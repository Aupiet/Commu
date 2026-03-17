#include "planner.h"
#include <math.h>
#include "globals.h"


#define MAPPING_DURATION 20000  // 20 secondes

// Pour la cnversion en binaire de la carte 
#define GRID_RESOLUTION 0.05f   // 5 cm
#define GRID_ORIGIN_X  -2.0f
#define GRID_ORIGIN_Y  -2.0f

typedef enum {
    MODE_MAPPING,
    MODE_AUTONOMOUS
} SystemMode;

SystemMode currentMode = MODE_MAPPING;

unsigned long startTime = 0;

uint8_t grid[MAX_H][MAX_W];
Point path[MAX_PATH];
int pathLength = 0;

typedef struct {
    uint16_t g, f;
    int16_t parent_x, parent_y;
    uint8_t open, closed;
} Node;

static Node nodes[MAX_H][MAX_W];

void setTestGrid() {

    for(int y=0;y<MAX_H;y++)
        for(int x=0;x<MAX_W;x++)
            grid[y][x]=0;

    for(int i=20;i<180;i++)
        grid[100][i]=1;
}

float heuristic(int x1,int y1,int x2,int y2){
    float dx=x1-x2, dy=y1-y2;
    return sqrtf(dx*dx+dy*dy);
}

int findLowestF(int w,int h){
    float best=1e9;
    int bx=-1,by=-1;

    for(int y=0;y<h;y++)
        for(int x=0;x<w;x++)
            if(nodes[y][x].open && nodes[y][x].f<best){
                best=nodes[y][x].f;
                bx=x;by=y;
            }

    return (bx==-1)?-1:(by*w+bx);
}

void reconstructPath(int gx,int gy,int sx,int sy){
    pathLength=0;
    int cx=gx, cy=gy;

    while(!(cx==sx && cy==sy) && pathLength<MAX_PATH){
        path[pathLength++] = (Point){cx,cy};
        int px=nodes[cy][cx].parent_x;
        int py=nodes[cy][cx].parent_y;
        cx=px; cy=py;
    }
}

int runAStar(int sx,int sy,int gx,int gy){

    for(int y=0;y<MAX_H;y++)
        for(int x=0;x<MAX_W;x++){
            nodes[y][x].g=1e9;
            nodes[y][x].open=0;
            nodes[y][x].closed=0;
        }

    nodes[sy][sx].g=0;
    nodes[sy][sx].f=heuristic(sx,sy,gx,gy);
    nodes[sy][sx].open=1;

    while(1){
        int idx=findLowestF(MAX_W,MAX_H);
        if(idx<0) return 0;

        int cx=idx%MAX_W;
        int cy=idx/MAX_W;

        if(cx==gx && cy==gy){
            reconstructPath(gx,gy,sx,sy);
            return 1;
        }

        nodes[cy][cx].open=0;
        nodes[cy][cx].closed=1;

        int dx[4]={1,-1,0,0};
        int dy[4]={0,0,1,-1};

        for(int i=0;i<4;i++){
            int nx=cx+dx[i], ny=cy+dy[i];

            if(nx<0||ny<0||nx>=MAX_W||ny>=MAX_H) continue;
            if(grid[ny][nx]) continue;
            if(nodes[ny][nx].closed) continue;

            float newG=nodes[cy][cx].g+1;

            if(!nodes[ny][nx].open || newG<nodes[ny][nx].g){
                nodes[ny][nx].g=newG;
                nodes[ny][nx].f=newG+heuristic(nx,ny,gx,gy);
                nodes[ny][nx].parent_x=cx;
                nodes[ny][nx].parent_y=cy;
                nodes[ny][nx].open=1;
            }
        }
    }
}

int runDijkstra(int sx, int sy, int gx, int gy)
{
    static uint8_t visited[MAX_H][MAX_W];
    static int16_t parent_x[MAX_H][MAX_W];
    static int16_t parent_y[MAX_H][MAX_W];

    // reset
    for(int y=0;y<MAX_H;y++)
        for(int x=0;x<MAX_W;x++)
            visited[y][x] = 0;

    int queue[MAX_W * MAX_H][2];
    int front = 0;
    int back = 0;

    queue[back][0] = sx;
    queue[back][1] = sy;
    back++;

    visited[sy][sx] = 1;

    int dx[4] = {1,-1,0,0};
    int dy[4] = {0,0,1,-1};

    while(front < back)
    {
        int x = queue[front][0];
        int y = queue[front][1];
        front++;

        if(x == gx && y == gy)
        {
            // reconstruction chemin
            pathLength = 0;
            int cx = gx, cy = gy;

            while(!(cx == sx && cy == sy) && pathLength < MAX_PATH)
            {
                path[pathLength++] = (Point){cx, cy};

                int px = parent_x[cy][cx];
                int py = parent_y[cy][cx];

                cx = px;
                cy = py;
            }

            path[pathLength++] = (Point){sx, sy};

            return 1;
        }

        for(int i=0;i<4;i++)
        {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if(nx<0 || ny<0 || nx>=MAX_W || ny>=MAX_H)
                continue;

            if(grid[ny][nx] || visited[ny][nx])
                continue;

            visited[ny][nx] = 1;
            parent_x[ny][nx] = x;
            parent_y[ny][nx] = y;

            queue[back][0] = nx;
            queue[back][1] = ny;
            back++;
        }
    }

    return 0;
}

void buildGridFromMap()
{
    // 1. reset grille
    for(int y=0;y<MAX_H;y++)
        for(int x=0;x<MAX_W;x++)
            grid[y][x] = 0;

    // 2. remplir avec obstacles LiDAR
    if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for(int i = 0; i < pointsAvailable; i++)
        {
            int idx = (pointWriteIndex - i + POINT_BUFFER_SIZE) % POINT_BUFFER_SIZE;

            float px = pointBuffer[idx].x; // mètres
            float py = pointBuffer[idx].y;

            int gx = (int)((px - GRID_ORIGIN_X) / GRID_RESOLUTION);
            int gy = (int)((py - GRID_ORIGIN_Y) / GRID_RESOLUTION);

            if(gx >= 0 && gy >= 0 && gx < MAX_W && gy < MAX_H)
            {
                grid[gy][gx] = 1; // obstacle
            }
        }

        xSemaphoreGive(bufferMutex);
    }

    Serial.println("[PLANNER] Grid built from LiDAR");
}

void plannerTask(void *pv) {

    startTime = millis();

    while (true) {

        // 🔄 Transition automatique
        if (currentMode == MODE_MAPPING &&
            millis() - startTime > MAPPING_DURATION) {

            currentMode = MODE_AUTONOMOUS;
            naifEnabled = false;

            Serial.println(">>> SWITCH TO AUTONOMOUS <<<");

            // Construire grille depuis map (à faire)
            //buildGridFromMap();

            // Exemple goal
            runAStar(10,10,150,150);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
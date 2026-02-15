#include "frontier_detector.hpp"
#include "naive_navigation.h"

// Transforme coordonnées robot -> cellule
inline void worldToGrid(float wx, float wy, const OccupancyGrid& map, int &gx, int &gy){
    gx = int(wx/map.resolution);
    gy = int(wy/map.resolution);
}

// Calcule distance euclidienne
inline float euclideanDistance(float x1,float y1,float x2,float y2){
    return sqrtf((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Explorer les frontiers et définir un goal global
void exploreFrontiers(const OccupancyGrid& map, float robot_x, float robot_y){
    int rx, ry;
    worldToGrid(robot_x, robot_y, map, rx, ry);

    std::unordered_map<int,bool> frontier_map;
    std::vector<Frontier> frontiers;

    // Parcours toute la map pour détecter frontiers
    for(int y=0;y<map.height;y++)
        for(int x=0;x<map.width;x++)
            if(is_new_frontier_cell(map,x,y,frontier_map))
                frontiers.push_back(buildNewFrontier(map,x,y,frontier_map));

    if(frontiers.empty()) return;

    // Choisir la meilleure frontier
    Frontier best = frontiers[0];
    float min_cost = euclideanDistance(robot_x, robot_y,
                                       best.centroid_x*map.resolution,
                                       best.centroid_y*map.resolution)/best.size;

    for(auto &f: frontiers){
        float cost = euclideanDistance(robot_x, robot_y,
                                       f.centroid_x*map.resolution,
                                       f.centroid_y*map.resolution)/f.size;
        if(cost<min_cost){ min_cost=cost; best=f; }
    }

    // Envoyer le goal à la navigation locale
    setGlobalGoal(best.centroid_x*map.resolution, best.centroid_y*map.resolution);
}

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

typedef struct {
    float x;
    float y;
} Waypoint;

void purePursuitTask(void *pvParameters);

#endif
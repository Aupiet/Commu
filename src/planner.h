#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>

#define MAX_W 80
#define MAX_H 80
#define MAX_PATH 1000

typedef struct {
    int x;
    int y;
} Point;

extern uint8_t grid[MAX_H][MAX_W];
extern Point path[MAX_PATH];
extern int pathLength;

// Algo
int runAStar(int sx, int sy, int gx, int gy);
int runDijkstra(int sx, int sy, int gx, int gy);

// Utils
void buildGridFromMap();   // TODO: conversion ROS map
void setTestGrid();        // test local

// Ensemble planning trajectory (incluant les algoritmes)
void plannerTask(void *pv);

#endif
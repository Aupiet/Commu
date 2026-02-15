#pragma once

#include <stdint.h>
#include <vector>

// Valeurs occupancy grid classiques
#define CELL_UNKNOWN   -1
#define CELL_FREE       0
#define CELL_OCCUPIED 100

// Seuil libre (comme dans ton pseudo-code)
#define FREE_THRESHOLD 50

struct GridCell {
  int x;
  int y;
};

// Vérifie si une cellule est une frontière
bool isFrontierCell(
  const std::vector<int8_t>& map,
  int width,
  int height,
  int x,
  int y,
  const std::vector<bool>& frontier_flags
);

// Détection globale des frontières
void detectFrontiers(
  const std::vector<int8_t>& map,
  int width,
  int height,
  std::vector<GridCell>& frontiers
);

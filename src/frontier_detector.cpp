#include "frontier_detector.h"

// =============================
// Helpers internes
// =============================

static inline bool inBounds(int width, int height, int x, int y) {
  return (x >= 0 && x < width && y >= 0 && y < height);
}

static inline int indexOf(int width, int x, int y) {
  return y * width + x;
}

// =============================
// isFrontierCell
// =============================

bool isFrontierCell(
  const std::vector<int8_t>& map,
  int width,
  int height,
  int x,
  int y,
  const std::vector<bool>& frontier_flags
) {
  // 1️⃣ Vérifier bornes
  if (!inBounds(width, height, x, y))
    return false;

  int idx = indexOf(width, x, y);

  // 2️⃣ Doit être UNKNOWN
  if (map[idx] != CELL_UNKNOWN)
    return false;

  // 3️⃣ Pas déjà frontier
  if (frontier_flags[idx])
    return false;

  // 4️⃣ Vérifier voisins 4-connexes
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};

  for (int i = 0; i < 4; i++) {
    int nx = x + dx[i];
    int ny = y + dy[i];

    if (!inBounds(width, height, nx, ny))
      continue;

    int nidx = indexOf(width, nx, ny);
    int8_t val = map[nidx];

    // voisin libre ?
    if (val >= 0 && val < FREE_THRESHOLD)
      return true;
  }

  return false;
}

// =============================
// detectFrontiers
// =============================

void detectFrontiers(
  const std::vector<int8_t>& map,
  int width,
  int height,
  std::vector<GridCell>& frontiers
) {
  frontiers.clear();

  // masque pour éviter doublons
  std::vector<bool> frontier_flags(width * height, false);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      if (isFrontierCell(map, width, height, x, y, frontier_flags)) {
        int idx = indexOf(width, x, y);
        frontier_flags[idx] = true;

        GridCell c;
        c.x = x;
        c.y = y;
        frontiers.push_back(c);
      }
    }
  }
}

#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>
#include "boids_omp_aos.h"

// hash key per celle della griglia
struct CellKey {
    int x, y;
    bool operator==(const CellKey& other) const {
        return x == other.x && y == other.y;
    }
};

// hash function per unordered_map
struct CellKeyHash {
    std::size_t operator()(const CellKey& k) const {
        return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1);
    }
};

// griglia uniforme 2D
class SpatialGrid {
public:
    float cellSize; // dimensione cella
    std::unordered_map<CellKey, std::vector<Boid*>, CellKeyHash> cells;

    SpatialGrid(float _cellSize) : cellSize(_cellSize) {}

    void clear() {
        cells.clear();
    }

    // calcola coordinate cella da posizione
    CellKey getCell(float x, float y) const {
        return { static_cast<int>(std::floor(x / cellSize)),static_cast<int>(std::floor(y / cellSize)) };
    }

    void insert(Boid* boid) {
        CellKey key = getCell(boid->x, boid->y);
        cells[key].push_back(boid);
    }

    // restituisce tutti i boid nella cella e nelle 8 celle adiacenti
    std::vector<Boid*> get_neighbors(Boid* boid) const {
        std::vector<Boid*> neighbors;
        CellKey base = getCell(boid->x, boid->y);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                CellKey key{base.x + dx, base.y + dy};
                auto it = cells.find(key);
                if (it != cells.end()) {
                    neighbors.insert(neighbors.end(), it->second.begin(), it->second.end());
                }
            }
        }
        return neighbors;
    }
};

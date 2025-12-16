#pragma once

#include <vector>
#include <cmath>

// Range leggero per iterare sugli indici dei boids
struct NeighborRange {
    const int* beginPtr;
    const int* endPtr;

    const int* begin() const
    {
        return beginPtr;
    }

    const int* end()   const
    {
        return endPtr;
    }
};

class SpatialGrid {
public:
    float cellSize;

    SpatialGrid(float cellSize, int worldWidth, int worldHeight, int maxBoids) : cellSize(cellSize), worldWidth(worldWidth), worldHeight(worldHeight), maxBoids(maxBoids)
    {
        gridWidth  = (int)std::ceil(worldWidth  / cellSize);
        gridHeight = (int)std::ceil(worldHeight / cellSize);
        numCells   = gridWidth * gridHeight;

        cellCount.resize(numCells);
        cellStart.resize(numCells + 1);
        boidIndices.resize(maxBoids);
    }

    // da chiamare ogni frame
    void clear() {
        std::fill(cellCount.begin(), cellCount.end(), 0);
    }

    // inserimento boid (equivalente di insert(Boid*))
    inline void insert(int boidIndex, float x, float y) {
        int c = cell_index(x, y);
        cellCount[c]++;
    }

    // chiamare dopo tutti gli insert()
    void build() {
        // prefix sum
        cellStart[0] = 0;
        for (int c = 0; c < numCells; ++c)
            cellStart[c + 1] = cellStart[c] + cellCount[c];

        // reset contatori temporanei
        std::fill(cellCount.begin(), cellCount.end(), 0);
    }

    // seconda fase di inserimento (thread-safe con atomics esterni)
    inline void insert_index(int boidIndex, float x, float y) {
        int c = cell_index(x, y);
        int offset = cellStart[c] + cellCount[c]++;
        boidIndices[offset] = boidIndex;
    }

    // restituisce i boids nella cella (non i vicini!)
    NeighborRange get_cell(int cell) const {
        return
        {
            &boidIndices[cellStart[cell]],
            &boidIndices[cellStart[cell + 1]]
        };
    }

    // equivalente di get_neighbors(Boid*): restituisce una cella alla volta
    inline int cell_of(float x, float y) const {
        return cell_index(x, y);
    }

    inline bool valid_cell(int cx, int cy) const {
        return cx >= 0 && cy >= 0 && cx < gridWidth && cy < gridHeight;
    }

    inline int flatten(int cx, int cy) const {
        return cy * gridWidth + cx;
    }

    inline void cell_coords(int cell, int& cx, int& cy) const {
        cx = cell % gridWidth;
        cy = cell / gridWidth;
    }

private:
    int worldWidth, worldHeight;
    int gridWidth, gridHeight;
    int numCells;
    int maxBoids;

    std::vector<int> cellCount;   // temporaneo / contatore
    std::vector<int> cellStart;   // prefix sum
    std::vector<int> boidIndices; // indici boids

    inline int cell_index(float x, float y) const {
        int cx = (int)std::floor(x / cellSize);
        int cy = (int)std::floor(y / cellSize);
        return cy * gridWidth + cx;
    }
};

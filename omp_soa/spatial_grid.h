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

class SpatialGrid
{
public:
    SpatialGrid(float cellSize, int worldWidth, int worldHeight, int maxBoids)
    {
        this->cellSize = cellSize;
        this->worldWidth = worldWidth;
        this->worldHeight = worldHeight;
        this->maxBoids = maxBoids;

        gridWidth  = static_cast<int>(std::ceil(worldWidth  / cellSize));
        gridHeight = static_cast<int>(std::ceil(worldHeight / cellSize));
        numCells   = gridWidth * gridHeight;

        cellCount.resize(numCells);
        cellStart.resize(numCells + 1);
        boidIndices.resize(maxBoids);
    }

    // svuota la griglia
    void clear()
    {
        std::fill(cellCount.begin(), cellCount.end(), 0);
    }

    // inserisci un boid (prima fase)
    void insert(float x, float y)
    {
        int cell = world_to_cell(x, y);
        cellCount[cell]++;
    }

    // calcola il contenuto della griglia
    void build()
    {
        cellStart[0] = 0;
        for (int c = 0; c < numCells; ++c)
            cellStart[c + 1] = cellStart[c] + cellCount[c];

        std::fill(cellCount.begin(), cellCount.end(), 0);
    }

    // inserisci un boid (seconda fase)
    void insert_index(int boidIndex, float x, float y)
    {
        int cell   = world_to_cell(x, y);
        int offset = cellStart[cell] + cellCount[cell]++;
        boidIndices[offset] = boidIndex;
    }

    // leggi il contenuto di una cella
    NeighborRange cell_content_at(float x, float y) const
    {
        int cell = world_to_cell(x, y);
        return {
            &boidIndices[cellStart[cell]],
            &boidIndices[cellStart[cell + 1]]
        };
    }

private:
    float cellSize;
    int worldWidth;
    int worldHeight;

    int gridWidth;
    int gridHeight;
    int numCells;
    int maxBoids;

    std::vector<int> cellCount;
    std::vector<int> cellStart;
    std::vector<int> boidIndices;

    // trasforma coordinate da world a grid 1d
    inline int world_to_cell(float x, float y) const
    {
        // trasformazione coordinate world to grid 2d
        int cx = static_cast<int>(std::floor(x / cellSize));
        int cy = static_cast<int>(std::floor(y / cellSize));

        // clamp indici griglia
        cx = std::clamp(cx, 0, gridWidth  - 1);
        cy = std::clamp(cy, 0, gridHeight - 1);

        // flatten da 2d a 1d
        return cy * gridWidth + cx;
    }
};

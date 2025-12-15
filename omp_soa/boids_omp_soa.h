#pragma once

// struttura per rappresentare dei boids
struct Boids {
    float* x;
    float* y;
    float* vx;
    float* vy;
    int count;
};

// funzione per aggiornare la posizione di tutti i boids
void update_all_boids(const Boids& boids, Boids& new_boids, float deltaTime, int windowWidth, int windowHeight);

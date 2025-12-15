#pragma once

// struttura per rappresentare un boid
typedef struct {
    float x, y;      // posizione
    float vx, vy;    // velocità
} Boid;

// funzione per aggiornare la posizione di tutti i boids
void update_all_boids(const Boid* boids, Boid* new_boids, int num_boids, float deltaTime, int windowWidth, int windowHeight);

// funzione per aggiornare la posizione di un boid
void update_boid_position(Boid* boid, const Boid* otherboids, int num_boids, float deltaTime, int windowWidth, int windowHeight);

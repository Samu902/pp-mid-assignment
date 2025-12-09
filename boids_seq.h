#ifndef BOIDS_SEQ_H
#define BOIDS_SEQ_H

// struttura per rappresentare un boid
typedef struct {
    float x, y;      // posizione
    float vx, vy;    // velocità
} Boid;

// funzione per aggiornare la posizione di un boid
void update_boid_position(Boid* boid, Boid* otherboids, int num_boids, float deltaTime, int windowWidth, int windowHeight);

#endif
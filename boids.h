#ifndef BOIDS_H
#define BOIDS_H

// struttura per rappresentare un boid
typedef struct {
    float x, y;      // posizione
    float vx, vy;    // velocità
    float biasval;   // bias
    int scout_group; // gruppo scout (1 o 2)
} Boid;

// funzione per aggiornare la posizione di un boid
void update_boid_position(Boid* boid, Boid* otherboids, int num_boids);

#endif
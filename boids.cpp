#include "boids.h"
#include <cmath>

#define visual_range 40.0f
#define visual_range_squared (visual_range * visual_range)
#define protected_range 8.0f
#define protected_range_squared (protected_range * protected_range)
#define centering_factor 0.0005f
#define matching_factor 0.05f
#define avoid_factor 0.05f
#define turn_factor 0.2f
#define max_bias 0.01f
#define bias_increment 0.00004f
#define min_speed 3.0f
#define max_speed 6.0f

// The user provides:
// - number of boids in scout group 1
// - number of boids in scout group 2
// - biasval for scout group 1
// - biasval for scout group 2
// default biasval: 0.001 (user-changeable, or updated dynamically)

// funzione per aggiornare la posizione di un boid
void update_boid_position(Boid* boid, Boid* otherboids, const int num_boids, float deltaTime) {
    // inizializza le variabili necessarie
    float xpos_avg = 0.0f, ypos_avg = 0.0f;
    float xvel_avg = 0.0f, yvel_avg = 0.0f;
    int neighboring_boids = 0;
    float close_dx = 0.0f, close_dy = 0.0f;

    // itera su tutti gli altri boids dello stormo
    for (int i = 0; i < num_boids; i++) {
        Boid* otherboid = &otherboids[i];

        // calcola la differenza di posizione con l'altro poid
        float dx = boid->x - otherboid->x;
        float dy = boid->y - otherboid->y;

        // le due differenze sono minori del visual range?
        if (std::fabs(dx) < visual_range && std::fabs(dy) < visual_range) {
            float squared_distance = dx * dx + dy * dy;

            // il quadrato della distanza è minore del quadrato del protected range?
            if (squared_distance < protected_range_squared) {
                close_dx += dx;
                close_dy += dy;
            } else if (squared_distance < visual_range_squared) { // il quadrato della distanza è minore del quadrato del visual range?
                // aggiungi i contributi per calcolare il centro dello stormo
                xpos_avg += otherboid->x;
                ypos_avg += otherboid->y;
                xvel_avg += otherboid->vx;
                yvel_avg += otherboid->vy;
                neighboring_boids++;
            }
        }
    }

    // se ci sono boids vicini, calcola il centro dello stormo
    if (neighboring_boids > 0) {
        xpos_avg /= static_cast<float>(neighboring_boids);
        ypos_avg /= static_cast<float>(neighboring_boids);
        xvel_avg /= static_cast<float>(neighboring_boids);
        yvel_avg /= static_cast<float>(neighboring_boids);

        // aggiungi alla velocità del boid il centering e il matching factor per avvicinarsi al centro dello stormo
        boid->vx += (xpos_avg - boid->x) * centering_factor + (xvel_avg - boid->vx) * matching_factor;
        boid->vy += (ypos_avg - boid->y) * centering_factor + (yvel_avg - boid->vy) * matching_factor;
    }

    // aggiungi alla velocità il avoid factor per allontanarsi dai boids vicini
    boid->vx += close_dx * avoid_factor;
    boid->vy += close_dy * avoid_factor;

    // gestione dei margini dello schermo
    if (boid->y < 0) boid->vy += turn_factor;
    if (boid->x > 100) boid->vx -= turn_factor;
    if (boid->x < 0) boid->vx += turn_factor;
    if (boid->y > 100) boid->vy -= turn_factor;

    // calcolo dei bias: scout group 1 con bias verso sx, l'altro con bias verso dx
    if (boid->scout_group == 1) {
        if (boid->vx > 0) {
            boid->biasval = std::fmin(max_bias, boid->biasval + bias_increment);
        } else {
            boid->biasval = std::fmax(bias_increment, boid->biasval - bias_increment);
        }
    } else if (boid->scout_group == 2) {
        if (boid->vx < 0) {
            boid->biasval = std::fmin(max_bias, boid->biasval + bias_increment);
        } else {
            boid->biasval = std::fmax(bias_increment, boid->biasval - bias_increment);
        }
    }

    // applica i bias alla velocità
    if (boid->scout_group == 1) {
        boid->vx = (1 - boid->biasval) * boid->vx + boid->biasval * 1;
    } else if (boid->scout_group == 2) {
        boid->vx = (1 - boid->biasval) * boid->vx + boid->biasval * (-1);
    }

    //# Calculate the boid's speed
    //# Slow step! Lookup the "alpha max plus beta min" algorithm
    // calcolo della norma della velocità e clamp fra minima e massima velocità
    float speed = sqrtf(boid->vx * boid->vx + boid->vy * boid->vy);
    if (speed < min_speed) {
        boid->vx = (boid->vx / speed) * min_speed;
        boid->vy = (boid->vy / speed) * min_speed;
    }
    if (speed > max_speed) {
        boid->vx = (boid->vx / speed) * max_speed;
        boid->vy = (boid->vy / speed) * max_speed;
    }

    // aggiornamento finale della posizione
    boid->x += boid->vx * deltaTime;
    boid->y += boid->vy * deltaTime;
}

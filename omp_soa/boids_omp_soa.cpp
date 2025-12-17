#include <cmath>

#ifdef _OPENMP
#include <omp.h> // for OpenMP library functions
#endif

#include "boids_omp_soa.h"
#include "spatial_grid.h"

#define visual_range 40.0f
#define visual_range_squared (visual_range * visual_range)
#define protected_range 8.0f
#define protected_range_squared (protected_range * protected_range)
#define centering_factor 0.002f
#define matching_factor 0.05f
#define avoid_factor 0.025f
#define turn_factor 0.4f
#define min_speed 3.0f
#define max_speed 6.0f

#define spatial_partitioning_on true

// algoritmo riadattato da https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html

// funzione per aggiornare le posizioni di tutti i boids
void update_all_boids(const Boids& boids, Boids& new_boids, float deltaTime, int windowWidth, int windowHeight)
{
    #if spatial_partitioning_on
    // inizializzazione grid
    SpatialGrid grid(visual_range, windowWidth, windowHeight, boids.count);
    grid.clear();

    // fase 1: conteggio
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < boids.count; ++i) {
        grid.insert(boids.x[i], boids.y[i]);
    }

    // Prefix sum (seriale, costo trascurabile)
    grid.build();

    // fase 2: riempimento
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < boids.count; ++i) {
        grid.insert_index(i, boids.x[i], boids.y[i]);
    }
    #endif

    #pragma omp parallel for schedule(static)
    for (int i = 0; i < boids.count; ++i) {

        // copia stato (double buffering)
        new_boids.x[i] = boids.x[i];
        new_boids.y[i] = boids.y[i];
        new_boids.vx[i] = boids.vx[i];
        new_boids.vy[i] = boids.vy[i];

        // inizializza le variabili necessarie
        float xpos_avg = 0.0f, ypos_avg = 0.0f;
        float xvel_avg = 0.0f, yvel_avg = 0.0f;
        int neighboring_boids = 0;
        float close_dx = 0.0f, close_dy = 0.0f;

        #if spatial_partitioning_on
        // visita cella + 8 adiacenti (in world space)
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {

                float sample_x = boids.x[i] + dx * visual_range;
                float sample_y = boids.y[i] + dy * visual_range;

                auto range = grid.cell_content_at(sample_x, sample_y);
                for (const int* it = range.begin(); it != range.end(); ++it)
                {
                    int j = *it;
                    if (j == i)
                        continue;

                    // calcola la differenza di posizione con l'altro boid
                    float dxw = boids.x[i] - boids.x[j];
                    float dyw = boids.y[i] - boids.y[j];

                    // le due differenze sono minori del visual range?
                    if (std::fabs(dxw) < visual_range && std::fabs(dyw) < visual_range) {
                        const float squared_distance = dxw * dxw + dyw * dyw;

                        // il quadrato della distanza è minore del quadrato del protected range?
                        if (squared_distance < protected_range_squared) {
                            close_dx += dxw;
                            close_dy += dyw;
                        } else if (squared_distance < visual_range_squared) { // il quadrato della distanza è minore del quadrato del visual range?
                            // aggiungi i contributi per calcolare il centro dello stormo
                            xpos_avg += boids.x[j];
                            ypos_avg += boids.y[j];
                            xvel_avg += boids.vx[j];
                            yvel_avg += boids.vy[j];
                            neighboring_boids++;
                        }
                    }
                }
            }
        }
        #else
        // itera su tutti gli altri boids dello stormo
        #pragma omp simd reduction(+:xpos_avg,ypos_avg,xvel_avg,yvel_avg,close_dx,close_dy,neighboring_boids)
        for (int j = 0; j < boids.count; ++j) {

            // calcola la differenza di posizione con l'altro boid
            float dx = boids.x[i] - boids.x[j];
            float dy = boids.y[i] - boids.y[j];

            // le due differenze sono minori del visual range?
            if (std::fabs(dx) < visual_range && std::fabs(dy) < visual_range) {
                const float squared_distance = dx * dx + dy * dy;

                // il quadrato della distanza è minore del quadrato del protected range?
                if (squared_distance < protected_range_squared) {
                    close_dx += dx;
                    close_dy += dy;
                } else if (squared_distance < visual_range_squared) { // il quadrato della distanza è minore del quadrato del visual range?
                    // aggiungi i contributi per calcolare il centro dello stormo
                    xpos_avg += boids.x[j];
                    ypos_avg += boids.y[j];
                    xvel_avg += boids.vx[j];
                    yvel_avg += boids.vy[j];
                    neighboring_boids++;
                }
            }
        }
        #endif

        float vx = new_boids.vx[i];
        float vy = new_boids.vy[i];

        // se ci sono boids vicini, calcola il centro dello stormo
        if (neighboring_boids > 0) {
            xpos_avg /= static_cast<float>(neighboring_boids);
            ypos_avg /= static_cast<float>(neighboring_boids);
            xvel_avg /= static_cast<float>(neighboring_boids);
            yvel_avg /= static_cast<float>(neighboring_boids);

            // aggiusta la velocità del boid con il centering factor per avvicinarsi al centro dei vicini (cohesion rule)
            vx += (xpos_avg - new_boids.x[i]) * centering_factor;
            vy += (ypos_avg - new_boids.y[i]) * centering_factor;

            // aggiusta la velocità del boid con il matching factor per avvicinarsi alla velocità media dei vicini (alignment rule)
            vx += (xvel_avg - vx) * matching_factor;
            vy += (yvel_avg - vy) * matching_factor;
        }

        // aggiusta la velocità con il avoid factor per allontanarsi dai boids vicini (separation rule)
        vx += close_dx * avoid_factor;
        vy += close_dy * avoid_factor;

        // gestione dei margini dello schermo
        if (new_boids.y[i] < windowHeight * 0.05f) vy += turn_factor;
        if (new_boids.x[i] > windowWidth  * 0.95f) vx -= turn_factor;
        if (new_boids.x[i] < windowWidth  * 0.05f) vx += turn_factor;
        if (new_boids.y[i] > windowHeight * 0.95f) vy -= turn_factor;

        //# Calculate the boid's speed
        //# Slow step! Lookup the "alpha max plus beta min" algorithm
        // calcolo della norma della velocità e clamp fra minima e massima velocità
        float speed = sqrtf(vx * vx + vy * vy);
        if (speed < min_speed) {
            vx = (vx / speed) * min_speed;
            vy = (vy / speed) * min_speed;
        }
        if (speed > max_speed) {
            vx = (vx / speed) * max_speed;
            vy = (vy / speed) * max_speed;
        }

        // aggiornamento finale della posizione (e della velocità)
        new_boids.vx[i] = vx;
        new_boids.vy[i] = vy;
        new_boids.x[i] += vx * deltaTime;
        new_boids.y[i] += vy * deltaTime;
    }
}

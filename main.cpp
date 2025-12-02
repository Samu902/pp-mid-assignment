#ifdef _OPENMP
#include <omp.h> // for OpenMP library functions
#endif
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "boids.h"

#define windowWidth 1280
#define windowHeight 720

#define speedUpSimulation 50

int main(int argc, char* argv[]) {
    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Boids Simulation");

    const int numBoids = 1000; // numero di agenti (boids)
    Boid* boids = new Boid[numBoids];
    sf::VertexArray boidsQuads(sf::Quads, numBoids * 4); // ogni boid è un quadrato (4 vertici)
    const int quadSize = 3;

    // inizializzazione boids
    for (int i = 0; i < numBoids; ++i) {
        // posizione iniziale random
        float x = rand() % windowWidth;
        float y = rand() % windowHeight;

        boids[i].x = x;
        boids[i].y = y;
        boids[i].vx = 0;
        boids[i].vy = 0;
        boids[i].scout_group = rand() % 2;
        boids[i].biasval = 0.001f;
    }

    // Ciclo principale
    sf::Clock clock;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        sf::Time deltaTime = clock.restart();

        // aggiorna i boids
        for (int i = 0; i < numBoids; ++i)
        {
            // calcola il nuovo stato del boid
            update_boid_position(&boids[i], boids, numBoids, deltaTime.asSeconds() * speedUpSimulation);

            float x = boids[i].x;
            float y = boids[i].y;

            // disegna i 4 vertici del quadrato (il boid)
            boidsQuads[i * 4].position = sf::Vector2f(x - quadSize * 0.5f, y - quadSize * 0.5f);
            boidsQuads[i * 4 + 1].position = sf::Vector2f(x + quadSize * 0.5f, y - quadSize * 0.5f);
            boidsQuads[i * 4 + 2].position = sf::Vector2f(x + quadSize * 0.5f, y + quadSize * 0.5f);
            boidsQuads[i * 4 + 3].position = sf::Vector2f(x - quadSize * 0.5f, y + quadSize * 0.5f);

            // colore del boid
            sf::Color boidColor(255, 255, 255);
            for (int j = 0; j < 4; ++j) {
                boidsQuads[i * 4 + j].color = boidColor;
            }
        }

        window.clear();
        window.draw(boidsQuads); // disegna tutti i quadrati (boids)
        window.display();
    }
}
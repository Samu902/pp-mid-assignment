#ifdef _OPENMP
#include <omp.h> // for OpenMP library functions
#endif
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#define windowWidth 1280
#define windowHeight 720

int main(int argc, char* argv[]) {
    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Boids Simulation");

    const int numBoids = 10; // numero di agenti (boids)
    sf::VertexArray boids(sf::Quads, numBoids * 4); // ogni boid è un quadrato (4 vertici)

    // Popoliamo i vertici con posizioni random (o la tua logica di movimento)
    for (int i = 0; i < numBoids; ++i) {
        // posizione iniziale random
        float x = rand() % windowWidth;
        float y = rand() % windowHeight;

        // disegna i 4 vertici del quadrato (il boid)
        boids[i * 4].position = sf::Vector2f(x, y);
        boids[i * 4 + 1].position = sf::Vector2f(x + 10, y);
        boids[i * 4 + 2].position = sf::Vector2f(x + 10, y + 10);
        boids[i * 4 + 3].position = sf::Vector2f(x, y + 10);

        // colore dei boids
        sf::Color boidColor(255, 255, 255);
        for (int j = 0; j < 4; ++j) {
            boids[i * 4 + j].color = boidColor;
        }
    }

    // Ciclo principale
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(boids); // Disegna tutti i quadrati (boids)
        window.display();
    }
}
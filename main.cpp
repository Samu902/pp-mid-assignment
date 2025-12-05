#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "boids.h"

int main(int argc, char* argv[])
{
    // esegui il programma per un numero diverso di agenti (boids)
    const int numberOfAgentsCases = 6;
    const int numberOfAgents[numberOfAgentsCases] = {100, 500, 1000, 2000, 5000, 10000};
    // esegui il ogni caso di agents per tot volte
    const int numberOfRuns = 10;
    // fattore di velocità globale di simulazione
    const float speedUpSimulation = 50;
    // esegui il programma per un certo numero di time steps
    const int maxTimeSteps = 2000;

    // array per raccogliere i tempi di esecuzione
    float simulationTimes[numberOfAgentsCases][numberOfRuns];

    // file di log per salvare i risultati
    std::ofstream logFile("logfile.txt");
    if (!logFile.is_open())
        std::cerr << "Error opening log file." << std::endl;

    const int windowWidth = 1280;
    const int windowHeight = 720;
    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Boids Simulation");

    for (int ai = 0; ai < numberOfAgentsCases; ai++)
    {
        for (int ri = 0; ri < numberOfRuns; ri++)
        {
            std::cout << "Inizio simulazione n. " << (ri + 1) << " su " << numberOfAgents[ai] << " boids." << std::endl;
            window.setTitle("Boids Simulation (n. " + std::to_string(ri + 1) + ", " + std::to_string(numberOfAgents[ai]) + " agents)");

            Boid* boids = new Boid[numberOfAgents[ai]];
            sf::VertexArray* boidsQuads = new sf::VertexArray(sf::Quads, numberOfAgents[ai] * 4); // ogni boid è un quadrato (4 vertici)
            const int quadSize = 3;

            // inizializzazione boids
            for (int i = 0; i < numberOfAgents[ai]; ++i) {
                // posizione iniziale random e velocità nulla
                boids[i].x = rand() % windowWidth;
                boids[i].y = rand() % windowHeight;
                boids[i].vx = 0;
                boids[i].vy = 0;

                // colore del boid
                sf::Color boidColor(255, 255, 255);
                for (int j = 0; j < 4; ++j) {
                    (*boidsQuads)[i * 4 + j].color = boidColor;
                }
            }

            // ciclo principale di esecuzione
            float totalSimulationTime = 0.0f;
            int elapsedTimeSteps = 0;
            sf::Clock clock;
            while (window.isOpen())
            {
                sf::Event event;
                while (window.pollEvent(event))
                {
                    if (event.type == sf::Event::Closed)
                        window.close();
                }
                // clock SFML per smoothing della simulazione
                sf::Time deltaTime = clock.restart();

                // aggiorna lo stato dei boids
                for (int i = 0; i < numberOfAgents[ai]; ++i)
                {
                    // campiona il punto di inizio di questa iterazione con un clock ad alta risoluzione
                    auto start = std::chrono::high_resolution_clock::now();

                    // calcola il nuovo stato del boid
                    update_boid_position(&boids[i], boids, numberOfAgents[ai], deltaTime.asSeconds() * speedUpSimulation, windowWidth, windowHeight);

                    // campiona il punto di fine di questa iterazione con un clock ad alta risoluzione
                    auto stop = std::chrono::high_resolution_clock::now();

                    // somma al tempo di esecuzione dell'algoritmo sugli altri boids
                    totalSimulationTime += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

                    float x = boids[i].x;
                    float y = boids[i].y;

                    // disegna i 4 vertici del quadrato (il boid)
                    (*boidsQuads)[i * 4].position = sf::Vector2f(x - quadSize * 0.5f, y - quadSize * 0.5f);
                    (*boidsQuads)[i * 4 + 1].position = sf::Vector2f(x + quadSize * 0.5f, y - quadSize * 0.5f);
                    (*boidsQuads)[i * 4 + 2].position = sf::Vector2f(x + quadSize * 0.5f, y + quadSize * 0.5f);
                    (*boidsQuads)[i * 4 + 3].position = sf::Vector2f(x - quadSize * 0.5f, y + quadSize * 0.5f);
                }

                // disegno dei quads (boids) nella finestra SFML
                window.clear();
                window.draw(*boidsQuads);
                window.display();

                // qua dopo tot time steps esce dal ciclo
                elapsedTimeSteps++;
                if (elapsedTimeSteps % 50 == 0)
                    std::cout << "Time steps: " << elapsedTimeSteps << std::endl;
                if (elapsedTimeSteps >= maxTimeSteps)
                    break;
            }

            // dealloca gli array per evitare memory leaks
            delete[] boids;
            delete boidsQuads;

            // stampa della misurazione ottenuta a schermo
            std::cout << "Simulazione terminata dopo " << elapsedTimeSteps << " time steps." << std::endl;
            std::cout << "Tempo di esecuzione per simulazione boids: " << totalSimulationTime / 1000000 << " secondi." << std::endl;
            simulationTimes[ai][ri] = totalSimulationTime / 1000000;
        }
    }
    // chiudi la finestra alla fine di tutte le simulazioni
    window.close();

    // calcolo tempo di esecuzione medio per ogni numero di agenti e stampa delle misurazioni ottenute a schermo e su file di log
    for (int ai = 0; ai < numberOfAgentsCases; ai++)
    {
        float meanTime = 0.0f;
        for (int ri = 0; ri < numberOfRuns; ri++)
            meanTime += simulationTimes[ai][ri];
        meanTime /= numberOfRuns;
        std::cout << "Tempo di esecuzione medio per " << numberOfAgents[ai] << " boids: " << meanTime << " secondi." << std::endl;
        logFile << "Tempo di esecuzione medio per " << numberOfAgents[ai] << " boids: " << meanTime << " secondi." << std::endl;
    }
    logFile.close();
}
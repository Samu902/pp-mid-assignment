#ifdef _OPENMP
#include <omp.h> // for OpenMP library functions
#endif
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "boids_omp_soa.h"

#define visuals_on true
#define spatial_partitioning_on true

#if spatial_partitioning_on
#define log_file_name "logfile_omp_soa_sp.txt"
#else
#define log_file_name "logfile_omp_soa.txt"
#endif

int main(int argc, char* argv[])
{
    // esegui il programma per un certo numero di threads
    const int numberOfThreadsCases = 4;
    const int numberOfThreads[numberOfThreadsCases] = {2, 4, 6, 8};
    // esegui il programma per un numero diverso di agenti (boids)
    const int numberOfAgentsCases = 6;
    const int numberOfAgents[numberOfAgentsCases] = {100, 500, 1000, 2000, 5000, 10000};
    // esegui il ogni caso di agents per tot volte
    const int numberOfRuns = 10;
    // fattore di velocità globale di simulazione
    const float speedUpSimulation = 50;
    // esegui il programma per un certo numero di time steps
    const int maxTimeSteps = 1500;

    // array per raccogliere i tempi di esecuzione
    float simulationTimes[numberOfThreadsCases][numberOfAgentsCases][numberOfRuns];

    // file di log per salvare i risultati
    std::ofstream logFile(log_file_name);
    if (!logFile.is_open())
        std::cerr << "Error opening log file." << std::endl;

    // check OpenMP
    int nthreads, tid;
    tid = -1;
    #ifdef _OPENMP
    std::cout << "_OPENMP is defined." << std::endl;
    std::cout << "Number of processors (Phys+HT): " << omp_get_num_procs() << std::endl;
    #endif

    const int windowWidth = 1280;
    const int windowHeight = 720;
    #if visuals_on
    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Boids Simulation");
    #endif

    for (int ti = 0; ti < numberOfThreadsCases; ti++)
    {
        omp_set_num_threads(numberOfThreads[ti]);
        for (int ai = 0; ai < numberOfAgentsCases; ai++)
        {
            for (int ri = 0; ri < numberOfRuns; ri++)
            {
                std::cout << "Inizio simulazione n. " << (ri + 1) << " su " << numberOfAgents[ai] << " boids con " << numberOfThreads[ti] << " threads." << std::endl;
                #if visuals_on
                window.setTitle("Boids Simulation (n. " + std::to_string(ri + 1) + ", " + std::to_string(numberOfAgents[ai]) + " agents, " + std::to_string(numberOfThreads[ti]) + " threads)");
                #endif

                // inizializzazione stato boids (posizione iniziale random e velocità nulla), i buffer rappresentano rispettivamente lo stato corrente e successivo
                Boids boids = Boids{
                    .x = new float[numberOfAgents[ai]],
                    .y = new float[numberOfAgents[ai]],
                    .vx = new float[numberOfAgents[ai]],
                    .vy = new float[numberOfAgents[ai]],
                    .count = numberOfAgents[ai],
                };
                Boids new_boids = Boids{
                    .x = new float[numberOfAgents[ai]],
                    .y = new float[numberOfAgents[ai]],
                    .vx = new float[numberOfAgents[ai]],
                    .vy = new float[numberOfAgents[ai]],
                    .count = numberOfAgents[ai],
                };
                for (int i = 0; i < numberOfAgents[ai]; ++i)
                {
                    boids.x[i] = rand() % windowWidth;
                    boids.y[i] = rand() % windowHeight;
                    boids.vx[i] = 0;
                    boids.vy[i] = 0;
                }

                #if visuals_on
                // inizializzazione grafica dei boids: ogni boid è un quadrato bianco (4 vertici)
                sf::VertexArray* boidsQuads = new sf::VertexArray(sf::Quads, numberOfAgents[ai] * 4);
                const int quadSize = 3;
                for (int i = 0; i < numberOfAgents[ai]; ++i)
                {
                    sf::Color boidColor(255, 255, 255);
                    for (int j = 0; j < 4; ++j) {
                        (*boidsQuads)[i * 4 + j].color = boidColor;
                    }
                }
                #endif

                // ciclo principale di esecuzione
                float totalSimulationTime = 0.0f;
                int elapsedTimeSteps = 0;
                sf::Clock clock;
                while
                (
                    elapsedTimeSteps < maxTimeSteps
                    #if visuals_on
                    && window.isOpen()
                    #endif
                )
                {
                    #if visuals_on
                    sf::Event event;
                    while (window.pollEvent(event))
                    {
                        if (event.type == sf::Event::Closed)
                            window.close();
                    }
                    #endif
                    // clock SFML per smoothing della simulazione
                    sf::Time deltaTime = clock.restart();

                    // campiona il punto di inizio di questo time step con un clock ad alta risoluzione
                    auto start = std::chrono::high_resolution_clock::now();

                    // aggiorna lo stato dei boids
                    update_all_boids(boids, new_boids, deltaTime.asSeconds() * speedUpSimulation, windowWidth, windowHeight);

                    // campiona il punto di fine di questo time step con un clock ad alta risoluzione
                    auto stop = std::chrono::high_resolution_clock::now();

                    // somma al tempo di esecuzione dell'algoritmo sugli altri boids
                    totalSimulationTime += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

                    #if visuals_on
                    // aggiorna i 4 vertici dei quadrati (i boids)
#pragma omp parallel for schedule(static)
                    for (int i = 0; i < numberOfAgents[ai]; ++i)
                    {
                        float x = new_boids.x[i];
                        float y = new_boids.y[i];

                        (*boidsQuads)[i * 4].position = sf::Vector2f(x - quadSize * 0.5f, y - quadSize * 0.5f);
                        (*boidsQuads)[i * 4 + 1].position = sf::Vector2f(x + quadSize * 0.5f, y - quadSize * 0.5f);
                        (*boidsQuads)[i * 4 + 2].position = sf::Vector2f(x + quadSize * 0.5f, y + quadSize * 0.5f);
                        (*boidsQuads)[i * 4 + 3].position = sf::Vector2f(x - quadSize * 0.5f, y + quadSize * 0.5f);
                    }

                    // disegno dei quads (boids) nella finestra SFML
                    window.clear();
                    window.draw(*boidsQuads);
                    window.display();
                    #endif

                    // ricopio il nuovo buffer nel vecchio per il prossimo time step
                    std::swap(boids, new_boids);

                    // incremento time steps e stampa intervalli intermedi
                    elapsedTimeSteps++;
                    if (elapsedTimeSteps % 50 == 0)
                        std::cout << "Time steps: " << elapsedTimeSteps << std::endl;
                }

                // dealloca gli array per evitare memory leaks
                delete[] boids.x;
                delete[] boids.y;
                delete[] boids.vx;
                delete[] boids.vy;
                delete[] new_boids.x;
                delete[] new_boids.y;
                delete[] new_boids.vx;
                delete[] new_boids.vy;
                #if visuals_on
                delete boidsQuads;
                #endif

                // stampa della misurazione ottenuta a schermo
                std::cout << "Simulazione terminata dopo " << elapsedTimeSteps << " time steps." << std::endl;
                std::cout << "Tempo di esecuzione per simulazione boids: " << totalSimulationTime / 1000000 << " secondi." << std::endl;
                simulationTimes[ti][ai][ri] = totalSimulationTime / 1000000;
            }
        }
    }
    #if visuals_on
    // chiudi la finestra alla fine di tutte le simulazioni
    window.close();
    #endif

    // calcolo tempo di esecuzione medio per ogni numero di threads e di agenti e stampa delle misurazioni ottenute a schermo e su file di log
    for (int ti = 0; ti < numberOfThreadsCases; ti++)
    {
        for (int ai = 0; ai < numberOfAgentsCases; ai++)
        {
            float meanTime = 0.0f;
            for (int ri = 0; ri < numberOfRuns; ri++)
                meanTime += simulationTimes[ti][ai][ri];
            meanTime /= numberOfRuns;
            std::cout << "Tempo di esecuzione medio per " << numberOfAgents[ai] << " boids con " << numberOfThreads[ti] << " threads: " << meanTime << " secondi." << std::endl;
            logFile << "Tempo di esecuzione medio per " << numberOfAgents[ai] << " boids con " << numberOfThreads[ti] << " threads: " << meanTime << " secondi." << std::endl;
        }
    }
    logFile.close();
}
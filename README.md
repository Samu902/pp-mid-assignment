# Parallelization of Boids Simulations

## Overview

This repository contains the implementation of a **Boids simulation** developed as a mid-term assignment for a **Parallel Programming** course at University of Florence.  
The main goal of the project is to analyze and compare different implementations of the same simulation:

- a **sequential version**
- **parallel OpenMP versions**
- different **data layouts**: *Array of Structures (AoS)* and *Structure of Arrays (SoA)*
- use of **spatial partitioning**

The project focuses on performance, scalability, and the impact of data organization on parallel execution.

Also, a visual representation was implementend using the **SFML** library.

---

## Boids Simulation

The Boids model simulates the collective behavior of autonomous agents (*boids*) following three simple rules:

1. **Separation** – avoid crowding neighbors
2. **Alignment** – steer towards the average heading of neighbors
3. **Cohesion** – move towards the average position of neighbors

Despite the simplicity of these rules, complex emergent behaviors arise, making the simulation computationally intensive and suitable for parallelization.

---

## Repository Structure

| Directory / File | Description |
|------------------|-------------|
| `seq/` | Sequential implementation of the Boids simulation. |
| `omp_aos/` | OpenMP parallel implementation using **Array of Structures (AoS)** data layout. |
| `omp_soa/` | OpenMP parallel implementation using **Structure of Arrays (SoA)** data layout. |
| `CMakeLists.txt` | CMake configuration file to build all versions of the project. |
| `.gitignore` | Git ignore rules. |

---

## Requirements

To build and run the project you need:

- **C++ 20** or newer
- **CMake 3.10+**
- A compiler with **OpenMP support** (e.g. `g++`, `clang++`)
- Build tools such as **Make** or **Ninja**

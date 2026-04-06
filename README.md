# Master's Degree - Simulations: RVDB & Robotics Dynamics

[![MATLAB](https://img.shields.io/badge/MATLAB-R2025a-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-Simulation-blue.svg)](https://www.mathworks.com/products/simulink.html)
[![LaTeX](https://img.shields.io/badge/LaTeX-Manuscript-008080.svg)](https://www.latex-project.org/)

This repository contains the source code, simulation models, and the complete LaTeX manuscript developed for my Master's Degree dissertation at the **Instituto Tecnológico de Aeronáutica (ITA)**. 

The research focuses on the integrated dynamic modeling of **Rendezvous and Docking/Berthing (RVDB)** missions, encompassing two-body orbital propagation and the complex dynamics of robotic manipulators.

## 📁 Repository Structure

The project is organized into two main branches: the academic manuscript and the simulation environment.

* **`/manuscript`**: Contains all LaTeX source files, figures, and bibliography used to compile the final dissertation document.
* **`/simulations`**: The core MATLAB scripts and Simulink models.
  * `/data`: Static variables, mass properties, robotic manipulator inertia matrices, and orbital constants.
  * `/src`: Isolated mathematical functions and dynamic scripts (e.g., 2-body propagation, kinematics).
  * `/models`: `.slx` Simulink files for the RVDB dynamics.
* **`/docs`**: Compiled PDF of the dissertation and additional theoretical documentation.

## 🚀 Getting Started

To run the simulations locally, ensure MATLAB and Simulink are installed. 

1. **Clone the repository:** run `git clone https://github.com/AlexandreAug/rvdb-robotics-simulink.git` in your terminal.
2. **Open MATLAB** and navigate to the `/simulations` folder.
3. **Run the initialization script:** Open and execute `main.m` (or `setup.m`). This script will automatically map all necessary subfolders to your MATLAB path (`addpath`) and load the required workspace variables (inertia matrices, orbital radius, masses) before opening the Simulink models.
4. **Run the simulation:** With the workspace populated, you can safely run the provided `.slx` models.

## ✉️ Contact

For questions or discussions regarding the dynamic models, feel free to reach out or open an issue in this repository.

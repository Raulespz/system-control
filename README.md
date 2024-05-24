# System and Control Project

## Overview
This project involves simulating a dynamic system described by a set of differential equations and implementing a Variable Structure/Sliding Mode (VS/SM) control system with PID error metrics. The project is developed using Python with the numpy and matplotlib libraries.

## Files
**main.py**: The primary script to execute the simulation and control system.
**controlsystem.py**: Contains functions for the control law and controlled simulation.
**plots.py**: Contains functions for plots for the control: Nominal and Realized Trajectorys, Tracking Errors, Phase Spaces, Control Signals.
**requirements.txt**: Lists the required Python packages.

## Usage
1. Ensure that Python is installed along with numpy and matplotlib.
2. Install the necessary packages using:
pip install -r requirements.txt
3. To execute Task 1: Simulate the system without control, run:
python main.py 
4. To execute Task 2: Simulate the system with control, run:
python controlsystem.py 
5. To execute the plots, that contain Nominal and Realized Trajectorys, Tracking Errors, Phase Spaces, Control Signals, run:
python plots.py 

## Description
- **Task 1**: Simulate the system without applying control, starting with the given initial conditions.
- **Task 2**: Simulate the system with the VS/SM control system using PID error metrics. The simulation generates various plots, including nominal and realized trajectories, tracking errors, phase spaces, and control signals.

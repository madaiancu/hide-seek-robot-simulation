# Hide and Seek Robot Simulation

This project simulates a **hide and seek scenario using TurtleBot3 in a Gazebo environment**.  
The robot navigates inside a simulated house and uses a **facial recognition module** to identify different people.

## Technologies
- ROS (Robot Operating System)
- Gazebo
- TurtleBot3
- Python

## Features
- Robot simulation in a virtual environment
- TurtleBot3 navigation inside a house map
- Facial recognition module for detecting people
- Integration between perception and robot behavior

## Project Structure

recunoastere_file/
├── recunoastere_faciala.py
├── pozaAbel.jpeg
├── pozaMadalina.jpeg
├── pozaRoberta.png
└── found_sound.mp3
turtlebot3_house.world


## How it Works

1. The TurtleBot3 robot is placed inside a **Gazebo simulation environment**.
2. The robot navigates through the house map.
3. A **facial recognition module** processes images to detect known individuals.
4. When a person is detected, the robot reacts according to the hide-and-seek logic.

## Learning Objectives

This project demonstrates:
- Robot simulation
- Integration of perception and robotics
- Basic AI concepts using facial recognition
- ROS-based development

## Author
Mădălina Iancu

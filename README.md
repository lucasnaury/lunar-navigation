# EURO2MOON - Space Robot Material Transfer

This repository contains all of the code for the Cranfield EURO2MOON Group Design project for the Robotics MSC.

Information about the project background and objectives can be found [here](./docs/project_background.md)

## Authors

- Aryan Dharne
- Nikhilanand Jha
- Pranjal Tikhe
- Lucas Naury

## How to run ?

This project is split into 2 tasks:
- Global path planning is performed only using Python, within the `scripts` folder
- Local planning is implemented using Nav2 on ROS2 Jazzy, with a simulation using Gazebo Harmonic. The ROS2 Package is `euro2moon`
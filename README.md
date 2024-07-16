# Conflict Simulation for Shared Autonomy in Autonomous Driving

This repository contains code for the report by Stefan Reitmann, Tsvetomila Mihaylova, Elin Anna Topp and Ville Kyrki "Conflict Simulation for Shared Autonomy in Autonomous Driving", LBR at HRI 2024.

## About

The project presents a Unity-based tool for conflict simulation in autonomous driving.

## Conflict Simulation Tool

The directory *conflict-simulation-tool* contains the code of the Unity-based tool. 

To run the simulation, load the project in Unity and open the scene *Assets/RCK2.3/Scenes/Simple*.


## Lane Detection

Simple control of the vehicle using lane detection has been implemented. 
It is based on [Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving).
The code for it can be accesssed in the folder *lane-detection*.

The file *lane-detection/README.md* contains instructions of how to connect the Unity simulation with the lane detection backend using ROS.

## Citation

```
@inproceedings{reitmann-2024-conflict-simulation,
author = {Reitmann, Stefan and Mihaylova, Tsvetomila and Topp, Elin Anna and Kyrki, Ville},
title = {Conflict Simulation for Shared Autonomy in Autonomous Driving},
year = {2024},
isbn = {9798400703232},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3610978.3640589},
doi = {10.1145/3610978.3640589},
booktitle = {Companion of the 2024 ACM/IEEE International Conference on Human-Robot Interaction},
pages = {882–887},
numpages = {6},
keywords = {autonomous driving, conflict detection, driving simulation, explainability, human-ai interaction, shared autonomy},
location = {Boulder, CO, USA},
series = {HRI '24}
}
```

Autonomous Infrastructure Inspection System
Overview

This repository presents an integrated autonomous mobile robotic system designed for structural health monitoring. Developed to replace hazardous and labor-intensive manual inspections, the system combines deep learning-based crack detection with memory-augmented coverage path planning to ensure real-time fault detection and comprehensive spatial coverage.
Key Features

    Vision-Based Defect Detection: Utilizes a lightweight YOLOv11n-seg model for real-time crack segmentation on embedded platforms.

    Autonomous Navigation: Employs grid-based boustrophedon coverage planning to ensure exhaustive spatial sweeps.

    Dynamic Replanning: Features memory-guided greedy replanning and infrared proximity sensors to efficiently navigate around unexpected obstructions.

    Sensor Fusion: Combines camera-based semantic detection with IR-based collision avoidance for anticipatory navigation.

Performance Metrics

Experimental validation in the Webots simulator, utilizing an E-puck differential-drive robot, yielded the following results:

    Vision Module: 88.8% Precision, 70.2% Recall, and 80.5% mAP@0.5 (trained on 4,029 images).

    Navigation Module: 94.17% Area Coverage and 92% Path Efficiency completed within 250 seconds in constrained, multi-obstacle environments.

System Architecture

The modular architecture tightly integrates the vision and navigation modules. Despite minor waypoint tracking instabilities in densely cluttered regions, the system successfully executes autonomous inspection missions with minimal human intervention, effectively maintaining the balance between coverage and collision avoidance.
Future Work

The simulation results validate the feasibility of this integrated approach for structural monitoring. This framework provides a robust foundation for future deployment on physical robotic platforms in real-world industrial environments, aiming to significantly reduce inspection costs and improve worker safety.

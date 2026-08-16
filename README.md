# ros2-ai-nltk-YOLOV8-based-pick-and-place-KUKA-iiwa-robotic-arm

# NLP & YOLOv8-Driven Autonomous Pick-and-Place Robotic Arm System

## Overview

This project integrates natural language processing (NLP), computer vision detection, and inverse kinematics (IK) mathematics to create an end-to-end intelligent pick-and-place system for the KUKA LBR iiwa 7-DOF robotic arm using ROS 2 Humble. Users can type natural language commands (e.g., "pick the bottle"), which are processed by a PyTorch-backed NLTK chatbot model, converted into ROS 2 topics, detected and localized in 3D space via YOLOv8, solved for joint radians using custom DH-parameter inverse kinematics, and finally executed on the physical or simulated robotic manipulator.

---

## System Architecture & Data Flow

The multi-node ROS 2 pipeline connects natural language intent parsing down to physical joint execution:

1. **Intent Processing & Command Node (`ChatbotAssistant` / PyTorch):**
* Ingests user input strings via terminal and parses patterns using NLTK tokenization and lemmatization against a reference `intents.json` dataset.


* Utilizes a trained PyTorch neural network model to classify the intent and extract the target object class.


* Publishes the recognized target object name across the `chatter` ROS 2 topic.




2. **Vision Localization Node (`obj_mesu`):**
* Subscribes to the `chatter` topic to identify which object to look for.


* Captures live webcam frames and runs YOLOv8 inference (`yolov8n.pt`) to locate the target object's 2D bounding box and pixel dimensions.


* Applies pinhole camera geometry parameters ($fx, fy, cx, cy$) and known real-world object dimensions to compute the 3D spatial coordinates ($X, Y, Z$) relative to the camera, publishing the vector over the `/mesure` topic.




3. **Inverse Kinematics & Transformation Node:**
* Subscribes to `/joint_states` and the `/mesure` 3D target topic.


* Computes Denavit-Hartenberg (DH) forward kinematics and Damped Least Squares (DLS) inverse kinematics to resolve target coordinates into precise joint radian angles ($\theta_1$ to $\theta_7$).


* Publishes the target joint displacements over the `/gesture` topic.




4. **Trajectory Execution Node (`WordListSubscriber`):**
* Subscribes to `/gesture` to receive the target joint radian vector.


* Constructs a `JointTrajectoryPoint` message and publishes it to `/iiwa_arm_controller/joint_trajectory` for smooth multi-joint path execution.





### Data Flow Pipeline

> Natural Language Input $\rightarrow$ PyTorch/NLTK Intent Classifier $\rightarrow$ `chatter` Topic $\rightarrow$ YOLOv8 3D Localization (`/mesure`) $\rightarrow$ IK & Transformation Solver (`/gesture`) $\rightarrow$ Trajectory Controller $\rightarrow$ `/iiwa_arm_controller/joint_trajectory` $\rightarrow$ KUKA LBR iiwa Arm
> 
> 

---

## Technologies Used

* **ROS 2 Humble:** Core robotic middleware facilitating distributed node communication.


* **PyTorch & NLTK:** Deep learning framework and natural language toolkit used for intent classification and pattern lemmatization.


* **Yolov8 (Ultralytics):** Real-time object detection model used for visual localization and 3D coordinate estimation.


* **KUKA LBR iiwa Stack:** Simulation and controller packages for the 7-DOF robotic manipulator (`iiwa_controllers`, `iiwa_bringup`).


* **OpenCV & NumPy:** Libraries handling image capture, spatial matrix transformations, and DH-parameter kinematic calculations.


* **Python:** Core programming language powering the autonomy and control pipeline.



---

## Setup & Execution Guide

### 1. Build and Source Workspace

Open a terminal in your ROS 2 workspace, build the packages, and source the environment:

```bash
cd ~/iiwa_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select iiwa_description iiwa_bringup iiwa_hardware iiwa_controllers
source install/setup.bash
```[cite: 42]

### 2. Launch the KUKA LBR iiwa Simulation
Start the Gazebo simulation environment and controller managers[cite: 39, 42]:
```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true
```[cite: 39, 42]

### 3. Run the NLP Chatbot Publisher Node
Execute the chatbot script to train or load the model, accept text inputs, and publish target object requests[cite: 46]:
```bash
python3 <your_chatbot_script_name>.py
```[cite: 46]

### 4. Run the YOLOv8 Vision Localization Node
Launch the vision node to begin processing webcam frames, detecting objects matching the intent, and publishing 3D coordinates[cite: 41]:
```bash
python3 <your_yolo_vision_script_name>.py
```[cite: 41]

### 5. Run the Inverse Kinematics and Arm Controller Nodes
Execute the IK solver script to calculate joint angles and the move execution script to drive the robotic arm to the target position[cite: 39, 40]:
```bash
python3 <your_ik_solver_script_name>.py
python3 <your_move_arm_script_name>.py
```[cite: 39, 40]

```

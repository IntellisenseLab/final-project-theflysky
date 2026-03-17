# 🤖 QBot Autonomous Vision-Based Control System

## 📌 Project Overview

This project implements an autonomous mobile robot system using **ROS 2**, where a QBot (Kobuki-based platform) interacts with humans using **computer vision**. The robot detects faces, recognizes hand gestures, and responds with appropriate movements such as approaching, stopping, spinning, or performing predefined actions.

The system is designed with a **modular ROS 2 architecture**, ensuring scalability, maintainability, and real-time performance.

---

## 🎯 Objectives

* Detect and track human faces using an RGB camera
* Recognize hand gestures for command input
* Control robot motion based on visual input
* Implement behavior-based decision making
* Integrate all components into a ROS 2 pipeline

---

## 🧠 System Architecture

The system consists of the following ROS 2 nodes:

### 1. Vision Node

* Captures image stream from camera
* Performs face detection using OpenCV
* Publishes processed image data

### 2. Gesture Node

* Uses MediaPipe/OpenCV for hand detection
* Classifies gestures (stop, move, etc.)
* Publishes gesture commands

### 3. Behavior Node

* Subscribes to face and gesture topics
* Decides robot action
* Publishes velocity commands (`/cmd_vel`)

### 4. QBot Controller

* Interfaces with Kobuki/QBot base
* Executes movement commands

---

## 🛠️ Technologies Used

* **ROS 2 (Humble / Jazzy)**
* **OpenCV**
* **MediaPipe**
* **Python 3**
* **Kobuki ROS Drivers**
* **VS Code (Development Environment)**

---

## 📂 Project Structure

```
qbot_ws/
├── src/
│   ├── vision_node/
│   ├── gesture_node/
│   ├── behavior_node/
│   ├── qbot_bringup/
├── build/
├── install/
├── log/
```

---

## ⚙️ Setup Instructions

### 1. Clone the Repository

```bash
git clone <your-repo-link>
cd qbot_ws
```

### 2. Build Workspace

```bash
colcon build
source install/setup.bash
```

### 3. Run the System

```bash
ros2 launch qbot_bringup system.launch.py
```

---

## 📷 Features

* Real-time face detection
* Hand gesture recognition
* Autonomous robot response
* Modular ROS 2 node architecture
* Easy integration and scalability

---

## 🧪 Testing Workflow

1. Verify camera feed
2. Test face detection node
3. Test gesture recognition
4. Validate robot movement using teleop
5. Run full integrated system

---

## ⚠️ Notes

* Ensure all dependencies are installed before running
* Avoid hardcoding paths or values
* System performance depends on lighting and camera quality

---

## 🚀 Future Improvements

* Add deep learning-based face recognition
* Improve gesture classification accuracy
* Integrate SLAM for navigation
* Add voice control

---

## 👥 Team

**Team FlySky**

---

## 📜 License

This project is for academic and research purposes.

---

## 🔗 Resources

ROS 2 Installation Guide:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

OpenCV Documentation:
https://docs.opencv.org/

MediaPipe Documentation:
https://ai.google.dev/edge/mediapipe

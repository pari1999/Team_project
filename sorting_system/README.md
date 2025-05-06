# Sorting System Team Project, for Master's Project under Technische Hochschule Ingolstadt

## Overview
This project is an autonomous sorting system that uses a Braccio robot and a DJI RoboMaster to detect and sort objects. The system is designed to run in parallel, with the Braccio handling object detection and sorting, and the DJI robot performing movement operations.

## Features
- **Object Detection**: The Braccio robot uses a YOLOv5 model to detect objects in its field of view.
- **Object Classification**: The Braccio robot classifies the detected objects (currently work in progress).
- **Parallel Processing**: The system runs the Braccio and DJI operations in separate threads.
- **Logging**: The system uses logging for traceability and debugging.

## Next Steps
1. **Object Detection for Braccio**:
   - Implement a robust object detection algorithm using the YOLOv5 model.
   - Train the model on a dataset of objects to be sorted.
   - Integrate the model with the Braccio camera to detect and classify objects in real-time.

2. **Safety Emergency Button**:
   - Implement a emergency stop button to halt all robot operations immediately.

3. **Handling Occlusions**:
   - Develop a system to detect when humans or other objects move between the robots.
   - Implement a pause mechanism when occlusions are detected to prevent accidents.
   - Use sensors or additional cameras to monitor the workspace for occlusions.

## Project Structure
- **`main.py`**: The entry point of the application, responsible for starting the Braccio and DJI threads.
- **`braccio/`**: Contains modules related to the Braccio robot.
  - **`braccio_main.py`**: Main module for the Braccio robot, coordinating object detection and sorting.
  - **`braccio_camera.py`**: Handles camera operations for the Braccio robot.
  - **`braccio_object.py`**: Contains logic for object detection.
  - **`braccio_serial.py`**: Manages serial communication with the Braccio robot.
- **`dji/`**: Contains modules related to the DJI RoboMaster.
  - **`dji_main.py`**: Main module for the DJI RoboMaster, coordinating movement operations.
  - **`dji_camera.py`**: Handles camera operations for the DJI RoboMaster.
  - **`dji_movement.py`**: Contains logic for controlling the movement of the DJI RoboMaster.
- **`models/`**: Directory for storing model file: `yolov5n.pt`.

## Getting Started
1. **Prerequisites**:
   - Python 3.x
   - Required Python packages (listed in `requirements.txt`)
   - Access to a Braccio robot and a DJI RoboMaster
   - RoboMaster-SDK: Ensure you have the RoboMaster-SDK installed and configured for your DJI RoboMaster.
   - YOLOv5n.pt: The YOLOv5 model file (`yolov5n.pt`) should be placed in the `models` directory of the project.

2. **Installation**:
   - Clone the repository.
   - Install the required packages using `pip install -r requirements.txt`.
   - Ensure the RoboMaster-SDK is properly set up and the YOLOv5 model file is in place.

3. **Running the System**:
   - Execute `main.py` to start the sorting system.
   - Monitor the logs for system status and any issues.

## License
TBA 
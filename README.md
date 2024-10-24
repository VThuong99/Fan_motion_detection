# Description
This is the first project of my [Introduction to Embedded Machine Learning course.](https://www.coursera.org/learn/introduction-to-embedded-machine-learning)
This project using accelerometer with microcontroller to detect the motion of a fan machine.
# Hardware
- Raspberry pi pico.
- ADXL345.
# Setup
This is my set up for this project.
![Screenshot 2024-06-06 161540](https://github.com/VThuong99/Fan_motion_detection/assets/146172312/2a850fc9-e80c-499c-a1ef-eda8c06cbf26)
# Data collection
I collect my own data using [Edge Impulse CLI.](https://docs.edgeimpulse.com/docs/tools/edge-impulse-cli) 
![image](https://github.com/VThuong99/Fan_motion_detection/assets/146172312/a4077d7d-5532-4695-90a6-841c229de802)
# Processing 
## Feature Extraction
Generate features from spectral analysis.
![image](https://github.com/VThuong99/Fan_motion_detection/assets/146172312/3c52901b-41b4-4020-83a8-c57587b10972)
## Model Training
Training with a simple NN model + K-means clustering algorithms to detect anomaly.
![image](https://github.com/VThuong99/Fan_motion_detection/assets/146172312/3a58a8fb-963b-4f5d-b5e4-4256c64d1442)

# Deployment
I deploy on the Raspberry pi pico with Arduino IDE (see tutorial on [Edge Impulse docs](https://docs.edgeimpulse.com/docs). 
Read data from accel sensor and then inference in real-time with RTOS supported in [Arm Mbed OS](https://os.mbed.com/mbed-os/).
# Demo


https://github.com/VThuong99/Fan_motion_detection/assets/146172312/15439cbc-dc8b-4fd4-8363-68f14cb7d886



# 

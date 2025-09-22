# Environmental Sensor Alarm System

[🎥 Video Demo](https://youtu.be/J7D62Jn6G84?si=YI9CXfdZGBBq4l6O)

The **Environmental Sensor Alarm System** is designed to trigger emergency alarms in homes and buildings when a **break-in, flooding, or fire** occurs. This ensures that facilities using the system are quickly notified of potential threats, allowing all occupants to evacuate safely.  

Motivated by the rising number of fires, break-ins, and floods affecting homes and workplaces, this project addresses the need for a **reliable, multi-sensor alarm system** that provides early warnings during emergencies.  

The system uses the **ESP-NOW peer-to-peer wireless communication protocol**, where one ESP32 handles **sensor monitoring** and another displays **alerts on an LCD screen**. With its ability to detect multiple types of hazards, this project offers a **comprehensive security solution** for homes, offices, and other buildings.  

---

## 🚨 Features
- **Break-in detection** using sound and motion sensors  
- **Flood detection** with a water sensor module  
- **Fire detection** using a temperature sensor  
- **ESP-NOW wireless communication** between ESP32 boards  
- **LCD display interface** for identifying the type of emergency  
- **Real-time, multi-sensor monitoring** with multitasking scheduling  

---

## 🎯 Learning Objectives
This project focuses on:  
- Applying **embedded system design principles**  
- Optimizing **system performance**  
- Developing **problem-solving skills**  
- Encouraging **innovation in IoT and safety systems**  

By integrating **four different sensors** with an ESP32 microcontroller, the system collects real-time data to determine whether an alarm should be triggered. Occupants can quickly identify the nature of the emergency through the LCD display interface.  

---

## 📂 Project Structure
- **`/receiver/`** – Contains the **ESP-NOW wireless communication** logic and the code that determines which warning message to display on the **LCD**  
- **`/sensorreading/`** – Contains the **multitasking scheduler** and the four dedicated task functions for each sensor (sound, motion, water, and temperature) 

---

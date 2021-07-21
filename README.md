# Radiation and CO2 mapping
![](https://i.imgur.com/4fIzN2B.jpg)

## Introduction
This package will record the co2 and radiation concentration.
The [following link](https://drive.google.com/drive/folders/1cHAI8AIlAs8HQwDsk-CN-zzBHxx-WQ2L?usp=sharing) is the data collected in experiment.

## Setup
1. Plug Arduino boards on husky 
(In this project, we plugged two sensors, so you should search for two USB ports)

3. Check the USB port 
```bash
$ cd /dev
$ ls | grep USB
```
3. Edit the run_arduino.launch file for the correct USB port
4. Launch ( set the correct USB port args)
```bash
$ roslaunch co2_rad_map run_arduino.launch radiation_port:=/dev/ttyUSB0 co2_port:=/dev/ttyUSB1
```
5. The launch file will enable you to collect the CO2 concentration message(ppm) and the radiation sensor message(uSv)


## Draw the marker on rviz map
1. Run the bag file you record (recommend) or run it realtime
2. Run the launch file, and set the topic you want to draw
```bash
$ roslaunch co2_rad_map draw.launch topic:=CO2_ppm
```
3. Open rviz, select the marker topic
4. The point will start to draw

![](https://i.imgur.com/tbWIchp.png)


## Raditaion with camera and draw the map
1. Run the bag file you record (recommend) or run it realtime
2. Run the launch file, and set the topic you want to draw
```bash
$ roslaunch co2_rad_map draw_radiation_meter.launch 
```
3. Open rviz, select the marker topic
4. The point will start to draw
![](https://i.imgur.com/DA2ZLDT.png)

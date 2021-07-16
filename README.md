# Radiation and CO2 mapping

## Setup
1. Plug Arduino Nano and Uno on husky 
2. Check the USB port 
```bash
cd /dev
ls | grep USB
```
3. Edit the run_arduino.launch file for the correct USB port
4. Launch
```bash
roslaunch co2_concentration run_arduino.launch
```


## Draw the marker on rviz map
1. Run the bag file or run it realtime
2. roslaunch co2_concentraion draw_co2
3. open rviz, select the marker

![](https://i.imgur.com/tbWIchp.png)

# Pre-mission check

## Power-on the vehicle

```
NEVER DIRECTLY CONNECT THE BATTERY HOUSING TO THE ELECTRONICS HOUSING.
```
- VIN port (a 4pin Blue trail Cobalt bulkhead connect) accept power voltage upto 21 volts.
    - Pin 1 & 3 are V-
    - Pin 2 & 4 are V+
- You can find the location of the VIN port on the [Endcap diagram](https://github.com/GSO-soslab/alpha_hardware_release/blob/main/Electronics/doc/v1_Endcap.pdf).
- Battery connection. All the batterys are parallelized using diodes. You can see there are cables running from the battery pressure housing to a small 2 inch Blue Robotics Pressure housing. 

## Vehicle connection
```
 All the ip address setup or change should be recorded in the log book.
 ```
- **WIFI connection**
    - Make sure the Communication mast is connected. Endcap diagram can be found [here](https://github.com/GSO-soslab/alpha_hardware_release/blob/main/Electronics/doc/v1_Endcap.pdf).
    -  Open a terminal and type `ssh [vehicle_name]@[ip_address]`. The vehicle name and the ip_address should be recorded on the log_book. Note: The ip address should start with 192.168.1.xx
- **Tether connection**
    - Make sure the tether is connected. Endcap diagram can be found [here](https://github.com/GSO-soslab/alpha_hardware_release/blob/main/Electronics/doc/v1_Endcap.pdf).
    -  Open a terminal and type `ssh [vehicle_name]@[ip_address]`. The vehicle name and the ip_address should be recorded on the log_book. Note: The ip address should start with 192.168.2.xx
- **Ping ethernet sensors**
    - The DVL, Ping360 and other deveices are connected using a ethernet swith inside the vehicle. 
    - Once you have connected to the vehicle, through either above way, you can ping these devices.
    

## Dry test
- **Sensor test:** `[vehicle_name]_bringup/launch/bringup_test.launch` is an empty launch file for user to test individual sensors drivers. All the launch commands for sensors are located `bringup_vehicle.launch`. Follow the instruction below to test a sensor.
    - copy the sensor launch commands they want from `bringup_vehicle.launch` to `bringup_test.launch`.
    - inside the terminal (vehicle system) type `roslaunch [vehicle_name]_bringup bringup_test.launch`
    - use `rostopic echo [topic_name]` to check if the driver is publishing data to the topics related to a sensor.
    - use `rostopic hz [topic_name]`, you can check how fast the data is publishing to a topic.
    
- **Thruster test:** 
    - Prepare launch file: copy the following lines into `bringup_test.launch` file.
    ```
    <include file="$(find alpha_rise_bringup)/launch/include/pi_pico.launch.xml">
        <arg name="robot_name" value="$(arg robot_name)"/>
    </include>
    ```
    - Launch the rosnode: run `roslaunch [vehicle_name]_bringup bringup_test.launch` to bringup vehicle ros driver that interfaces with the MCU which sends PWM signals to the motor ESCs.
    - Check thruster topics: run `rostopic list`, all the configured thruster are listed under topics `/[vehicle_name]/control/thruster/[thruster_name]`. 
    - Move the thruster: run `rostopic pub /[vehicle_name]/control/thruster/[thruster_name] ` then press the "tab" key to auto-fill the command. You can change the value between -1.0 to 1.0. 
        ```
        DO NOT SEND A COMMAND OVER 0.2 IN AIR, IT WILL CAUSE DAMAGE TO THE THRUSTER
        ```
        - *Corresponding thruster is spinning?* you can hit control+c in the `rostopic pub` terminal.
        - *Wrong thruster is spinning?* you can adjust the mapping between PWM channels and thruster topics in the `[vehicle_name]_config/config/pi_pico.yaml`, under `PWM_Control` section.    **Please only change the channel number**
        - *Which direction the thruster should spin?* All the thrusters on the vehicle have **Clockwise** propeller (see the figure below). When a positve command is sent, the propeller should spin **clockwise** when you looking from the **cable side**, and it should generate a thrust towards the **cable side**. If this is not right you have two options.
            - Switch two motor cables connected to the ESC
            - Update the `[vehicle_name]_description/urdf/base.urdf.xacro` file. You can invert the x-axis of the thruster by rotating the thruster link around z axis by 180 deg(pi). 

![Thruster directions](https://github.com/GSO-soslab/sosl_auv_manual/blob/main/pictures/thruster_direction.png)

## Power down the vehicle
- You can type `sudo poweroff` in the terminal [vehicle side] to turnoff the Pi inside the vehicle.
- Wait for 1-2 minutes before unplug the power cable on the VIN port.
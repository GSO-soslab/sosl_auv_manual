# Vehicle operation instruction
## Bringup the vehicle
- Power on the vehicle by plug-in the power cable from the battery management housing (2in housing) to the **VIN** port on the electronics housing.
- Connect your laptop to the same WIFI network (name found on the station box).
- `Ping [vehicle_ip_address]`. The IP address can be found on the vehicle log book. Wireless ip address should start with 192.168.1.xxx. Tether connection is under 192.168.2.xxx
- `ssh [vehicle_name]@[ip_address]`
- In the terminal, run `roslaunch [vehicle_name]_bringup bringup_vehicle.launch`. All sensors should be up running and the MVP helm should be active.
- Enable/disable controller: Open another terminal and type, `rosservice call /[vehicle_name]/controller/enable` or `rosservice call /[vehicle_name]/controller/disable`. You should see "controller enabled" or "controller disabled" in the previous terminal. By default, the controller is disabled.

## Get information about the FSM in helm
- Check your current state: type `rosservice call /[vehicle_name]/helm/get_state`
- Check available states: type  `rosservice call /[vehicle_name]/helm/get_states`
- Go to a different state: type  `rosservice call /[vehicle_name]/helm/change_state` then use tab to auto-fill and entry the state name inside quotation marks.
- The states are defined inside the file `[vehicle_name_config]/mission/config/helm.yaml`. 
    - `finite_state_machine` section shows all the states, state connections, and the control modes used in the state.
    - `behaviors` section shows the helm bhv_plugins used, and their priorities in each state. Priority is used for the case that multiple behaviors may try to command different desired setpoints for a controlled state. Then, the mvp_control will only take the one from the behavior with the highest priority. 
    - global_link should be kept at `world_ned` 
    - local_link should be kept with `cg_link`
    - These two above links indicate which frames the desired values from the behaviors will be in and will be used by the mvp_control.
- The configuration for each behavior is loaded inside `[vehicle_name]_bringup/launch/bringup_helm.launch` with additional param from yaml files located in `[vehicle_name]_config/mission/param`.
- Normally, we will have `start, survey_3d, kill, and direct_control` states.
    - `start` is the initial state when the MVP-helm is up running. Its control mode is normally idle, meaning no states are controlled. THe vehicle will return to this state if it has finished all the waypoints.
    - `survey_3d` the MVP_helm will guide the vehicle to go through a list of waypoints. Initial waypoint list can be found in `[vehicle_name]_config/mission/param/path_local.yaml`. BUt can be updated via topics and services. Check **waypoint programming** section for information.
    - `direct_control` state will allow user to direct command the desired setpoints in different DOFs based on the control modes. Typically, z, yaw, pitch, surge are enabled. see more information in **Desired pose programming** section.
    - `kill` state is similar to `start` state where no thruster will be commanded. and the vehicle will stay idle.

## Waypoint programming
`survey_3d` uses path_following_3d bevahior plugin from helm. This plug-in will guide the AUV to track line segments between the waypoints definde by the user. At the same time, it will guide the vehicle to keep its depth around the z-value of the next waypoint. 
We have two options to allow user to update waypoints on-the-fly.

- Option-1: Good for adaptive sampling.
    - User could direct publish waypoints in `geometry_msgs::PolygonStamped` format to the topic name `/[vehicle_name]/helm/path_3d/update_waypoints`. The behavior will convert the waypoints into the helm's target id then overwritten its current waypoints and start to track them. **Note** If the vehicle is in the `start` state, user has to switch the vehicle state to `survey_3d` to active waypoint tracking.
- Option-2: Good for regular AUV survyes.
    - User could pre-load or upload waypoint files to the folder `[vehicle_name]_config/mission/param/goto_list/`. 
    - User load new waypoints by running `rosservice call /[vehicle_name]/helm/path_3d/load_waypoints "file: '[filename]'"` to load the waypoint files. If successful, the vehicle's waypoints will be updated immediately. 
        - If the vehicle is in `survey_3d` state, it will start to track the new waypoints
        - If the vehicle is in `start` state, you need to switch the state back to `survey_3d`.
        - User can define different frame_id in the waypoint yaml file. 
        - User can define waypoint in latitude(lat), longitude(lon) and altitude(alt) format inside `ll_waypoint` section with an example shown below. **Note**: altitude is positive upward as it uses ENU convention.
        ```
        #frame id has to be keep in world as from LL will convert waypoint into lat lon
        frame_id: "world" 

        ll_waypoints:
        - {lat: 41.0000,  lon: -71.001,  alt: -10}
        - {lat: 41.0001,  lon: -71.002,  alt: -5}
        - {lat: 41.0002,  lon: -71.001,  alt: -20}
        ```
- Check next waypoint.
    - User can check which waypoint the vehicle is moving towards by calling the a service. `rosservice call /[vehicle_name]/helm/path_3d/get_next_waypoint "{}" `
    - It will retunr the waypoint's x, y, z with a frame_id, and the latitude, longitude and altitude of the waypoint calculated using `ToLL` service from the robot localization package.
- When the vehicle finish all the waypoints, it will return to `start` state.

- ***NavSat node conifugration***  
    - In our default setting, the `world` frame and `odom` frame are placed at the same location. 
    - User can define where is the Latitude and Longitude of their `world` frame in the yaml file located under `[vehicle_name]/[vehicle_name]_config/config/navsat.yaml`.
    - Change the values for the `datum` paramter.
    - ToLL and FromLL services will response x, y, z position when sending the request with latitude, longitude and altitude. These services are used extensively in our MVP_mission.

## Desired pose setpoints programming
`direct_control` state is designed for the users who don't want to use built-in waypoint following behavior. They can directly command desired setpoints to a vehicle state and use MVP_control to control the vehicle pose. 

- `direct_control` control mode can be found in `[vehicle_name_config]/mission/config/helm.yaml`, e.g., you will see `mode: hold_dof` in `direct_control` section.
- You can find which DOFs are controlled and PID gain in `[vehicle_name]_config/config/control.yaml` or `[vehicle_name]_config/config/control_sim.yaml`. The later one is just for simulation environment.
- The initial desired setpoints are defined in file `[vehicle_name]_config/config/mission/param/direct_control.yaml`.
- To change the desired setpoints on-the-fly, you can publish a customized msgs to the topic `/[vehicle_name]/helm/direct_control/desired_setpoints`. 
- We suggest user to use `rqt_ez_publisher` to adjust the desired setpoints when testing the basic behavior.
- Alternatively, user can use **Logitec F170** and `joy` package to control vehicle's surge, sway, yaw, pitch, and depth. The surge and sway are controlled by the left joystick, and the yaw, pitch and depth are controlled by the buttons on the right. (see the [figure](https://github.com/GSO-soslab/sosl_auv_manual/blob/main/pictures/tele_op2.png) below) 
- User could program the increments (deg or meters) for the yaw, pitch and depth when the button is pressed once, and could up or down scale the joystick commands for surge and sway velocity mapping. Check the `[vehicle_name]_config/mission/param/direct_control.yaml` file for the values.
<img src="https://github.com/GSO-soslab/sosl_auv_manual/blob/main/pictures/tele_op2.png" width="700">
- When launching the joy node, we also need to remap the topics. An example of the commands in the launch file is shown below.
```
 <node ns="$(arg robot_name)" name="joy_node" pkg="joy" 
        type="joy_node" output="screen">
        <remap from="joy" to="helm/direct_control/joy" />
  </node>
```
- ***Note: Logitech has to be in Direction mode***

## Localization
- Local odometry is available in `ENU` format under `[vehicle_name]odometry/filtered/local` topic
- A separate node should be created in alpha_core localization package to convert local to lat lon.

## Acoustic modem
under development

# Vehicle operation instruction
## Bringup the vehicle
- Power on the vehicle by plug-in the power cable from the battery management housing (2in housing) to the VIN port in the electronics housing.
- Connect your laptop to the same WIFI network (name found on the station box).
- `Ping [vehicle_ip_address]`. The IP address can be found on the vehicle log book. Wireless ip address should start with 192.168.1.xxx. Tether connection is under 192.168.2.xxx
- `ssh [vehicle_name]@[ip_address]`
- In the terminal, run `roslaunch [vehicle_name]_bringup bringup_vehicle.launch`. All sensors should be up running and the MVP helm should be active.
- Enable/disable controller: Open another terminal and type, `rosservice call /[vehicle_name]/controller/enable` or `rosservice call /[vehicle_name]/controller/disable`. You should see "controller enabled" or "controller disabled" in the previous terminal.

## Change states
- Check your current state: type `rosservice call /[vehicle_name]/helm/get_state`
- Check available states: type  `rosservice call /[vehicle_name]/helm/get_states`
- Go to a different state: tyep  `rosservice call /[vehicle_name]/helm/change_state` then use tab to auto-fill. and inside quote symbol you put the actual name of the state, e.g., survey_global.
- The states are defined inside the file `[vehicle_name_config]/mission/config/helm.yaml`. 
    - The finite state machine section shows all the states, state connections, and the control modes used.
    - Behaviors section shows the helm plugins used, and their priorities in each state. Priority is used for the case that two behaviors may try to command different desired value for a controlled state, then the mvp_control will only take the one from the behavior with the highest priority. 
    - global_link should be kept at `world_ned` 
    - local_link should be kept with `cg_link`
    - These two links indicate which frame the desired values from the behaviors will be in and will be used by the mvp_control.
- The configuration for each behavior is loaded by `[vehicle_name]_bringup/launch/bringup_helm.launch`. The actual yaml files are located in `[vehicle_name]_config/mission/param`.
- Normally, we will have `start, survey_local, survey_global, kill, and direct_control` states.
    - `start` is the initial state when the MVP-helm is up running.
    - `survey_local` the MVP_helm will guide the vehicle to go through a list of waypoints defined in `[vehicle_name]_config/mission/param/path_local.yaml` file where x and y are in the world_ned frame or the global frame defined in the `[vehicle_name]_config/mission/config/helm.yaml`
    - `survey_global` the MVP_helm will guide the vehicle to go through a list of waypoints defined in `[vehicle_name]_config/mission/param/gps_wpt.yaml` file. Other path following parameters are located in `path_global.yaml` in the same folder.
    - In both survey mode, the depth is defined in `depth_tracking.yaml`. Currently, it is a constant depth. We are currently updating the depth tracking behavior such that different depth can be defined along with the waypoints.
    - `direct_control` state will allow user to direct command the desired pose in different DOFs based on the control modes.

## Waypoint programming
- Option-1: 
    - stop the helm
    - update the waypoints in `gps_wpt.yaml` file   
    - restart the helm by `roslaunch [vehicle_name]_bringup bringup_vehicle.launch`
- Option-2 (to_do)
    - `update_waypoint` topic is already exist but there is no ros node or service avaiable to update the values based on a file.
    - Create a new rosnode called mvp_utilities to have services that user can call to update the waypoints based on yaml file (with latlon or local defined as a param).


## Desired pose programming
- We recommend to use `rqt_ez_publisher` to adjust the desired pose under the topic `continuous_command_topic`

## Localization
- Local odometry is available in `ENU` format under `[vehicle_name]odometry/filtered/local` topic
- A separate node should be created in alpha_core localization package to convert local to lat lon.

## Acoustic modem
under development

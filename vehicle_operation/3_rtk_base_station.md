# RTK base station
The RTK base station is a SparkFun RTK Surveyor in base mode connected to the base station computer in the MVP station. The computer runs Ubuntu Server 24.04 and the [rtk_base](https://github.com/GSO-soslab/rtk_base/tree/jazzy-devel) package, which forwards the receiver's RTCM corrections to the vehicles over TCP and UDP. The RTCM path does not go through ROS. The node also publishes the base position and status as ROS topics for monitoring, and the computer serves GNSS time to the shore network over NTP.

```
antenna (outside) --coax--> Surveyor --USB--> base station computer: rtk_base_node
                                                |-> RTCM over TCP / UDP --> vehicles (unicore_rtk_driver rtcm.inputs)
                                                |-> /rtk_base/rtk_base/{fix, nmea}, /diagnostics
                                                '-> NMEA over UDP --> gpsd --> chrony --> NTP for the shore network
```

This page lists the steps for our station. Parameters, file formats and options are described in the rtk_base README.

The IP address and user of the base station computer can be found in the station log book. Below they are written as `<base_station_ip>` and `<user>`; the shore network is `<shore_subnet>` (for example `192.168.x.0/24`) with broadcast address `<shore_broadcast>`.

| Item | Value |
|---|---|
| ROS namespace / node | `rtk_base` / `rtk_base` |
| ROS_DOMAIN_ID | same as the shore network |
| RTCM outputs | TCP server on port 2101, UDP broadcast on port 2101 |
| Station files | `~/ros2_ws/src/rtk_base/site/` (not in git) |

## Antenna
- The Surveyor is inside the station; the GNSS antenna (L1/L2) is mounted outside with a clear view of the sky in all directions, and connected to the Surveyor with a coax cable. The Surveyor has an SMA antenna connector; the SparkFun L1/L2 antennas have a TNC connector.
- Use a GNSS rated low loss cable, as short as possible. The cable also carries the antenna supply from the receiver, so any adapter or splitter in the line must pass DC.
- The base position is the position of the antenna. Do not move the antenna after the survey-in; survey again after it has been moved.
- Check the signal with the antenna in place: `satellites used` in `/diagnostics` (see Base configuration) should be above 10.

## Operating system
- Install **Ubuntu Server 24.04 LTS (64-bit)**. On a Raspberry Pi use Raspberry Pi Imager; in the customisation settings set the hostname, the user, the network and the Wi-Fi country, and enable SSH with public-key authentication only.
- After a reinstall, remove the old host key on your laptop: `ssh-keygen -R <base_station_ip>`

### Automatic upgrades
Turn off automatic upgrades. An upgrade of a single ROS package (for example `ros-jazzy-nmea-msgs`) without the rest of ROS stops the node with an `undefined symbol` error, and an upgrade running at boot holds the apt lock.
```sh
sudo systemctl disable --now unattended-upgrades apt-daily.timer apt-daily-upgrade.timer
```
Upgrade by hand, all ROS packages together, then rebuild the workspace:
```sh
sudo apt update && sudo apt full-upgrade
cd ~/ros2_ws && colcon build --symlink-install
```

### Wi-Fi country
Only for a computer on Wi-Fi. Without a regulatory domain the Wi-Fi cannot use the upper 5 GHz channels and falls back to 2.4 GHz, where it may not be reachable from the 5 GHz clients.
- stop cloud-init from rewriting the network configuration at boot:
```sh
echo 'network: {config: disabled}' | sudo tee /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
```
- add `regulatory-domain` to `/etc/netplan/50-cloud-init.yaml`:
```yaml
network:
  version: 2
  wifis:
    wlan0:
      regulatory-domain: "US"
      dhcp4: true
      access-points:
        "<ssid>":
          password: "..."
```
- on a Raspberry Pi, set the same country on the kernel command line: append ` cfg80211.ieee80211_regdom=US` to the single line in `/boot/firmware/cmdline.txt`
- apply and reboot: `sudo netplan generate && sudo netplan apply && sudo reboot`
- check: `iw dev wlan0 link` shows the expected frequency

## ROS 2 Jazzy
- follow the official guide: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html and install `ros-jazzy-ros-base` and `ros-dev-tools`
- some Ubuntu images (for example the Raspberry Pi image) only enable the `noble` apt suite. Before installing, check `grep Suites /etc/apt/sources.list.d/ubuntu.sources`; if needed change the line to `Suites: noble noble-updates noble-backports` and run `sudo apt update && sudo apt full-upgrade -y` (see the warning in the guide)
- initialise rosdep:
```sh
sudo rosdep init
rosdep update
```
- serial port access: `sudo usermod -aG dialout $USER`, then log out and back in

## rtk_base
```sh
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b jazzy-devel https://github.com/GSO-soslab/rtk_base.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
```

### Station files
The station's own files are kept in `site/` in the repository folder. Git ignores `site/`, so `git pull` does not change them and they are never committed. Create them from the examples once:
```sh
cd ~/ros2_ws/src/rtk_base
mkdir -p site
cp config/base_generic.yaml site/base.yaml
cp systemd/rtk_base.service site/
cp config/gpsd.example site/gpsd
cp config/chrony-rtk_base.conf.example site/chrony-rtk_base.conf
```
They are edited in the sections below. `git clean -x` deletes `site/`.

## Surveyor connection
Connect the **Configure u-blox** USB-C port of the Surveyor to the base station computer. This port is the GNSS receiver's own USB port and carries NMEA and RTCM. It has a stable device name on any computer:
```
/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00
```
Use this name, not `/dev/ttyACM0`, which can be another device after a reboot. The Surveyor also charges over this cable. If the computer cannot supply the charging current, use a powered USB hub.

The Surveyor **Data** port (3.3 V serial, 115200 baud) can be used instead, for example on a Raspberry Pi GPIO UART. See "Receiver on a Raspberry Pi UART" in the `unicore_rtk_driver` README, and enable RTCM on the Data port (Surveyor configuration below). Do not connect the **Radio** port: its pin 1 supplies 3.5 V to 5.5 V.

## Surveyor configuration
The Surveyor has two USB-C ports: **Configure ESP32** (Surveyor menu) and **Configure u-blox** (the GNSS receiver itself). The settings below are made once per unit, from a laptop.
- base type: connect the laptop to the **Configure ESP32** port, open a terminal at 115200 (`python3 -m serial.tools.miniterm --dtr 0 --rts 0 /dev/ttyUSB0 115200`, exit with `Ctrl+]`), press any key, open **Configure Base** and select **Survey-In**. Use a long minimum time and a tight mean accuracy (see Base position below). Our unit was set to a fixed base at 0, 0, 0, which gives no fix and no RTCM in base mode.
- NMEA messages (firmware v4.4 does not output GST by default; the node publishes `fix` on GST): in the menu select **Configure GNSS Messages**, **Reset to Surveying Defaults** (GGA, GSA, GST, GSV, RMC). This applies to the Data port. For the Configure u-blox USB port also select **Configure System**, **Configure RTK operation**, **Mirror ZED-F9x's UART1 settings to USB**.
- RTCM on the Data port (only when the Data port is used): connect the laptop to the **Configure u-blox** port and enable RTCM output on the receiver UART1. The setting is stored in the receiver flash and is kept after a power cycle.
```sh
sudo apt install gpsd-clients
export UBXOPTS="-f /dev/ttyACM0 -s 115200 -P 27"
ubxtool -g CFG-UART1OUTPROT-RTCM3X     # 0 = disabled
ubxtool -z CFG-UART1OUTPROT-RTCM3X,1
```
- set the SETUP switch to **Base**. The yellow PPS LED blinks with a fix, the red BASE LED blinks during the survey-in and is solid when RTCM is sent (1005, 1074, 1084, 1124 and 1230).

### Surveyor firmware
Our unit came with firmware v1.1 (no WiFi configuration, no Bluetooth menu) and has 4 MB flash (sold before September 2021), so it can only be updated over USB. Updated to v4.4 on 2026-11-18:
- connect the laptop to the **Configure ESP32** port and check the flash size: `esptool --no-stub --port /dev/ttyUSB0 flash_id` (the Ubuntu `esptool` package needs `--no-stub`); power-cycle the Surveyor afterwards
- download the uploader and the firmware:
```sh
mkdir -p ~/rtk_fw && cd ~/rtk_fw
wget https://github.com/sparkfun/SparkFun_RTK_Firmware_Uploader/releases/download/v1.7.0/RTKUploader.linux.gz
wget https://github.com/sparkfun/SparkFun_RTK_Firmware_Binaries/raw/main/RTK_Surveyor_Firmware_v4_4.bin
gunzip RTKUploader.linux.gz && chmod +x RTKUploader && ./RTKUploader
```
- in the uploader select the `.bin`, port `/dev/ttyUSB0`, **Upload Firmware** (it detects the 4 MB flash). Then set Survey-In and the NMEA messages again (above).

## Base configuration
- edit `~/ros2_ws/src/rtk_base/site/base.yaml`:
```yaml
/**:
  ros__parameters:
    device: /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00
    baudrate: 115200
    gps_frame_id: rtk_base/gps

    # stamp with GNSS time, as on the vehicles
    use_gnss_time: true

    # NMEA to gpsd (DEVICES="udp://127.0.0.1:5006" in site/gpsd)
    use_gpsd_udp: true
    virtual_port: 5006

    receiver: generic

    rtcm:
      outputs: ["tcpserver://0.0.0.0:2101", "udp://<shore_broadcast>:2101"]
```
- test it in a terminal before installing the service (stop it with `Ctrl+C`):
```sh
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=<shore network domain id>
ros2 launch rtk_base start.launch.py namespace:=rtk_base node_name:=rtk_base \
    params_file:=$HOME/ros2_ws/src/rtk_base/site/base.yaml
```
- check: `ros2 topic echo /diagnostics --once` shows `solution: single`, `rtcm out bytes/s` above 0 and `base station: id 0: <lat>, <lon>, <height>`
- check from a laptop with `unicore_rtk_driver`: `ros2 run unicore_rtk_driver unicore_rtk_probe tcp://<base_station_ip>:2101` lists the RTCM messages (about 400 bytes per second) and the base position

### Base position
Rovers are accurate to centimeters relative to the base; the absolute position has the same offset as the base. A survey-in position is only accurate to meters (2.1 m mean accuracy with 60 s and 5 m on 2026-11-18) and changes with every survey. When a rover loses the corrections, its position can jump by up to that offset. For positions that agree between days, survey once with a long time and a tight accuracy, then enter that position as a fixed base in the Surveyor menu and record it in the station log book. See "Base position accuracy" in the rtk_base README.

## Start at boot
- edit `~/ros2_ws/src/rtk_base/site/rtk_base.service`: the user, `ROS_DOMAIN_ID`, `namespace:=rtk_base` and the paths, for example:
```sh
cd ~/ros2_ws/src/rtk_base
sed -i -e 's/CHANGE_ME_USER/<user>/g' -e 's/ROS_DOMAIN_ID=0/ROS_DOMAIN_ID=<shore network domain id>/' \
       -e 's/namespace:=base /namespace:=rtk_base /' site/rtk_base.service
grep -E "User=|DOMAIN|namespace|params_file" site/rtk_base.service
```
- link it into systemd and start it (stop any base started in a terminal first, the port can only have one reader):
```sh
sudo systemctl enable --now ~/ros2_ws/src/rtk_base/site/rtk_base.service
journalctl -u rtk_base -f
```
- after editing `site/rtk_base.service`: `sudo systemctl daemon-reload && sudo systemctl restart rtk_base`
- check: reboot, wait 2 minutes without logging in, then check `/diagnostics` from the laptop

## Time synchronization
Same setup as the vehicle Pis (see [Time Synchronization](2_operation.md#time-synchronization)), with gpsd reading the NMEA that the node forwards over UDP. chrony uses Ubuntu's own `/etc/chrony/chrony.conf` (internet servers, drift file) and a drop-in file for the GNSS time and the shore network.
- install dependency: `sudo apt install --no-install-recommends gpsd gpsd-tools chrony`
- `site/gpsd`, copied to `/etc/default/gpsd`:
```sh
START_DAEMON="true"

# The node owns the receiver serial port; gpsd must not open it.
USBAUTO="false"

DEVICES="udp://127.0.0.1:5006"

GPSD_OPTIONS="-n"
```
- `site/chrony-rtk_base.conf`, copied to `/etc/chrony/conf.d/rtk_base.conf`:
```sh
# GNSS time from NMEA through gpsd (no PPS)
refclock SHM 0 refid GPS offset 0.0

# Serve time to the shore network, also when no source is reachable
allow <shore_subnet>
local stratum 10

# Step the clock on the first updates (a computer without a battery-backed clock starts with a wrong time)
makestep 1.0 3
```
- copy, enable gpsd at boot (Ubuntu only starts it when a client connects) and restart:
```sh
cd ~/ros2_ws/src/rtk_base
sudo cp site/gpsd /etc/default/gpsd
sudo cp site/chrony-rtk_base.conf /etc/chrony/conf.d/rtk_base.conf
sudo systemctl enable --now gpsd
sudo systemctl restart gpsd chrony
```
- check: `chronyc sources -v` shows `*` on the GPS line (reach 377) once the node is running. Without internet the internet servers show `?`.

<!-- PENDING: measured GPS offset for the Surveyor -->

## Vehicles
- rover corrections, in the vehicle's `unicore_rtk_driver` parameters: `rtcm.inputs: ["udp://:2101"]` for the UDP broadcast, or `["tcp://<base_station_ip>:2101"]`; both can be listed, the first one that delivers is used
- check on the vehicle: `/diagnostics` of the driver shows `base station: id 0: ...` and `rtcm in bytes/s` around 400, and `solution: RTK fixed` with a clear sky
- correction timeouts and other receiver settings go in `receiver_commands`, see "Receiver configuration" in the `unicore_rtk_driver` README
- vehicle and shore laptop time: chrony client with the base station as server, see [RTK base station and shore laptop](2_operation.md#rtk-base-station-and-shore-laptop)

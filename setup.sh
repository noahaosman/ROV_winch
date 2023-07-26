#!/bin/bash

# Configure wifi network settings
WPA_FILE='/etc/wpa_supplicant/wpa_supplicant.conf'
read -r -d '' WPA_LINE << EOM
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
  ssid="mixz"
  psk="chi10eps9"
  priority=3
}
network={
  ssid="ScienceShare"
  psk="SwimRobotSwim"
  priority=2
}
network={
  ssid="ROV_winch"
  mode=2
  key_mgmt=WPA-PSK
  psk="raspberry"
  frequency=2437
  priority=1
}
EOM
echo "$WPA_LINE" > "$WPA_FILE"

# Configure static IP
ip r | grep default
cat /etc/resolv.conf
IP_FILE='/etc/dhcpcd.conf'
read -r -d '' IP_LINE << EOM
interface wlan0
static ip_address=10.0.1.150/24
static routers=10.0.1.1
static domain_name_servers=10.0.1.1
EOM
echo "$IP_LINE" >> "$IP_FILE"

# enable hardware serial port
raspi-config nonint do_serial 2
# enable i2c
raspi-config nonint do_i2c 0

# Install required Python packages
apt-get update
apt-get install -y python3-pip
apt-get install -y i2c-tools
apt-get install -y python3-dev
apt-get install -y python3-rpi.gpio
pip3 install gpiozero
apt-get install -y build-essential python3-smbus
pip3 install adafruit-ads1x15
pip3 install pyserial
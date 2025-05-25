# Beaglebone blue image
## Download
``` bash
mkdir -p downloads
cd downloads
wget https://files.beagle.cc/file/beagleboard-public-2021/images/bone-debian-10.3-console-armhf-2020-04-06-1gb.img.xz
```

https://www.beagleboard.org/distros
https://forum.beagleboard.org/t/debian-10-x-buster-monthly-snapshot-2023-07-01-final/31203

https://rcn-ee.com/rootfs/release/2023-07-01/buster-console-armhf/bone-debian-10.13-console-armhf-2023-07-01-1gb.img.xz
https://rcn-ee.com/rootfs/release/2023-07-01/buster-console-armhf/bone-eMMC-flasher-debian-10.13-console-armhf-2023-07-01-1gb.img.xz

## Flash image
press "SD" button while power attach

**login:** debian<br>
**pwd:** temppwd
``` bash
vi /boot/uEnv.txt
sync
```
uncomment last line to flash emmc. power cycle.

# Download toolchain
``` bash
./build.sh --install_toolchain
```
or

``` bash
mkdir -p toolchain
cd toolchain/
wget https://developer.arm.com/-/media/Files/downloads/gnu-a/8.3-2019.02/gcc-arm-8.3-2019.02-x86_64-arm-linux-gnueabihf.tar.xz
tar -xf gcc-arm-8.3-2019.02-x86_64-arm-linux-gnueabihf.tar.xz
```


https://developer.arm.com/-/media/Files/downloads/gnu-a/8.3-2019.02/gcc-arm-8.3-2019.02-x86_64-arm-linux-gnueabihf.tar.xz

# Configure bbb

## WLAN
Connect via serial terminal:
``` bash
sudo -s
connmanctl services
connmanctl
tether wifi off
enable wifi
scan wifi
services
agent on
connect wifi_XXXXXXXXXX
quit

sudo ifconfig
```

### Generate ssh key on target
Target:
``` bash
ssh-keygen -t rsa
```

## Setup /etc/hosts
Host:
``` bash
sudo su
./build.sh --env
exit
```

### Copy ssh key to host
Host:
``` bash
ssh-copy-id debian@bbb
ssh debian@bbb
ssh debian@beaglebone.lan
sshb
```

### Update image, install
Target:
``` bash
sudo apt update
#sudo apt install libc6
sudo apt-get install i2c-tools
```

# PX4
## Clone repo
``` bash
./build.sh --clone_px4
```
or
``` bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.15.4
git submodule sync --recursive
git submodule update --init --recursive
```


## Build
https://blog.aviumtechnologies.com/topics/px4-autopilot/building-and-running-px4-autopilot-on-beaglebone-r-blue
https://docs.px4.io/main/en/flight_controller/beaglebone_blue.html

edit PX4-Autopilot/boards/beaglebone/blue/default.px4board
``` bash
CONFIG_BOARD_TESTING=n
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS2"
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS5"
CONFIG_DRIVERS_MAGNETOMETER_AKM_AK8963=y
CONFIG_DRIVERS_MAGNETOMETER_QMC5883L=y
```

``` bash
./build.sh --build
```
or
``` bash
. ./px4build.h
make beaglebone_blue_default
make beaglebone_blue_default upload
```

## Build, upload and start px4
``` bash
./build.sh --clean
./build.sh --build
./build.sh --upload

cd /home/debian/px4
sudo ./bin/px4 -s px4.config
```

# librobotcontrol

## Build and upload

``` bash
./build.sh --build_robot
./build.sh --upload_robot
```


## Run
``` bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/debian/robot/lib
./rc_calibrate_gyro
./rc_calibrate_accel
./rc_calibrate_mag

./rc_test_mpu -t -m
```


## I2C
``` bash
/usr/sbin/i2cdetect -l
/usr/sbin/i2cdetect -y -a -r 2
```

``` bash
debian@beaglebone:~/robot/bin$ /usr/sbin/i2cdetect -y -a -r 0
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- UU -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: UU -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
debian@beaglebone:~/robot/bin$ /usr/sbin/i2cdetect -y -a -r 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
debian@beaglebone:~/robot/bin$ /usr/sbin/i2cdetect -y -a -r 2
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- 76 -- -- -- -- -- -- -- -- --

```

**0x68:** MPU-9250 (not MPU-6050)<br>
**0x76:** BMP280


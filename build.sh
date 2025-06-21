SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PX4_DIR=$SCRIPT_DIR/.

export PATH=/gcc-arm-8.3-2019.02-x86_64-arm-linux-gnueabihf/bin:$PATH

reset
#set -e

function help() {
    echo "build.sh [OPTION]..."
    echo "  -h|--help       Show help."
}

opt_clean=""
opt_upload=""
opt_build=""
opt_install_toolchain=""
opt_env=""
opt_build_robot=""
opt_upload_robot=""
opt_clone_px4=""
opt_upload_pru=""

#if ! grep -q "bbb" /etc/hosts; then
#    echo "192.168.0.194 bbb" >> /etc/hosts
#    echo "192.168.0.194 beaglebone.lan" >> /etc/hosts
#fi


#if [ "$#" -eq 0 ]; then
#alias sshb="ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null debian@bbb"
#fi

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -h|--help) help ;;
        -c|--clean) opt_clean=1 ;;
        -u|--upload) opt_upload=1 ;;
        -b|--build) opt_build=1 ;;
        --build_robot) opt_build_robot=1 ;;
        --upload_robot) opt_upload_robot=1 ;;
        --upload_pru) opt_upload_pru=1 ;;
        --env) opt_env=1 ;;
        --install_toolchain) opt_install_toolchain=1 ;;
        --clone_px4) opt_clone_px4=1 ;;
#        -t|--target) opt_target="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

if [[ $opt_env ]] ; then
    cp $SCRIPT_DIR/.devcontainer/config ~/.ssh/config
    echo 'alias sshb="ssh bbb"' >> ~/.bashrc
fi

if [[ $opt_install_toolchain ]] ; then
    mkdir -p $SCRIPT_DIR/toolchain
    cd $SCRIPT_DIR/toolchain
    #wget https://snapshots.linaro.org/gnu-toolchain/13.0-2022.06-1/arm-linux-gnueabihf/gcc-linaro-13.0.0-2022.06-x86_64_arm-linux-gnueabihf.tar.xz
    #tar -xf gcc-linaro-13.0.0-2022.06-x86_64_arm-linux-gnueabihf.tar.xz
    wget https://developer.arm.com/-/media/Files/downloads/gnu-a/8.3-2019.02/gcc-arm-8.3-2019.02-x86_64-arm-linux-gnueabihf.tar.xz
    tar -xf gcc-arm-8.3-2019.02-x86_64-arm-linux-gnueabihf.tar.xz

    cd $SCRIPT_DIR/download
    wget https://github.com/Kitware/CMake/releases/download/v3.18.4/cmake-3.18.4.tar.gz
    tar zxvf cmake-3.18.4.tar.gz
    cd cmake-3.18.4
    sudo ./bootstrap
    sudo make
    sudo make install
    sudo ln -s /usr/local/bin/cmake /usr/bin/cmake
    cd $SCRIPT_DIR
fi

if [[ $opt_clone_px4 ]] ; then
    cd $SCRIPT_DIR
    git clone https://github.com/snst/PX4-Autopilot.git --recursive
    cd $SCRIPT_DIR/PX4-Autopilot
    #git checkout v1.15.4
    git checkout bbbl1
    git submodule sync --recursive
    git submodule update --init --recursive
    cd $SCRIPT_DIR
fi


if [[ $opt_clean ]] ; then
    cd $PX4_DIR
    make beaglebone_blue_default clean
fi

if [[ $opt_build ]] ; then
    cd $PX4_DIR
    make beaglebone_blue_default
fi


if [[ $opt_upload ]] ; then
    #PX4-Autopilot/boards/beaglebone/blue/cmake/upload.cmake
    cd $PX4_DIR
    make beaglebone_blue_default upload
fi


if [[ $opt_build_robot ]] ; then
    ROBOT_DIR=$PX4_DIR/build/beaglebone_blue_default/librobotcontrol-prefix/src/librobotcontrol
    sed -i 's/:= gcc/:= arm-linux-gnueabihf-gcc/g' $ROBOT_DIR/examples/Makefile
    sed -i 's/:= gcc/:= arm-linux-gnueabihf-gcc/g' $ROBOT_DIR/library/Makefile
    sed -i 's/:= gcc/:= arm-linux-gnueabihf-gcc/g' $ROBOT_DIR/services/rc_battery_monitor/Makefile
    sed -i 's/:= gcc/:= arm-linux-gnueabihf-gcc/g' $ROBOT_DIR/services/robotcontrol/Makefile

    cd $PX4_DIR/build/beaglebone_blue_default/librobotcontrol-prefix/src/librobotcontrol
    make
    #cd $PX4_DIR/build/beaglebone_blue_default/librobotcontrol-prefix/src/librobotcontrol/examples
    #make
fi

if [[ $opt_upload_robot ]] ; then
    #cd $PX4_DIR
    ROBOT_DIR=$PX4_DIR/build/beaglebone_blue_default/librobotcontrol-prefix/src/librobotcontrol
#    rsync -arh --progress $ROBOT_DIR/examples/bin $ROBOT_DIR/library/lib debian@beaglebone.lan:/home/debian/robot
    rsync -arh --progress $ROBOT_DIR/examples/bin debian@bbb:/home/debian/robot
    rsync -arh --progress $ROBOT_DIR/library/lib debian@bbb:/home/debian/robot
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/debian/robot/lib"
fi


if [[ $opt_upload_pru ]] ; then
    #cd $PX4_DIR
    SRC_FILE=$PX4_DIR/build/beaglebone_blue_default/librobotcontrol-prefix/src/librobotcontrol/pru_firmware/fw/am335x-pru1-rc-servo-fw
    rsync -arh --progress $SRC_FILE debian@bbb:/home/debian/px4
    echo "sudo cp am335x-pru1-rc-servo-fw /lib/firmware"
fi


export PATH=/home/runner/.platformio/penv/bin:/usr/local/bin/:$PATH

# 替换VERSION_CODE
sed -i "s/{VERSION}/$GITHUB_REF_NAME/g" include/fishbot_config.h
pip3 install esptool
pio lib install # Install dependencies
cp .config/microros/colcon.meta .pio/libdeps/fishbot_motion_control_four_driver_v2/micro_ros_platformio/metas/colcon.meta
rm -rf .pio/libdeps/fishbot_motion_control_four_driver_v2/micro_ros_platformio/libmicroros/
pio run

rm -rf bin && mkdir bin
export TNAME='fishbot_motion_control_four_driver_v2'
export TVERSION=$GITHUB_REF_NAME
export TDATA=`date +%y%m%d`
export BINNAME=`echo $TNAME`_$TVERSION.$TDATA.bin
export ELFNAME=`echo $TNAME`_$TVERSION.$TDATA.elf

# export boot_app0_dir="/root/.platformio/packages/framework-arduinoespressif32/tools/partitions"
esptool.py  --chip esp32s3 merge_bin -o bin/$BINNAME --flash-mode=keep --flash-size 4MB 0x0000 .pio/build/fishbot_motion_control_four_driver_v2/bootloader.bin 0x8000 .pio/build/fishbot_motion_control_four_driver_v2/partitions.bin  0xe000 ./boards/boot_app0.bin  0x10000 .pio/build/fishbot_motion_control_four_driver_v2/firmware.bin 
cp .pio/build/fishbot_motion_control_four_driver_v2/firmware.elf bin/$ELFNAME
echo "Build Finish bin/`ls bin`"


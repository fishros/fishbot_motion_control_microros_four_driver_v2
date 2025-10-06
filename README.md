# FishBot运动控制程序四驱V2版本


# 本程序及资料仅供参考学习使用，非授权禁止商用。

配套运动控制板（可以在小鱼的店铺直接购买，性价比接地气，直达链接：[https://item.taobao.com/item.htm?id=695473143304](https://item.taobao.com/item.htm?id=695473143304)）：

## 配套开发教程


## 其他指令 

### Merge BootLoad&Frameware

```
export boot_app0_dir="$HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions"
docker run -it --rm --privileged -v=/dev:/dev  -v $boot_app0_dir:$boot_app0_dir -v `pwd`:`pwd` -w `pwd` fishros2/fishbot-tool esptool.py  --chip esp32 merge_bin -o bin/fishbot_motion_control_v1.0.0.`date +%y%m%d`.bin --flash_mode dio --flash_size 4MB 0x1000 .pio/build/featheresp32/bootloader.bin 0x8000 .pio/build/featheresp32/partitions.bin 0xe000 $boot_app0_dir/boot_app0.bin 0x10000 .pio/build/featheresp32/firmware.bin
```


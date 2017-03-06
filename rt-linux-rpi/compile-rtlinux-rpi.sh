#!/bin/bash

echo "script for compiling rtlinux-rpi by wesleysliao"

cd linux
git fetch origin
git reset --hard origin/rpi-4.4.y

wget https://www.kernel.org/pub/linux/kernel/projects/rt/4.4/patch-4.4.47-rt58.patch.gz
zcat patch-4.4.47-rt58.patch.gz | patch -p1

KERNEL=kernel7

make ARCH=arm CROSS_COMPILE=/home/wesley/Sources/rtlinux-rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf- bcm2709_defconfig

make menuconfig



make ARCH=arm CROSS_COMPILE=/home/wesley/Sources/rtlinux-rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf- zImage modules dtbs -j 6



mount ext4

sudo make ARCH=arm CROSS_COMPILE=/home/wesley/Sources/rtlinux-rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf- INSTALL_MOD_PATH=/home/wesley/Sources/rtlinux-rpi/ext4 modules_install

sudo cp ../fat32/$KERNEL.img ../fat32/$KERNEL-backup.img
sudo scripts/mkknlimg arch/arm/boot/zImage ../fat32/$KERNEL.img

sudo cp arch/arm/boot/dts/*.dtb ../fat32/
sudo cp arch/arm/boot/dts/overlays/*.dtb* ../fat32/overlays/
sudo cp arch/arm/boot/dts/overlays/README ../fat32/overlays/

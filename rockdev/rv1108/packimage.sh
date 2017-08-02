#! /bin/bash

usage() {
	echo "usage: ./mkimage.sh <product>" 
}

if [ ! -n "$1" ]; then 
  usage
  exit 0
fi 

OUT=$2
KERNEL_DIR=$(pwd)/$OUT
IMAGE=$(cd `dirname $0`; pwd)
PRODUCT=$1

cat ${KERNEL_DIR}/arch/arm/boot/zImage ${KERNEL_DIR}/arch/arm/boot/dts/$PRODUCT.dtb > $IMAGE/zImage-dtb
$IMAGE/kernelimage --pack --kernel $IMAGE/zImage-dtb $IMAGE/kernel.img 0x62000000 > /dev/null && \
echo "Image: kernel image is ready ${PRODUCT}"
$IMAGE/firmwareMerger -p $IMAGE/setting.ini $IMAGE/

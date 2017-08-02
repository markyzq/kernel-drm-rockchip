#!/bin/sh
DIR=$(cd `dirname $0`; pwd)
upgrade_tool db ${DIR}/RV1108_usb_boot.bin
upgrade_tool wl 0x200 ${DIR}/kernel.img
upgrade_tool rd

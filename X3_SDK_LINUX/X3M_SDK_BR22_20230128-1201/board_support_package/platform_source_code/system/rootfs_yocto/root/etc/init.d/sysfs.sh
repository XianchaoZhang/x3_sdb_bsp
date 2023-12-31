#!/bin/sh
### BEGIN INIT INFO
# Provides:          mountvirtfs
# Required-Start:
# Required-Stop:
# Default-Start:     S
# Default-Stop:
# Short-Description: Mount kernel virtual file systems.
# Description:       Mount initial set of virtual filesystems the kernel
#                    provides and that are required by everything.
### END INIT INFO

if [ -e /proc ] && ! [ -e /proc/mounts ]; then
  mount -t proc proc /proc
fi

if [ -e /sys ] && grep -q sysfs /proc/filesystems && ! [ -e /sys/class ]; then
  mount -t sysfs sysfs /sys
fi

if [ -e /sys/kernel/debug ] && grep -q debugfs /proc/filesystems; then
  mount -t debugfs debugfs /sys/kernel/debug
fi

if [ -e /sys/kernel/config ] && grep -q configfs /proc/filesystems; then
  mount -t configfs configfs /sys/kernel/config
fi

if ! [ -e /dev/zero ] && [ -e /dev ] && grep -q devtmpfs /proc/filesystems; then
  mount -n -t devtmpfs devtmpfs /dev
fi
setprop sysfs.ready 1

# set min_free_kbytes to (MemTotal - CmaTotal) * 3%
MEMORY=`cat /proc/meminfo | awk '/MemTotal/ || /CmaTotal/ {print $2}'`
MEM=`echo $MEMORY | cut -d \  -f1`
CMA=`echo $MEMORY | cut -d \  -f2`
MIN=`awk -v x=$MEM -v y=$CMA 'BEGIN {printf ("%.0f", (x-y)*0.03)}'`
echo $MIN > /proc/sys/vm/min_free_kbytes

#!/bin/sh
### BEGIN INIT INFO
# Provides:          hostname
# Required-Start:
# Required-Stop:
# Default-Start:     S
# Default-Stop:
# Short-Description: Set hostname based on /sys/socinfo/board_name
### END INIT INFO
HOSTNAME=$(/bin/hostname)

# Busybox hostname doesn't support -b so we need implement it on our own
if [ -f /sys/class/socinfo/board_name ];then
	str=`cat /sys/class/socinfo/board_name`
	name=${str}
	hostname ${name}
elif [ -z "$HOSTNAME" -o "$HOSTNAME" = "(none)" -o ! -z "`echo $HOSTNAME | sed -n '/^[0-9]*\.[0-9].*/p'`" ] ; then
	hostname localhost
fi
setprop hostname.ready 1

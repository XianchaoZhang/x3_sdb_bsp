#!/bin/sh
### BEGIN INIT INFO
# Provides:             dmesg
# Required-Start:
# Required-Stop:
# Default-Start:        S
# Default-Stop:
### END INIT INFO

if [ -f /tmp/log/dmesg ]; then
	if LOGPATH=$(which logrotate); then
		$LOGPATH -f /etc/logrotate-dmesg.conf
	else
		mv -f /tmp/log/dmesg /tmp/log/dmesg.old
	fi
fi
mkdir -p /tmp/log
dmesg -s 131072 > /tmp/log/dmesg

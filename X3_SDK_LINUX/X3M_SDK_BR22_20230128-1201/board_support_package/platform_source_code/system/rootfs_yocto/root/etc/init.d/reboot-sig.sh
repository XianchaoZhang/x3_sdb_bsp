#!/bin/sh

# send SIGTERM signal
echo "Sending all process the TERM signal..."
killall5 -15
sleep 1
echo "Sending all process the KILL signal..."
killall5 -9
sleep 1
/etc/rc6.d/S40umountfs
# trigger real reboot
setprop sysreboot.start 1

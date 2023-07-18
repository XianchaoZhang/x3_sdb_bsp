#!/bin/sh
### BEGIN INIT INFO
# Provides:          mountall
# Required-Start:    mountvirtfs
# Required-Stop: 
# Default-Start:     S
# Default-Stop:
# Short-Description: Mount all filesystems.
# Description:
### END INIT INFO

# Check /userdata and /app partiton first(except Quickboot mode)
check_fs_script=/etc/init.d/1mkfs.sh
target_mode=$(cat /etc/version | awk '{print $2}')

if [ -f $check_fs_script ];then
	[ "$target_mode" != "quickboot" ] && sh $check_fs_script
else
	echo "$check_fs_script not exist!" > /dev/ttyS0
fi

. /etc/default/rcS

#
# Mount local filesystems in /etc/fstab. For some reason, people
# might want to mount "proc" several times, and mount -v complains
# about this. So we mount "proc" filesystems without -v.
#
test "$VERBOSE" != no && echo "Mounting local filesystems..."
mount -at nonfs,nosmbfs,noncpfs 2>/dev/null

# Check whether the /userdata and /app partiton mount is successful.
app_mount=`cat /proc/mounts |grep -w "/app" |awk '{print $2}'`
userdata_mount=`cat /proc/mounts |grep -w "/userdata" |awk '{print $2}'`
if [ -z "$userdata_mount" ] || [ -z "$app_mount" ];then
	echo "mount /userdata or /app failed, retry..." > /dev/ttyS0
	sh $check_fs_script
	mount -at nonfs,nosmbfs,noncpfs 2>/dev/null
fi
setprop mount.ready 1

#
# We might have mounted something over /dev, see if /dev/initctl is there.
#
if test ! -p /dev/initctl
then
	rm -f /dev/initctl
	mknod -m 600 /dev/initctl p
fi
kill -USR1 1

#
# Execute swapon command again, in case we want to swap to
# a file on a now mounted filesystem.
#
[ -x /sbin/swapon ] && swapon -a

: exit 0


#!/bin/sh
##set init selinux file context in /app /userdata /map /app_param /log

app_mount=`cat /proc/mounts |grep -w "/app" |awk '{print $2}'`
if [ -n $app_mount ]; then
    app_contexts=`ls -Z / | grep -w "app" |awk '{print $1}' |grep unlabeled`
    if [ -n $app_contexts ]; then
        #set context info in /app
        mount -o remount rw /app > /dev/null 2>&1
        setfiles /etc/selinux/targeted/contexts/files/file_contexts  /app > /dev/null 2>&1
        mount -o remount ro /app > /dev/null 2>&1
    fi
fi

userdata_mount=`cat /proc/mounts |grep -w "/userdata" |awk '{print $2}'`
if [ -n $userdata_mount ]; then
    userdata_contexts=`ls -Z / | grep -w "userdata" |awk '{print $1}' |grep unlabeled`
    if [ -n $userdata_contexts ]; then
        #set context info in /userdata
        setfiles /etc/selinux/targeted/contexts/files/file_contexts  /userdata > /dev/null 2>&1
    fi
fi

map_mount=`cat /proc/mounts |grep -w "/map" |awk '{print $2}'`
if [ -n $map_mount ]; then
    map_contexts=`ls -Z / | grep -w "map" |awk '{print $1}' |grep unlabeled`
    if [ -n $map_contexts ]; then
        #set context info in /map
        setfiles /etc/selinux/targeted/contexts/files/file_contexts  /map > /dev/null 2>&1
    fi
fi

app_param_mount=`cat /proc/mounts |grep -w "/app_param" |awk '{print $2}'`
if [ -n $app_param_mount ]; then
    app_param_contexts=`ls -Z / | grep -w "app_param" |awk '{print $1}' |grep unlabeled`
    if [ -n $app_param_contexts ]; then
        #set context info in /app_param
        setfiles /etc/selinux/targeted/contexts/files/file_contexts  /app_param > /dev/null 2>&1
    fi
fi

log_mount=`cat /proc/mounts |grep -w "/log" |awk '{print $2}'`
if [ -n $log_mount ]; then
    log_contexts=`ls -Z / | grep -w "log" |awk '{print $1}' |grep unlabeled`
    if [ -n $log_contexts ]; then
        #set context info in /log
        setfiles /etc/selinux/targeted/contexts/files/file_contexts  /log > /dev/null 2>&1
    fi
fi

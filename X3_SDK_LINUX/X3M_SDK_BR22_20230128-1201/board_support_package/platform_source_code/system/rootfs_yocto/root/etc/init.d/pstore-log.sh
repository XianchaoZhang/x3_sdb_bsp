#!/bin/sh
### BEGIN INIT INFO
# Provides:             pstore-log
# Required-Start:
# Required-Stop:
# Default-Start:        S
# Default-Stop:
### END INIT INFO

PSTORE_FS=`cat /proc/mounts |grep "^pstore" |awk '{print $2}'`
PSTORE_LOG=/userdata/log/pstore
PSTORE_LOGMAX=100
PSTORE_LOGINDEX=0

if [ ! -z "${PSTORE_FS}" ] && [ -f ${PSTORE_FS}/console-ramoops-0 ] && [ ! -e ${PSTORE_LOG}/disable ]; then
	if [ ! -f ${PSTORE_FS}/dmesg-ramoops-0 ] && [ ! -e ${PSTORE_LOG}/always ]; then
		exit 0
	fi
	mkdir -p ${PSTORE_LOG}
	if [ -f ${PSTORE_LOG}/max ]; then
		PSTORE_LOGMAX=`cat ${PSTORE_LOG}/max`
	else
		echo ${PSTORE_LOGMAX} > ${PSTORE_LOG}/max
	fi
	if [ -f ${PSTORE_LOG}/index ]; then
		PSTORE_LOGINDEX=`cat ${PSTORE_LOG}/index`
		PSTORE_LOGINDEX=`expr ${PSTORE_LOGINDEX} + 1`
	fi
	if [ ${PSTORE_LOGMAX} -gt 0 ]; then
		PSTORE_LOGCNT=`ls ${PSTORE_LOG} -l |grep "^d" |wc -l`
		if [ ${PSTORE_LOGCNT} -ge ${PSTORE_LOGMAX} ]; then
			PSTORE_LOGCNT=`expr ${PSTORE_LOGCNT} - ${PSTORE_LOGMAX} + 1`
			PSTORE_LOGDEL=`ls ${PSTORE_LOG} -l |grep "^d" |awk '{print $9}' |sort -n |head -n ${PSTORE_LOGCNT}`
			for d in ${PSTORE_LOGDEL}; do
				rm -fr ${PSTORE_LOG}/${d}
			done
		fi
	fi
	PSTORE_LOGDATE=`date +%Y%m%d-%H%M%S`
	PSTORE_LOGDIR=${PSTORE_LOG}/${PSTORE_LOGINDEX}.${PSTORE_LOGDATE}
	if [ -d ${PSTORE_LOGDIR} ]; then
		rm -fr ${PSTORE_LOGDIR}
	fi
	mkdir -p ${PSTORE_LOGDIR}
	echo "pstore log to ${PSTORE_LOGDIR}"
	mv ${PSTORE_FS}/* ${PSTORE_LOGDIR}/
	echo ${PSTORE_LOGINDEX} > ${PSTORE_LOG}/index
	sync
fi

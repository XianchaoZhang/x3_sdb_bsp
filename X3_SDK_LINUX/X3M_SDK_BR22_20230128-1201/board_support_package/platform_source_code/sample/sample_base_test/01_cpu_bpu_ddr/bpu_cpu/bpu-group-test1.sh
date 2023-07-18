curPath=$(pwd)
export BPLAT_BUSY_THRES=1
PORTION=100
PORTION_TH_LOW=0
PORTION_TH_HIGH=0
TH_VALUE1=80   #for two bpu core to correct portion value
TH_VALUE2=50   #for single bpu core to correct portion value1
TH_VALUE3=80   #for single bpu core to correct portion value2
FULL=100
COR1=2
COR2=5
BPU=2
RATIO_BPU0=0
RATIO_BPU1=0
usage="Usage: $0 [-b bpu core] [-p portion]"

while getopts "p:b:h" opt
do
	case $opt in
		p)
			PORTION=$OPTARG
			;;
		b)
			BPU=$OPTARG
			;;
		h)
			echo $usage
			exit 0
			;;
		\?)
			echo $usage
			exit 1
			;;
	esac
done
if [ -f "/sys/devices/system/bpu/profiler_frequency" ]; then
	echo 5 > /sys/devices/system/bpu/profiler_n_seconds
	echo 250 > /sys/devices/system/bpu/profiler_frequency
	echo 1   > /sys/devices/system/bpu/profiler_enable
fi
echo "bpu:$BPU"
if [ $PORTION -lt 30 ]; then
	PORTION_TH_HIGH=`expr $PORTION + 3`
	PORTION_TH_LOW=`expr $PORTION - 10`
elif [ $PORTION -ge 30 ] && [ $PORTION -lt 60 ]; then
	PORTION_TH_HIGH=`expr $PORTION + 3`
	PORTION_TH_LOW=`expr $PORTION - 30`
else
	PORTION_TH_HIGH=`expr $PORTION + 4`
	PORTION_TH_LOW=`expr $PORTION - 45`
fi
if [ $BPU -eq 1 ] || [ $BPU -eq 0 ]; then
	if [ $PORTION -lt $TH_VALUE2 ]; then
		echo "portion:$PORTION,execute I0001"
		$curPath/libbpu/HBDK3_MODEL_I0001/run.sh $PORTION $BPU  &
	elif [ $PORTION -ge $TH_VALUE2 ] && [ $PORTION -lt $TH_VALUE3 ]; then
		echo "portion:$PORTION,execute I0012"
		$curPath/libbpu/HBDK3_MODEL_I0012/run.sh $PORTION $BPU  &
	elif [ $PORTION -eq $FULL ]; then
		echo "portion:$PORTION,execute 2K"
		$curPath/libbpu/HBDK3_MODEL_2K/run.sh $PORTION $BPU  &
	else
		echo "portion:$PORTION,execute P0000"
		$curPath/libbpu/HBDK3_MODEL_P0000/run.sh $PORTION $BPU  &
	fi
	sleep 20
	for i in $(seq 1 3)
	do
		RATIO_BPU0=`cat /sys/devices/system/bpu/bpu$BPU/ratio`
		echo "RATIO BPU:$RATIO_BPU0"
		if [ $RATIO_BPU0 -gt $PORTION_TH_HIGH ] || [ $RATIO_BPU0 -lt $PORTION_TH_LOW ];then
			echo "test bpu$BPU group $i time failed"
			break
		else
			echo "test bpu$BPU group $i time success"
		fi
		if [ $i -lt 3 ]; then
			sleep 15
		else
			i=4
		fi
	done
	if [ $i -lt 4 ]; then
		echo "bpu group test failed"
		if [ -f "/sys/devices/system/bpu/profiler_n_seconds" ]; then
			echo 1 > /sys/devices/system/bpu/profiler_n_seconds
		fi
		killall tc_hbdk3
		exit 1
	else
		echo "bpu broup test success"
		if [ -f "/sys/devices/system/bpu/profiler_n_seconds" ]; then
			echo 1 > /sys/devices/system/bpu/profiler_n_seconds
		fi
		killall tc_hbdk3
	fi
elif [ $BPU -eq 2 ]; then
	if [ $PORTION -lt $TH_VALUE1 ]; then
		echo "portion:$PORTION,execute HBDK3_MODEL_P0000"
		$curPath/libbpu/HBDK3_MODEL_P0000/run.sh $PORTION $BPU &
	elif [ $PORTION -eq $FULL ]; then
		echo "portion:$PORTION,execute HBDK3_MODEL_2K"
		$curPath/libbpu/HBDK3_MODEL_2K/run.sh $PORTION $BPU &
	else
		echo "portion is $PORTION"
		$curPath/libbpu/HBDK3_MODEL_P0000/run.sh $PORTION $BPU  &
	fi
	echo "holding 3 seconds..."
	sleep 3
	i=1
	while [ True ]
	#for i in $(seq 1 2147483647)
	do
		RATIO_BPU0=`cat /sys/devices/system/bpu/bpu0/ratio`
		RATIO_BPU1=`cat /sys/devices/system/bpu/bpu1/ratio`
		echo "RATIO BPU0:$RATIO_BPU0"
		echo "RATIO BPU1:$RATIO_BPU1"
		if [ $RATIO_BPU0 -gt $PORTION_TH_HIGH ] || [ $RATIO_BPU0 -lt $PORTION_TH_LOW ];then
			echo "test bpu0 $i time failed"
			break
		else
			echo "test bpu0 group $i time success"
		fi
		if [ $RATIO_BPU1 -gt $PORTION_TH_HIGH ] || [ $RATIO_BPU1 -lt $PORTION_TH_LOW ];then
			echo "test bpu1 $i time failed"
			break
		else
			echo "test bpu1 group $i time success"
		fi
		if [ $i -lt 2147483647 ]; then
			sleep 0.5
			i=$(($i+1))
		else
			i=4
		fi

	done
	if [ $i -lt 4 ]; then
		echo "bpu group test failed"
		if [ -f "/sys/devices/system/bpu/profiler_n_seconds" ]; then
			echo 1 > /sys/devices/system/bpu/profiler_n_seconds
		fi
		killall tc_hbdk3
		exit 1
	else
		echo "bpu broup test success"
		if [ -f "/sys/devices/system/bpu/profiler_n_seconds" ]; then
			echo 1 > /sys/devices/system/bpu/profiler_n_seconds
		fi
		killall tc_hbdk3
	fi
else
	echo "select bpu parameter 0/1/2"
fi


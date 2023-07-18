echo 3 > /proc/sys/vm/drop_caches
nohup ../bin/stressapptest -s 172800 -M 100 -f /tmp/sat.io1 -f /tmp/sat.io2 -i 4 -m 8 -C 2 -W > ../output/cpu-stress.log &
nohup ../bin/memtester 100M 440 >> /userdata/mem.log 2>&1 &
echo userspace > /sys/class/devfreq/devfreq0/governor
echo cr5 > /sys/class/devfreq/devfreq0/device/method
while true
do
        let count+=1
        echo "switch 1333 start"
        echo 1333000000 > /sys/class/devfreq/devfreq0/userspace/set_freq
        echo "switch 1333 done"
        sleep 0.5
        echo "switch 333 start"
        echo 333000000 > /sys/class/devfreq/devfreq0/userspace/set_freq
        echo "switch 333 done"
        sleep 0.5
        echo "switch 2666 start"
        echo 2666000000 > /sys/class/devfreq/devfreq0/userspace/set_freq
        echo "switch 2666 done"
        sleep 0.5
        echo "switch 333 start"
        echo 333000000 > /sys/class/devfreq/devfreq0/userspace/set_freq
        echo "switch 333 done"
        sleep 0.5
        echo "switch 1333 start"
        echo 1333000000 > /sys/class/devfreq/devfreq0/userspace/set_freq
        echo "switch 1333 done"
        sleep 0.5
        echo "switch 2666 start"
        echo 2666000000 > /sys/class/devfreq/devfreq0/userspace/set_freq
        echo "switch 2666 done"
        sleep 0.5
        echo "enter :$count"
	hrut_somstatus  > ../output/hrut_somstatus.log
done


#!bin/sh

function run
{
    name=$1
    cmd=$2
    timeout=$3
    echo -e -n "$name\r\033[40C[Running]"
    rm -f /tmp/success
    $cmd >> /app/factory_test.log 2>&1 && touch /tmp/success &
    pid=$!
    while [ $timeout -ne 0 ]
    do
        sleep 1
        let timeout--
        if [ ! -d /proc/$pid ];then
            if [ -f /tmp/success ];then
                echo -e "\r$name\r\033[40C[pass]\033[K"
                echo "test $name pass" >> /app/factory_test.log
            else
                echo -e "\r$name\r\033[40C[fail]\033[K"
                echo "test $name fail" >> /app/factory_test.log
            fi
            return
        fi
    done
    kill -9 $pid
    echo -e "\r$name\r\033[40C[timeout]\033[K"
    echo "test $name timeout" >> /app/factory_test.log
}

if [ ! -f /app/factory_test ];then
    echo "skip factory test"
    exit 0
fi

rm -f /app/factory_test
if [ -f /sys/socinfo/board_id ]; then
	boardid=$(cat /sys/socinfo/board_id |sed -e 's/board_id://')
else
	boardid=$(cat /sys/class/socinfo/board_id)
fi

echo 0 > /proc/sys/kernel/printk
echo -e "\n\n   ========== factory test for $boardid start =========="

#test start
run "memtester" "/app/bin/memtester 5M 1" 15
run "storagetest" "/app/scripts/storage_test 5" 80
if [ "$boardid" != "204" ] && [ "$boardid" != "205" ]; then
	run "norflashtest" "/app/scripts/norflash.sh" 15
fi
run "i2ctest" "/app/scripts/i2c_test.sh" 5
run "pmictest" "/app/scripts/pmic_test.sh" 5
if [ "$boardid" != "204" ] && [ "$boardid" != "205" ]; then
	run "tempsensortest" "/app/scripts/temp_test.sh" 5
fi
run "stressapptest" "/app/bin/stressapptest -s 20 -M 100 -m 8 -C 2 -W" 30
run "bputest" "/app/libbpu/MODEL-070/run.sh" 60
run "isptest" "/app/bin/isp_test" 50
#test end

echo -e "   ========== factory test for $boardid finish =========\n\n"

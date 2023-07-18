1、驱动支持
方法一：
	加载预编译驱动，找到对应SDK版本的patch，把 rtc-pcf8563.ko 上传到设备上，然后执行以下命令加载驱动
	insmod rtc-pcf8563.ko

方法二：
	修改内核配置，支持PCF8563驱动
	CONFIG_RTC_DRV_PCF8563=y

驱动加载成功后，会出现 /dev/rtc1 设备节点，对应的就是pcf-8563

使用：
建立 /dev/rtc1 到 /dev/rtc1 的软连接

rm /dev/rtc
ln -s /dev/rtc1 /dev/rtc

测试方法：
date -s 2022.01.21-21:24:00   	# 设置系统时间
hwclock -w       				# 将系统时间写入RTC
hwclock -r       				# 读取RTC时间，确认时间是否写入成功
hwclock -s       				# 如果rtc电池存在，断电后，再上电，将RTC时间更新到系统时间
date             				# 读取系统时间
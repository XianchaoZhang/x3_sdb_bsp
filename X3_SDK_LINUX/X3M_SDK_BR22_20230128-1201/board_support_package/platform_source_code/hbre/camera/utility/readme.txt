1、编译cam_utility.c,得到可执行文件camera_config
2、camera_config主要做的事情就是根据设置的sensor_name,配置对应sensor的setting
3、执行步骤：
	./camera_config [cam_ctl] [i2c_bus] [i2c_addr] [power_mode] [gpio_num] [sensor_name]
	cam_ctl:如果sensor支持多size,可以根据cam_size选择对应的setting, 例如 : 720,1080;
	i2c_bus:根据实际硬件，设置对应cam i2c bus 例如 : 0、1;
	i2c_addr:根据实际硬件，设置对应cam i2c 地址 例如 : 0x36 0x38;
	power_mode:设备是否需要单独对camerasensor 上电，0 不需要单独上电，1需要单独上电
	gpio_num:启动sensor使能引脚gpio, 例如 : 0x56;
	sensor_name:需要config的camera名,如 ov13855,ov10635;
	
	cam_format:如果sensor支持多种格式,可以根据cam_format选择对应的setting,例如 : yuv_10,raw_10;
	
	
4、如果需要其他对sensor定制化的设置,utility可拓展
5、
	power_mode = 是否单独使能camera
	i2c_bus + i2c_addr = i2c读写 setting
	sensor_name = 对应camerasetting
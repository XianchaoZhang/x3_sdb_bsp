#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>

#include "utils/common_utils.h"

#include "x3_utils.h"
#include "utils/cJSON.h"

// 获取主芯片类型
E_CHIP_TYPE x3_get_chip_type(void)
{
	int ret = 0;
	FILE *stream;
	char chip_id[16] = {0};

	stream = fopen("/sys/class/socinfo/chip_id", "r");
	if (!stream) {
		printf("open fail: %s\n", strerror(errno));
		return -1; 
	}   
	ret = fread(chip_id, sizeof(char), 9, stream);
	if (ret != 9) {
		printf("read fail: %s\n", strerror(errno));
		fclose(stream);
		return -1; 
	}   
	fclose(stream);

	if (strncmp("0xab36300", chip_id, 9) == 0)
		return E_CHIP_X3M; // x3m
	else if (strncmp("0xab37300", chip_id, 9) == 0)
		return E_CHIP_X3E; // x3e
	else return E_CHIP_X3M;
}

typedef struct sensor_id {
  int i2c_bus;           // sensor挂在哪条总线上
  int i2c_dev_addr;      // sensor i2c设备地址
  int i2c_addr_width;
  int det_reg;
  char sensor_name[10];
  int enable_bit; // 本sensor对应的list中的bit位
} sensor_id_t;

#define I2C_ADDR_8		1
#define I2C_ADDR_16		2

sensor_id_t sensor_id_list[] =
{
	{3, 0x36, I2C_ADDR_16, 0x0100, "ov8856", SENSOR_OV8856_SUPPORT}, // ov8856
	{2, 0x40, I2C_ADDR_8, 0x0B, "f37", SENSOR_F37_SUPPORT},           // F37
	{1, 0x40, I2C_ADDR_8, 0x0B, "f37", SENSOR_F37_SUPPORT},           // F37
	{2, 0x36, I2C_ADDR_16, 0x0100, "os8a10", SENSOR_OS8A10_SUPPORT},     // os8a10
	{2, 0x1a, I2C_ADDR_16, 0x0000, "imx415", SENSOR_IMX415_SUPPORT},     // imx415
	{2, 0x29, I2C_ADDR_16, 0x03F0, "gc4663", SENSOR_GC4663_SUPPORT},     // gc4663
	{1, 0x29, I2C_ADDR_16, 0x03F0, "gc4663", SENSOR_GC4663_SUPPORT},     // gc4663
	{2, 0x36, I2C_ADDR_16, 0x0000, "imx415_bv", SENSOR_IMX415_BV_SUPPORT} // second imx415
};

int x3_get_hard_capability(hard_capability_t *capability)
{
	int i = 0, ret = 0;
	char cmd[256];
	char result[1024];
	char file_name[128];
	struct stat file_stat;
	FILE *stream;

	E_CHIP_TYPE chip_id = x3_get_chip_type();
	printf("chip_type: %s\n", chip_id == E_CHIP_X3M ? "X3M" : "X3E");
	capability->m_chip_type = chip_id;

	/* sdb 生态开发板  ，使能sensor       mclk, 否则i2c 通信不会成功 */
	for (i = 0; i < 4; i++) {
		/* 读取 /sys/class/vps/mipi_host[i]/param/snrclk_freq  的值 \
		 * 如果该mipi host可以使用则会是一个大于1000的值，否则为0 \
		 * 通过判断该值不为0作为可以使能该路mipi mclk的依据
		 */
		memset(file_name, '\0', sizeof(file_name));
		memset(result, '\0', sizeof(result));
		sprintf(file_name, "/sys/class/vps/mipi_host%d/param/snrclk_freq", i);
		stream = fopen(file_name, "r");
		if (!stream) {
			continue;
		}
		ret = fread(result, sizeof(char), 32, stream);
		if (ret <= 0) {
			printf("read fail\n");
			fclose(stream);
			return -1; 
		}
		fclose(stream);

		/* 如果频率不为0   就使能该路mclk */
		if (strncmp(result, "0", 1) != 0)
			HB_MIPI_EnableSensorClock(i);
	}

	for (i = 0; i < ARRAY_SIZE(sensor_id_list); i++) {
		/* 判断I2C总线是否使能，如果没有使能则直接continue */
		memset(file_name, '\0', sizeof(file_name));
		sprintf(file_name, "/dev/i2c-%d", sensor_id_list[i].i2c_bus);
		if (stat(file_name, &file_stat) == -1)
			continue;
		/* 通过i2ctransfer命令读取特定寄存器，判断是否读取成功来判断是否支持相应的sensor */
		memset(cmd, '\0', sizeof(cmd));
		memset(result, '\0', sizeof(result));
		if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_8) {
			sprintf(cmd, "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1", sensor_id_list[i].i2c_bus,
			sensor_id_list[i].i2c_dev_addr, sensor_id_list[i].det_reg);
		} else if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_16){
			sprintf(cmd, "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1", sensor_id_list[i].i2c_bus,
			sensor_id_list[i].i2c_dev_addr,
			sensor_id_list[i].det_reg >> 8, sensor_id_list[i].det_reg & 0xFF);
		} else {
			continue;
		}
		exec_cmd_ex(cmd, result, sizeof(result));
		if (strstr(result, "Error") == NULL && strstr(result, "error") == NULL) { // 返回结果中不带Error, 说明sensor找到了
			printf("--------match sensor:%s\n", sensor_id_list[i].sensor_name);
			capability->m_sensor_list |= sensor_id_list[i].enable_bit;
		}
	}

	printf("support sensor: %d\n", capability->m_sensor_list);
	return 0;
}

/* 因为有相同的sensor接在不同的i2c总线上，比如SDB板用的是i2c2,   X3 PI用的i2c1 \
 * 本接口用来判断Sensor具体挂在哪条总线上，并返回总线号 */
int x3_get_sensor_on_which_i2c_bus(const char *sensor_name)
{
	int i = 0;
	char cmd[256];
	char result[1024];
	char file_name[128];
	struct stat file_stat;

	/* 遍历sensor列表，对比sensor_name做判断 */
	for (i = 0; i < ARRAY_SIZE(sensor_id_list); i++) {
		if (strcasecmp(sensor_name, sensor_id_list[i].sensor_name) != 0)
			continue;
		/* 判断I2C总线是否使能，如果没有使能则直接continue */
		memset(file_name, '\0', sizeof(file_name));
		sprintf(file_name, "/dev/i2c-%d", sensor_id_list[i].i2c_bus);
		if (stat(file_name, &file_stat) == -1)
			continue;
		/* 通过i2ctransfer命令读取特定寄存器，判断是否读取成功来判断是否支持相应的sensor */
		memset(cmd, '\0', sizeof(cmd));
		memset(result, '\0', sizeof(result));
		if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_8) {
			sprintf(cmd, "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1", sensor_id_list[i].i2c_bus,
			sensor_id_list[i].i2c_dev_addr, sensor_id_list[i].det_reg);
		} else if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_16){
			sprintf(cmd, "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1", sensor_id_list[i].i2c_bus,
			sensor_id_list[i].i2c_dev_addr,
			sensor_id_list[i].det_reg >> 8, sensor_id_list[i].det_reg & 0xFF);
		} else {
			continue;
		}
		exec_cmd_ex(cmd, result, sizeof(result));
		if (strstr(result, "Error") == NULL && strstr(result, "error") == NULL) { // 返回结果中不带Error, 说明sensor找到了
			return sensor_id_list[i].i2c_bus;
		}
	}

	return -1;
}


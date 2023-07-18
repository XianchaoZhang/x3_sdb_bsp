#ifndef COMMON_H
#define COMMON_H

#include <sys/types.h>
#include <stdio.h>
#include <sys/time.h>

#define vmon_err(format, ...) printf("[%s]%s[%d] viomonitor E: "format"\n",__TIME__, __func__, __LINE__, ##__VA_ARGS__)
#define vmon_info(format, ...) printf("[%s]%s[%d] viomonitor L: "format"\n",__TIME__, __func__, __LINE__, ##__VA_ARGS__)

//#define vmon_dbg(format, ...) printf("[%s]%s[%d] viomonitor D: "format"\n",__TIME__, __func__, __LINE__, ##__VA_ARGS__)
#define vmon_dbg(format, ...)

enum RAW_BIT {
	RAW_8 = 0,
	RAW_10 = 1,
	RAW_12 = 2,
	RAW_14 = 3,
	RAW_16 = 4,
};

enum YUV_TYEP {
	YUVNV12 = 0,
	YUV420 = 1,
	YUV422 = 2,
	YUV444 = 3,
};

enum VIDEO_TYPE {
	H264 = 0,
	H265 = 1,
	PPS = 2,
};

enum DateType {
	RAW_DATA = 0,
	YUV_DATA = 1,
	JEPG_DATA = 2,
	VIDEO_DATA = 3,
	VIDEO_CFG = 4,
	STATS_AWB_DATA = 5,
	STATS_AEfull_DATA = 6,
	STATS_AE5bin_DATA = 6,
	STATS_LUMVAR_DATA = 7,
	STATS_AF_DATA = 8,
};

enum sensor_mode {
	NORMAL_HDR = 1,
	DOL2 = 2,
	DOL3 = 3,
	DOL4 = 4,
};

#endif // COMMON_H

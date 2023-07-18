#ifndef __USB_CAMERA_UTILS_H__
#define __USB_CAMERA_UTILS_H__

#include <time.h>

#define interval_cost_trace_in() \
	struct timespec t1, t2; \
	static long t1_last = 0; \
 \
	clock_gettime(CLOCK_MONOTONIC, &t1); \
	long t1_ms = t1.tv_sec * 1000 + t1.tv_nsec / (1000 * 1000); \
	long intv = t1_ms - t1_last;

#define interval_cost_trace_out() \
	clock_gettime(CLOCK_MONOTONIC, &t2); \
 \
	long t2_ms = t2.tv_sec * 1000 + t2.tv_nsec / (1000 * 1000); \
	long dur = t2_ms - t1_ms; \
	printf("fill buffer(%d bytes) t1_ms(%ldms) t2_ms(%ldms) " \
		"interval (%ldms), and cost (%ldms)\n", \
		*buf_len, t1_ms, t2_ms, intv, dur); \
	t1_last = t1_ms;



#endif	/* __USB_CAMERA_UTILS_H__ */

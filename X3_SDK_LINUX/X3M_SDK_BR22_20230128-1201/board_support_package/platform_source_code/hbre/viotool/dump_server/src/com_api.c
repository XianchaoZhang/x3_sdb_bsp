#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <fcntl.h>
#include <signal.h>

#include "com_api.h"
#include "common.h"

void print_timestamp(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	vmon_dbg("tv = %ld.%ld\n", tv.tv_sec, tv.tv_usec);
}

/*
 * Copyright (c) 2019 Horizon Robotics
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *file_socuid = "/sys/class/socinfo/soc_uid";

int main(int argc, char **argv)
{
    size_t ret = 0;
	FILE *stream;
	char socuid[40] = {0};

	stream = fopen(file_socuid, "r");
	if (!stream) {
		printf("open soc_uid fail\n");
		return -1;
	}
	ret = fread(socuid, sizeof(char), 32, stream);
    if (ret != 32) {
        printf("read soc_uid fail\n");
        fclose(stream);
        return -1;
    }
	fclose(stream);
    printf("soc_uid: %s\n", socuid);
	return 0;
}

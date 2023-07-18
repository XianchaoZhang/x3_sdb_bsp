#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <signal.h>

#include "server_tcp.h"

int create_tcp_socket(void)
{
	int fd = -1;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		printf("<%s: %d>socket error\n", __FUNCTION__, __LINE__);
	}

	return fd;
}

int ipaddr_to_int(const char *ipaddr)
{
	int ip = 0;
	int a, b, c, d;

	sscanf(ipaddr, "%d.%d.%d.%d.", &a, &b, &c, &d);

	if (0 <= a && a <= 255 && 0 <= b && b <= 255 &&
		0 <= c && c <= 255 & 0 <= d && d <= 255)
	ip = (a & 0xff) << 24 | (b & 0xff) << 16 | (c & 0xff) << 8 | (d & 0xff);

	return ip;
}

int read_packet(int fd, char *buf, int len)
{
	int bytes_left = len;
	int bytes_read = 0;
	uint32_t loop = 0;
	char *ptr = NULL;

	if (fd < 0)
		return -1;

	ptr = buf;
	while (bytes_left > 0) {
		bytes_read = read(fd, ptr, bytes_left);

		if (bytes_read < 0) {
			if ((errno == EINTR) || (errno == EAGAIN)
				|| (errno == EWOULDBLOCK)) {
				bytes_read = 0;
			} else {
				vmon_err("read pactet err is %s\n", strerror(errno));
				return -1;
			}
		} else if (bytes_read == 0) {
				vmon_err("read pactet err is %s\n", strerror(errno));
				return -1;
		}

		bytes_left -= bytes_read;
		ptr += bytes_read;

		loop++;
		if (loop > 3) {
			break;
		}
	}

	return len - bytes_left;
}

int write_packet(int fd, char *buf, int len)
{
	int bytes_left = len;
	int bytes_write = 0;
	char *ptr = NULL;

	if (fd < 0)
		return -1;

	ptr = buf;
	while(bytes_left > 0) {
		bytes_write = write(fd, ptr, bytes_left);
		if (bytes_write < 0) {
			if ((errno == EINTR) || (errno == EAGAIN) ||
				(errno == EWOULDBLOCK)) {
				bytes_write = 0;
			} else {
				vmon_err("write pactet err is %s\n", strerror(errno));
				return -1;
			}
		} else if (bytes_write == 0) {
				vmon_err("write pactet err is %s\n", strerror(errno));
			return -1;
		}

		bytes_left -= bytes_write;
		ptr += bytes_write;
	}

	return len - bytes_left;
}

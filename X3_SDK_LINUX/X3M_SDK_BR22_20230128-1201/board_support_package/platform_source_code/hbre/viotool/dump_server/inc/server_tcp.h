#ifndef __SERVER_TCP_H
#define __SERVER_TCP_H

#include "common.h"

int create_tcp_socket(void);
int ipaddr_to_int(const char *ipaddr);
int read_packet(int fd, char *buf, int len);
int write_packet(int fd, char *buf, int len);

#endif // TCPDUMPSERVER_H

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <signal.h>

#include "server_core.h"
#include "server_tcp.h"
#include "server_cmd.h"
#include "code_func.h"
#include "vio_func.h"
#include "server_config.h"
#include "common.h"

#define FD_NUM 10

static int socket_fd[FD_NUM];
static int socket_num;
pthread_mutex_t send_mutex;
struct tranfer_info vmonitor_info;
pthread_t dump_server_core_pid;
static uint32_t server_running = 0;

/*get yuv channel info*/
uint32_t send_yuv_enable(void)
{
	return vmonitor_info.yuv_enable;
}

uint32_t send_jepg_enable(void)
{
	return vmonitor_info.jepg_enable;
}

uint32_t send_video_enable(void)
{
	return vmonitor_info.video_enable;
}

uint32_t send_raw_enable(void)
{
	return vmonitor_info.raw_enable;
}

uint32_t get_raw_serial_num(void)
{
	return (vmonitor_info.raw_serial_num + 2);
}

uint32_t get_yuv_serial_num(void)
{
	return (vmonitor_info.yuv_serial_num + 2);
}

void get_yuv_pipeline_info(uint32_t *pipe_line, uint32_t *channel)
{
	*pipe_line = (uint32_t)vmonitor_info.pipe_line;
	*channel = (uint32_t)vmonitor_info.channel_id;
}

void get_raw_pipeline_info(uint32_t *pipe_line)
{
	*pipe_line = (uint32_t)vmonitor_info.pipe_line;
}

static void change_video_status(void)
{
	int ret = 0;
	uint32_t pipe_line = (uint32_t)vmonitor_info.pipe_line;
	uint32_t channel = (uint32_t)vmonitor_info.channel_id;
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t encode_mode = (uint32_t)vmonitor_info.video_code;
	uint32_t bit_rate = (uint32_t)vmonitor_info.bit_stream;
	uint32_t intra_period = (uint32_t)vmonitor_info.fream_interval;

	//struct param_buf video_cfg;
	if (vmonitor_info.video_enable == 1) {
		ret = get_yuv_pipe_info(pipe_line, channel, &width, &height);
		if (ret) {
			return;
		}
		set_video_param(width, height, encode_mode, bit_rate, intra_period);

		ret = video_start();
		if (ret < 0) {
			vmon_err("start_video failed.\n");
		}
	} else {
		video_stop();
	}
}

/*mutex*/
static void mutex_init(void)
{
	pthread_mutex_init(&send_mutex, NULL);
}

void acquire_mutex(void)
{
	pthread_mutex_lock(&send_mutex);
}

void release_mutex(void)
{
	pthread_mutex_unlock(&send_mutex);
}

static void info_init(void)
{
	uint32_t tmp = 0;
	uint32_t raw_enable, pipe_line;
	for(tmp = 0; tmp < FD_NUM; tmp++) {
		socket_fd[tmp] = -1;
	}
	socket_num = 0;

	memset(&vmonitor_info, 0, sizeof(struct tranfer_info));
	raw_config_info(&raw_enable, &pipe_line);
	vmonitor_info.pipe_line = pipe_line; /* use defaut config of json file */
	vmonitor_info.raw_enable = 0;
	vmonitor_info.yuv_enable = 1;
	vmonitor_info.video_enable = 0;
	vmonitor_info.jepg_enable = 0;
	vmonitor_info.channel_id = 2;
}

void err_handler(uint32_t err_flag)
{
	if (socket_fd[0] > 0) {
		close(socket_fd[0]);
		socket_fd[0] = -1;
		socket_num--;
		vmon_err("close socket connect.\n");
	}
}

int send_data(void *ptr, uint32_t len)
{
	int ret = 0;

	if(socket_fd[0] > 0) {
		vmon_dbg("send data enable, fd %d \n", socket_fd[0]);
		ret = write_packet(socket_fd[0], ptr, len);
	} else {
		vmon_dbg("fd is error, fd %d \n", socket_fd[0]);
		ret = -1;
	}

	return ret;
}

static int set_nonblocking(int fd)
{
    int flags;
    if ((flags = fcntl(fd, F_GETFL, 0)) == -1)
        flags = 0;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static void *server_thread_client_process(void *param)
{
	int len = 0;
	int fd = *(int *)param;
	struct tranfer_info cmd_info;
	char *cmd_buff = NULL;

	//set tcp as nonblock
	//len = set_nonblocking(fd);

	//socket_fd[socket_num] = fd;
	acquire_mutex();
	socket_fd[0] = fd;
	socket_num++;
	release_mutex();
	vmon_dbg("new net fd %d, socket_num %d.\n", socket_fd[0], socket_num);

	pthread_detach(pthread_self());
	while (1) {
		//receive cmd header
		vmon_dbg("receive loop.\n");
		//usleep(300*1000);
		memset(&cmd_info, 0x00, sizeof(struct tranfer_info));
		len = read_packet(socket_fd[0], (char *)&cmd_info, sizeof(struct tranfer_info));
		if (len >= 0) {
			if (len != sizeof(struct tranfer_info)) {
				vmon_err("read failed, len = %d\n", len);
			} else {
				memcpy(&vmonitor_info, &cmd_info, sizeof(struct tranfer_info));
				vmon_info("read success, len = %d\n", len);
				vmon_dbg("tcp info, rawenable = %d \n", cmd_info.raw_enable);
				vmon_dbg("tcp info, yuvenable = %d \n", cmd_info.yuv_enable);
				vmon_dbg("tcp info, raw_num = %d \n", cmd_info.raw_serial_num);
				vmon_dbg("tcp info, yuv_num = %d \n", cmd_info.yuv_serial_num);
				vmon_dbg("tcp info, pipe_line = %d \n", cmd_info.pipe_line);
				vmon_dbg("tcp info, channel = %d \n", cmd_info.channel_id);
				vmon_dbg("tcp info, video_enable = %d \n", cmd_info.video_enable);
				vmon_dbg("tcp info, jepg_enable = %d \n", cmd_info.jepg_enable);
				vmon_dbg("tcp info, bit_stream = %d \n", cmd_info.bit_stream);
				vmon_dbg("tcp info, fream_interval = %d \n", cmd_info.fream_interval);
				change_video_status();

				vmon_err("-------0x%04x-------%d-------\n",
						vmonitor_info.video_cfg.param_id,
						vmonitor_info.video_cfg.param_data);
				set_dynamic_param(vmonitor_info.video_cfg);

				if (cmd_info.tcp_open == 0) {
					vmon_err("close tcp, status = %d \n", cmd_info.tcp_open);
					break;//close tcp
				}
			}
		} else {
			vmon_err("socket is disconnect. \n");
			break;
		}
	}

	acquire_mutex();
	err_handler(1);
	release_mutex();
	vmon_info("close fd. \n");

	pthread_exit(NULL);
}


static void *dump_server_thread_start(void *para)
{
	pthread_t pid;
	int client_fd = -1, listen_fd = -1;
	struct sockaddr_in server_addr, client_addr;
	socklen_t addr_len = 0;
	int on = 1;

	if ((listen_fd = create_tcp_socket()) < 0) {
		vmon_err("tcp_dump_server_create_socket failed\n");
		exit(1);
	}
	setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	bzero(&server_addr, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	if (strcmp("INADDR_ANY", dump_config_get_server_ip()) == 0) {
		server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	} else {
		struct in_addr addr;
		inet_aton(dump_config_get_server_ip(), &addr);
		server_addr.sin_addr.s_addr = addr.s_addr;
	}
	server_addr.sin_port = htons(dump_config_get_server_port());
	if (bind(listen_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
		vmon_err("bind error\n");
		goto err;
	}

	if (listen(listen_fd, FD_NUM) < 0) {
		vmon_err("listen error\n");
		goto err;
	}

	pthread_detach(pthread_self());

	socket_num = 0;
	while(server_running) {
		vmon_info("wait new socket connect.\n");
		addr_len = sizeof(client_addr);
		if((client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &addr_len)) < 0) {
			vmon_err("accept error\n");
			break;
		} else {
			vmon_dbg("ip = %s, port = %d, fd = %d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), client_fd);
			if (pthread_create(&pid, NULL, server_thread_client_process, &client_fd) != 0) {
				vmon_err("server_thread_client_process start failed \n");
				close(client_fd);
			}
		}
		//usleep(300*1000);
	}

err:
	vmon_dbg("dump_server_thread_start thread end\n");
	if (listen_fd > 0)
		close(listen_fd);
	pthread_exit(NULL);
}

int dump_server_core_start_services(void)
{
	info_init();
	mutex_init();

	signal(SIGPIPE, SIG_IGN);

	server_running = 1;
	if (pthread_create(&dump_server_core_pid, NULL, dump_server_thread_start, NULL) != 0) {
		vmon_err("start_dump_server_thread start failed. \n");
		server_running = 0;
		return -1;
	}

	return 0;
}

void dump_server_core_stop_services(void)
{
	server_running = 0;
	if (dump_server_core_pid) {
		pthread_cancel(dump_server_core_pid);
		pthread_join(dump_server_core_pid, NULL);
		dump_server_core_pid = 0;
	}
	return;
}


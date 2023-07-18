/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <sys/ioctl.h>

#include <netdb.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <dlfcn.h>
#include <linux/ioctl.h>

#include <hbipc_cp.h>
#include <hbipc_errno.h>


#define	VC_DEBUG	(0)
#define	vc_log(format, ...)	printf("[vc_daemon]: line %d ! "format "\n" , __LINE__, ##__VA_ARGS__)
#define	vc_err(format, ...)	printf("[vc_daemon]: line %d ! "format "\n" , __LINE__, ##__VA_ARGS__)
#if VC_DEBUG
#define	vc_dbg(format, ...)	printf("[vc_daemon]: line %d "format "\n" , __LINE__, ##__VA_ARGS__)
#else
#define	vc_dbg(format, ...)
#endif

#define LISTEN_BACKLOG	(50)
#define VCAM_IOC_MAGIC  'v'

#define HB_VCAM_INIT  _IO(VCAM_IOC_MAGIC, 0)
#define HB_VCAM_DEINIT  _IO(VCAM_IOC_MAGIC, 1)
#define HB_VCAM_START  _IO(VCAM_IOC_MAGIC, 2)
#define HB_VCAM_STOP  _IO(VCAM_IOC_MAGIC, 3)
#define HB_VCAM_NEXT_GROUP _IO(VCAM_IOC_MAGIC, 4)
#define HB_VCAM_FRAME_DONE _IO(VCAM_IOC_MAGIC, 5)

#define BUF_LEN (128)
#define VCAM_FN "/dev/vcam"
#define SOCKET_SUN_PATH "/userdata/server_socket"
UUID_DEFINE(server_id, 0x00, 0x10, 0x20, 0x30, 0x40,
	0x50, 0x60, 0x70, 0x8f, 0x9f, 0xaf, 0xbf, 0xcf, 0xdf, 0xef, 0xff);



struct domain_configuration domain_config = {
	2,
	{
		{"X2BIF001", 0, "/dev/x2_bif"},
		{"X2SD001", 1, "/dev/x2_sd"},
		{},
	},
};


enum vcam_cmd {
	CMD_VCAM_INIT = 1,
	CMD_VCAM_START,
	CMD_VCAM_NEXT_REQUST,
	CMD_VCAM_STOP,
	CMD_VCAM_DEINIT,
	CMD_VCAM_MAX
};


struct Setting_config{
	int fps;
	int size;
};

/* for vcam image info */
struct vcam_img_info_t {
	int width;
	int height;
	int stride;
	int format;
};

/* for vcam slot info */
struct vcam_slot_info_t {
	int slot_id;
	int cam_id;
	int	frame_id;
	int64_t timestamp;
	struct vcam_img_info_t img_info;
};

/*
 * for vcam group info
 * group_size = slot_size * slot_num;
 * every_group_addr = base + g_id * group_size;
*/
struct vcam_group_info_t {
	int   g_id;   // group id
	uint64_t base;   // first group paddr
	int   slot_size;  // a slot size
	int   slot_num;  // a group have slot_num slot
	int   flag;   // 0 free 1 busy
};

/* for vcam msg info */
struct hb_vcam_msg_t {
	int info_type;
	struct vcam_group_info_t group_info;
	struct vcam_slot_info_t  slot_info;
};



typedef struct vcam_dres_s {
	int domain_id;
	int vcam_fd;
	struct session v_connect;
	pthread_t pid_s;
	int nl_socket;
	pthread_t pid_skt_read;
}vcam_dres_t;

static vcam_dres_t d_res;

int vcamera_deinit()
{
	pthread_cancel(d_res.pid_s);
	pthread_join(d_res.pid_s, NULL);
	hbipc_cp_deinit(d_res.domain_id, server_id);
	return 0;
}

int vcamera_send2ap(struct hb_vcam_msg_t* p_vccam)
{
	int ret = -1;
	int len = sizeof(*p_vccam);
	if ((ret = hbipc_cp_sendframe(&d_res.v_connect, (char *)p_vccam, len)) < 0) {
			vc_err("hbipc_cp_sendframe error: %d\n", ret);
			return -1;
		}

	return 0;
}

void *vc_bifsd_thread(void *arg)
{
	int ret = 0;

	struct hb_vcam_msg_t vcam_rcv;

	while(1) {
		ret = hbipc_cp_recvframe(&d_res.v_connect, (char *)&vcam_rcv, sizeof(vcam_rcv));
		if (ret < 0) {
			vc_err("hbipc_cp_recvframe error: %d\n", ret);
			break;
		}
		vc_dbg("hbipc_cp_recvframe framedone info ok,slot id is %d\n",
			vcam_rcv.slot_info.slot_id);
	    if (ioctl(d_res.vcam_fd, HB_VCAM_FRAME_DONE, &vcam_rcv)) {
			vc_err("ioctl  HB_VCAM_FRAME_DONE error\n");
			break;
	    }
	}
	return NULL;
}

int vcamera_init(struct hb_vcam_msg_t* p_vccam)
{
	int ret = -1;
	int len = 0;
	if ((ret = hbipc_cp_init("X2BIF001", server_id, TF_BLOCK,
		0, &domain_config)) < 0) {
		vc_err("hbipc_cp_init error: %d\n", ret);
		goto init_error;
	}
	d_res.domain_id = ret;

	if ((ret = hbipc_cp_accept(d_res.domain_id, server_id,
		&d_res.v_connect, TF_BLOCK)) < 0) {
		vc_err("hbipc_cp_accept error: %d\n", ret);
		goto accept_error;
	}
	len = sizeof(*p_vccam);

	ret = pthread_create(&d_res.pid_s, NULL, (void *)vc_bifsd_thread, NULL);
	if (ret) {
	   vc_err("vc_bifsd_thread	create failed\n");
	   goto accept_error;
	}

	if ((ret = hbipc_cp_sendframe(&d_res.v_connect, (void *)p_vccam, len)) < 0) {
			vc_err("hbipc_cp_sendframe error: %d\n", ret);
			goto accept_error;
		}
	d_res.vcam_fd = open(VCAM_FN, O_RDWR | O_CLOEXEC);
	 if (d_res.vcam_fd < 0) {
		 vc_err("open vcam node fail,vcam_fd is %d\n", d_res.vcam_fd);
		 ret = -1;
      	 goto accept_error;
    }

	 vc_log("vcamera_init ok\n");

	return 0;

accept_error:
	hbipc_cp_deinit(d_res.domain_id , server_id);
init_error:
	return ret;
}

void *vc_skt_rd_thread(void *arg)
{
	int* p_client_sockfd = (int*)arg;
	int recvbytes;
	int ret = 0;
	struct hb_vcam_msg_t *p_vcaminfo = NULL;
	p_vcaminfo = malloc(sizeof(*p_vcaminfo));
	if(p_vcaminfo == NULL) {
		vc_err("p_vcaminfo malloc error\n");
		return NULL;
	}
	while (1) {
		recvbytes = read(*p_client_sockfd, p_vcaminfo, sizeof(*p_vcaminfo));
		if(recvbytes < 0) {
			vc_err("socket recv error\n");
			break;
		} else if (recvbytes == 0) {
			vc_err("client exception exit\n");
			p_vcaminfo->info_type = CMD_VCAM_DEINIT;
			ret += vcamera_send2ap(p_vcaminfo);
			sleep(1);
			ret += vcamera_deinit();
			break;
		} else {
			vc_dbg("p_vcaminfo->info_type is %d\n", p_vcaminfo->info_type);
			vc_dbg("p_vcaminfo->group_info.base is 0x%x\n", p_vcaminfo->group_info.base);
			vc_dbg("p_vcaminfo->group_info.g_id %d\n", p_vcaminfo->group_info.g_id);
			vc_dbg("p_vcaminfo->group_info.slot_num %d\n", p_vcaminfo->group_info.slot_num);
			vc_dbg("p_vcaminfo->group_info.slot_size %d\n", p_vcaminfo->group_info.slot_size);

			switch(p_vcaminfo->info_type) {
				case CMD_VCAM_INIT:
					ret += vcamera_init(p_vcaminfo);
					break;
				case CMD_VCAM_START:
				case CMD_VCAM_NEXT_REQUST:
				case CMD_VCAM_STOP:
					ret += vcamera_send2ap(p_vcaminfo);
					break;

				case CMD_VCAM_DEINIT:
					ret += vcamera_send2ap(p_vcaminfo);
					sleep(1);  // let ap hbipc deinit resource
					ret += vcamera_deinit();
					ret = -1;
					break;

				default:
					vc_err("recv camera cmd NOT invalid\n");
					break;
			}
			if(ret)
				break;
			memset(p_vcaminfo, 0, sizeof(*p_vcaminfo));
		}
	}

	pthread_exit(NULL);
}

int main(int argc, char *argv[])
{
	int server_sockfd;
	int client_sockfd;
	struct sockaddr_un server_addr;
	int ret = 0;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	if ((server_sockfd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
		vc_err("socket create error\n");
		return -1;
	}
	unlink(SOCKET_SUN_PATH);

	bzero(&server_addr, sizeof(server_addr));
	server_addr.sun_family = AF_UNIX;
	strncpy(server_addr.sun_path, SOCKET_SUN_PATH, sizeof(server_addr.sun_path)-1);


	if (bind(server_sockfd, (struct sockaddr *)&server_addr,
		SUN_LEN(&server_addr)) < 0) {
		vc_err("socket bind error\n");
		goto out;
	}

	if (listen(server_sockfd, LISTEN_BACKLOG) < 0) {
		vc_err("socket listen error\n");
		goto out;
	}


	 while (1) {
		 if ((client_sockfd = accept(server_sockfd, NULL, NULL)) < 0) {
			 vc_err("socket accept error\n");
			 goto out1;
		 }
		 vc_log("socket ACCEPT a new client SUCCESS!\n");

		 ret = pthread_create(&d_res.pid_skt_read, &attr,
			(void *)vc_skt_rd_thread, &client_sockfd);
		if (ret) {
	   		vc_err("vc_skt_rd_thread create failed\n");
	   		continue;
		}
	}
out1:
	close(client_sockfd);
out:
	close(server_sockfd);
	return -1;
}

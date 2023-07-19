#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <stdint.h>
#include <pthread.h>
#ifdef __cplusplus
extern "C"{
#endif

//ִ�п���ָ̨��
int exec_cmd(const char *cmd);	
int exec_cmd_ex(const char *cmd, char* res, int max);
//ִ�п���ָ̨��, �ж�ִ�н���Ƿ����str�ַ���
int exec_cmd_chstr_exist(char* cmd, char* str);
//��ȡϵͳʣ���ڴ�
unsigned long get_system_mem_freeKb();		
//��ȡĿ¼ʣ��洢�ռ�
unsigned long long get_system_tf_freeKb(char* dir);
//��ȡϵͳ������ʱ�䵽���ڵ�tick��
int32_t get_tick_count();
//��ȫ·���ļ����л�ȡ�ļ���
void  get_file_pure_name(char* full_name,char * dest);
//ʹ��select��ʱ
void select_delay_ms(int nMillisecond);
//�ж��ļ�/�ļ����Ƿ����
int is_file_exist(const char* file_path);
int is_dir_exist(const char* dir_path);

int int2hex2str(char *pValue,int lValue,int lCharLen);
int int2str(char *pValue,int lValue,int lCharLen);
//�ַ�����ת
char* strrev(char* s);
//��Χ�����
int random_range(int min, int max);

void localtime_string(char* tstr);

//����4Mջ��С���߳�
int pthread_create_4m(pthread_t *pt_id, void *(*proc)(void *), void* arg);
int get_addr_ip(char* demain, char* ip, int socktype);
int str_splite(char* str, char* split, char* des, int rows, int row_size);

#ifdef __cplusplus
}
#endif

#endif
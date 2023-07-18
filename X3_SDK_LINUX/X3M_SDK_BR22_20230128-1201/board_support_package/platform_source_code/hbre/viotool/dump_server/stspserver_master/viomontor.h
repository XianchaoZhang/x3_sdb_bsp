#ifndef VIO_MONTOR_H
#define VIO_MONTOR_H

int vio_montor_start(const char *pathname, uint32_t gdc_rotation, uint32_t video, uint32_t stats);
int vio_montor_stop(void);

#ifdef __cplusplus
extern "C"
{
#endif
int get_video_data(uint8_t* frame);
#ifdef __cplusplus
}
#endif

#endif // COMMON_H

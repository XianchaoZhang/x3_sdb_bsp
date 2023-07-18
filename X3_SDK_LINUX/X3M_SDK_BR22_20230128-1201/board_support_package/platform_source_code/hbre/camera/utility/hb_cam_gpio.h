#ifndef __HB_CAM_GPIO_H__
#define __HB_CAM_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

int gpio_export(unsigned int gpio);
int gpio_unexport(unsigned int gpio);
int gpio_set_dir(unsigned int gpio, unsigned int out_flag);
int gpio_set_value(unsigned int gpio, unsigned int value);
int gpio_get_value(unsigned int gpio, unsigned int *value);
int gpio_set_edge(unsigned int gpio, char *edge);

#ifdef __cplusplus
}
#endif

#endif
/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_HB_SPI_H_
#define UTILITY_HB_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

extern int hb_spi_write_block(int fd, char *buf, int count);
extern int hb_spi_read_block(int fd, char *buf, int count);

#ifdef __cplusplus
}
#endif

#endif  // UTILITY_HB_SPI_H_

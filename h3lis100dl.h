 /*
  * STMicroelectronics h3lis100dl Accelerometer driver
  * Based on lis331dlh input driver
  *
  * Copyright 2016 STMicroelectronics Inc.
  *
  * Armando Visconti <armando.visconti@st.com>
  *
  * Licensed under the GPL-2.
  */

#ifndef __H3LIS100DL_H__
#define __H3LIS100DL_H__

#define INPUT_EVENT_TYPE		EV_MSC
#define INPUT_EVENT_X			MSC_SERIAL
#define INPUT_EVENT_Y			MSC_PULSELED
#define INPUT_EVENT_Z			MSC_GESTURE
#define INPUT_EVENT_TIME_MSB		MSC_SCAN
#define INPUT_EVENT_TIME_LSB		MSC_MAX

#define SAD0L				0x00
#define SAD0H				0x01
#define H3LIS100DL_I2C_SADROOT	0x0C
#define H3LIS100DL_I2C_SAD_L	((H3LIS100DL_I2C_SADROOT<<1)|SAD0L)
#define H3LIS100DL_I2C_SAD_H	((H3LIS100DL_I2C_SADROOT<<1)|SAD0H)
#define	H3LIS100DL_DEV_NAME	"h3lis100dl"

/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define	H3LIS100DL_FS_MASK		0x30
#define H3LIS100DL_G_100G 		0x00

/* Accelerometer Sensor Operating Mode */
#define H3LIS100DL_ENABLE		0x01
#define H3LIS100DL_DISABLE		0x00
#define H3LIS100DL_PM_NORMAL		0x20
#define H3LIS100DL_PM_OFF		H3LIS100DL_DISABLE

#ifdef __KERNEL__
struct h3lis100dl_platform_data {

	int poll_interval;
	int min_interval;

	u8 g_range;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif /* __KERNEL__ */
#endif  /* __H3LIS100DL_H__ */

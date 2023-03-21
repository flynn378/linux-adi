// SPDX-License-Identifier: GPL-2.0
/*
 * adis165x IMU driver
 *
 * Copyright 2019 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/imu/adis.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/irq.h>
#include <linux/lcm.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>

#define ADIS165X_BURST32_MAX_DATA	32
#define ADIS165X_BURST_MAX_DATA		20
#define ADIS165X_MAX_SCAN_DATA		28

#define ADIS165X_CHECKSUM_SIZE		2  /* in bytes */
#define ADIS165X_16_BIT_BURST_SIZE	0
#define ADIS165X_32_BIT_BURST_SIZE	1
#define ADIS165X_MSG_SIZE_16_BIT_BURST 	20 /* in bytes */
#define ADIS165X_MSG_SIZE_32_BIT_BURST 	32 /* in bytes */

#define ADIS165X_BURST_MAX_SPEED	1000000

enum {
	ADIS165X_SYNC_DEFAULT = 0,
	ADIS165X_SYNC_DIRECT,
	ADIS165X_SYNC_SCALED,
	ADIS165X_SYNC_OUTPUT,
	ADIS165X_SYNC_PULSE = 5,
};

/**
 * @brief Bitfield struct which maps on the diagnosis register
 */
struct _adis165x_diag_flags {
	u8 DATA_PATH_OVERRUN	:1;
	u8 SPI_COMM_ERR		:1;
	u8 CHECKSUM_ERR		:1;
	u8 STANDBY_MODE		:1;
	u8 CLOCK_ERR		:1;
	u8 FLS_MEM_UPDATE_ERR	:1;
	u8 FLS_MEM_TEST_ERR	:1;
	u8 FLS_MEM_WR_CNT_EXCEED:1;
	u8 SNSR_SELF_TEST_ERR	:1;
	u8 GYRO1_SELF_TEST_ERR	:1;
	u8 GYRO2_SELF_TEST_ERR	:1;
	u8 ACCL_SELF_TEST_ERR	:1;
};

/**
 * @brief Union for device diagnosis flags
 */
union adis165x_diag_flags {
	struct _adis165x_diag_flags adis165x_diag_flags_bits;
	u16 			value;
};

struct adis165x_sync_rate_limit {
	u16 min_rate;
	u16 max_rate;
};

struct adis165x_chip_info {
	const struct adis165x_sync_rate_limit *sync_rate_limit;
	const struct adis165x_data_def *reg_map;

	const struct iio_chan_spec *channels;
	const char *name;
	const struct adis_data adis165x_data;
	u32 num_channels;
	u32 gyro_max_val;
	u32 gyro_max_scale;
	u32 accel_max_val;
	u32 accel_max_scale;
	u32 temp_scale;
	u32 rot_max_val;
	u32 rot_max_scale_log2;
	u32 vel_max_val;
	u32 vel_max_scale_log2;

	u32 int_clk;
	u16 dec_rate_max;
	u16 filt_ctrl_max;
	u16 filter_update_us;
	u16 msc_reg_update_us;
	u16 sens_bw_update_ms;
	u8  burst_size_max;
	u8  burst_sel_max;
	u8  sens_bw_max;
	u8  sync_mode_max;
	u16 flshcnt_max;
	u8  pt_of_perc_algnmt_max;
	u8  linear_accl_comp_max;
};

/**
 * @brief ADIS165X device register structure
 */
struct adis165x_reg {
	u8 	addr;
	u8 	size;
	u32 	mask;
};

/**
 * @brief ADIS165X memory map data definition structure
 */
struct adis165x_data_def {
	/* Measured data */
	struct adis165x_reg x_gyro;
	struct adis165x_reg y_gyro;
	struct adis165x_reg z_gyro;
	struct adis165x_reg x_accl;
	struct adis165x_reg y_accl;
	struct adis165x_reg z_accl;
	struct adis165x_reg temp_out;

	/* Timing data */
	struct adis165x_reg time_stamp;
	struct adis165x_reg data_cntr;

	/* Delta measured data */
	struct adis165x_reg x_deltang;
	struct adis165x_reg y_deltang;
	struct adis165x_reg z_deltang;
	struct adis165x_reg x_deltvel;
	struct adis165x_reg y_deltvel;
	struct adis165x_reg z_deltvel;

	/* Calibration data */
	struct adis165x_reg xg_bias;
	struct adis165x_reg yg_bias;
	struct adis165x_reg zg_bias;
	struct adis165x_reg xa_bias;
	struct adis165x_reg ya_bias;
	struct adis165x_reg za_bias;

	/* Filter data */
	struct adis165x_reg filt_ctrl;
	struct adis165x_reg up_scale;
	struct adis165x_reg dec_rate;

	/* Identification data */
	struct adis165x_reg rang_mdl;
	struct adis165x_reg firm_rev;
	struct adis165x_reg firm_dm;
	struct adis165x_reg firm_y;
	struct adis165x_reg prod_id;
	struct adis165x_reg serial_num;
	struct adis165x_reg usr_scr_1;
	struct adis165x_reg usr_scr_2;
	struct adis165x_reg usr_scr_3;
	struct adis165x_reg flshcnt;

	/* Configuration data */
	struct adis165x_reg msc_ctrl;
	struct adis165x_reg dr_polarity;
	struct adis165x_reg sync_polarity;
	struct adis165x_reg sync_mode;
	struct adis165x_reg sens_bw;
	struct adis165x_reg pt_of_perc_algnmt;
	struct adis165x_reg linear_accl_comp;
	struct adis165x_reg burst_sel;
	struct adis165x_reg burst_size;

	/* Commands */
	struct adis165x_reg glob_cmd;
	struct adis165x_reg sw_res;
	struct adis165x_reg fls_mem_test;
	struct adis165x_reg fls_mem_update;
	struct adis165x_reg snsr_self_test;
	struct adis165x_reg fact_calib_restore;

	/* Diagnosis data */
	struct adis165x_reg diag_stat;

	/* Diagnosis flags masks */
	u16 data_path_overrun_mask;
	u16 fls_mem_update_mask;
	u16 spi_comm_err_mask;
	u16 standby_mode_mask;
	u16 snsr_self_test_mask;
	u16 fls_mem_test_mask;
	u16 clock_err_mask;
	u16 gyro1_self_test_mask;
	u16 gyro2_self_test_mask;
	u16 accl_self_test_mask;
};

struct adis165x {
	const struct adis165x_chip_info *info;
	struct adis adis;
	u16 sync_mode;
	__be16 data[ADIS165X_MAX_SCAN_DATA];
	union adis165x_diag_flags 	diag_flags;

	u8 burst_size;
	u8  burst_sel;
	u32 clk_freq;
	u32 ext_clk;
	u16 data_cntr;
	u32 samples_lost;
	u32 sampling_frequency;
};

enum {
	ADIS165X_GYRO_X,
	ADIS165X_GYRO_Y,
	ADIS165X_GYRO_Z,
	ADIS165X_ACCEL_X,
	ADIS165X_ACCEL_Y,
	ADIS165X_ACCEL_Z,
	ADIS165X_TEMP,
	ADIS165X_DELTA_ANGL_X,
	ADIS165X_DELTA_ANGL_Y,
	ADIS165X_DELTA_ANGL_Z,
	ADIS165X_DELTA_VEL_X,
	ADIS165X_DELTA_VEL_Y,
	ADIS165X_DELTA_VEL_Z,
	ADIS165X_DATA_COUNTER,
};

static const struct adis165x_data_def adis165x_def = {
	.x_gyro 		= {.addr = 0x04, .size = 0x04, .mask = 0xFFFFFFFF},
	.y_gyro 		= {.addr = 0x08, .size = 0x04, .mask = 0xFFFFFFFF},
	.z_gyro 		= {.addr = 0x0C, .size = 0x04, .mask = 0xFFFFFFFF},
	.x_accl 		= {.addr = 0x10, .size = 0x04, .mask = 0xFFFFFFFF},
	.y_accl 		= {.addr = 0x14, .size = 0x04, .mask = 0xFFFFFFFF},
	.z_accl 		= {.addr = 0x18, .size = 0x04, .mask = 0xFFFFFFFF},
	.temp_out 		= {.addr = 0x1C, .size = 0x02, .mask = 0x0000FFFF},
	.time_stamp 		= {.addr = 0x1E, .size = 0x02, .mask = 0x0000FFFF},
	.data_cntr 		= {.addr = 0x22, .size = 0x02, .mask = 0x0000FFFF},
	.x_deltang 		= {.addr = 0x24, .size = 0x04, .mask = 0xFFFFFFFF},
	.y_deltang 		= {.addr = 0x28, .size = 0x04, .mask = 0xFFFFFFFF},
	.z_deltang 		= {.addr = 0x2C, .size = 0x04, .mask = 0xFFFFFFFF},
	.x_deltvel 		= {.addr = 0x30, .size = 0x04, .mask = 0xFFFFFFFF},
	.y_deltvel 		= {.addr = 0x34, .size = 0x04, .mask = 0xFFFFFFFF},
	.z_deltvel 		= {.addr = 0x38, .size = 0x04, .mask = 0xFFFFFFFF},
	.xg_bias 		= {.addr = 0x40, .size = 0x04, .mask = 0xFFFFFFFF},
	.yg_bias 		= {.addr = 0x44, .size = 0x04, .mask = 0xFFFFFFFF},
	.zg_bias 		= {.addr = 0x48, .size = 0x04, .mask = 0xFFFFFFFF},
	.xa_bias 		= {.addr = 0x4C, .size = 0x04, .mask = 0xFFFFFFFF},
	.ya_bias 		= {.addr = 0x50, .size = 0x04, .mask = 0xFFFFFFFF},
	.za_bias 		= {.addr = 0x54, .size = 0x04, .mask = 0xFFFFFFFF},
	.filt_ctrl 		= {.addr = 0x5C, .size = 0x02, .mask = 0x00000007},
	.up_scale 		= {.addr = 0x62, .size = 0x02, .mask = 0x0000FFFF},
	.dec_rate 		= {.addr = 0x64, .size = 0x02, .mask = 0x000007FF},
	.rang_mdl 		= {.addr = 0x5E, .size = 0x02, .mask = 0x0000000C},
	.firm_rev 		= {.addr = 0x6C, .size = 0x02, .mask = 0x0000FFFF},
	.firm_dm 		= {.addr = 0x6E, .size = 0x02, .mask = 0x0000FFFF},
	.firm_y 		= {.addr = 0x70, .size = 0x02, .mask = 0x0000FFFF},
	.prod_id 		= {.addr = 0x72, .size = 0x02, .mask = 0x0000FFFF},
	.serial_num 		= {.addr = 0x74, .size = 0x02, .mask = 0x0000FFFF},
	.usr_scr_1 		= {.addr = 0x76, .size = 0x02, .mask = 0x0000FFFF},
	.usr_scr_2 		= {.addr = 0x78, .size = 0x02, .mask = 0x0000FFFF},
	.usr_scr_3 		= {.addr = 0x7A, .size = 0x02, .mask = 0x0000FFFF},
	.flshcnt		= {.addr = 0x7C, .size = 0x04, .mask = 0xFFFFFFFF},
	.msc_ctrl		= {.addr = 0x60, .size = 0x02, .mask = 0x0000FFFF},
	.dr_polarity 		= {.addr = 0x60, .size = 0x02, .mask = BIT(0)},
	.sync_polarity 		= {.addr = 0x60, .size = 0x02, .mask = BIT(1)},
	.sync_mode 		= {.addr = 0x60, .size = 0x02, .mask = 0x0000000C},
	.sens_bw		= {.addr = 0x60, .size = 0x02, .mask = BIT(4)},
	.pt_of_perc_algnmt 	= {.addr = 0x60, .size = 0x02, .mask = BIT(6)},
	.linear_accl_comp 	= {.addr = 0x60, .size = 0x02, .mask = BIT(7)},
	.burst_sel 		= {.addr = 0x60, .size = 0x02, .mask = BIT(8)},
	.burst_size 		= {.addr = 0x60, .size = 0x02, .mask = BIT(9)},
	.glob_cmd		= {.addr = 0x68, .size = 0x02, .mask = 0x0000FFFF},
	.sw_res 		= {.addr = 0x68, .size = 0x02, .mask = BIT(7)},
	.fls_mem_test 		= {.addr = 0x68, .size = 0x02, .mask = BIT(4)},
	.fls_mem_update 	= {.addr = 0x68, .size = 0x02, .mask = BIT(3)},
	.snsr_self_test 	= {.addr = 0x68, .size = 0x02, .mask = BIT(2)},
	.fact_calib_restore 	= {.addr = 0x68, .size = 0x02, .mask = BIT(1)},
	.diag_stat 		= {.addr = 0x02, .size = 0x02, .mask = 0x000007FE},
	.data_path_overrun_mask = BIT(1),
	.fls_mem_update_mask 	= BIT(2),
	.spi_comm_err_mask 	= BIT(3),
	.standby_mode_mask 	= BIT(4),
	.snsr_self_test_mask 	= BIT(5),
	.fls_mem_test_mask	= BIT(6),
	.clock_err_mask 	= BIT(7),
	.gyro1_self_test_mask 	= BIT(8),
	.gyro2_self_test_mask 	= BIT(9),
	.accl_self_test_mask 	= BIT(10),
};

#define ADIS165X_MOD_CHAN(_type, _mod, _address, _si, _r_bits, _s_bits) \
	{ \
		.type = (_type), \
		.modified = 1, \
		.channel2 = (_mod), \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_CALIBBIAS), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
		.address = (_address), \
		.scan_index = (_si), \
		.scan_type = { \
			.sign = 's', \
			.realbits = (_r_bits), \
			.storagebits = (_s_bits), \
			.endianness = IIO_BE, \
		}, \
	}

#define ADIS165X_MOD_CHAN_DELTA(_type, _mod, _address, _si, _r_bits, _s_bits) \
	{ \
		.type = (_type), \
		.modified = 1, \
		.channel2 = (_mod), \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
		.address = (_address), \
		.scan_index = (_si), \
		.scan_type = { \
			.sign = 's', \
			.realbits = (_r_bits), \
			.storagebits = (_s_bits), \
			.endianness = IIO_BE, \
		}, \
	}

#define ADIS165X_GYRO_CHAN(regdef, addr, _mod) \
	ADIS165X_MOD_CHAN(IIO_ANGL_VEL, IIO_MOD_ ## _mod, addr, \
			   ADIS165X_GYRO_ ## _mod, 32, 32)

#define ADIS165X_ACCEL_CHAN(regdef, addr, _mod) \
	ADIS165X_MOD_CHAN(IIO_ACCEL, IIO_MOD_ ## _mod, addr, \
			   ADIS165X_ACCEL_ ## _mod, 32, 32)

#define ADIS165X_DELTA_ANGL_CHAN(regdef, addr, _mod) \
	ADIS165X_MOD_CHAN_DELTA(IIO_ROT, IIO_MOD_ ## _mod, addr, \
			   ADIS165X_DELTA_ANGL_ ## _mod, 32, 32)

#define ADIS165X_DELTA_VEL_CHAN(regdef, addr, _mod) \
	ADIS165X_MOD_CHAN_DELTA(IIO_VELOCITY, IIO_MOD_ ## _mod, addr, \
			   ADIS165X_DELTA_VEL_ ## _mod, 32, 32)

#define ADIS165X_TEMP_CHAN(regdef) { \
		.type = IIO_TEMP, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
		.address = regdef.temp_out.addr, \
		.scan_index = ADIS165X_TEMP, \
		.scan_type = { \
			.sign = 's', \
			.realbits = 32, \
			.storagebits = 32, \
			.endianness = IIO_BE, \
		}, \
	}

#define ADIS165X_DATA_COUNTER_CHAN(_si) {					\
	.type = IIO_COUNT,						\
	.scan_index = _si,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.endianness = IIO_BE, 					\
		},							\
}

static const struct iio_chan_spec adis165x_channels[] = {
	ADIS165X_GYRO_CHAN(adis165x_def, adis165x_def.x_gyro.addr, X),
	ADIS165X_GYRO_CHAN(adis165x_def, adis165x_def.y_gyro.addr, Y),
	ADIS165X_GYRO_CHAN(adis165x_def, adis165x_def.z_gyro.addr, Z),
	ADIS165X_ACCEL_CHAN(adis165x_def, adis165x_def.x_accl.addr, X),
	ADIS165X_ACCEL_CHAN(adis165x_def, adis165x_def.y_accl.addr, Y),
	ADIS165X_ACCEL_CHAN(adis165x_def, adis165x_def.z_accl.addr, Z),
	ADIS165X_TEMP_CHAN(adis165x_def),
	ADIS165X_DELTA_ANGL_CHAN(adis165x_def, adis165x_def.x_deltang.addr, X),
	ADIS165X_DELTA_ANGL_CHAN(adis165x_def, adis165x_def.y_deltang.addr, Y),
	ADIS165X_DELTA_ANGL_CHAN(adis165x_def, adis165x_def.z_deltang.addr, Z),
	ADIS165X_DELTA_VEL_CHAN(adis165x_def, adis165x_def.x_deltvel.addr, X),
	ADIS165X_DELTA_VEL_CHAN(adis165x_def, adis165x_def.y_deltvel.addr, Y),
	ADIS165X_DELTA_VEL_CHAN(adis165x_def, adis165x_def.z_deltvel.addr, Z),
	ADIS165X_DATA_COUNTER_CHAN(ADIS165X_DATA_COUNTER)
};

enum adis165x_variant {
	ADIS165X_05_1,
	ADIS165X_05_2,
	ADIS165X_05_3,
};

#define ADIS165X_DATA(regdef, _prod_id, _timeouts)			\
{									\
	.msc_ctrl_reg = regdef.msc_ctrl.addr,				\
	.glob_cmd_reg = regdef.glob_cmd.addr,				\
	.diag_stat_reg = regdef.diag_stat.addr,				\
	.prod_id_reg = regdef.prod_id.addr,				\
	.prod_id = (_prod_id),						\
	.self_test_mask = BIT(2),					\
	.self_test_reg = regdef.glob_cmd.addr,				\
	.cs_change_delay = 16,						\
	.read_delay = 5,						\
	.write_delay = 5,						\
	.unmasked_drdy = true,						\
	.timeouts = (_timeouts),					\
	.burst_reg_cmd = regdef.glob_cmd.addr,				\
	.burst_len = ADIS165X_BURST_MAX_DATA,				\
	.burst_max_len = ADIS165X_BURST32_MAX_DATA,			\
	.burst_max_speed_hz = ADIS165X_BURST_MAX_SPEED			\
}

static const struct adis165x_sync_rate_limit adis165x_sync_mode[] = {
	[ADIS165X_SYNC_DEFAULT] = { },
	[ADIS165X_SYNC_DIRECT] = { 1900, 2100 },
	[ADIS165X_SYNC_SCALED] = { 1, 128 },
	[ADIS165X_SYNC_OUTPUT] = { },
};

static const struct adis_timeout adis165x_timeouts = {
	.reset_ms = 200,
	.sw_reset_ms = 200,
	.self_test_ms = 20,
};

static const struct adis_timeout adis1650x_timeouts = {
	.reset_ms = 260,
	.sw_reset_ms = 260,
	.self_test_ms = 30,
};

static const struct adis165x_chip_info adis165x_chip_info[] = {
	[ADIS165X_05_1] = {
		.name = "adis16505",
		.num_channels = ARRAY_SIZE(adis165x_channels),
		.channels = adis165x_channels,
		.gyro_max_val = 1,
		.gyro_max_scale = IIO_RAD_TO_DEGREE(160 << 16),
		.accel_max_val = 78,
		.accel_max_scale = 32000 << 16,
					 .temp_scale = 100,
					 .int_clk = 2000,
					 .sync_rate_limit = adis165x_sync_mode,
					 .adis165x_data = ADIS165X_DATA(adis165x_def, 16505, &adis1650x_timeouts),
					 .reg_map = &adis165x_def,
					 .burst_sel_max = 1,
					 .burst_size_max = 1,
					 .sens_bw_max = 1,
					 .pt_of_perc_algnmt_max = 1,
					 .linear_accl_comp_max = 1,
					 .sync_mode_max = 3,
					 .flshcnt_max = 10000,
					 .filt_ctrl_max = 6,
					 .dec_rate_max = 1999,
					 .filter_update_us = 30,
					 .msc_reg_update_us = 200,
					 .sens_bw_update_ms = 250,
	},
	[ADIS165X_05_2] = {
		.name = "adis16505",
		.num_channels = ARRAY_SIZE(adis165x_channels),
		.channels = adis165x_channels,
		.gyro_max_val = 1,
		.gyro_max_scale = IIO_RAD_TO_DEGREE(40 << 16),
		.accel_max_val = 78,
		.accel_max_scale = 32000 << 16,
					 .temp_scale = 100,
					 .rot_max_val = 720u,
					 .rot_max_scale_log2 = 31u,
					 .vel_max_val = 100u,
					 .vel_max_scale_log2 = 31u,
					 .int_clk = 2000,
					 .sync_rate_limit = adis165x_sync_mode,
					 .adis165x_data = ADIS165X_DATA(adis165x_def, 16505, &adis1650x_timeouts),
					 .reg_map = &adis165x_def,
					 .burst_sel_max = 1,
					 .burst_size_max = 1,
					 .pt_of_perc_algnmt_max = 1,
					 .linear_accl_comp_max = 1,
					 .sync_mode_max = 3,
					 .flshcnt_max = 10000,
					 .filt_ctrl_max = 6,
					 .dec_rate_max = 1999,
					 .filter_update_us = 30,
					 .msc_reg_update_us = 200,
					 .sens_bw_update_ms = 250,
	},
	[ADIS165X_05_3] = {
		.name = "adis16505",
		.num_channels = ARRAY_SIZE(adis165x_channels),
		.channels = adis165x_channels,
		.gyro_max_val = 1,
		.gyro_max_scale = IIO_RAD_TO_DEGREE(10 << 16),
		.accel_max_val = 78,
		.accel_max_scale = 32000 << 16,
					 .temp_scale = 100,
					 .int_clk = 2000,
					 .sync_rate_limit = adis165x_sync_mode,
					 .adis165x_data = ADIS165X_DATA(adis165x_def, 16505, &adis1650x_timeouts),
					 .reg_map = &adis165x_def,
					 .burst_sel_max = 1,
					 .burst_size_max = 1,
					 .pt_of_perc_algnmt_max = 1,
					 .linear_accl_comp_max = 1,
					 .sync_mode_max = 3,
					 .flshcnt_max = 10000,
					 .filt_ctrl_max = 6,
					 .dec_rate_max = 1999,
					 .filter_update_us = 30,
					 .msc_reg_update_us = 200,
					 .sens_bw_update_ms = 250,
	},
};


#define adis165x_field_get(_reg, _mask) (((_reg) & (_mask)) >> __ffs(_mask))
#define adis165x_field_prep(_val, _mask) (((_val) << __ffs(_mask)) & (_mask))

static void adis165x_update_diag_flags(struct adis165x *st, u16 diag_stat)
{
	const struct adis165x_data_def *reg_map = st->info->reg_map;
	st->diag_flags.adis165x_diag_flags_bits.DATA_PATH_OVERRUN = diag_stat &
			reg_map->data_path_overrun_mask ? 1 : 0;
	st->diag_flags.adis165x_diag_flags_bits.SPI_COMM_ERR = diag_stat &
			reg_map->spi_comm_err_mask ? 1 : 0;
	st->diag_flags.adis165x_diag_flags_bits.STANDBY_MODE = diag_stat &
			reg_map->standby_mode_mask ? 1 : 0;
	st->diag_flags.adis165x_diag_flags_bits.CLOCK_ERR = diag_stat &
			reg_map->clock_err_mask ? 1 : 0;
	st->diag_flags.adis165x_diag_flags_bits.FLS_MEM_UPDATE_ERR = diag_stat &
			reg_map->fls_mem_update_mask ? 1 : 0;
	st->diag_flags.adis165x_diag_flags_bits.FLS_MEM_TEST_ERR = diag_stat &
			reg_map->fls_mem_test_mask ? 1 : 0;
	st->diag_flags.adis165x_diag_flags_bits.SNSR_SELF_TEST_ERR = diag_stat &
			reg_map->snsr_self_test_mask ? 1 : 0;
	st->diag_flags.adis165x_diag_flags_bits.GYRO1_SELF_TEST_ERR = diag_stat &
			reg_map->gyro1_self_test_mask ? 1 : 0;
	st->diag_flags.adis165x_diag_flags_bits.GYRO2_SELF_TEST_ERR = diag_stat &
			reg_map->gyro2_self_test_mask ? 1 : 0;
	st->diag_flags.adis165x_diag_flags_bits.ACCL_SELF_TEST_ERR = diag_stat &
			reg_map->accl_self_test_mask ? 1 : 0;
}

static int adis165x_read_sync_mode(void *arg, u64 *sync_mode)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->sync_mode;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*sync_mode = adis165x_field_get(reg_val, reg.mask);

	return 0;
}

static int adis165x_read_dec_rate(void *arg, u64 *dec_rate)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->dec_rate;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*dec_rate = reg_val & reg.mask;

	return 0;
}

static int adis165x_read_up_scale(void *arg, u64 *up_scale)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->up_scale;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*up_scale = reg_val & reg.mask;

	return 0;
}

static int adis165x_read_sampling_freq(struct adis165x *st, u32 *freq)
{
	int ret;
	u64 dec_rate;
	u64 sync_mode;
	u64 up_scale;
	u32 sample_rate = st->clk_freq;

	ret = adis165x_read_sync_mode(st, &sync_mode);
	if (ret)
		return ret;

	if (sync_mode == ADIS165X_SYNC_SCALED) {
		ret = adis165x_read_up_scale(st, &up_scale);
		if (ret)
			return ret;

		sample_rate = st->clk_freq * up_scale;
	}

	ret = adis165x_read_dec_rate(st, &dec_rate);
	if (ret)
		return ret;

	*freq = DIV_ROUND_CLOSEST(sample_rate, (u32)dec_rate + 1);

	return 0;
}

static int adis165x_write_up_scale(void *arg, u64 up_scale)
{
	struct adis165x *st = arg;
	int ret;
	u64 sync_mode;
	struct adis165x_reg reg = st->info->reg_map->up_scale;

	if(up_scale > reg.mask)
		return -EINVAL;

	ret = adis165x_read_sync_mode(st, &sync_mode);
	if (ret)
		return ret;

	/* Allow for any value to be written unless the device is in SYNC_SCALED synchronization mode.
	 * If the device is in SYNC_SCALED syncronization mode, make sure the result for clk_freq * up_scale
	is between 1900 and 2100 Hz, otherwise return -EINVAL.
	*/
	if (sync_mode == ADIS165X_SYNC_SCALED && (st->clk_freq*up_scale > 2100
			|| st->clk_freq*up_scale < 1900))
		return -EINVAL;

	ret = adis_write_reg(&st->adis, reg.addr, up_scale, reg.size);
	if (ret)
		return ret;

	return adis165x_read_sampling_freq(st, &st->sampling_frequency);
}

static int adis165x_update_sync_mode(struct adis165x *st,
				     unsigned int sync_mode, unsigned int ext_clk)
{
	int ret;
	struct adis165x_reg reg = st->info->reg_map->sync_mode;

	if(sync_mode > st->info->sync_mode_max)
		return -EINVAL;

	if (sync_mode != ADIS165X_SYNC_DEFAULT && sync_mode != ADIS165X_SYNC_OUTPUT) {
		/* Sync pulse is external */
		if (ext_clk < st->info->sync_rate_limit[sync_mode].min_rate
		    || ext_clk > st->info->sync_rate_limit[sync_mode].max_rate)
			return -EINVAL;

		st->ext_clk = ext_clk;
		st->clk_freq = ext_clk;

		if (sync_mode == ADIS165X_SYNC_SCALED) {
			/*
			 * In sync scaled mode, the IMU sample rate is the clk_freq * sync_scale.
			 * Hence, default the IMU sample rate to the highest multiple of the input
			 * clock lower than the IMU max sample rate. The optimal range is
			 * 1900-2100 sps...
			 */
			ret = adis165x_write_up_scale(st, 2100 / st->clk_freq);
			if (ret)
				return ret;
		}

	} else {
		st->clk_freq = st->info->int_clk;
	}

	ret = adis_update_bits_base(&st->adis, reg.addr, reg.mask,
				    adis165x_field_prep(sync_mode, reg.mask),
				    reg.size);
	if(ret)
		return ret;

	return adis165x_read_sampling_freq(st, &st->sampling_frequency);
}

static int adis165x_write_dec_rate(void *arg, u64 dec_rate)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->dec_rate;
	int ret;
	if(dec_rate > st->info->dec_rate_max)
		return -EINVAL;

	ret =  adis_update_bits_base(&st->adis, reg.addr, reg.mask,
				     adis165x_field_prep(dec_rate, reg.mask), reg.size);
	if(ret)
		return ret;

	fsleep(st->info->filter_update_us);

	return adis165x_read_sampling_freq(st, &st->sampling_frequency);
}

static int adis165x_write_sampling_freq(struct adis165x *st, const u32 freq)
{
	u16 dec_rate;
	int ret;
	u32 sample_rate = st->clk_freq;
	u64 sync_mode;
	unsigned long scaled_rate;
	int up_scale;
	if (!freq)
		return -EINVAL;

	ret = adis165x_read_sync_mode(st, &sync_mode);
	if (ret)
		return ret;

	if (sync_mode == ADIS165X_SYNC_SCALED) {
		scaled_rate = lcm(st->clk_freq, freq);

		/*
		 * If lcm is bigger than the IMU maximum sampling rate there's no perfect
		 * solution. In this case, we get the highest multiple of the input clock
		 * lower than the IMU max sample rate.
		 */
		if (scaled_rate > 2100)
			scaled_rate = 2100 / st->clk_freq * st->clk_freq;
		else
			scaled_rate = 2100 / scaled_rate * scaled_rate;

		up_scale = scaled_rate / st->clk_freq;
		ret = adis165x_write_up_scale(st, up_scale);
		if (ret)
			return ret;

		sample_rate = scaled_rate;
	}

	dec_rate = DIV_ROUND_CLOSEST(sample_rate, freq);

	if (dec_rate)
		dec_rate--;

	if (dec_rate > st->info->dec_rate_max)
		dec_rate = st->info->dec_rate_max;

	return adis165x_write_dec_rate(st, dec_rate);
}

static int adis165x_read_filt_ctrl(void *arg, u64 *val)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->filt_ctrl;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*val = reg_val & reg.mask;

	return 0;
}

static int adis165x_write_filt_ctrl(void *arg, u64 filt_ctrl)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->filt_ctrl;
	int ret;

	if(filt_ctrl > st->info->filt_ctrl_max)
		return -EINVAL;

	ret = adis_write_reg(&st->adis, reg.addr, filt_ctrl, reg.size);
	if (ret)
		return ret;

	fsleep(st->info->filter_update_us);

	return 0;
}

static int adis165x_write_dr_polarity(void *arg, u64 dr_polarity)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->dr_polarity;
	int ret;
	if(dr_polarity > 1)
		return -EINVAL;

	ret =  adis_update_bits_base(&st->adis, reg.addr, reg.mask,
				     adis165x_field_prep(dr_polarity, reg.mask), reg.size);
	if(ret)
		return ret;

	fsleep(st->info->msc_reg_update_us);

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static int adis165x_read_flshcnt(void *arg, u64 *flshcnt)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->flshcnt;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*flshcnt = reg_val;

	if(*flshcnt > st->info->flshcnt_max)
		st->diag_flags.adis165x_diag_flags_bits.FLS_MEM_WR_CNT_EXCEED = true;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_flshcnt_fops,
			 adis165x_read_flshcnt, NULL, "%lld\n");

static int adis165x_read_usr_scr_3(void *arg, u64 *usr_scr_3)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->usr_scr_3;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*usr_scr_3 = reg_val;

	return 0;
}
static int adis165x_write_usr_scr_3(void *arg, u64 usr_scr_3)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->usr_scr_3;

	if (usr_scr_3 > reg.mask)
		return -EINVAL;

	return adis_write_reg(&st->adis, reg.addr, usr_scr_3, reg.size);
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_usr_scr_3_fops,
			 adis165x_read_usr_scr_3, adis165x_write_usr_scr_3, "%llu\n");

static int adis165x_read_usr_scr_2(void *arg, u64 *usr_scr_2)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->usr_scr_2;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*usr_scr_2 = reg_val;

	return 0;
}

static int adis165x_write_usr_scr_2(void *arg, u64 usr_scr_2)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->usr_scr_2;

	if (usr_scr_2 > reg.mask)
		return -EINVAL;

	return adis_write_reg(&st->adis, reg.addr, usr_scr_2, reg.size);
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_usr_scr_2_fops,
			 adis165x_read_usr_scr_2, adis165x_write_usr_scr_2, "%llu\n");

static int adis165x_read_usr_scr_1(void *arg, u64 *usr_scr_1)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->usr_scr_1;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*usr_scr_1 = reg_val;

	return 0;
}

static int adis165x_write_usr_scr_1(void *arg, u64 usr_scr_1)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->usr_scr_1;

	if (usr_scr_1 > reg.mask)
		return -EINVAL;

	return adis_write_reg(&st->adis, reg.addr, usr_scr_1, reg.size);
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_usr_scr_1_fops,
			 adis165x_read_usr_scr_1, adis165x_write_usr_scr_1, "%llu\n");

static int adis165x_read_serial_num(void *arg, u64 *serial_num)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->serial_num;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*serial_num = reg_val;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_serial_num_fops,
			 adis165x_read_serial_num, NULL, "%llu\n");

static int adis165x_read_prod_id(void *arg, u64 *prod_id)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->prod_id;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*prod_id = reg_val;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_prod_id_fops, adis165x_read_prod_id, NULL,
			 "%llu\n");

static ssize_t adis165x_read_fw_date(struct file *file,
				     char __user *userbuf,
				     size_t count, loff_t *ppos)
{
	struct adis165x *st = file->private_data;
	u16 md, year;
	char buf[12];
	size_t len;
	int ret;

	ret = adis_read_reg_16(&st->adis, st->info->reg_map->firm_y.addr, &year);
	if (ret)
		return ret;

	ret = adis_read_reg_16(&st->adis, st->info->reg_map->firm_dm.addr, &md);
	if (ret)
		return ret;

	len = snprintf(buf, sizeof(buf), "%.2x-%.2x-%.4x\n", md >> 8, md & 0xff,
		       year);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations adis165x_fw_date_fops = {
	.open = simple_open,
	.read = adis165x_read_fw_date,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

static ssize_t adis165x_read_fw_rev(struct file *file,
				    char __user *userbuf,
				    size_t count, loff_t *ppos)
{
	struct adis165x *st = file->private_data;
	char buf[7];
	size_t len;
	u16 rev;
	int ret;

	ret = adis_read_reg_16(&st->adis, st->info->reg_map->firm_rev.addr, &rev);
	if (ret)
		return ret;

	len = scnprintf(buf, sizeof(buf), "%x.%x\n", rev >> 8, rev & 0xff);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations adis165x_fw_rev_fops = {
	.open = simple_open,
	.read = adis165x_read_fw_rev,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

static int adis165x_cmd_fact_calib_restore(void *arg, u64 val)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->fact_calib_restore;
	int ret;

	ret = adis_write_reg(&st->adis, reg.addr, reg.mask, reg.size);
	if(ret)
		return ret;

	return adis165x_read_sampling_freq(st, &st->sampling_frequency);
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_fact_calib_restore_fops,
			 NULL, adis165x_cmd_fact_calib_restore, "%llu\n");

static int adis165x_cmd_snsr_self_test(void *arg, u64 val)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->snsr_self_test;
	return adis_write_reg(&st->adis, reg.addr, reg.mask, reg.size);
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_cmd_snsr_self_test_fops,
			 NULL, adis165x_cmd_snsr_self_test, "%llu\n");

static int adis165x_cmd_fls_mem_update(void *arg, u64 val)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->fls_mem_update;
	return adis_write_reg(&st->adis, reg.addr, reg.mask, reg.size);
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_cmd_fls_mem_update_fops,
			 NULL, adis165x_cmd_fls_mem_update, "%llu\n");

static int adis165x_cmd_fls_mem_test(void *arg, u64 val)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->fls_mem_test;
	return adis_write_reg(&st->adis, reg.addr, reg.mask, reg.size);
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_cmd_fls_mem_test_fops,
			 NULL, adis165x_cmd_fls_mem_test, "%llu\n");

static int adis165x_cmd_sw_res(void *arg, u64 val)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->sw_res;
	return adis_write_reg(&st->adis, reg.addr, reg.mask, reg.size);
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_cmd_sw_res_fops,
			 NULL, adis165x_cmd_sw_res, "%llu\n");


DEFINE_DEBUGFS_ATTRIBUTE(adis165x_dec_rate_fops,
			 adis165x_read_dec_rate, adis165x_write_dec_rate, "%llu\n");


DEFINE_DEBUGFS_ATTRIBUTE(adis165x_up_scale_fops,
			 adis165x_read_up_scale, adis165x_write_up_scale, "%llu\n");

static int adis165x_read_dr_polarity(void *arg, u64 *dr_polarity)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->dr_polarity;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*dr_polarity = reg_val & reg.mask ? 1 : 0;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_dr_polarity_fops,
			 adis165x_read_dr_polarity, adis165x_write_dr_polarity, "%llu\n");

static int adis165x_read_sync_polarity(void *arg, u64 *sync_polarity)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->sync_polarity;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*sync_polarity = reg_val & reg.mask ? 1 : 0;

	return 0;
}

static int adis165x_write_sync_polarity(void *arg, u64 sync_polarity)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->sync_polarity;
	int ret;
	if(sync_polarity > 1)
		return -EINVAL;

	ret =  adis_update_bits_base(&st->adis, reg.addr, reg.mask,
				     adis165x_field_prep(sync_polarity, reg.mask), reg.size);
	if(ret)
		return ret;

	fsleep(st->info->msc_reg_update_us);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_sync_polarity_fops,
			 adis165x_read_sync_polarity, adis165x_write_sync_polarity, "%llu\n");

static int adis165x_write_sync_mode(void *arg, u64 sync_mode)
{
	struct adis165x *st = arg;
	if(sync_mode > st->info->sync_mode_max)
		return -EINVAL;

	return adis165x_update_sync_mode(st, sync_mode, st->ext_clk);
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_sync_mode_fops,
			 adis165x_read_sync_mode, adis165x_write_sync_mode, "%llu\n");

static int adis165x_read_sens_bw(void *arg, u64 *sens_bw)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->sens_bw;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*sens_bw = adis165x_field_get(reg_val, reg.mask);

	return 0;
}

static int adis165x_write_sens_bw(void *arg, u64 sens_bw)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->sens_bw;
	int ret;

	if(sens_bw > st->info->sens_bw_max)
		return -EINVAL;

	ret =  adis_update_bits_base(&st->adis, reg.addr, reg.mask,
				     adis165x_field_prep(sens_bw, reg.mask), reg.size);
	if(ret)
		return ret;

	msleep(st->info->sens_bw_update_ms);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_sens_bw_fops,
			 adis165x_read_sens_bw, adis165x_write_sens_bw, "%llu\n");

static int adis165x_read_pt_of_perc_algnmt(void *arg, u64 *pt_of_perc_algnmt)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->pt_of_perc_algnmt;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*pt_of_perc_algnmt = reg_val & reg.mask ? 1 : 0;

	return 0;
}

static int adis165x_write_pt_of_perc_algnmt(void *arg, u64 pt_of_perc_algnmt)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->pt_of_perc_algnmt;
	int ret;

	if(pt_of_perc_algnmt > st->info->pt_of_perc_algnmt_max)
		return -EINVAL;

	ret =  adis_update_bits_base(&st->adis, reg.addr, reg.mask,
				     adis165x_field_prep(pt_of_perc_algnmt, reg.mask), reg.size);
	if(ret)
		return ret;

	fsleep(st->info->msc_reg_update_us);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_pt_of_perc_algnmt_fops,
			 adis165x_read_pt_of_perc_algnmt, adis165x_write_pt_of_perc_algnmt, "%llu\n");

static int adis165x_read_linear_accl_comp(void *arg, u64 *linear_accl_comp)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->linear_accl_comp;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*linear_accl_comp = reg_val & reg.mask ? 1 : 0;

	return 0;
}

static int adis165x_write_linear_accl_comp(void *arg, u64 linear_accl_comp)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->linear_accl_comp;
	int ret;

	if(linear_accl_comp > st->info->linear_accl_comp_max)
		return -EINVAL;

	ret =  adis_update_bits_base(&st->adis, reg.addr, reg.mask,
				     adis165x_field_prep(linear_accl_comp, reg.mask), reg.size);
	if(ret)
		return ret;

	fsleep(st->info->msc_reg_update_us);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_linear_accl_comp_fops,
			 adis165x_read_linear_accl_comp, adis165x_write_linear_accl_comp, "%llu\n");

static int adis165x_read_burst_sel(void *arg, u64 *burst_sel)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->burst_sel;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*burst_sel = reg_val & reg.mask ? 1 : 0;
	st->burst_sel = *burst_sel;

	return 0;
}

static int adis165x_write_burst_sel(void *arg, u64 burst_sel)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->burst_sel;
	int ret;

	if(burst_sel > st->info->burst_sel_max)
		return -EINVAL;

	ret =  adis_update_bits_base(&st->adis, reg.addr, reg.mask,
				     adis165x_field_prep(burst_sel, reg.mask), reg.size);
	if(ret)
		return ret;

	st->burst_sel = burst_sel;

	fsleep(st->info->msc_reg_update_us);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_burst_sel_fops,
			 adis165x_read_burst_sel, adis165x_write_burst_sel, "%llu\n");

static int adis165x_read_burst_size(void *arg, u64 *burst_size)
{
	struct adis165x *st = arg;
	struct adis *adis = &st->adis;
	struct adis165x_reg reg = st->info->reg_map->burst_size;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*burst_size = reg_val & reg.mask ? 1 : 0;
	st->burst_size = *burst_size;

	if(*burst_size)
		adis->burst_extra_len = 6 * sizeof(u16);
	else
		adis->burst_extra_len = 0;

	return 0;
}

static int adis165x_write_burst_size(void *arg, u64 burst_size)
{
	struct adis165x *st = arg;
	struct adis *adis = &st->adis;
	struct adis165x_reg reg = st->info->reg_map->burst_size;
	int ret;

	if(burst_size > st->info->burst_size_max)
		return -EINVAL;

	ret =  adis_update_bits_base(&st->adis, reg.addr, reg.mask,
				     adis165x_field_prep(burst_size, reg.mask), reg.size);
	if(ret)
		return ret;

	st->burst_size = burst_size;

	fsleep(st->info->msc_reg_update_us);
	if(burst_size)
		adis->burst_extra_len = 6 * sizeof(u16);
	else
		adis->burst_extra_len = 0;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_burst_size_fops,
			 adis165x_read_burst_size, adis165x_write_burst_size, "%llu\n");

#define MAX_TXT_SIZE 50
static const char * const rang_mdl_txt[] = {
	"+/-125_degrees_per_sec",
	"+/-500_degrees_per_sec",
	"+/-2000_degrees_per_sec",
};

static ssize_t adis165x_read_rang_mdl(struct file *file,
				      char __user *userbuf,
				      size_t count, loff_t *ppos)
{
	struct adis165x *st = file->private_data;
	size_t len;
	int ret;
	struct adis165x_reg reg = st->info->reg_map->rang_mdl;
	unsigned int reg_val;
	char buf[MAX_TXT_SIZE];

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	reg_val = adis165x_field_get(reg_val, reg.mask);

	len = snprintf(buf, strlen(rang_mdl_txt[reg_val]) + 1, "%s\n",
		       rang_mdl_txt[reg_val]);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations adis165x_rang_mdl_fops = {
	.open = simple_open,
	.read = adis165x_read_rang_mdl,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

DEFINE_DEBUGFS_ATTRIBUTE(adis165x_filt_ctrl_fops,
			 adis165x_read_filt_ctrl, adis165x_write_filt_ctrl, "%llu\n");

static int adis165x_read_data_cntr(void *arg, u64 *data_cntr)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->data_cntr;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*data_cntr = reg_val;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_data_cntr_fops, adis165x_read_data_cntr, NULL,
			 "%llu\n");

static int adis165x_read_time_stamp(void *arg, u64 *time_stamp)
{
	struct adis165x *st = arg;
	struct adis165x_reg reg = st->info->reg_map->time_stamp;
	unsigned int reg_val;
	int ret;

	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	*time_stamp = reg_val;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_time_stamp_fops, adis165x_read_time_stamp,
			 NULL,
			 "%llu\n");

static int adis165x_read_diag_stat(struct adis165x *st,
				   union adis165x_diag_flags *diag_flags)
{
	unsigned int reg_val;
	struct adis165x_reg reg = st->info->reg_map->diag_stat;
	int ret;
	ret = adis_read_reg(&st->adis, reg.addr, &reg_val, reg.size);
	if (ret)
		return ret;

	adis165x_update_diag_flags(st, reg_val);
	*diag_flags = st->diag_flags;

	return 0;
}

static int adis165x_read_diag_data_path_overrun(void *arg,
		u64 *data_path_overrun_err)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*data_path_overrun_err = diag_flags.adis165x_diag_flags_bits.DATA_PATH_OVERRUN;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_data_path_overrun_fops,
			 adis165x_read_diag_data_path_overrun, NULL, "%llu\n");

static int adis165x_read_diag_fls_mem_update_err(void *arg,
		u64 *fls_mem_update_err)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*fls_mem_update_err = diag_flags.adis165x_diag_flags_bits.FLS_MEM_UPDATE_ERR;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_fls_mem_update_err_fops,
			 adis165x_read_diag_fls_mem_update_err, NULL, "%llu\n");

static int adis165x_read_diag_spi_comm_err(void *arg, u64 *spi_comm_err)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*spi_comm_err = diag_flags.adis165x_diag_flags_bits.SPI_COMM_ERR;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_spi_comm_err_fops,
			 adis165x_read_diag_spi_comm_err, NULL, "%llu\n");

static int adis165x_read_diag_standby_mode(void *arg, u64 *standby_mode)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*standby_mode = diag_flags.adis165x_diag_flags_bits.STANDBY_MODE;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_standby_mode_fops,
			 adis165x_read_diag_standby_mode, NULL, "%llu\n");

static int adis165x_read_diag_snsr_self_test_err(void *arg,
		u64 *snsr_self_test_err)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*snsr_self_test_err = diag_flags.adis165x_diag_flags_bits.SNSR_SELF_TEST_ERR;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_snsr_self_test_err_fops,
			 adis165x_read_diag_snsr_self_test_err, NULL, "%llu\n");

static int adis165x_read_diag_fls_mem_test_err(void *arg, u64 *fls_mem_test_err)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*fls_mem_test_err = diag_flags.adis165x_diag_flags_bits.FLS_MEM_TEST_ERR;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_fls_mem_test_err_fops,
			 adis165x_read_diag_fls_mem_test_err, NULL, "%llu\n");

static int adis165x_read_diag_clock_err(void *arg, u64 *clock_err)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*clock_err = diag_flags.adis165x_diag_flags_bits.CLOCK_ERR;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_diag_clock_err_fops,
			 adis165x_read_diag_clock_err, NULL, "%llu\n");

static int adis165x_read_diag_gyro1_self_test_err(void *arg,
		u64 *gyro1_self_test_err)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*gyro1_self_test_err = diag_flags.adis165x_diag_flags_bits.GYRO1_SELF_TEST_ERR;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_gyro1_self_test_err_fops,
			 adis165x_read_diag_gyro1_self_test_err, NULL, "%llu\n");

static int adis165x_read_diag_gyro2_self_test_err(void *arg,
		u64 *gyro2_self_test_err)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*gyro2_self_test_err = diag_flags.adis165x_diag_flags_bits.GYRO2_SELF_TEST_ERR;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_gyro2_self_test_err_fops,
			 adis165x_read_diag_gyro2_self_test_err, NULL, "%llu\n");

static int adis165x_read_diag_accl_self_test_err(void *arg,
		u64 *accl_self_test_err)
{
	struct adis165x *st = arg;
	union adis165x_diag_flags diag_flags;
	int ret;

	ret = adis165x_read_diag_stat(st, &diag_flags);
	if (ret)
		return ret;

	*accl_self_test_err = diag_flags.adis165x_diag_flags_bits.ACCL_SELF_TEST_ERR;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_diag_accl_self_test_err_fops,
			 adis165x_read_diag_accl_self_test_err, NULL, "%llu\n");

static int adis165x_read_diag_fls_mem_wr_cnt_exceed(void *arg,
		u64 *fls_mem_wr_cnt_exceed)
{
	struct adis165x *st = arg;

	*fls_mem_wr_cnt_exceed =
		st->diag_flags.adis165x_diag_flags_bits.FLS_MEM_WR_CNT_EXCEED;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_read_diag_fls_mem_wr_cnt_exceed_fops,
			 adis165x_read_diag_fls_mem_wr_cnt_exceed, NULL, "%llu\n");

static int adis165x_read_diag_checksum_err(void *arg, u64 *checksum_err)
{
	struct adis165x *st = arg;

	*checksum_err = st->diag_flags.adis165x_diag_flags_bits.CHECKSUM_ERR;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_read_diag_checksum_err_fops,
			 adis165x_read_diag_checksum_err, NULL, "%llu\n");

static int adis165x_read_lost_samples_count(void *arg, u64 *samples_lost)
{
	struct adis165x *st = arg;

	*samples_lost = st->samples_lost;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adis165x_read_lost_samples_count_fops,
			 adis165x_read_lost_samples_count, NULL, "%llu\n");

static void adis165x_debugfs_init(struct iio_dev *indio_dev)
{
	struct adis165x *st = iio_priv(indio_dev);
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);

	debugfs_create_file_unsafe("flash_counter", 0664,
				   d, st, &adis165x_flshcnt_fops);
	debugfs_create_file_unsafe("scratch_pad_register3", 0664,
				   d, st, &adis165x_usr_scr_3_fops);
	debugfs_create_file_unsafe("scratch_pad_register2", 0664,
				   d, st, &adis165x_usr_scr_2_fops);
	debugfs_create_file_unsafe("scratch_pad_register1", 0664,
				   d, st, &adis165x_usr_scr_1_fops);
	debugfs_create_file_unsafe("serial_number", 0444,
				   d, st, &adis165x_serial_num_fops);
	debugfs_create_file_unsafe("product_id", 0444,
				   d, st, &adis165x_prod_id_fops);
	debugfs_create_file("firmware_date", 0444, d,
			    st, &adis165x_fw_date_fops);
	debugfs_create_file("firmware_revision", 0444,
			    d, st, &adis165x_fw_rev_fops);
	debugfs_create_file_unsafe("factory_calibration_restore", 0664,
				   d, st, &adis165x_fact_calib_restore_fops);
	debugfs_create_file_unsafe("sensor_self_test", 0664,
				   d, st, &adis165x_cmd_snsr_self_test_fops);
	debugfs_create_file_unsafe("flash_memory_update", 0664,
				   d, st, &adis165x_cmd_fls_mem_update_fops);
	debugfs_create_file_unsafe("flash_memory_test", 0664,
				   d, st, &adis165x_cmd_fls_mem_test_fops);
	debugfs_create_file_unsafe("software_reset", 0664,
				   d, st, &adis165x_cmd_sw_res_fops);
	debugfs_create_file_unsafe("decimation_filter", 0664,
				   d, st, &adis165x_dec_rate_fops);
	debugfs_create_file_unsafe("sync_signal_scale", 0664,
				   d, st, &adis165x_up_scale_fops);
	debugfs_create_file_unsafe("data_ready_polarity", 0664,
				   d, st, &adis165x_dr_polarity_fops);
	debugfs_create_file_unsafe("sync_polarity", 0664,
				   d, st, &adis165x_sync_polarity_fops);
	debugfs_create_file_unsafe("sync_mode_select", 0664,
				   d, st, &adis165x_sync_mode_fops);
	debugfs_create_file_unsafe("internal_sensor_bandwidth", 0664,
				   d, st, &adis165x_sens_bw_fops);
	debugfs_create_file_unsafe("point_of_percussion_alignment", 0664,
				   d, st, &adis165x_pt_of_perc_algnmt_fops);
	debugfs_create_file_unsafe("linear_acceleration_compensation", 0664,
				   d, st, &adis165x_linear_accl_comp_fops);
	debugfs_create_file_unsafe("burst_data_selection", 0664,
				   d, st, &adis165x_burst_sel_fops);
	debugfs_create_file_unsafe("burst_size_selection", 0664,
				   d, st, &adis165x_burst_size_fops);
	debugfs_create_file("gyroscope_measurement_range", 0444, d,
			    st, &adis165x_rang_mdl_fops);
	debugfs_create_file_unsafe("filter_size", 0664,
				   d, st, &adis165x_filt_ctrl_fops);
	debugfs_create_file_unsafe("data_counter", 0664,
				   d, st, &adis165x_data_cntr_fops);
	debugfs_create_file_unsafe("time_stamp", 0664,
				   d, st, &adis165x_time_stamp_fops);
	debugfs_create_file_unsafe("diag_data_path_overrun", 0664,
				   d, st, &adis165x_diag_data_path_overrun_fops);
	debugfs_create_file_unsafe("diag_flash_memory_update_error", 0664,
				   d, st, &adis165x_diag_fls_mem_update_err_fops);
	debugfs_create_file_unsafe("diag_spi_communication_error", 0664,
				   d, st, &adis165x_diag_spi_comm_err_fops);
	debugfs_create_file_unsafe("diag_standby_mode", 0664,
				   d, st, &adis165x_diag_standby_mode_fops);
	debugfs_create_file_unsafe("diag_sensor_self_test_error", 0664,
				   d, st, &adis165x_diag_snsr_self_test_err_fops);
	debugfs_create_file_unsafe("diag_flash_memory_test_error", 0664,
				   d, st, &adis165x_diag_fls_mem_test_err_fops);
	debugfs_create_file_unsafe("diag_clock_error", 0664,
				   d, st, &adis165x_diag_diag_clock_err_fops);
	debugfs_create_file_unsafe("diag_gyroscope1_self_test_error", 0664,
				   d, st, &adis165x_diag_gyro1_self_test_err_fops);
	debugfs_create_file_unsafe("diag_gyroscope2_self_test_error", 0664,
				   d, st, &adis165x_diag_gyro2_self_test_err_fops);
	debugfs_create_file_unsafe("diag_acceleration_self_test_error", 0664,
				   d, st, &adis165x_diag_accl_self_test_err_fops);
	debugfs_create_file_unsafe("diag_flash_memory_write_count_exceeded_error", 0664,
				   d, st, &adis165x_read_diag_fls_mem_wr_cnt_exceed_fops);
	debugfs_create_file_unsafe("diag_checksum_error_flag", 0664,
				   d, st, &adis165x_read_diag_checksum_err_fops);
	debugfs_create_file_unsafe("lost_samples_count", 0664,
				   d, st, &adis165x_read_lost_samples_count_fops);
}
#else
static void adis165x_debugfs_init(struct iio_dev *indio_dev)
{
}
#endif

/* The values are approximated. */
static const u32 adis165x_3db_freqs[] = {
	720, /* Filter disabled, full BW (~720Hz) */
	360,
	164,
	80,
	40,
	20,
	10,
};

static int adis165x_read_lpf(struct adis165x *st, u32 *filter)
{
	u64 filter_sz;
	int ret;

	ret = adis165x_read_filt_ctrl(st, &filter_sz);
	if (ret)
		return ret;

	*filter = adis165x_3db_freqs[filter_sz];

	return 0;
}

static int adis165x_write_filter(struct adis165x *st, const u32 filter)
{
	int i = ARRAY_SIZE(adis165x_3db_freqs);

	while (--i) {
		if (adis165x_3db_freqs[i] >= filter)
			break;
	}

	return adis165x_write_filt_ctrl(st, i);
}

static int adis165x_read_raw(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     int *val, int *val2, long info)
{
	struct adis165x *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val = st->info->gyro_max_val;
			*val2 = st->info->gyro_max_scale;
			return IIO_VAL_FRACTIONAL;
		case IIO_ACCEL:
			*val = st->info->accel_max_val;
			*val2 = st->info->accel_max_scale;
			return IIO_VAL_FRACTIONAL;
		case IIO_TEMP:
			*val = st->info->temp_scale;
			return IIO_VAL_INT;
		case IIO_ROT:
			*val = st->info->rot_max_val;
			*val2 = st->info->rot_max_scale_log2;;
			return IIO_VAL_FRACTIONAL_LOG2;
		case IIO_VELOCITY:
			*val = st->info->vel_max_val;
			*val2 = st->info->vel_max_scale_log2;
			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = adis_read_reg_32(&st->adis,
				       st->info->reg_map->xg_bias.addr + 4 * chan->scan_index,
				       val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = adis165x_read_lpf(st, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = adis165x_read_sampling_freq(st, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adis165x_write_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      int val, int val2, long info)
{
	struct adis165x *st = iio_priv(indio_dev);
	u32 tmp;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		tmp = val;
		return adis165x_write_sampling_freq(st, tmp);
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		return adis165x_write_filter(st, val);
	case IIO_CHAN_INFO_CALIBBIAS:
		return adis_write_reg_32(&st->adis,
					 st->info->reg_map->xg_bias.addr + 4 * chan->scan_index,
					 val);
	default:
		return -EINVAL;
	}
}

static int adis165x_buff_preenable(struct iio_dev *indio_dev)
{
	struct adis165x *st = iio_priv(indio_dev);

	st->samples_lost = 0;
	st->data_cntr = 0;

	return 0;
}

const struct iio_buffer_setup_ops adis165x_buff_ops = {
	.preenable = adis165x_buff_preenable,
};

static const struct iio_info adis165x_info = {
	.read_raw = &adis165x_read_raw,
	.write_raw = &adis165x_write_raw,
	.update_scan_mode = adis_update_scan_mode,
	.debugfs_reg_access = adis_debugfs_reg_access,
};

static bool adis165x_validate_checksum(const u8 *buffer, u8 size)
{
	u8 i;
	u16 checksum = get_unaligned_be16(&buffer[size-ADIS165X_CHECKSUM_SIZE]);

	for (i = 0; i < size - ADIS165X_CHECKSUM_SIZE; i++)
		checksum -= buffer[i];

	return checksum == 0;
}

static irqreturn_t adis165x_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adis165x *st = iio_priv(indio_dev);
	struct adis *adis = &st->adis;
	int ret, bit, i = 0;
	__be16 *buffer;
	bool valid;
	u8 msg_size;
	u8 buff_idx;
	u32 res1;
	u32 res2;
	u16 current_data_cntr;

	const u8 offset = st->burst_size ? 13 : 7;
	const u8 data_cntr_offset = st->burst_size ? 14 : 8;

	if (st->burst_size == ADIS165X_16_BIT_BURST_SIZE)
		msg_size = ADIS165X_MSG_SIZE_16_BIT_BURST;
	else
		msg_size = ADIS165X_MSG_SIZE_32_BIT_BURST;

	ret = spi_sync(adis->spi, &adis->msg);
	if (ret)
		goto trig_done;

	buffer = adis->buffer;

	valid = adis165x_validate_checksum(adis->buffer, msg_size);
	if (!valid) {
		st->diag_flags.adis165x_diag_flags_bits.CHECKSUM_ERR = true;
		dev_err(&adis->spi->dev, "Invalid crc\n");
		goto trig_done;
	}
	st->diag_flags.adis165x_diag_flags_bits.CHECKSUM_ERR = false;
	adis165x_update_diag_flags(st, get_unaligned_be16(adis->buffer));

	current_data_cntr =  get_unaligned_be16(&buffer[data_cntr_offset]);

	if(st->data_cntr) {
		if (current_data_cntr > st->data_cntr) {
			if (st->sync_mode != ADIS165X_SYNC_SCALED)
				st->samples_lost += current_data_cntr - st->data_cntr - 1;
			else {
				res1 = (current_data_cntr - st->data_cntr) * 49;
				res2 = DIV_ROUND_CLOSEST(1000000, st->sampling_frequency);

				if(res1 > res2) {
					st->samples_lost += res1 / res2;
					if(res1 % res2 < res2 / 2)
						st->samples_lost--;
				}
			}
		} else {
			if (st->sync_mode != ADIS165X_SYNC_SCALED)
				st->samples_lost += 0xFFFFU - st->data_cntr + current_data_cntr;
		}
	}

	st->data_cntr = current_data_cntr;

	for_each_set_bit(bit, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		switch (bit) {
		case ADIS165X_TEMP:
			/* upper not used */
			st->data[i++] = 0;
			st->data[i++] = buffer[offset];
			break;
		case ADIS165X_GYRO_X ... ADIS165X_ACCEL_Z:
			/*
			 * The first 2 bytes on the received data are the
			 * DIAG_STAT reg, hence the +1 offset here...
			 */
			if(st->burst_sel) {
				st->data[i++] = 0;
				st->data[i++] = 0;
			} else {
				if (st->burst_size) {
					/* upper 16 */
					st->data[i++] = buffer[bit * 2 + 2];
					/* lower 16 */
					st->data[i++] = buffer[bit * 2 + 1];
				} else {
					st->data[i++] = buffer[bit + 1];
					/* lower not used */
					st->data[i++] = 0;
				}
			}
			break;

		case ADIS165X_DELTA_ANGL_X ... ADIS165X_DELTA_VEL_Z:
			if(!st->burst_sel) {
				st->data[i++] = 0;
				st->data[i++] = 0;
			} else {
				buff_idx = bit - ADIS165X_DELTA_ANGL_X;
				if (st->burst_size) {
					/* upper 16 */
					st->data[i++] = buffer[buff_idx * 2 + 2];
					/* lower 16 */
					st->data[i++] = buffer[buff_idx * 2 + 1];
				} else {
					st->data[i++] = buffer[buff_idx + 1];
					/* lower not used */
					st->data[i++] = 0;
				}
			}
			break;
		case ADIS165X_DATA_COUNTER:
			st->data[i++] = 0;
			st->data[i++] = buffer[data_cntr_offset];
			break;
		}
	}
	iio_push_to_buffers(indio_dev, st->data);
trig_done:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static void adis165x_disable_clk(void *data)
{
	clk_disable_unprepare((struct clk *)data);
}

static int adis165x_config_sync_mode(struct adis165x *st)
{
	int ret;
	struct device *dev = &st->adis.spi->dev;
	u32 sync_mode;
	u32 ext_clk_freq = 0;

	/* default to internal clk */
	st->clk_freq = st->info->int_clk;

	ret = device_property_read_u32(dev, "adi,sync-mode", &sync_mode);
	if (ret)
		return 0;

	if (sync_mode > st->info->sync_mode_max) {
		dev_err(dev, "Invalid sync mode: %u for %s\n", sync_mode,
			st->info->name);
		return -EINVAL;
	}

	/* All the other modes require external input signal */
	if (sync_mode != ADIS165X_SYNC_OUTPUT) {
		struct clk *clk = devm_clk_get(dev, NULL);

		if (IS_ERR(clk))
			return PTR_ERR(clk);

		ret = clk_prepare_enable(clk);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(dev, adis165x_disable_clk, clk);
		if (ret)
			return ret;

		ext_clk_freq = clk_get_rate(clk);
	}

	return adis165x_update_sync_mode(st, sync_mode, ext_clk_freq);
}

static int adis165x_config_irq_pin(struct adis165x *st)
{
	int ret;
	struct irq_data *desc;
	u32 irq_type;
	u8 polarity;
	struct spi_device *spi = st->adis.spi;

	desc = irq_get_irq_data(spi->irq);
	if (!desc) {
		dev_err(&spi->dev, "Could not find IRQ %d\n", spi->irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(desc);
	if (irq_type == IRQ_TYPE_EDGE_RISING) {
		polarity = 1;
		st->adis.irq_flag = IRQF_TRIGGER_RISING;
	} else if (irq_type == IRQ_TYPE_EDGE_FALLING) {
		polarity = 0;
		st->adis.irq_flag = IRQF_TRIGGER_FALLING;
	} else {
		dev_err(&spi->dev, "Invalid interrupt type 0x%x specified\n",
			irq_type);
		return -EINVAL;
	}

	/* We cannot mask the interrupt so ensure it's not enabled at request */
	st->adis.irq_flag |= IRQF_NO_AUTOEN;

	ret = adis165x_write_dr_polarity(st, polarity);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id adis165x_of_match[] = {
	{
		.compatible = "adi,adis165x-05-1",
		.data = &adis165x_chip_info[ADIS165X_05_1]
	},
	{
		.compatible = "adi,adis165x-05-2",
		.data = &adis165x_chip_info[ADIS165X_05_2]
	},
	{
		.compatible = "adi,adis165x-05-3",
		.data = &adis165x_chip_info[ADIS165X_05_3]
	},
	{ },
};
MODULE_DEVICE_TABLE(of, adis165x_of_match);

static int adis165x_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adis165x *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->info = device_get_match_data(&spi->dev);
	if (!st->info)
		return -EINVAL;

	ret = adis_init(&st->adis, indio_dev, spi, &st->info->adis165x_data);
	if (ret)
		return ret;

	indio_dev->name = st->info->name;
	indio_dev->channels = st->info->channels;
	indio_dev->num_channels = st->info->num_channels;
	indio_dev->info = &adis165x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = __adis_initial_startup(&st->adis);
	if (ret)
		return ret;

	ret = adis165x_config_irq_pin(st);
	if (ret)
		return ret;

	ret = adis165x_config_sync_mode(st);
	if (ret)
		return ret;

	ret = devm_adis_setup_buffer_and_trigger(&st->adis, indio_dev,
			adis165x_trigger_handler,  &adis165x_buff_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return ret;

	adis165x_debugfs_init(indio_dev);

	return 0;
}

static struct spi_driver adis165x_driver = {
	.driver = {
		.name = "adis165x",
		.of_match_table = adis165x_of_match,
	},
	.probe = adis165x_probe,
};
module_spi_driver(adis165x_driver);

MODULE_AUTHOR("Ramona Bolboaca <ramona.bolboaca@analog.com>");
MODULE_DESCRIPTION("Analog Devices adis165x IMU driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_ADISLIB);

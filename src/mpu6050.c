/*
 * Copyright (c) 2019 Liviu Nicolescu <nliviu@gmail.com>
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http//www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <common/cs_dbg.h>

#include <mgos_i2c.h>

#include "mpu6050.h"

/* MPU-6000-Register-Map1.pdf page 14 */
#define REG_GYRO_CONFIG 0x1B
/* MPU-6000-Register-Map1.pdf page 15 */
#define REG_ACCEL_CONFIG 0x1C
/* MPU-6000-Register-Map1.pdf page 29 */
#define REG_ACCEL_OUT 0x3B
#define REG_ACCEL_OUT_REG_COUNT 6
/* MPU-6000-Register-Map1.pdf page 30 */
#define REG_TEMP_OUT 0x41
#define REG_TEMP_OUT_REG_COUNT 2
/* MPU-6000-Register-Map1.pdf page 31 */
#define REG_GYRO_OUT 0x43
#define REG_GYRO_OUT_REG_COUNT 6
/* MPU-6000-Register-Map1.pdf page 40 */
#define REG_PWR_MGMT_1 0x6B
/* MPU-6000-Register-Map1.pdf page 42 */
#define REG_PWR_MGMT_2 0x6C
/* MPU-6000-Register-Map1.pdf page 45 */
#define REG_WHO_AM_I 0x75

struct mpu6050 {
  struct mgos_i2c *i2c;
  uint16_t addr;

  double g_scale; /* deg/s */
  double a_scale; /* g */
};

struct mpu6050 *s_dev = NULL;

static const char *str_err = "--- MPU6050 error - ";

static inline uint16_t be2le(uint16_t be) {
  return (((be << 8) & 0xFF00) | ((be >> 8) & 0x00FF));
}

bool mpu6050_init(uint16_t addr) {
  struct mgos_i2c *i2c = mgos_i2c_get_global();
  if (NULL == i2c) {
    LOG(LL_ERROR, ("%s%s", str_err, "i2c is null!"));
    return false;
  }

  int me = mgos_i2c_read_reg_b(i2c, addr, REG_WHO_AM_I);
  if (addr != me) {
    LOG(LL_ERROR, ("%s%02X != 0x68!", str_err, me));
    return false;
  }

  // init
  /* CLKSEL=1 PLL with X axis gyroscope reference */
  bool init_ok = mgos_i2c_write_reg_b(i2c, addr, REG_PWR_MGMT_1, 0x01);
  if (!init_ok) {
    LOG(LL_ERROR, ("%s%s", str_err, "failed to init!"));
    return false;
  }

  // get gyro full scale
  /*
   * Bit7  | Bit6  | Bit5  | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 
   * XG_ST | YG_ST | ZG_ST | FS_SEL[1:0] | -    | -    | -
   */
  uint8_t gfs_sel;
  bool ok= mgos_i2c_getbits_reg_b(i2c, addr, REG_GYRO_CONFIG, 3, 2, &gfs_sel);
  if (!ok) {
    LOG(LL_ERROR, ("%s%s", str_err, "failed to read gyro scale!"));
    return false;
  }

  // get accel full scale
  /*
   * Bit7  | Bit6  | Bit5  | Bit4 | Bit3  | Bit2 | Bit1 | Bit0
   * XA_ST | YA_ST | ZA_ST | AFS_SEL[1:0] |         -
   */
  uint8_t afs_sel;
  ok= mgos_i2c_getbits_reg_b(i2c, addr, REG_ACCEL_CONFIG, 3, 2, &afs_sel);
  if (!ok) {
    LOG(LL_ERROR, ("%s%s", str_err, "failed to read accel scale!"));
    return false;
  }

  s_dev = (struct mpu6050 *) calloc(1, sizeof(*s_dev));
  if (NULL == s_dev) {
    LOG(LL_ERROR, ("%s%s", str_err, "failed to allocate memory!"));
    return false;
  }

  s_dev->i2c = i2c;
  s_dev->addr = addr;

  switch (gfs_sel) {
    case 0:
      s_dev->g_scale = 250;
      break;
    case 1:
      s_dev->g_scale = 500;
      break;
    case 2:
      s_dev->g_scale = 1000;
      break;
    case 3:
      s_dev->g_scale = 2000;
      break;
  }
  s_dev->g_scale /= 32768.0;

  switch (afs_sel) {
    case 0:
      s_dev->a_scale = 2;
      break;
    case 1:
      s_dev->a_scale = 4;
      break;
    case 2:
      s_dev->a_scale = 8;
      break;
    case 3:
      s_dev->a_scale = 16;
      break;
  }
  s_dev->a_scale /= 32768.0;

  return true;
}

bool mpu6050_read_accel(struct accel *accel) {
  if ((NULL == s_dev) || (NULL == accel)) {
    return false;
  }

  union {
    uint8_t b[REG_ACCEL_OUT_REG_COUNT];
    int16_t w[REG_ACCEL_OUT_REG_COUNT / 2];
  } data;

  bool ok = mgos_i2c_read_reg_n(s_dev->i2c, s_dev->addr, REG_ACCEL_OUT,
                                REG_ACCEL_OUT_REG_COUNT, data.b);
  if (!ok) {
    LOG(LL_ERROR, ("%s%s", str_err, "failed to read accel!"));
    return false;
  }

  accel->x = be2le(data.w[0]);
  accel->y = be2le(data.w[1]);
  accel->z = be2le(data.w[2]);
  accel->scale = s_dev->a_scale;

  return true;
}

bool mpu6050_read_gyro(struct gyro *gyro) {
  if ((NULL == s_dev) || (NULL == gyro)) {
    return false;
  }

  union {
    uint8_t b[REG_GYRO_OUT_REG_COUNT];
    int16_t w[REG_GYRO_OUT_REG_COUNT / 2];
  } data;

  bool ok = mgos_i2c_read_reg_n(s_dev->i2c, s_dev->addr, REG_GYRO_OUT,
                                REG_GYRO_OUT_REG_COUNT, data.b);
  if (!ok) {
    LOG(LL_ERROR, ("%s%s", str_err, "failed to read gyro!"));
    return false;
  }

  gyro->x = be2le(data.w[0]);
  gyro->y = be2le(data.w[1]);
  gyro->z = be2le(data.w[2]);
  gyro->scale = s_dev->g_scale;

  return true;
}

bool mpu6050_read_temperature(double *temperature) {
  if ((NULL == s_dev) || (NULL == temperature)) {
    return false;
  }

  int16_t t = mgos_i2c_read_reg_w(s_dev->i2c, s_dev->addr, REG_TEMP_OUT);
  *temperature = (float) t / 340.0 + 36.53;

  return true;
}

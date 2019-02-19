/*
 * Copyright (c) 2019 Liviu Nicolescu <nliviu@gmail.com>
 * All rights reserved
 *
 * Licensed under the Apache License, Version 2.0 (the ""License"");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an ""AS IS"" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mgos.h"

#include "mpu6050.h"

static void timer_cb(void *arg) {
  (void) arg;
  struct accel accel = {.x = 0, .y = 0, .z = 0, .scale = 0};
  if (mpu6050_read_accel(&accel)) {
    LOG(LL_INFO, ("%20s - x=%5hd, y=%5hd, z=%5hd", "raw accel", accel.x,
                  accel.y, accel.z));
    LOG(LL_INFO,
        ("%20s - x=%5.2f, y=%5.2f, z=%5.2f", "scaled accel (g)",
         accel.x * accel.scale, accel.y * accel.scale, accel.z * accel.scale));
  }

  struct gyro gyro = {.x = 0, .y = 0, .z = 0, .scale = 0};
  if (mpu6050_read_gyro(&gyro)) {
    LOG(LL_INFO,
        ("%20s - x=%5hd, y=%5hd, z=%5hd", "raw gyro", gyro.x, gyro.y, gyro.z));
    LOG(LL_INFO,
        ("%20s - x=%5.2f, y=%5.2f, z=%5.2f", "scaled gyro (deg/s)",
         gyro.x * gyro.scale, gyro.y * gyro.scale, gyro.z * gyro.scale));
  }

  double temperature = 0.0;
  if (mpu6050_read_temperature(&temperature)) {
    LOG(LL_INFO, ("temperature=%.2f", temperature));
  }
}

enum mgos_app_init_result mgos_app_init(void) {
  bool ok = mpu6050_init(mgos_sys_config_get_mpu6050_addr());
  if (ok) {
    mgos_set_timer(mgos_sys_config_get_mpu6050_interval(), MGOS_TIMER_REPEAT,
                   timer_cb, NULL);
  }
  return MGOS_APP_INIT_SUCCESS;
}

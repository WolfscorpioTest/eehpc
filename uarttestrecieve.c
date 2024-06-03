/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * exptest.c - Testing of expansion port.
 */
#define DEBUG_MODULE "GTGPS"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "log.h"

#include "estimator_kalman.h"
#include "system.h"

static bool isInit;

#define BUFFER_LEN 255
static char linebuffer[BUFFER_LEN];

static positionMeasurement_t ext_pos;

static float x, y, z;

void lhTask(void *param)
{
  char ch;

  atof("4.4");

  uart1Init(57600);

  systemWaitStart();

  while(1)
  {
    // Read line ...
    ch = 0;
    int i = 0;
    for (i=0; i<(BUFFER_LEN-1) && ch!='\n'; i++) {
      uart1Getchar(&ch);
      linebuffer[i] = ch;
    }
    linebuffer[i] = '\0';

    // Test if the line is well-formed OBJ0 position
    if (strncmp(linebuffer, "OBJ0", 4)) {
      continue;
    }

    int ntab = 0;
    for(int i=0; i<BUFFER_LEN && linebuffer[i]!='\0'; i++) {
      if (linebuffer[i] == '\t') {
        ntab ++;
      }
    }
    if (ntab != 6) {
      continue;
    }

    // Start decoding ...
    char *ptr = &linebuffer[4];

    strtof(ptr, &ptr);  // Timestamp
    strtof(ptr, &ptr);  // Some code
    x = strtof(ptr, &ptr);
    z = strtof(ptr, &ptr);
    y = strtof(ptr, &ptr);

    // Push position in EKF
    ext_pos.x = x;
    ext_pos.y = -1*y;
    ext_pos.z = z;
    ext_pos.stdDev = 0.01;
    estimatorKalmanEnqueuePosition(&ext_pos);
  }
}


static void lhInit(DeckInfo *info)
{
  if(isInit)
    return;

  DEBUG_PRINT("Enabling reading from GlobalTop GPS\n");
  // uart1Init(57600);

  xTaskCreate(lhTask, "LIGHTHOUSE",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);

  isInit = true;
}

static bool lhTest()
{
  bool status = true;

  if(!isInit)
    return false;

  return status;
}

static const DeckDriver lhDeck = {
  .vid = 0,
  .pid = 0,
  .name = "bcLighthouse",

  .usedPeriph = 0,
  .usedGpio = 0,               // FIXME: Edit the used GPIOs
  .requiredEstimator = kalmanEstimator,

  .init = lhInit,
  .test = lhTest,
};

DECK_DRIVER(lhDeck);

LOG_GROUP_START(lighthouse)
LOG_ADD(LOG_FLOAT, x, &ext_pos.x)
LOG_ADD(LOG_FLOAT, y, &ext_pos.y)
LOG_ADD(LOG_FLOAT, z, &ext_pos.z)
LOG_GROUP_STOP(lighthouse)

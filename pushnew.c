/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * push.c - App layer application of the onboard push demo. The crazyflie 
 * has to have the multiranger and the flowdeck version 2.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>


#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "PUSH"

//function adding together various positioning data of drone to determine

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

// fsm states
typedef enum {
    idle,
    lowUnlock,
    unlocked,
    stopping
} State;


// default state is unlocked
static State state = unlocked;

//int values for various states (idle is 0 so isn't here)
static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 500;

//default constant values for calculations
static const float velMax = 1.0f;
static const uint16_t radius = 300;
static const uint16_t radius_up_down = 100;
static const float up_down_delta = 0.002f;

static float height_sp = 0.2f;

// "functions for min max"
#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)


// appMain is run by kbuild by default
void appMain()
{
  static setpoint_t setpoint;

  vTaskDelay(M2T(3000));

    //logging from multiranger?
  logVarId_t idUp = logGetVarId("range", "up");
  logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack = logGetVarId("range", "back");
  
  // selecting flow and multiranger decks
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");

    //default const
  float factor = velMax/radius;

  //DEBUG_PRINT("%i", idUp);

  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(10));
    //DEBUG_PRINT(".");

    // just getting values

    //understand direction of movement
    uint8_t positioningInit = paramGetUint(idPositioningDeck);

    //understand positioning of objects around
    uint8_t multirangerInit = paramGetUint(idMultiranger);

    uint16_t up = logGetUint(idUp);

    if (state == unlocked) {

        //finding distances of something close
      uint16_t left = logGetUint(idLeft);
      uint16_t right = logGetUint(idRight);
      uint16_t front = logGetUint(idFront);
      uint16_t back = logGetUint(idBack);

        // if left is right there (we detect it very close, then left_o is radius) 
        // if left is very far (we detect it very far away, left_o = 0 and we will focus on the right)
      uint16_t left_o = radius - MIN(left, radius);
      // vice versa
      uint16_t right_o = radius - MIN(right, radius); 
      // can keep these because our goal is just to keep moving forward anyways

      //adding sign and "speed factor"
      float l_comp = (-1) * left_o * factor;
      float r_comp = right_o * factor;

      //finding distances and adding together to see which direction to go into
      float velSide = r_comp + l_comp;


        // can change this to be nothing because we want to be moving forward
      uint16_t front_o = radius - MIN(front, radius);
      uint16_t back_o = radius - MIN(back, radius);
      float f_comp = (-1) * front_o * factor;
      float b_comp = back_o * factor;
      float velFront = b_comp + f_comp;

      //default vel (change with the inputs from nn)
      velFront = 5;
      // set conditional

      // we want to go up when there are obstacles (hands) closer than radius_up_down on both sides
      if(left < radius_up_down && right < radius_up_down)
      {
        height_sp += up_down_delta;
      }

      // we want to go down when there are obstacles (hands) closer than radius_up_down in front and back (or there is something on top)
      if((front < radius_up_down && back < radius_up_down) || up < radius)
      {
        height_sp -= up_down_delta;
      }

      uint16_t up_o = radius - MIN(up, radius);
      float height = height_sp - up_o/1000.0f;


      /*DEBUG_PRINT("l=%i, r=%i, lo=%f, ro=%f, vel=%f\n", left_o, right_o, l_comp, r_comp, velSide);
      DEBUG_PRINT("f=%i, b=%i, fo=%f, bo=%f, vel=%f\n", front_o, back_o, f_comp, b_comp, velFront);
      DEBUG_PRINT("u=%i, d=%i, height=%f\n", up_o, height);*/

      if (1) {
        setHoverSetpoint(&setpoint, velFront, velSide, height, 0);
        commanderSetSetpoint(&setpoint, 3);
      }

      if (height < 0.1f) {
        state = stopping;
        DEBUG_PRINT("X\n");
      }

    } else {

      if (state == stopping && up > stoppedTh) {
        DEBUG_PRINT("%i", up);
        state = idle;
        DEBUG_PRINT("S\n");
      }

      if (up < unlockThLow && state == idle && up > 0.001f) {
        DEBUG_PRINT("Waiting for hand to be removed!\n");
        state = lowUnlock;
      }

      if (up > unlockThHigh && state == lowUnlock && positioningInit && multirangerInit) {
        DEBUG_PRINT("Unlocked!\n");
        state = unlocked;
      }

      if (state == idle || state == stopping) {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
      }
    }
  }
}

/*
 * mcp23017.c:
 *	Extend wiringPi with the MCP 23017 I2C GPIO expander chip
 *	Copyright (c) 2013 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"
#include "mcp23x0817.h"

#include "mcp23017.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

static int fdDev = 0;

/**
 * Register address, port dependent, for a given PIN
 */
static uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr){
  return(pin<8) ?portAaddr:portBaddr;
}

/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
static void updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
  uint8_t regValue;
  uint8_t regAddr=regForPin(pin,portAaddr,portBaddr);
  regValue = wiringPiI2CReadReg8 (fdDev, regAddr);

  // set or clear the value for the particular bit
  bitWrite(regValue,pin%8,pValue);

  wiringPiI2CWriteReg8 (fdDev, regAddr, regValue) ;
}

/*
 * myInterruptRead:
 * pinBase should be passed in via pins so we can get the node structure.
 *********************************************************************************
 */

static int myInterruptRead (struct wiringPiNodeStruct *node, int *pins, int *values)
{
  if ((node = wiringPiFindNode (*pins)) == NULL)
    return -1;

  *pins = 0;

  // get which pins have an interrupt
  // try port A
  *pins = wiringPiI2CReadReg8 (node->fd, MCP23x17_INTFA);
  *values = wiringPiI2CReadReg8(node->fd, MCP23x17_INTCAPA);

  // try port B
  *pins |= (wiringPiI2CReadReg8 (node->fd, MCP23x17_INTFB)) << 8;
  *values |= wiringPiI2CReadReg8(node->fd, MCP23x17_INTCAPB) << 8;

  return 0;
}


/*
 * myPinIntPolarity:
 * Set's up a pin for interrupt. uses MODEs: INT_EDGE_FALLING, INT_EDGE_RISING, and INT_EDGE_BOTH
 *
 * Note that the interrupt condition finishes when you read the information about the port / value
 * that caused the interrupt or you read the port itself. Check the datasheet as it can be confusing.
 *
 *********************************************************************************
 */
static void myPinIntPolarity (struct wiringPiNodeStruct *node, int pin, int mode)
{
  pin -= node->pinBase ;

  // set the pin interrupt control
  // 0 means change, 1 means compare against given value
  updateRegisterBit(pin,(mode!=INT_EDGE_FALLING),MCP23x17_IOCON,MCP23x17_IOCONB);

  // In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
  // In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
  updateRegisterBit(pin,(mode==INT_EDGE_FALLING),MCP23x17_DEFVALA,MCP23x17_DEFVALB);

  // enable the pin for interrupt
  updateRegisterBit(pin,HIGH,MCP23x17_GPINTENA,MCP23x17_GPINTENB);
}

/*
 * myPinMode:
 *********************************************************************************
 */

static void myPinMode (struct wiringPiNodeStruct *node, int pin, int mode)
{
  pin -= node->pinBase ;

  updateRegisterBit(pin,(mode==INPUT),MCP23x17_IODIRA,MCP23x17_IODIRB);
}


/*
 * myPullUpDnControl:
 *********************************************************************************
 */

static void myPullUpDnControl (struct wiringPiNodeStruct *node, int pin, int mode)
{
  pin -= node->pinBase ;

  updateRegisterBit(pin,(mode==PUD_UP),MCP23x17_GPPUA,MCP23x17_GPPUB);
}


/*
 * myDigitalWrite:
 *********************************************************************************
 */

static void myDigitalWrite (struct wiringPiNodeStruct *node, int pin, int value)
{
  pin -= node->pinBase ;	// Pin now 0-15

  updateRegisterBit(pin,(value==HIGH),MCP23x17_GPIOA,MCP23x17_GPIOB);
}


/*
 * myDigitalRead:
 *********************************************************************************
 */

static int myDigitalRead (struct wiringPiNodeStruct *node, int pin)
{
  uint8_t regAddr;

  pin -= node->pinBase ;

  regAddr=regForPin(pin,MCP23x17_GPIOA,MCP23x17_GPIOB);
  return (wiringPiI2CReadReg8 (node->fd, regAddr) >> (pin%8)) & 1;
}


/*
 * mcp23017Setup:
 *	Create a new instance of an MCP23017 I2C GPIO interface. We know it
 *	has 16 pins, so all we need to know here is the I2C address and the
 *	user-defined pin base.
 *********************************************************************************
 */

int mcp23017Setup (const int pinBase, const int i2cAddress)
{
  int fd ;
  struct wiringPiNodeStruct *node ;

  if ((fd = wiringPiI2CSetup (i2cAddress)) < 0)
    return fd ;

  fdDev = fd;

  // set defaults
  wiringPiI2CWriteReg8 (fd, MCP23x17_IODIRA, 0xFF) ;
  wiringPiI2CWriteReg8 (fd, MCP23x17_IODIRB, 0xFF) ;
  wiringPiI2CWriteReg8 (fd, MCP23x17_GPPUA, 0) ;
  wiringPiI2CWriteReg8 (fd, MCP23x17_GPPUB, 0) ;
  wiringPiI2CWriteReg8 (fd, MCP23x17_IOCON, 0) ;
  wiringPiI2CWriteReg8 (fd, MCP23x17_GPINTENA, 0);
  wiringPiI2CWriteReg8 (fd, MCP23x17_GPINTENB, 0);
  wiringPiI2CWriteReg8 (fd, MCP23x17_INTCONA, 0);
  wiringPiI2CWriteReg8 (fd, MCP23x17_INTCONB, 0);
  wiringPiI2CWriteReg8 (fd, MCP23x17_DEFVALA, 0);
  wiringPiI2CWriteReg8 (fd, MCP23x17_DEFVALB, 0);
  // clear any interrupts that may be pending
  wiringPiI2CReadReg8 (fd, MCP23x17_GPIOA);
  wiringPiI2CReadReg8 (fd, MCP23x17_GPIOB);
  
  wiringPiI2CWriteReg8 (fd, MCP23x17_IOCON, IOCON_INIT) ;

  node = wiringPiNewNode (pinBase, 16) ;

  node->fd              = fd ;
  node->pinMode         = myPinMode ;
  node->pinIntPolarity  = myPinIntPolarity ;
  node->pullUpDnControl = myPullUpDnControl ;
  node->digitalRead     = myDigitalRead ;
  node->digitalWrite    = myDigitalWrite ;
  node->interruptRead   = myInterruptRead ;
  node->data2           = wiringPiI2CReadReg8 (fd, MCP23x17_OLATA) ;
  node->data3           = wiringPiI2CReadReg8 (fd, MCP23x17_OLATB) ;

  return 0 ;
}

/*
 * mcp23017SetupInts:
 * Configures the interrupt system. both port A and B are assigned the same configuration.
 * Mirroring will OR both INTA and INTB pins.
 * Opendrain will set the INT pin to value or open drain.
 * polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false,false, LOW)
 *
 *********************************************************************************
 */

int mcp23017SetupInts (int pinBase, uint8_t mirroring, uint8_t openDrain, uint8_t polarity) {
  uint8_t ioconfValue;
  struct wiringPiNodeStruct *node;

  if ((node = wiringPiFindNode (pinBase)) == NULL)
    return -1;
        
  // configure the port A
  ioconfValue = wiringPiI2CReadReg8 (node->fd, MCP23x17_IOCON);
  bitWrite(ioconfValue,6,mirroring);
  bitWrite(ioconfValue,2,openDrain);
  bitWrite(ioconfValue,1,polarity);
  wiringPiI2CWriteReg8 (node->fd, MCP23x17_IOCON,ioconfValue);

  // Configure the port B
  ioconfValue = wiringPiI2CReadReg8 (node->fd, MCP23x17_IOCONB);
  bitWrite(ioconfValue,6,mirroring);
  bitWrite(ioconfValue,2,openDrain);
  bitWrite(ioconfValue,1,polarity);
  wiringPiI2CWriteReg8 (node->fd, MCP23x17_IOCONB,ioconfValue);
  return 0;
}

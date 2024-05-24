/* MCP23S17 - drive the Microchip MCP23S17 16-bit Port Extender using SPI
 * mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef  MCP23017_H
#define  MCP23017_H

#include "mbed.h"
#include "../lib/GPIOLib/inc/GPIOExpansionInterface.h"
#include <list>

#define MCP23017_INTERRUPT_FLAG 0x01

// all register addresses assume IOCON.BANK = 0 (POR default)

#define IODIR   0x00
#define IPOL    0x02
#define GPINTEN 0x04
#define DEFVAL  0x06
#define INTCON  0x08
#define IOCON   0x0A
#define GPPU    0x0C
#define INTF    0x0E
#define INTCAP  0x10
#define GPIO    0x12
#define OLAT    0x14

// Control settings

#define IOCON_BANK			0x80 // Banked registers
#define IOCON_MIRROR		0x40
#define IOCON_BYTE_MODE	    0x20 // Disables sequential operation. If bank = 0, operations toggle between A and B registers
#define IOCON_HAEN			0x08 // Hardware address enable
#define IOCON_ODR			0x04 // INT Polarity
#define IOCON_INT			0x02 // INT Polarity

/** Typedef struct for external pin configuration
 *
 *  Used to block double initializing and for save callback function
 */
typedef struct {
	ExpPortName port;
	ExpPinName pin;
	Callback<void(uint32_t,gpio_irq_event)> func;
	uint32_t id;
}ExpGPIO_t;

/** Typedef struct for save mcp configuration
 *
 */
typedef struct {
	uint32_t id;
    char opcode;
    ExpPortWidth portWidth;
    bool intConfigured;
	PinName interrupt;
}ExpObj_t;


/** A MCP23S17 implementation to control register and interrupt via SPI Interface
 *
 * @note Synchronization level: Interrupt safe
 *
 */
class MCP23017 : public GPIOExpansionInterface {

public:

	/** Create an MCP23S17 with creating new SPI object
	 *
	 *  @param sda					SPI MOSI pin
	 *  @param scl 				SPI MSIO pin
	 *  @param address 				SPI SCLK pin
	 *  @param interrupt 		Interrupt pin for interruptControl
	 *  @param pw 					ExpPortWidth for configurating PortWidth parameter
	 */
	 MCP23017(PinName sda,
		 PinName scl,
		 int address,
		 PinName interrupt = NC,
		 ExpPortWidth pw = PW_8);

	/** Create an MCP23S17 with creating new SPI object
	 *
	 *  @param i2c					SPI-pointer to preinitialized SPI-object
	 *  @param address 					SPI CS pin
	 *  @param interrupt 		Interrupt pin for interruptControl
	 *  @param pw 					ExpPortWidth for configurating PortWidth parameter
	 */
	 MCP23017(mbed::I2C *i2c,
		 int address,
		 PinName interrupt = NC,
		 ExpPortWidth pw = PW_8);

	 ~MCP23017();

	 ExpError getRegister(char address, char *data);

	 ExpError getRegister(char address, short *data);

	 ExpError setRegister(char address, char *data);

	 ExpError setRegister(char address, short *data);

	 /** Get the PortWidth of GPIOExpander
 	  *
 	  *  Provide Port Width for iteration throw int return values
 	  *
 	  *  @return				Typedef Enum of ExpPortWidth
 	  */
	 ExpPortWidth getPortWidth(void);

	 /** Get the direction register of expected port
		*
		*  Provide port register to pointer destination.
		*  For ErrorHandling function returns typdef enum ExpError
		*
		*  @param port		Typdef Enum of ExpPortName for selecting port register
		*  @param data		int-pointer to write data throw
		*  @return				Typedef Enum of ExpPortWidth
		*/
	 ExpError getDirection(ExpPortName port, int *data);

	 /** Set the direction register of expected port
 	  *
 	  *  Used to write port register to set direction.
 	  *  For ErrorHandling function returns typdef enum ExpError
 	  *
 	  *  @param port							Typdef Enum of ExpPortName for selecting port register
 	  *  @param directionMask		int for writing direction to port
 	  *  @return									Typedef Enum of ExpPortWidth
 	  */
	 ExpError setDirection(ExpPortName port, int directionMask);

		/** Get the PullUp register of expected port
		 *
		 *  Provide port register to pointer destination.
		 *  For ErrorHandling function returns typdef enum ExpError
		 *
		 *  @param port		Typdef Enum of ExpPortName for selecting port register
		 *  @param mode		PinMode param to select, which mode register should be read
		 *  @param data		int-pointer to write data throw
		 *  @return				Typedef Enum of ExpPortWidth
		*/
	 ExpError getConfigureMode(ExpPortName port, PinMode mode, int *data);

		/** Set the Pullup register of expected port
		 *
		 *  Used to write port register to set pullup.
		 *  For ErrorHandling function returns typdef enum ExpError
		 *
		 *  @param port		Typdef Enum of ExpPortName for selecting port register
		 *  @param mode		PinMode param to select, which mode register should be changed
		 *  @param mask		int to write pullup configuration to port
		 *  @return			Typedef Enum of ExpPortWidth
		 */
	 ExpError setConfigureMode(ExpPortName port, PinMode mode, int pullupMask);

	 /** Get the Interrupt controlling register of expected port
 	  *
 	  *  Provide port register for InterruptEnabling, risingEdge and fallingEdge to pointer destination.
 	  *  For ErrorHandling function returns typdef enum ExpError
 	  *
 	  *  @param port		Typdef Enum of ExpPortName for selecting port register
 	  *  @param data		int-pointer to write data throw. Therefore an int[3]-array will be filled
 	  *  @return				Typedef Enum of ExpPortWidth
 	  */
	 ExpError getInterruptEnable(ExpPortName port, int *data);

	 /** Set the Interrupt register of expected port
 	  *
 	  *  Used to write port register to set InterruptEnabling, risingEdge and fallingEdge.
 	  *  For ErrorHandling function returns typdef enum ExpError
 	  *
 	  *  @param port										Typdef Enum of ExpPortName for selecting port register
 	  *  @param interruptsEnabledMask	int to write InterruptEnable configuration to port
 	  *  @param risingEdgeMask					int to write risingEdge configuration to port
 	  *  @param fallingEdgeMask				int to write fallingEdge configuration to port
 	  *  @return												Typedef Enum of ExpPortWidth
 	  */
	 ExpError setInterruptEnable(ExpPortName port, int interruptsEnabledMask, int risingEdgeMask, int fallingEdgeMask);

	 /** Provide function for Interrupt execution
 	  *
 	  *  Used to select callback functions of attached pins
 	  */
	 void interruptControl(void);

	 /** Get the Input register of expected port
 	  *
 	  *  Provide port register for ChannelInput to pointer destination.
 	  *  For ErrorHandling function returns typdef enum ExpError
 	  *
 	  *  @param port		Typdef Enum of ExpPortName for selecting port register
 	  *  @param data		int-pointer to write data throw
 	  *  @return				Typedef Enum of ExpPortWidth
 	  */
	 ExpError read(ExpPortName port, int *data);

	 /** Set the Ouput register of expected port
	  *
	  *  Used to write port register to set ChannelOutput.
	  *  For ErrorHandling function returns typdef enum ExpError
	  *
	  *  @param port		Typdef Enum of ExpPortName for selecting port register
	  *  @param data		int to write InterruptEnable configuration to port
	  *  @return				Typedef Enum of ExpPortWidth
	  */
	 ExpError write(ExpPortName port, int data);

	 /** Attach selected pin of special port to internal list
 	  *
 	  *  Update attached configuration if available
 	  *  Save function callback and object-identification for InterruptIn to provide callback throw
 	  *  For ErrorHandling function returns typdef enum ExpError
 	  *
 	  *  @param port		Typdef Enum of ExpPortName for selecting port register
 	  *  @param pin		Typdef Enum of ExpPinName for selecting pin
 	  *  @param func		Callback<void(uint32_t,gpio_irq_event)> object for InterruptIn
 	  *  @param id			uint32_t for identification of Interrupt Callback Object
 	  *  @return				Typedef Enum of ExpPortWidth
 	  */
	 ExpError attach(ExpPortName port, ExpPinName pin, Callback<void(uint32_t,gpio_irq_event)> func = NULL, uint32_t id = 0);

	 /** Get state of attachment of pin at port
 	  *
 	  *  Provide boolean, whether pin at port is registered or not
 	  *  For ErrorHandling function returns typdef enum ExpError
 	  *
 	  *  @param port		Typdef Enum of ExpPortName for selecting port register
 	  *  @param pin		Typdef Enum of ExpPinName for selecting pin
 	  *  @return				Boolean of registration state
 	  */
	 bool isAttached(ExpPortName port, ExpPinName pin);

//	 bool isAttached(ExpPortName port);

	 /** Detach selected pin of special port from internal list
    *
    *  Remove attached configuration if available
    *  For ErrorHandling function returns typdef enum ExpError
    *
    *  @param port		Typdef Enum of ExpPortName for selecting port register
    *  @param pin		Typdef Enum of ExpPinName for selecting pin
    *  @return				Typedef Enum of ExpPortWidth
    */
	 ExpError detach(ExpPortName port, ExpPinName pin);

protected:
    void _init();
    void _deinit(void);
    void _write(char regAddress, char data);
    void _write(char regAddress, short data);
    int _read(char regAddress);
    void _read(char regAddress, char *data);
    void _read(char regAddress, short *data);
    void _enableCallback(bool state);
    void _threadControl(void);
    void _interruptControl(void);
    std::list<ExpGPIO_t>::iterator _getExpGPIO(ExpPortName port, ExpPinName pin);
    Thread _intThread;
	volatile bool _threadrunning;
	EventFlags _intFlags;
	ExpObj_t _config;
	mbed::I2C *_i2c;
    int _address;
    bool _created;
    InterruptIn _int;
    std::list<ExpGPIO_t> _intList;
    // Mutex for thread safety
    mutable PlatformMutex _mutex;
};

#endif
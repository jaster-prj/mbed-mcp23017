/* MCP23017 - drive the Microchip MCP23S17 16-bit Port Extender using SPI
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
#include "mbed.h"
#include "mbed_error.h"
#include "../inc/MCP23017.h"

union {
    uint8_t  value8[2];
    uint16_t value16;
} tmp_data;

MCP23017::MCP23017(PinName sda,
		PinName scl,
		int address,
		PinName interrupt,
		ExpPortWidth pw) : _int(interrupt) {
    _address = address;
	_config.id = 0;
	_config.portWidth = pw;
	_config.interrupt = interrupt;
	_i2c = new mbed::I2C(sda, scl);
    _created = true;
    _init();
}

MCP23017::MCP23017(mbed::I2C* i2c,
		int address,
		PinName interrupt,
		ExpPortWidth pw) : _int(interrupt) {
    _address = address;
	_config.id = 0;
	_config.portWidth = pw;
	_config.interrupt = interrupt;
	_i2c = i2c;
    _init();
}

MCP23017::~MCP23017(void) {
	_deinit();
}

void MCP23017::_init(void) {
	if (_config.interrupt!=NC){
	    _write(IOCON, (char)(IOCON_MIRROR | IOCON_INT));
        _enableCallback(true);
        _threadrunning = true;
        _intThread.start(callback(this, &MCP23017::_threadControl));
    	_config.intConfigured = true;
	}
	else{
	    _write(IOCON, (char)(0x0));
	    _config.intConfigured = false;
	}
}

void MCP23017::_deinit(void) {
	setDirection(EXPPORT_A, 0xFF);
	setDirection(EXPPORT_B, 0xFF);
	if (_config.intConfigured == true){
	    _enableCallback(false);
		setInterruptEnable(EXPPORT_A, 0, 0, 0);
		setInterruptEnable(EXPPORT_B, 0, 0, 0);
	    _threadrunning = false;
        _intThread.join();
	}
    if (_created) {
        delete _i2c;
    }
}

void MCP23017::_write(char regAddress, char data) {
    _mutex.lock();    
    char  buffer[2];

    buffer[0] = regAddress;
    buffer[1] = data;
    _i2c->write(_address, buffer, 2);
    _mutex.unlock();
}

void MCP23017::_write(char regAddress, short data) {
    _mutex.lock();

    char  buffer[3];

    buffer[0] = regAddress;
    tmp_data.value16 = data;
    buffer[1] = tmp_data.value8[0];
    buffer[2] = tmp_data.value8[1];

    _i2c->write(_address, buffer, 3);
    _mutex.unlock();
}

int MCP23017::_read(char regAddress) {
    _mutex.lock();
    char buffer[2];

    buffer[0] = regAddress;
    _i2c->write(_address, buffer, 1);
    _i2c->read(_address, buffer, 2);

    _mutex.unlock();
    return ((int)(buffer[0] + (buffer[1]<<8)));
}

void MCP23017::_read(char regAddress, char *data) {
    _mutex.lock();

    _i2c->write(_address, &regAddress, 1);
    _i2c->read(_address, data, 1);

    _mutex.unlock();
}

void MCP23017::_read(char regAddress, short *data) {
    char buffer[2];
    _mutex.lock();
    _i2c->write(_address, &regAddress, 1);
    _i2c->read(_address, buffer, 2);

    _mutex.unlock();
    *data = (short)(buffer[0] + (buffer[1]<<8));
}

void MCP23017::_enableCallback(bool state) {
	if (_config.interrupt!=NC){
		if (state) {
		    _int.rise(callback(this, &MCP23017::interruptControl));
		}
		else{
		    _int.rise(NULL);
		}
	}
}

void MCP23017::_threadControl(void) {
	while(_threadrunning){
		if (_intFlags.wait_all(MCP23017_INTERRUPT_FLAG, 100)){
			_interruptControl();
		}
		if (_config.interrupt!=NC && _int){
			_read(GPIO | 0);
			_read(GPIO | 1);
		}
	}
}

void MCP23017::_interruptControl(void) {
	char intMask = 0;
	char gpio = 0;
	int position = 0;
	gpio_irq_event event;
	std::list<ExpGPIO_t>::iterator it;
	if (_config.intConfigured == true) {
		for(int i = EXPPORT_A; i <= EXPPORT_B; i++) {
			intMask = _read(INTF | i);
			if (intMask != 0){
				for(int j = 0; j < _config.portWidth; j++){
					if (intMask >> j & 0x01){
						position = j;
						break;
					}
				}
				gpio = _read(GPIO | i);
				if ((gpio&intMask) != 0) {
					event = IRQ_RISE;
				}else{
					event = IRQ_FALL;
				}
				if (position != 0){
					it = _getExpGPIO((ExpPortName)i, (ExpPinName)(position));
					if (it != _intList.end() && it->func){
						it->func(it->id,event);
					}
//					_intArray[i<<4 | intMask].func();
					break;
				}
			}
		}
	}
}

std::list<ExpGPIO_t>::iterator MCP23017::_getExpGPIO(ExpPortName port, ExpPinName pin) {
	std::list<ExpGPIO_t>::iterator it;
	if (!_intList.empty()) {
		for(it = _intList.begin(); it != _intList.end(); ++it){
			if (it->pin == pin && it->port == port){
				break;
			}
		}
	}
	return it;
}

ExpError MCP23017::getRegister(char address, char *data) {
	_read(address, data);
	return ExpError_OK;
}

ExpError MCP23017::getRegister(char address, short *data) {
	_read(address, data);
	return ExpError_OK;
}

ExpError MCP23017::setRegister(char address, char *data) {
	_write(address, *data);
	return ExpError_OK;
}

ExpError MCP23017::setRegister(char address, short *data) {
	_write(address, *data);
	return ExpError_OK;
}

ExpPortWidth MCP23017::getPortWidth(void){
	return _config.portWidth;
}

ExpError MCP23017::getDirection(ExpPortName port, int *data) {
	*data = (int)(~(char)(_read(IODIR | (char)port)&0xFF)&0xFF);
	return ExpError_OK;
}

ExpError MCP23017::setDirection(ExpPortName port, int directionMask) {
	if (directionMask > 0xFF) {
		return ExpError_MASK;
	}
    _write(IODIR | (char)port, (char)((~directionMask)&0xFF));
	return ExpError_OK;
}

ExpError MCP23017::getConfigureMode(ExpPortName port, PinMode mode, int *data) {
	if (mode != PullUp) {
		return ExpError_MASK;
	}
	*data = (int)_read(GPPU | (char)port);
	return ExpError_OK;
}

ExpError MCP23017::setConfigureMode(ExpPortName port, PinMode mode, int pullupMask) {
	if (pullupMask > 0xFF || mode != PullUp) {
		return ExpError_MASK;
	}
    _write(GPPU | (char)port, (char)pullupMask);
	return ExpError_OK;
}

ExpError MCP23017::getInterruptEnable(ExpPortName port, int *data) {
	_enableCallback(false);
	*(data+0) = (int)_read(INTCON | (char)port);
	*(data+1) = (int)_read(DEFVAL | (char)port);
	*(data+2) = (int)_read(GPINTEN | (char)port);
    _enableCallback(true);
	return ExpError_OK;
}
ExpError MCP23017::setInterruptEnable(ExpPortName port, int interruptsEnabledMask, int risingEdgeMask, int fallingEdgeMask) {
	if (interruptsEnabledMask > 0xFF || risingEdgeMask > 0xFF || fallingEdgeMask > 0xFF) {
		return ExpError_MASK;
	}
	_enableCallback(false);
	_write(INTCON | (char)port, (char)((char)risingEdgeMask^(char)fallingEdgeMask));
    _write(DEFVAL | (char)port, (char)fallingEdgeMask);
    _write(GPINTEN | (char)port, (char)interruptsEnabledMask);
    _enableCallback(true);
	return ExpError_OK;
}

void MCP23017::interruptControl(void) {
	_intFlags.set(MCP23017_INTERRUPT_FLAG);
}

ExpError MCP23017::read(ExpPortName port, int *data) {
	*data = (int)(_read(GPIO | (char)port)&0xFF);
	return ExpError_OK;
}

ExpError MCP23017::write(ExpPortName port, int data) {
	if (data > 0xFF) {
		return ExpError_MASK;
	}
    _write(GPIO | (char)port, (char)data);
	return ExpError_OK;
}

ExpError MCP23017::attach(ExpPortName port, ExpPinName pin, Callback<void(uint32_t,gpio_irq_event)> func, uint32_t id) {
    if (!((pin <= EXPPIN_7) && (port == EXPPORT_A || port == EXPPORT_B))) {
//    	MBED_ERROR( MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT), "Buffer pointer is Null" );
    	return ExpError_MASK;
    }

    std::list<ExpGPIO_t>::iterator it = _getExpGPIO(port, pin);
    if (!_intList.empty() && it !=  _intList.end()){
    	it->id = id;
    	it->func = func;
    }
    else{
    	ExpGPIO_t temp;
    	temp.id = id;
    	temp.port = port;
    	temp.pin = pin;
    	temp.func = func;
    	_intList.push_back(temp);
    }
	return ExpError_OK;
}

bool MCP23017::isAttached(ExpPortName port, ExpPinName pin) {
    if (!((pin <= EXPPIN_7) && (port == EXPPORT_A || port == EXPPORT_B))) {
    	MBED_ERROR( MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT), "Buffer pointer is Null" );
//    	return ExpError_MASK;
    }
	if(_intList.empty()) {
		return false;
	}
    std::list<ExpGPIO_t>::iterator it = _getExpGPIO(port, pin);
    if (it != _intList.end()){
    	return true;
    }
    return false;
}

ExpError MCP23017::detach(ExpPortName port, ExpPinName pin) {
    if (!((pin <= EXPPIN_7) && (port == EXPPORT_A || port == EXPPORT_B))) {
//    	MBED_ERROR( MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT), "Buffer pointer is Null" );
    	return ExpError_MASK;
    }
    if (isAttached(port, pin)){
    	std::list<ExpGPIO_t>::iterator it;
    	for(it = _intList.begin(); it != _intList.end(); ++it){
    		if (it->pin == pin && it->port == port){
    			it = _intList.erase(it);
    			break;
    		}
    	}
        return ExpError_OK;
    }
	return ExpError_NOTINITIALIZED;
}

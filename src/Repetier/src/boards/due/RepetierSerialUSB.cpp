/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.


    Main author: repetier

*/

/**
 * RepetierSerialDue.cpp - Hardware serial library for Arduino DUE
 * Copyright (c) 2017 Eduardo José Tagle. All right reserved
 * Modified for repetier 2021 Roland Littwin
 */
#include "Repetier.h"

// Imports from Atmel USB Stack/CDC implementation
extern "C" {
bool usb_task_cdc_isenabled();
bool usb_task_cdc_dtr_active();
bool udi_cdc_is_rx_ready();
int udi_cdc_getc();
bool udi_cdc_is_tx_ready();
int udi_cdc_putc(int value);
};

// Pending character
static int pending_char = -1;

// Public Methods
void RepetierSerialUSB::begin(const long) { }

void RepetierSerialUSB::end() { }

int RepetierSerialUSB::peek() {
    if (pending_char >= 0) {
        return pending_char;
    }
    // If USB CDC not enumerated or not configured on the PC side
    if (!usb_task_cdc_isenabled()) {
        return -1;
    }
    // If no bytes sent from the PC
    if (!udi_cdc_is_rx_ready()) {
        return -1;
    }
    pending_char = udi_cdc_getc();

    return pending_char;
}

int RepetierSerialUSB::read() {
    if (pending_char >= 0) {
        int ret = pending_char;
        pending_char = -1;
        return ret;
    }

    // If USB CDC not enumerated or not configured on the PC side
    if (!usb_task_cdc_isenabled()) {
        return -1;
    }
    // If no bytes sent from the PC
    if (!udi_cdc_is_rx_ready()) {
        return -1;
    }

    int c = udi_cdc_getc();

    return c;
}

int RepetierSerialUSB::available() {
    /* If Pending chars */
    return pending_char >= 0 ||
        /* or USB CDC enumerated and configured on the PC side and some
       bytes where sent to us */
        (usb_task_cdc_isenabled() && udi_cdc_is_rx_ready());
}

void RepetierSerialUSB::flush() { }

size_t RepetierSerialUSB::write(const uint8_t c) {

    /* Do not even bother sending anything if USB CDC is not enumerated
     or not configured on the PC side or there is no program on the PC
     listening to our messages */
    if (!usb_task_cdc_isenabled() || !usb_task_cdc_dtr_active())
        return 0;

    /* Wait until the PC has read the pending to be sent data */
    while (usb_task_cdc_isenabled() && usb_task_cdc_dtr_active() && !udi_cdc_is_tx_ready()) {
    };

    /* Do not even bother sending anything if USB CDC is not enumerated
     or not configured on the PC side or there is no program on the PC
     listening to our messages at this point */
    if (!usb_task_cdc_isenabled() || !usb_task_cdc_dtr_active())
        return 0;

    // Fifo full
    //  udi_cdc_signal_overrun();
    udi_cdc_putc(c);
    return 1;
}

RepetierSerialUSB SerialUSB;

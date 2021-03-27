/**
 * \file
 *
 * \brief USB configuration file
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef _CONF_USB_H_
#define _CONF_USB_H_

#define USB_DEVICE_VENDOR_ID 0x03EB  /* ATMEL VID */
#define USB_DEVICE_PRODUCT_ID 0x2424 /* MSC / CDC */

#undef UNUSED /* To avoid a macro clash as macros.h already defines it */
#include "compiler.h"

/**
 * USB Device Configuration
 * @{
 */

//! Device definition (mandatory)
#define USB_DEVICE_MAJOR_VERSION 1
#define USB_DEVICE_MINOR_VERSION 0
#define USB_DEVICE_POWER 100 // Consumption on Vbus line (mA)
#define USB_DEVICE_ATTR \
    (USB_CONFIG_ATTR_SELF_POWERED)
// (USB_CONFIG_ATTR_BUS_POWERED)
//  (USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_SELF_POWERED)
//  (USB_CONFIG_ATTR_REMOTE_WAKEUP|USB_CONFIG_ATTR_BUS_POWERED)

/**
 * Device speeds support
 * Low speed not supported by CDC and MSC
 * @{
 */

//! To define a Low speed device
//#define  USB_DEVICE_LOW_SPEED

//! To define a Full speed device
#define USB_DEVICE_FULL_SPEED

//! To authorize the High speed
#ifndef USB_DEVICE_FULL_SPEED
#if (UC3A3 || UC3A4)
#define USB_DEVICE_HS_SUPPORT
#elif (SAM3XA || SAM3U)
#define USB_DEVICE_HS_SUPPORT
#endif
#endif
//@}

/**
 * USB Device Callbacks definitions (Optional)
 * @{
 */
#define UDC_VBUS_EVENT(b_vbus_high)
#define UDC_SOF_EVENT()
#define UDC_SUSPEND_EVENT()
#define UDC_RESUME_EVENT()
#define UDC_GET_EXTRA_STRING() usb_task_extra_string()
#define USB_DEVICE_SPECIFIC_REQUEST() usb_task_other_requests()
//@}

//@}

//@}

/**
 * USB Interface Configuration
 * @{
 */
/**
 * Configuration of CDC interface
 * @{
 */

//! Define one USB communication ports
#define UDI_CDC_PORT_NB 1

//! Interface callback definition
#define UDI_CDC_ENABLE_EXT(port) usb_task_cdc_enable(port)
#define UDI_CDC_DISABLE_EXT(port) usb_task_cdc_disable(port)
#define UDI_CDC_RX_NOTIFY(port) usb_task_cdc_rx_notify(port)
#define UDI_CDC_TX_EMPTY_NOTIFY(port)
#define UDI_CDC_SET_CODING_EXT(port, cfg) usb_task_cdc_config(port, cfg)
#define UDI_CDC_SET_DTR_EXT(port, set) usb_task_cdc_set_dtr(port, set)
#define UDI_CDC_SET_RTS_EXT(port, set)

//! Define it when the transfer CDC Device to Host is a low rate (<512000 bauds)
//! to reduce CDC buffers size
//#define  UDI_CDC_LOW_RATE

//! Default configuration of communication port
#define UDI_CDC_DEFAULT_RATE 115200
#define UDI_CDC_DEFAULT_STOPBITS CDC_STOP_BITS_1
#define UDI_CDC_DEFAULT_PARITY CDC_PAR_NONE
#define UDI_CDC_DEFAULT_DATABITS 8

//! Enable id string of interface to add an extra USB string
#define UDI_CDC_IAD_STRING_ID 4

#include "udi_cdc_conf.h"

#include "usb_task.h"

#endif // _CONF_USB_H_

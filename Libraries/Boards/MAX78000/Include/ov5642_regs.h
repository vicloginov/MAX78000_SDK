/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*
******************************************************************************/
#ifndef __OV5642_REGS_H__
#define __OV5642_REGS_H__

#define OV5642_I2C_SLAVE_ADDR   0x3C // 0x78>>1

#define OV5642_CHIPID_HIGH  0x300a
#define OV5642_CHIPID_LOW   0x300b

#define OV5642_ID           0x5642
#define SYS_IO_PAD_CNTRL_REG_START (0x3000)

#define SYS_CNTRL_REG   (0x3008)
#define RESET_MODE      0x80
#define SLEEP_MODE      0x40

#define TIME_CNTRL_18_REG   (0x3818)
#define HORIZONTAL_MIRROR   0x40
#define VERTICAL_FLIP       0x20
#define THUMBNAIL           0x10
#define COMPRESSION         0x08

#define ARRAY_CTRL_01_REG   (0x3621)


// Special effects

typedef enum {
    NORMAL = 0,
    BLUE,
    GREEN,
    RED,
    BLACK_WHITE,
    NEGATIVE,
    GRAY,
    SEPIA
} ov5642_effect_t;

#endif //__OV5642_H__

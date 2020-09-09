#ifndef _headfile_h
#define _headfile_h

#include <stdarg.h>
#include <stdio.h>
#include <SKEAZ1284.h>
#include "sysinit.h"
#include "systick.h"
#include "common.h"
#include "ics.h"
#include "rtc.h"
#include "io.h"
#include "uart.h"
#include "sim.h"
#include "gpio.h"       //IO口操作
#include "pit.h"
#include "adc.h"
#include "uart.h"
#include "ftm.h"
#include "kbi.h"
#include "i2c.h"
#include "spi.h"
#include "adc.h"
#include "acmp.h"
#include "mscan.h"
#include "pwt.h"
#include "wdog.h"
#include "pmc.h"
#include "adc.h"
#include "adc.h"


//用户自定义头文件   
#include "LQ12864.h"
#include "isr.h"   
#include "LQLED.h"
#include "LQKEY.h"  
#include "LQ9AX.h" 
#include "LQI2C.h"
#include "MPUIIC.h"
#include "MPU6050.h"
#include "Serial_oscilloscope.h"
#include "8700_2100.h"

//新添头文件
#include "Parameter.h"
#include "Control.h"
#include "Init.h"
#include "Event.h"
#include "newFlash.h"


#endif
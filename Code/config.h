/*
 * File:   config.h
 * Author: Syed Tahmid Mahbub
 *
 * Created on October 10, 2014
 */

#ifndef CONFIG_H
#define	CONFIG_H
#define _SUPPRESS_PLIB_WARNING 
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#define _SUPPRESS_PLIB_WARNING 
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#include "plib.h"
// serial stuff
#include <stdio.h>

#define	SYS_FREQ 40000000

#pragma config FNOSC = FRCPLL, POSCMOD = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20
#pragma config FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF, JTAGEN = OFF, FSOSCEN = OFF

#endif	/* CONFIG_H */


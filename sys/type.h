/*
*****************************************************************************
*				All rights reserved.
*
* Filename:			type.h
* Description:		type head file
*
* Change History:
*			Goldjun     -- 06/17'2012 - Ver0.1
*			            -- created
*
*
*
*
******************************************************************************
*/
#ifndef __TYPE_H__
#define __TYPE_H__

//#define MCU_51
//#define MCU_PIC
#define MCU_STM8


#ifdef	MCU_51
#include <reg51.h>
#endif

#ifdef	MCU_PIC
#include <htc.h>
#endif

#ifdef	MCU_STM8
#include "stm8s.h"
#include "stm8s_gpio.h"
#endif



#define	HIGH			    1			
#define	LOW			        0

#define				        IN
#define				        OUT

#define	BYTE_HIGH		    0			
#define	BYTE_LOW		    1	


#define SetGpioRegBit(reg,mask)		((reg) |= (mask))
#define ClrGpioRegBit(reg,mask)		((reg) &= (~(mask)))
#define SetGpioReg(reg,mask)		(reg = mask)
#define GetGpioReg(reg)			    ((reg))

typedef void			    VOID, *PVOID;
typedef char			    BOOL;
//typedef bit			    BOOLEAN;
typedef unsigned char		BYTE,MESSAGE;
typedef unsigned short		WORD;
typedef unsigned long		DWORD;

typedef signed char 		SBYTE;
typedef signed int			SWORD;
typedef signed long			SDWORD;

#endif


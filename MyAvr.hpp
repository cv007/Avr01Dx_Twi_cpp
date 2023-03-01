#pragma once
//======================================================================
//  MyAvr.hpp - global includes/defines, etc.
//======================================================================
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/interrupt.h>

//======================================================================
//  global F_CPU for delays, or any timing calculations
//======================================================================
#define F_CPU 3333333ul
#include <util/delay.h>

using u8  = uint8_t;
using i8  = int8_t;
using u16 = uint16_t;
using i16 = int16_t;
using u32 = uint32_t;
using i32 = int32_t;

//======================================================================
//  defines for TWI use- enables master/slave isr in TwiISR.cpp
//======================================================================
#define TWIM0_ISR_ENABLE 1
#define TWIS0_ISR_ENABLE 1
#if defined(TWI1)
#define TWIM1_ISR_ENABLE 0
#define TWIS1_ISR_ENABLE 0
#endif

//======================================================================
//  common utility functions
//======================================================================
template<typename T, int N>
int arraySize(T(&t)[N]){ (void)t; return N; }

//======================================================================
//  TwiISR.cpp
//======================================================================
#include "MyAvr.hpp"
#include "Twis.hpp"
#include "Twim.hpp"

#if defined(TWIM0_ISR_ENABLE) && TWIM0_ISR_ENABLE
ISR             (TWI0_TWIS_vect) { Twis::isr(0);} //0 = TWI0
#endif
#if defined(TWIS0_ISR_ENABLE) && TWIS0_ISR_ENABLE
ISR             (TWI0_TWIM_vect) { Twim::isr(0); }
#endif


#if defined( TWI1 )

#if defined(TWIM1_ISR_ENABLE) && TWIM1_ISR_ENABLE
ISR             (TWI1_TWIS_vect) { Twis::isr(1);} //1 = TWI1
#endif
#if defined(TWIM1_ISR_ENABLE) && TWIM1_ISR_ENABLE
ISR             (TWI1_TWIM_vect) { Twim::isr(1); }
#endif

#endif

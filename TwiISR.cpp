//======================================================================
//  TwiISR.cpp
//======================================================================
#include "MyAvr.hpp"
#include "Twis.hpp"
#include "Twim.hpp"

//call the static function isr with the instance number
//the Twix class will use the instance number to lookup
//the object, then call the object's isr_ function
//(default instance number is 0)

//this method is used so the class is not required to be
//all static, and allows the same use when the mcu has
//more than one twi instance


#if defined(TWIM0_ISR_ENABLE) && TWIM0_ISR_ENABLE
ISR             (TWI0_TWIS_vect) { Twis::isr();} //TWI0
#endif
#if defined(TWIS0_ISR_ENABLE) && TWIS0_ISR_ENABLE
ISR             (TWI0_TWIM_vect) { Twim::isr(); }
#endif


#if defined( TWI1 )

#if defined(TWIM1_ISR_ENABLE) && TWIM1_ISR_ENABLE
ISR             (TWI1_TWIS_vect) { Twis::isr(1);} //1 = TWI1
#endif
#if defined(TWIM1_ISR_ENABLE) && TWIM1_ISR_ENABLE
ISR             (TWI1_TWIM_vect) { Twim::isr(1); }
#endif

#endif

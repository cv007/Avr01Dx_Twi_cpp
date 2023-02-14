//======================================================================
//  TwiISR.cpp
//======================================================================
#include "MyAvr.hpp"
#include "Twis.hpp"
#include "Twim.hpp"

//change define names as needed to match instance created elsewhere, 
//or comment out either if not being used
//most mcu's will only have TWI0

        //TWI0            [name]
        #define TWIS0_NAME twis0 //slave instance name
        #define TWIM0_NAME twim0 //master instance name

                #if defined(TWIS0_NAME)
                extern Twis TWIS0_NAME;                
ISR             (TWI0_TWIS_vect) { TWIS0_NAME.isr();}
                #endif

                #if defined(TWIM0_NAME)
                extern Twim TWIM0_NAME;
ISR             (TWI0_TWIM_vect) { TWIM0_NAME.isr(); }
                #endif



        #if defined( TWI1 )

        //TWI1            [name]
        #define TWIS1_NAME twis1 //slave instance name
        #define TWIM1_NAME twim1 //master instance name

                #if defined(TWIS1_NAME)
                extern Twis TWIS1_NAME;
ISR             (TWI1_TWIS_vect) { TWIS1_NAME.isr(); }
                #endif

                #if defined(TWIM1_NAME)
                extern Twim TWIM1_NAME;
ISR             (TWI1_TWIM_vect) { TWIM1_NAME.isr(); }
                #endif

        #endif

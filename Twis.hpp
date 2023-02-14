#pragma once
//======================================================================
//  Twis.hpp
//======================================================================
#include "MyAvr.hpp"
#include "TwiPins.hpp"

/*------------------------------------------------------------------------------
    Twis.hpp - Twi slave

    1. uncomment the appropriate set of pins for your mcu
        in TwiPins.hpp to select the twi pins to use
    2. create a class instance (name of your choosing)
        Twis twis0; //if only TWI0 available, no argument needed
        Twis twis1{ TWI1 }; //else need to specify which peripheral instance
    3. in TwiISR.cpp, modify/match the appropriate define using the instance
        name you created in step 2
    3. turn on with address and callback function
        twis0.on( 0x40, myCallback );
    4. enable interrupts via sei() (avr/interrupts.h)

    optional - set a 2nd address, or an address mask, can be set at anytime


    callback function, isr provides twi instance, state and status register
    (status register can be used get more info when in TWIS_ERROR state)

    bool myCalback(Twis& twis, Twis::IRQ_STATE state, u8 statusReg){
        you have the enum states to deal with as needed
        return true if everything ok, false if you want to stop the transaction
        you have 3 functions to use with twis0-
            twis.lastAddress() - when in TWIS_ADDRESSED, this will give address
                               seen to get here (could be 0, address, address2,
                               or a mask match)
            twis.write() - when in TWIS_MREAD, you can reply with a write
            twis.read() - when in TWIS_MWRITE, you can read what was sent
    }

    NOTE: gencall is enabled by default, so check the address in the callback
          when in TWIS_ADDRESSED state (simply enabled to eliminate one more
          option to set- most likely never seen but if so you can ignore in the
          callback by only returning true when dealing with an address you
          want to respond to)

          FM+ mode is always used but if do not want it you can modify
          the initPins() and on() functions


public enum, callback type, functions-

enum
IRQ_STATE       { TWIS_ADDRESSED, TWIS_MREAD, TWIS_MWRITE, TWIS_STOPPED, TWIS_ERROR };
                using
CallbackT       = bool(*)(Twis& twi, IRQ_STATE state, u8 statusReg);

Twis            (TWI_t); //constructor argument required if mcu has more than single instance of TWI
                void
on              (u8 SlaveAddress, twis_callback_t callbackFunction);
                void
off             ();
                void
write           (u8 value);
                u8
read            ();
                u8
lastAddress     ();
                void
address2        (u8 SlaveAddress2);
                void
addressMask     (u8 SlaveAddressMask); //no 2nd address when using this option

------------------------------------------------------------------------------*/


//======================================================================
//   Twis - Twi slave
///======================================================================
                //declare isr functions with C linkage so Twim class can
                //friend them and give access to the private isr function
                extern "C" void TWI0_TWIS_vect();
                #if defined(TWI1)
                extern "C" void TWI1_TWIS_vect();
                #endif

class
Twis            {

    public:

                enum
IRQ_STATE       { TWIS_ADDRESSED, TWIS_MREAD, TWIS_MWRITE, TWIS_STOPPED, TWIS_ERROR };

                using
CallbackT       = bool(*)(Twis& twi, IRQ_STATE state, u8 statusReg);

    private:
                volatile u8 lastAddress_; //last address we responded as
                CallbackT   isrFuncCallback_;
                #if defined(TWI1)           //more than 1 twi instance available
                TWI_t&      twi_;           //so contructor will set which instance
                #else                       //only 1 instance, can optimize by setting
                static inline               //(create inside header w/C++17)
                TWI_t&      twi_{ TWI0 };   //the fixed value here
                #endif

                auto
address1        (u8 v) { twi_.SADDR = (v<<1) bitor 1; } //gencall enabled, so check address in callback
                auto
address2        (u8 v, bool nomask) { twi_.SADDRMASK = (v<<1) bitor nomask; }
                auto
off_            () { twi_.SCTRLA and_eq compl 1; }
                auto
on_             ()
                { //if master/slave pins are the same, not in dual mode so set FMPEN in CTRLA
                  //if not the same, initPins already took care of FMPEN in the DUALCTRL register
                if( twi0_pins.MpinSCL == twi0_pins.SpinSCL ) twi_.CTRLA or_eq 2;
                twi_.SCTRLA or_eq 1;
                }
                auto
irqAllOn        () { twi_.SCTRLA or_eq 0xE0; }
                auto
irqAllOff       () { twi_.SCTRLA and_eq compl 0xE0; }
                static u8
status          () { return twi_.SSTATUS; }
                auto
clearFlags      () { twi_.SSTATUS = 0xCC; }
                auto
nackComplete    () { twi_.SCTRLB = 6; } //COMPTRANS, NACK
                auto
ack             () { twi_.SCTRLB = 3; } //RESPONSE, ACK

                //local enum
                //DIF:APIF:CLKHOLD:RXACK:COLL:BUSERR:DIR:AP
                enum { DIF_DIRbm = 0x82, APIF_APbm = 0x41, RXNACKbm = 0x10, ERRbm = 0x0C,
                       DIF_R = 0x82, DIF_W = 0x80, APIF_ADDR = 0x41, APIF_STOP = 0x40 };

                //v = a copy of SSTATUS (used in isr)
                auto
isDataRead      (u8 v) { return (v bitand DIF_DIRbm) == DIF_R; }         //DIF, DIR(1=R)
                auto
isDataWrite     (u8 v) { return (v bitand DIF_DIRbm) == DIF_W; }         //DIF, DIR(0=W)
                auto
isAddress       (u8 v) { return (v bitand APIF_APbm) == APIF_ADDR; }     //APIF, AP(1=addr)
                auto
isStop          (u8 v) { return (v bitand APIF_APbm) == APIF_STOP; }     //APIF, AP(0=stop)
                auto
isRxNack        (u8 v) { return (v bitand RXNACKbm); }                   //RXACK(0=ACK,1=NACK)
                auto
isError         (u8 v) { return (v bitand ERRbm); }                      //COLL,BUSERR

                auto
initPins        ()
                {
                uint8_t
                    scl = twi0_pins.SpinSCL & 7,        //extract all values for easier use/reading
                    sca = twi0_pins.SpinSCA & 7,
                    clrbm = ~twi0_pins.pmux_clrbm,      //inverted for bitand use
                    setbm = twi0_pins.pmux_setbm;
                volatile uint8_t *pinctrl = &twi0_pins.Sport->PIN0CTRL;
                volatile uint8_t *pmux = twi0_pins.pmux;

                //enable pullups and set portmux as needed (some have no alt pins, so no twi portmux)
                pinctrl[scl] or_eq PORT_PULLUPEN_bm;
                pinctrl[sca] or_eq PORT_PULLUPEN_bm;
                if( pmux ) *pmux = (*pmux bitand clrbm) bitor setbm; //compiler will optimize if bitfield is a single bit

                //turn on dual mode if master/slave pins are not the same
                //will assume the user has setup twi0_pins properly, so wants to use dual mode
                //not all mcu's have dual mode, so using offset from CTRLA to access DUALCTRL so
                //can compile for all mcu's
                if( twi0_pins.MpinSCL != twi0_pins.SpinSCL ) (&twi_.CTRLA)[1] or_eq 1; //dual enable
                }

                //friend isr() so ISR in TwiISR.cpp can access this private function
                //if TWI1 available, friend it also (even if unused)
                friend void TWI0_TWIS_vect();
                #if defined(TWI1)
                friend void TWI1_TWIS_vect();
                #endif

                //callback function returns true if want transaction to proceed
                auto
isr             ()
                {
                static bool is1st;      //so can ignore rxack on first master read
                u8 s = status();        //get a copy of status
                IRQ_STATE state =       isError(s)     ? TWIS_ERROR : //do first
                                        isStop(s)      ? TWIS_STOPPED :
                                        isAddress(s)   ? TWIS_ADDRESSED :
                                        isDataRead(s)  ? TWIS_MREAD :
                                        isDataWrite(s) ? TWIS_MWRITE : TWIS_ERROR;
                bool nacked = isRxNack(s);
                bool done = false; //assume not done

                if( state == TWIS_ADDRESSED ) {
                    lastAddress_ = read()>>1;
                    is1st = true;
                    }
                else if( state == TWIS_MREAD ) {
                    if( is1st ) is1st = false; else done = nacked;
                    }
                else if( state != TWIS_MWRITE ) done = true; //error or stopped

                if( false == isrFuncCallback_(*this, state, s) ) done = true;
                done ? nackComplete() : ack();
                }

public:

                #if defined(TWI1)                   //more than 1 twi instance available
Twis            (TWI_t& twi = TWI0) : twi_(twi){}   //specify when creating instance (default is TWI0)
                #else                               //else twi_ is already set to TWI0
Twis            (TWI_t& twi = TWI0){ (void)twi; }   //provide a constructor that does nothing so if user
                #endif                              //does create with TWI0, it will be harmless

                auto
on              (u8 addr, CallbackT cb)
                {
                if( ! cb ) return;                  //do not accept callback set to 0
                isrFuncCallback_ = cb;
                off_();                             //will also clear flags
                initPins();                         //init pins
                address1( addr );
                irqAllOn();
                on_();
                }
                auto
off             ()
                {
                irqAllOff();
                off_();
                }
                auto
write           (u8 v) { twi_.SDATA = v; }
                u8
read            () { return twi_.SDATA; }
                u8
lastAddress     () { return lastAddress_; }         //last address we responded to
                auto
address2        (u8 v) { address2(v, true); }       //2nd address
                auto
addressMask     (u8 v) { address2(v, false); }      //address mask (no 2nd address)

                }; // Twis class



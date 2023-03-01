#pragma once
//======================================================================
//  Twim.hpp - Twi master - avr mega0, tiny0/1, da, etc.
//======================================================================
#include "MyAvr.hpp"
#include "TwiPins.hpp"


/*------------------------------------------------------------------------------
    * in MyAvr.hpp, enable the isr via the define-
        #define TWIM0_ISR_ENABLE 1
    
    * add TwiISR.cpp to the project
    
    * include this header-
        #include "Twim.hpp"

    * uncomment/create the appropriate set of pins for your mcu
      in twiPins.h to allow the use of the twi pins (name given not important)
        static const TwiPinsT TWI0_pins = { ... };
    * create a class instance (name of your choosing), specify pins to use
      and twi instance if mcu has TWI1 available
        Twim twim0{ TWI0_pins }; //if only TWI0 available, no need to specify
        Twim twim1{ TWI0_pins, TWI1 }; //else need to specify which peripheral instance
    * set baud
        twim0.baud( 100000ul ); //100kHz (optional second arg- cpu speed, defaults to F_CPU)
    * turn on, specifying slave address
        twim0.on( 0x44 );
    * enable interrupts via sei() (avr/interrupts.h)


    optionally set a callback function if not polling via twim0.waitUS()
        twim0.callback( myCallback );

    you can now use one of four functions to move data-

    twim0.writeRead - write wbuffer of wN length, read into rbuffer of rN length
    twim0.writeWrite - write wbuffer of wN length, write wbuffer2 of w2N length
    twim0.write - write wbuffer of wN length (alias to writeRead with no read)
    twim0.read - read into rbuffer of rN length (alias to writeRead with no write)

    if not using a callback function, you can poll for completion-

        twim0.baud( 100000ul );             //second rgument defaults to F_CPU
        twim0.on( 0x44 );
        sei();

        u8 wbuf[1] = { 0x55 };              //command to read 4 bytes, as an example
        u8 rbuf[4];                         //no need to clear/init read buffer
        twim0_writeRead( wbuf, 1, rbuf, 4 );//write 1 byte (0x55), read 4 bytes

        //blocking until done or a timeout (us)
        if( twim0_waitUS(3000) ){           //result ok, rbuf has 4 bytes
            }
        else if( twim0.isBusy() ){          //was timeout, (twim irqs may still be on)
            twim0.busRecovery();            //can do bus recovery if wanted
            }
        else {                              //was nack'd or bus error/collision (twim irqs are off)
            }

        twim0.off();

        NOTE: FM+ mode is always used but if do not want it you can modify
              the on_() function


public type, functions-

                using
CallbackT       = void (*)(Twim&);

                void
on              (u8 address);
                void
off             ();
                bool
isBusy          ();
                bool
resultOK        ();
                void
callback        (CallbackT callbackFunction);
                void
writeRead       (const u8* writeBuffer, u16 writeLength, u8* readBuffer, u16 readLength);
                void
writeWrite      (const u8* writeBuffer, u16 writeLength, const u8* writeBuffer2, u16 writeLength2);
                void
write           (const u8* writeBuffer, u16 writeLength);
                void
read            (u8* readBuffer, u16 readLength);
                bool
waitUS          (u16 microseconds);
                void
busRecovery     ();
                void
baud            (uint32_t twiHz, uint32_t cpuHz = F_CPU)

------------------------------------------------------------------------------*/


//if the isr is not enabled via defines, do not allow this class
//to be used as it depends on the isr to function
#if defined(TWIM0_ISR_ENABLE) && TWIM0_ISR_ENABLE
//======================================================================
//  Twim master
//======================================================================
                //declare isr functions with C linkage so Twim class can
                //friend them and give access to the private isr function
                extern "C" void TWI0_TWIM_vect();
                #if defined(TWI1)
                extern "C" void TWI1_TWIM_vect();
                #endif

class Twim      {

    public:
                using               //pass reference to Twim so callback
CallbackT       = void (*)(Twim&);  //has class instance without the need
                                    //to know the class object name
    private:
                CallbackT       isrFuncCallback_;
                const u8*       txbuf_;     //we do not write to tx buffer(s)
                const u8*       txbufEnd_;
                const u8*       txbuf2_;
                const u8*       txbuf2End_;
                u8*             rxbuf_;     //we need to write to rx buffer
                const u8*       rxbufEnd_;
                volatile bool   lastResult_; //1=ok,0=fail
                const TwiPinsT& pins_;

                #if defined(TWI1)           //more than 1 twi instance available
                TWI_t&          twi_;       //so contructor will set twi instance
                static inline Twim* instance_[2];
                #else                       //only 1 instance, can optimize by setting
                static inline               //create inside header w/C++17 (-std=c++17)
                TWI_t&          twi_{ TWI0 }; //the twi_ reference value here
                static inline Twim* instance_[1];
                #endif

                //local enums

                //MCTRLB flush3|ack2|cmd1:0
                enum { ACK = 0, READ = 2, STOP = 3, NACK = 4,  FLUSH = 8 };
                //MSTATUS RIF7|WIF6|CLKHOLD5|RXACK4|ARBLOST3|BUSERR2|BUSSTATE1:0
                enum { RIF = 0x80, WIF = 0x40, CLKHOLD = 0x20, RXNACK = 0x10, ARBLOST = 0x8, BUSERR = 0x4 };
                enum { ALLFLAGS = RIF|WIF|CLKHOLD|ARBLOST|BUSERR };
                enum { ANYERR = ARBLOST|BUSERR }; //error bits
                enum { RIEN = RIF, WIEN = WIF, RWIEN = RIEN|WIEN }; //irq bits
                enum { RW = 1 }; //address bit0
                enum { UNKNOWN = 0, IDLE, OWNER, BUSBUSY, BUSMASK = 3 }; //bus state
                enum { READOK = RIF|CLKHOLD|OWNER, WRITEOK = WIF|CLKHOLD|OWNER };
                enum { ENABLE = 1, FMPEN = 2 }; //on/off, FM+ enable

                auto
enable          ()
                {
                twi_.CTRLA or_eq FMPEN; //FM+ enable
                twi_.MCTRLA or_eq ENABLE; //twim enable
                }
                auto
disable         () { twi_.MCTRLA = 0; }
                auto
irqAllOn        () { twi_.MCTRLA or_eq RWIEN; }
                auto
irqAllOff       () { twi_.MCTRLA and_eq compl RWIEN; }
                auto
toStateIdle     () { twi_.MSTATUS = ALLFLAGS bitor IDLE; } //clear flags, set to IDLE
                auto
ackActionACK    () { twi_.MCTRLB = ACK; }
                auto
ACKread         () { twi_.MCTRLB = READ; }
                auto
NACKstop        () { twi_.MCTRLB = NACK bitor STOP; }
                auto
address         (u8 v) { disable(); twi_.MADDR = v<<1; } //off so no start produced
                auto
startRead       () { ackActionACK(); twi_.MADDR or_eq RW; } //reuse existing address
                auto
startWrite      () { twi_.MADDR and_eq compl RW; } //reuse existing address
                auto
write1          (u8 v) { twi_.MDATA = v; }
                auto
read1           () { return twi_.MDATA; }
                auto
status          () { return twi_.MSTATUS; }

                auto
startIrq        (bool wr) //start a read (wr=0) or write (wr=1), enable irq
                {
                lastResult_ = false;
                wr ? startWrite() : startRead();
                irqAllOn();
                }

                auto
finished        (bool tf) //for isr use, tf=true if success
                {
                lastResult_ = tf;
                //NACKstop works for write also (nack not done, harmless)
                NACKstop();
                irqAllOff(); //do before callback in case call back starts another xfer
                if( isrFuncCallback_ ) isrFuncCallback_( *this );
                }

                auto
initPins        ()
                {
                uint8_t
                    scl = pins_.MpinSCL & 7,    //extract all values for easier use/reading
                    sca = pins_.MpinSCA & 7,
                    clrbm = ~pins_.pmux_clrbm,  //inverted for bitand use
                    setbm = pins_.pmux_setbm;
                volatile uint8_t *pinctrl = &pins_.Mport->PIN0CTRL;
                volatile uint8_t *pmux = pins_.pmux;

                //enable pullups and set portmux as needed (some have no alt pins, so no twi portmux)
                pinctrl[scl] = PORT_PULLUPEN_bm; //assignment, will set all other bits to 0
                pinctrl[sca] = PORT_PULLUPEN_bm; // if need invert or isc bits for some reason, change to |=
                if( pmux ) *pmux = (*pmux bitand clrbm) bitor setbm; //compiler will optimize if bitfield is a single bit
                }


                //friend isr() so ISR in TwiISR.cpp can access this private function
                //if TWI1 available, friend it also (even if unused)
                friend void TWI0_TWIM_vect();
                #if defined(TWI1)
                friend void TWI1_TWIM_vect();
                #endif

                void
isr_            ()
                {
                u8 s = status();
                //error
                if( s & ANYERR ) return finished( false );
                //read
                if( s == READOK ){
                    *rxbuf_++ = read1();
                    return rxbuf_ < rxbufEnd_ ? ACKread() : finished( true );
                    }
                //write
                if( s == WRITEOK ){
                    if( txbuf_ < txbufEnd_ ) return write1( *txbuf_++ ); //more data
                    if( txbuf2_ < txbuf2End_ ) return write1( *txbuf2_++ ); //more data
                    return rxbuf_ ? startRead() : finished( true ); //switch to read? or done
                    }
                //unknown, or a write nack
                finished( false );
                }

                //static function so C isr function can call function without object
                //we will lookup the object and call the isr_() function
                static void
isr             (u8 n = 0){ instance_[n]->isr_(); }

    public:

                #if defined(TWI1)                   //more than 1 twi instance available
Twim            (const TwiPinsT& pins, TWI_t& twi = TWI0)
                : twi_(twi)
                {
                initPins();
                instance_[twi == TWI0 ? 0 : 1] = this;
                }
                #else                               //else twi_ is already set to TWI0
Twim            (const TwiPinsT& pins, TWI_t& twi = TWI0)
                : pins_(pins)
                {
                (void)twi;
                initPins();
                instance_[0] = this;
                }
                #endif                              //can also specify TWI0, not needed but harmless

                auto
callback        (CallbackT cb) { isrFuncCallback_ = cb; } //optional, else use waitUS
                void
off             () { disable(); }
                auto
on              (u8 addr)
                {
                address(addr);
                toStateIdle();
                enable();
                }
                auto
isBusy          () { return twi_.MCTRLA bitand RWIEN; } //if irq on, is busy
                auto
resultOK        () { return lastResult_; }


                //write+read (or write only, or read only)
                //pointers with user provided lengths
                auto
writeRead       (const u8* wbuf, u16 wn, u8* rbuf, u16 rn)
                {
                txbuf_ = wbuf; txbufEnd_ = &wbuf[wn];
                rxbuf_ = rbuf; rxbufEnd_ = &rbuf[rn];
                txbuf2_ = 0; txbuf2End_ = 0;
                startIrq( wn ); //if no write (wn==0), then will start a read irq
                }
                //references with template where length is deduced from type
                template<int Nw, int Nr> auto
writeRead       (const u8 (&wbuf)[Nw], u8(&rbuf)[Nr]) { writeRead( wbuf, Nw, rbuf, Nr ); }
                //write 1 byte, read >1 bytes
                template<int Nr> auto
writeRead       (const u8& wbyte, u8(&rbuf)[Nr]) { writeRead( &wbyte, 1, rbuf, Nr ); }
                //write >1 bytes, read 1 byte
                template<int Nw> auto
writeRead       (const u8(&wbuf)[Nw], u8& rbyte) { writeRead( wbuf, Nw, &rbyte, 1 ); }
                //write 1 byte, read 1 byte
                auto
writeRead       (const u8& wbyte, u8& rbyte) { writeRead( &wbyte, 1, &rbyte, 1 ); }


                //write/write (such as a command, then a buffer)
                //pointers with user provided lengths
                auto
writeWrite      (const u8* wbuf, u16 wn, const u8* wbuf2, u16 wn2)
                {
                txbuf_ = wbuf; txbufEnd_ = &wbuf[wn];
                txbuf2_ = wbuf2; txbuf2End_ = &wbuf2[wn2];
                rxbuf_ = 0; rxbufEnd_ = 0; //no read
                startIrq( 1 ); //write only
                }
                //references with template where length is deduced from type
                template<int Nw, int Nw2> auto
writeWrite      (const u8(&wbuf)[Nw], const u8(&wbuf2)[Nw2]) { writeWrite( wbuf, Nw, wbuf2, Nw2 ); }
                //1 byte write, >1 bytes write
                template<int Nw2> auto
writeWrite      (const u8& wbyte, const u8(&wbuf2)[Nw2]) { writeWrite( &wbyte, 1, wbuf2, Nw2 ); }
                //>1 bytes write, 1 byte write
                template<int Nw> auto
writeWrite      (const u8(&wbuf)[Nw], const u8& wbyte2) { writeWrite( wbuf, Nw, &wbyte2, 1 ); }
                //>1 bytes write, 1 byte write
                auto
writeWrite      (const u8 &wbyte, const u8& wbyte2) { writeWrite( &wbyte, 1, &wbyte2, 1 ); }


                //write only alias
                //pointer with user provided length
                auto
write           (const u8* wbuf, u16 wn) { writeRead( wbuf, wn, 0, 0); }
                //reference with template where length is deduced from type
                template<int Wn> auto
write           (const u8(&wbuf)[Wn]) { writeRead( wbuf, Wn, 0, 0); }
                //1 byte only
                auto
write           (const u8& wbyte) { writeRead( &wbyte, 1, 0, 0); }


                //read only alias
                //pointer with user provided length
                auto
read            (u8* rbuf, u16 rn) { writeRead( 0, 0, rbuf, rn); }
                //reference with template where length is deduced from type
                template<int Rn> auto
read            (u8(&rbuf)[Rn]) { writeRead( 0, 0, rbuf, Rn ); }
                //1 byte only
                auto
read            (u8& rbyte) { writeRead( 0, 0, &rbyte, 1 ); }


                //blocking wait with timeout
                //if false is returned, caller can check isBusy() to see
                //if was a timeout or an error (isBusy will be true if timeout)
                //caller then can do a bus recovery if wanted
                auto
waitUS          (u16 us)
                {
                while( _delay_us(1), --us && isBusy() ){}
                return resultOK(); //true = ok, false = error or timeout
                }

                void
baud            (uint32_t twiHz, uint32_t cpuHz = F_CPU)
                {
                int32_t v = cpuHz/twiHz/2 - 5;
                twi_.MBAUD = v >= 0 ? v : 0;
                }

                void
busRecovery     () //twi will be off when done
                {
                disable(); //turn off twi so we have control of pins
                uint8_t sclbm = 1<<(pins_.MpinSCL & 7), scabm = 1<<(pins_.MpinSCA & 7);
                PORT_t* pt = pins_.Mport;

                pt->OUTSET = sclbm;             //scl high
                pt->DIRSET = sclbm;             //scl output
                for( u8 i = 0; i < 19; i++ ){   //10 clocks (20 toggles, but leave low so 19)
                    pt->OUTTGL = sclbm;
                    _delay_us( 5 );             //5us half cycle = 100khz
                    }
                //produce a stop
                pt->OUTCLR = scabm;             //sca low
                pt->DIRSET = scabm;             //sca output
                _delay_us( 30 );
                pt->DIRCLR = sclbm;             //scl back to input w/pullup
                _delay_us( 30 );
                pt->DIRCLR = scabm;             //sca back to input w/pullup
                }

                }; //Twim class

#endif //defined(TWIM0_ISR_ENABLE) && TWIM0_ISR_ENABLE
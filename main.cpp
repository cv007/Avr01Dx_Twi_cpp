//======================================================================
//  main.c
//
//  demonstrate twi0 usage, both master and slave communicating on same
//  device, tested with an ATtiny416 Xplained Nano (led is PB5, inverted)
//======================================================================
#include "Twis.hpp"
#include "Twim.hpp"

/*------------------------------------------------------------------------------
    wait - uses _delay_ms (so we can have a simple callable delay with
           variable runtime values)
------------------------------------------------------------------------------*/
                static void
waitMS          (u16 ms){ while( ms-- ) _delay_ms(1); }


/*------------------------------------------------------------------------------
    Led 
------------------------------------------------------------------------------*/
class
Led             {

                PORT_t& port_;
                u8 pinbm_;
    public:

Led             (PORT_t& port, u8 pin, bool invert = false)
                : port_(port), pinbm_(1<<pin)
                {
                (&port.PIN0CTRL)[pin] = invert ? 0x80 : 0; //enable invert if needed
                port_.DIRSET = pinbm_; //output
                }
                //constructor invert took care of high/low state needed for
                //on/off, so user can now simply use on/off
                auto
on              () { port_.OUTSET = pinbm_; }
                auto
off             () { port_.OUTCLR = pinbm_; }
                auto
toggle          () { port_.OUTTGL = pinbm_; }

                }; //Led


/*------------------------------------------------------------------------------
    Blinker I2C device - example i2c device on this mcu

        twi slave address 0x51

        our slave device has registers from address 0x00 to 0x07 named ram,
            which can be used to store byte values
        our slave device has a register at address 0x08 named onTime,
            and an address of 0x09 named offTime
        the onTime value will determine the led blink on time-
            0 = off
            1 = 10ms on
            255 = 2550ms on
        the offTime value will determine the led off time-
            0 = off
            1 = 10ms off
            255 = 2550ms off

        register address will auto increment after a write or read (if write
            was not a register address write), will roll over from end to beginning

        to write a new value to onTime or offTime
            write 0x08 or 0x09, write new value
        to read the current value of onTime or offTime
            write 0x08 or 0x09, read value
        to write a new value to onTime and offTime
            write 0x08, write new onTime value, write new offTime value
        to read the current value of onTime and offTime
            write 0x08, read onTime value, read offTime value

------------------------------------------------------------------------------*/
class
BlinkerI2C      {

                //device registers 0-9
                u8 ram_[8];
                u8 onTime_;         //[8], MS x10
                u8 offTime_;        //[9], MS x10
                u8* regPtr_;        //current device register address (pointer)

                Twis& twi_;         //Twis instance
                const u8 myAddress_;//slave address
                Led& led_;          //led to blink

                //so static callback can access this object, store object in static array
                //in this case we only allow 1 instance, but could have 2 instances
                //if device also has TWI1 available
                #if defined(TWI1)
                static inline BlinkerI2C* instance_[2];
                #else
                static inline BlinkerI2C* instance_[1];
                #endif

    public:

BlinkerI2C      (Twis& twis, u8 myAddress, Led& led)
                : twi_(twis), myAddress_(myAddress), led_(led)
                {
                instance_[twis.twiN()] = this;
                on();
                }

                void
on              () { twi_.on( myAddress_, twiCallback ); }
                auto
off             () { twi_.off(); }

                static bool
twiCallback     (Twis& twis, Twis::IRQ_STATE state, u8 statusReg)
                {
                static bool is1stWrite;
                BlinkerI2C& my{ *instance_[twis.twiN()] }; //get our object, as a reference
                //keep regPtr inside the range of registers that can write/read
                if( my.regPtr_ > &my.offTime_ or
                    my.regPtr_ < &my.ram_[0] ) my.regPtr_ = &my.ram_[0];

                bool ret = true; //assume ok to continue transaction

                switch( state ) {
                    //check address here, could be general call (0) or maybe we
                    //have a second address or address mask
                    case Twis::TWIS_ADDRESSED:
                    if( twis.lastAddress() != my.myAddress_ ) ret = false; //for us?
                    else is1stWrite = true;     //yes, expect a register address write
                    break;

                    case Twis::TWIS_MREAD:      //master read, so slave writes
                    twis.write( *my.regPtr_++ );
                    break;

                    case Twis::TWIS_MWRITE: {   //parens so we can create a var inside case without error
                        u8 v = twis.read();
                        if( is1stWrite ){       //if first write, is a register address write
                            is1stWrite = false;
                            my.regPtr_ = &my.ram_[v];  //ram is base address 0, so v is offset from that
                            break;              //regPtr will be validated in the next isr
                            }
                        //else is a register write
                        *my.regPtr_++ = v;
                        }
                    break;

                    case Twis::TWIS_STOPPED:
                    case Twis::TWIS_ERROR:
                    ret = false;
                    break;

                    }
                return ret;
                }

                void
run             (u8 n) //do what we are designed to do- blink led
                {
                for( u8 i = 0; i < n; i++ ){
                    led_.on();  waitMS( onTime_ * 10 );
                    led_.off(); waitMS( offTime_ * 10 );
                    }
                }

                };


/*------------------------------------------------------------------------------
    BlinkerCmd - twi master communications to slave device
------------------------------------------------------------------------------*/
class
BlinkerCmd      {

                Twim& twi_;
                u8 address_;

                auto
twiOn           () { twi_.baud( 100000ul ).on(address_); }

    public:

                //register addresses of BlinkerI2C device
                enum
REGISTERS       { RAM, ONTIME = 8, OFFTIME };

BlinkerCmd      (Twim& twim, u8 address )
                : twi_(twim), address_(address)
                {
                twiOn();
                }

                void
reset           ()
                {
                //special case for bus recovery since we are using the same pins
                //for master/slave (in this example), so caller will have to disable
                //the slave to also release the pins (and re-enable slave when done)
                twi_.busRecovery();
                twiOn();
                }

                //caller will check return value, and if false
                //they are in charge of calling reset if needed
                template<int N> bool
write           (const u8 reg, const u8(&wbuf)[N]) //multi-byte write starting at reg
                { //write register address, write value(s)
                twi_.writeWrite(reg, wbuf);
                return twi_.waitUS( 3000 );
                }

                bool //single byte write to reg
write           (u8 reg, const u8& wbyte)
                {
                u8 wbuf[1] = { wbyte }; //convert to a 1 byte array for above write
                return write( reg, wbuf );
                }

                template<int N> bool
read            (u8 reg, u8(&rbuf)[N]) //multi-byte read starting at reg
                { //write register address, read value(s)
                u8 wr[1] = { reg }; //make it a 1 byte array for writeRead
                twi_.writeRead( wr, rbuf );
                return twi_.waitUS( 3000 );
                }

                bool //single byte read from reg
read            (u8 reg, u8& rbyte)
                {
                u8 rbuf[1] = { rbyte }; //convert to a 1 byte array for above read
                return read( reg, rbuf );
                }

                };


/*------------------------------------------------------------------------------
    main
------------------------------------------------------------------------------*/

                static Twim         twim0       { TWI0_pins };
                static Twis         twis0       { TWI0_pins };
                static Led          led         { PORTB, 5, true }; //tiny416 xplained nano led
                static BlinkerCmd   blinkercmd  { twim0, 0x51 };
                static BlinkerI2C   blinkeri2c  { twis0, 0x51, led };

                int
main            ()
                {
                sei();

                //lambda function for any errors w/twi, used twice below
                auto error = []{
                    blinkeri2c.off(); //need slave to also release pins (in this example)
                    blinkercmd.reset();
                    blinkeri2c.on();
                    led.on();
                    waitMS(10000);
                    led.off();
                    };

                //blinker device has ram registers, will use ram[0]
                //to store a value so we can test reading the slave also
                //if fails, keep trying
                u8 blinkN = 5; //blink N times in loop below
                while( not blinkercmd.write(blinkercmd.RAM, blinkN) ){ //write 1 byte, 5 -> ram[0]
                    error();
                    }

                //table of led on/off value pairs
                using TblT = struct { u8 ms10[2]; };
                const TblT onOffTbl[] = {
                    { 2, 40 },      //20ms on, 400ms off
                    { 100, 100 },   //1000ms on, 1000ms off
                    { 10, 200 }     //100ms on, 2000ms off
                    };

                while( 1 ) {

                    for( auto& tbl : onOffTbl ){
                        //write 2 values starting at ONTIME register (master->slave)
                        //then get value from register ram[0] (should be same as blinkN initial value above)
                        if( blinkercmd.write(blinkercmd.ONTIME, tbl.ms10) and
                            blinkercmd.read(blinkercmd.RAM, blinkN) ) {
                            //then blink N times (blinkeri2c is doing this)
                            blinkeri2c.run( blinkN );
                            waitMS( 2000 ); //2 seconds between blink 'sets'
                            continue; //next pair
                            }

                        //if any errors, do bus recovery (also turns on led for 10 seconds)
                        error();
                        //and start over
                        break;
                        }

                    } //while

                } //main


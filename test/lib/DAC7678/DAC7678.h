#ifndef _DAC7678_LIB_H
#define _DAC7678_LIB_H

#include <Arduino.h>
#include <Wire.h>

#define DAC7678_DEFAULT_WIRE &Wire

// Registers, commands, etc. Don't cares are set high unless theyre intersecting with a channel code.
#define DAC7678_DEFAULT_ADDRESS     0b1001000 // address when all ADDR pins are pulled low

#define DAC7678_WRITE_CHANNEL       0x00      // 0000 XXXX - access a channel's input register - OR with Channel
#define DAC7678_UPDATE_CHANNEL      0x10      // 0001 XXXX - update or read from a channel - OR with Channel
#define DAC7678_WRITE_CH_UPDATE_ALL 0x20      // 0010 XXXX - access a channel, then update all channels - OR with Channel
#define DAC7678_WRITE_CH_UPDATE_CH  0x30      // 0011 XXXX - access a channel, then update only that channel - OR with Channel
#define DAC7678_PDMODE_COMMAND      0x4F      // 0100 XXXX - access PDMode register
#define DAC7678_CLRCODE_COMMAND     0x5F      // 0101 XXXX - access ClrCode register
#define DAC7678_LDAC_COMMAND        0x6F      // 0110 XXXX - access LDAC register
#define DAC7678_RESET_COMMAND       0x7F      // 0111 XXXX - software reset
#define DAC7678_STATICREF_COMMAND   0x8F      // 1000 XXXX - access internal reference static mode register
#define DAC7678_FLEXIBLEREF_COMMAND 0x9F      // 1001 XXXX - access internal reference flexible mode register

#define DAC7678_STATICMODE_SW       0xBFFF    // (1001) - X0XX XXXX XXXX XXXX - switch to static mode from flexible mode


/**
 * @brief Class object for managing a TI DAC7678 chip.
 * @details
 * This class can be used to configure, control, and communicate with the DAC7678 chip.
 * It may work with DAC5578, DAC6578, and DAC7578 too! However, these have not been tested or even really considered.
 * 
 * This class uses the Arduino TwoWire library, meaning that read and write commands are BLOCKING 
 * (although it will stop reading if the wire becomes unavailable so it won't get caught in a read loop). 
 * If you don't know what 'blocking' means, you probably don't have to worry about it.
 * 
 * This class can not used to read the analog voltage outputs from the DAC7678 chip - just use the AnalogIn (ADC)
 * pins of your Arduino if you want to do that.
 * 
 * I try to provide adequate documentation in the source code, but more in-depth instructions are available
 * in the README for this project's GitHub repo, including function descriptions and a start-up guide. Go read that.
 * 
 * Reading the datasheet is recommended.
 * DAC7678 datasheet: https://www.ti.com/lit/ds/symlink/dac7678.pdf
 * 
 * @author Darius Fieschko
 */
class DAC7678
{
    public:

        // Register codes

        // Channel - used when writing to a channel (setting its output voltage)
        enum Channel {A = 0b0000, B = 0b0001, C = 0b0010, D = 0b0011, 
                      E = 0b0100, F = 0b0101, G = 0b0110, H = 0b0111, 
                      ALL = 0b1111, NONE = -1}; 
        
        // ClrMode - codes written to ClrMode register that decide what value to set DAC channels to when they're cleared
        enum ClrMode {ZERO_SCALE = 0xFFCF, MID_SCALE = 0xFFDF, FULL_SCALE = 0xFFEF, CLR_DISABLE = 0xFFFF};

        // PDMode - codes written to PDMode register - powers on registers, or powers them down and determines what impedance to pull pins to GND with
        enum PDMode {POWER_ON = 0b00, PD_1K = 0b01, PD_100K = 0b10, PD_HIGHZ = 0b11};    // PD_1K - 1k pulldown, PD_100K - 100k pulldown, PD_HIGHZ - no pulldown, just disconnects pin

        // SpeedMode - codes written to SpeedMode register - sets I2C communication speed (requires reset)
        enum SpeedMode {LOW_SPEED = 0x3FFF, HIGH_SPEED = 0x7FFF, MAINTAIN_SPEED = 0xBFFF};

        // ReferenceMode - Choose between static mode or flexible mode for the DAC7678's internal reference.
        // Static mode has two options - reference on, reference off. The reference will automatically turn on (if it is off) when a channel is powered on, 
        // and will automatically turn off (if it is on) when all channels are powered off.
        // Flexible mode has four options - powered on (acts same as static case), ALWAYS powered on, or ALWAYS powered off.
        enum ReferenceMode {ON_STATIC = 0xFFFF, OFF_STATIC = 0xFFEF, 
                            ON_FLEXIBLE = 0xCFFF, ALWAYS_ON = 0xDFFF, ALWAYS_OFF = 0xEFFF};


        // UpdateMode - not a register code; determines behavior of this class when writing to DAC registers
        enum UpdateMode {UPDATE_ONE, UPDATE_ALL, UPDATE_NONE};

        /* ~ Function documentation is in .cpp ~ */

        // Constructors
        DAC7678();
        DAC7678(uint8_t address);
        DAC7678(TwoWire &wire);
        DAC7678(uint8_t address, TwoWire &wire);

        // Class configuration functions - doesn't do anything to communicate with DAC, 
        //                                 just determines behavior of this class
        void setDefaults();
        void setAddress(uint8_t address);
        void connectWire(TwoWire &wire);
        void chooseUpdateMode(UpdateMode mode);

        // I2C functions
        void write(uint8_t command, uint16_t data);
        uint16_t read(uint8_t command);

        // Command functions
        void setDAC(Channel chan, uint16_t value);
        void update(Channel chan);
        void setWithoutUpdating(Channel chan, uint16_t value);
        void setAndUpdateAll(Channel chan, uint16_t value);
        void setAndUpdate(Channel chan, uint16_t value);
        void reset(SpeedMode mode = LOW_SPEED);
        void setClrMode(ClrMode mode);
        void setLDACMode(uint8_t chan_code);
        void setPDMode(PDMode mode, uint8_t chan_code);
        void setReferenceMode(ReferenceMode ref_mode);
        void setToStaticMode();

        // Read functions
        uint16_t readChannelInput(Channel chan);
        uint16_t readChannelOutput(Channel chan);
        uint16_t readPDRegister();
        uint16_t readClrModeRegister();
        uint16_t readLDACRegister();
        uint16_t readStaticReference();
        uint16_t readFlexibleReference(); 

        // Misc functions
        bool isValidAddress(uint8_t address);
        uint8_t getChannelCode(Channel chan1, Channel chan2 = NONE, Channel chan3 = NONE, Channel chan4 = NONE, 
                            Channel chan5 = NONE, Channel chan6 = NONE, Channel chan7 = NONE, Channel chan8 = NONE);

        // Access private variables
        uint8_t getAddressVar() { return address; }
        UpdateMode getUpdateModeVar() { return update_mode; }
        ReferenceMode getReferenceModeVar() { return ref_mode; }

        TwoWire * wire; // TwoWire Arduino object used for I2C communication


    private:
        // The DAC7678 device's address is set via the ADDR pins. Depending on your chip, you may have one or two ADDR pins.
        // See Tables 9 and 10 in DAC7678 datasheet for node addresses. https://www.ti.com/lit/ds/symlink/dac7678.pdf
        const uint8_t VALID_ADDRESSES[8] =
        {
            0b1001000, 0b1001001, 0b1001010, 0b1001011,
            0b1001100, 0b1001101, 0b1001110, 0b1001111
        };

        void writeToChannel(uint8_t command, Channel chan, uint16_t value);
        uint8_t chanCode(Channel chan);
        bool isStaticMode(ReferenceMode mode)   { return mode == ON_STATIC   || mode == OFF_STATIC; }
        bool isFlexibleMode(ReferenceMode mode) { return mode == ON_FLEXIBLE || mode == ALWAYS_ON || mode == ALWAYS_OFF; }

        uint8_t address;                        // 7-bit I2C node address
        UpdateMode update_mode;                 // Determines whether just one or all channels are updated when a channel is written to
        ReferenceMode ref_mode = OFF_STATIC;    // Keeps track of what mode the internal reference is in (boots up in static mode)
};

#endif

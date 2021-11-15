#include "DAC7678.h"

/**
 * @brief Sets DAC7678 class variables to default values. Does not do anything to DAC7678 chip!
 */
void DAC7678::setDefaults()
{
    wire = DAC7678_DEFAULT_WIRE;
    address = DAC7678_DEFAULT_ADDRESS;
    update_mode = UPDATE_ONE; // update only the channel written to
}

/**
 * @brief Construct a new DAC7678 object with default values.
 */
DAC7678::DAC7678()
{
    setDefaults();
}

/**
 * @brief Construct a new DAC7678 object with a specified node address.
 * @details Will default to Wire for TwoWire interface.
 *
 * @param address Node address of DAC7678 to communicate with.
 */
DAC7678::DAC7678(uint8_t address)
{
    setDefaults();
    setAddress((uint8_t) address);
}

/**
 * @brief Construct a new DAC7678 object a specified TwoWire object.
 * @details Will default to default node address.
 * 
 * @param wire TwoWire object to use for communication.
 */
DAC7678::DAC7678(TwoWire &wire)
{
    setDefaults();
    connectWire(wire);
}

/**
 * @brief Construct a new DAC7678 object a specified address and a TwoWire object.
 * 
 * @param adress Node address of DAC7678 to communicate with.
 * @param wire TwoWire object to use for communication.
 */
DAC7678::DAC7678(uint8_t address, TwoWire &wire)
{
    setDefaults();
    setAddress((uint8_t) address);
    connectWire(wire);
}

/**
 * @brief Choose the address of the DAC7678 chip you're communicating with.
 *        Does NOT check for address validity - use isValidAddress() for that
 * 
 * @param address I2C node address of chip
 */
void DAC7678::setAddress(uint8_t addr)
{
    address = (uint8_t) addr; // set address
}


/**
 * @brief Passes Arduino TwoWire object to DAC7678 object so it knows how to talk to the DAC7678 chip.
 * 
 * @param wire TwoWire object to pass to DAC7678 class
 */
void DAC7678::connectWire(TwoWire &wire_object)
{
    wire = &wire_object;
}

/**
 * @brief Set update mode between updating just the channel being written to (UPDATE_ONE), all channels (UPDATE_ALL) 
 *        or not updating any channels at all (UPDATE_NONE - you will have to call update() manually).
 * @details This affects which channels update when just one channel is set to a new output voltage.
 *          in general, you only want to update the channel you're writing to, because there's no reason to update
 *          channels that you're not changing the values of.
 * 
 * @param mode The UpdateMode to use - UPDATE_ONE, UPDATE_ALL, or UPDATE_NONE
 */
void DAC7678::chooseUpdateMode(UpdateMode mode)
{
    update_mode = mode;
}

/**
 * @brief Transmit arbitrary command to DAC7678 node address via I2C. Only recommended for advanced users.
 * @details This function transmits whatever your heart may desire to the DAC7678. This leaves a lot of room
 *          for error, unless you know exactly what you're doing. Recommended for advanced users only.
 * 
 *          The transmission that the DAC7678 receives consists of 4 bytes:
 *          1. Address byte
 *          2. Command byte
 *          2. Data byte 1
 *          3. Data byte 0
 *          Since the address byte is determined by the address variable set with setAddress(), this function
 *          requires an argument of three bytes - hence the 8-bit command argument and 16-bit data argument.
 * 
 * @param command the command byte to send
 * @param data the data bytes to send
 */
void DAC7678::write(uint8_t command, uint16_t data)
{
    wire->beginTransmission(address);   // transmit to DAC7678 node address
    wire->write((uint8_t) command);     // transmit command byte
    wire->write((uint16_t) data);       // transmit data bytes
    wire->endTransmission();            // end transmission
}

uint16_t DAC7678::read(uint8_t command)
{ // See 'DAC7678 I2C READ SEQUENCE' in datasheet (page 30) for more info
    uint16_t data = 0;
    wire->beginTransmission(address);
    wire->write((uint8_t) command); // send command byte to device
    wire->endTransmission(false);   // done sending command byte, but DON'T release I2C bus
    wire->requestFrom(address, 2);  // request 2 bytes from device
    while(wire->available())
    {
        uint8_t byte_in = wire->read(); // read byte
        data = (data << 8) | byte_in;   // add byte to data
    }
    return data;
}

/**
 * @brief Writes to a Channel's input register and updates (or doesn't update) channels based on which UpdateMode
 *        has been chosen via chooseUpdateMode() (defaults to UPDATE_ONE if you haven't called chooseUpdateMode())
 * 
 * @param chan DAC Channel to write to 
 * @param value Value to write to DAC channel
 */
void DAC7678::setDAC(Channel chan, uint16_t value)
{
    switch(update_mode)
    {
        case UPDATE_ONE:
            setAndUpdate(chan, value);
            break;
        case UPDATE_ALL:
            setAndUpdateAll(chan, value);
            break;
        case UPDATE_NONE:
            setWithoutUpdating(chan, value);
            break;
        default:
            break;
    }
}

/**
 * @brief Writes to the specified Channel's input register, but doesn't update the DAC's output from the register.
 *        You might want to use chooseUpdateMode() and setDAC() instead of these specific functions!
 * 
 * @param chan Channel whose input register we want to write to
 * @param value Value to write to input register. 12-bit value - the 4 most significant bits aren't used!
 */
void DAC7678::setWithoutUpdating(Channel chan, uint16_t value)
{   
    writeToChannel(DAC7678_WRITE_CHANNEL, chan, value);
}

/**
 * @brief Updates the specified Channel's output from its input register.
 * 
 * @param chan Channel to update
 */
void DAC7678::update(Channel chan)
{
    writeToChannel(DAC7678_UPDATE_CHANNEL, chan);
}

/**
 * @brief Writes to the specified Channel's input register, then updates ALL channels from their input registers.
 *        You might want to use chooseUpdateMode() and setDAC() instead of these specific functions!
 * 
 * @param chan Channel whose input register we want to write to
 * @param value Value to write to input register. 12-bit value - the 4 most significant bits aren't used!
 */
void DAC7678::setAndUpdateAll(Channel chan, uint16_t value)
{
    writeToChannel(DAC7678_WRITE_CH_UPDATE_CH, chan, value);
}

/**
 * @brief Writes to the specified Channel's input register, then updates the channel's output from the input register.
 *        You might want to use chooseUpdateMode() and setDAC() instead of these specific functions!
 * 
 * @param chan Channel whose input register we want to write to
 * @param value Value to write to input register. 12-bit value - the 4 most significant bits aren't used!
 */
void DAC7678::setAndUpdate(Channel chan, uint16_t value)
{
    writeToChannel(DAC7678_WRITE_CH_UPDATE_CH, chan, value);
}

/**
 * @brief (Private) used for writing to or updating channel registers.
 * 
 * @param command Command code to send - intended commands are DAC7678_WRITE_CHANNEL, DAC7678_UPDATE_CHANNEL, 
 *                                                             DAC7678_WRITE_CH_UPDATE_ALL, DAC7678_WRITE_CH_UPDATE_CH
 *                but this will technically work with other commands as well if you finagle it right (not that you should).
 * @param chan Which channel to write to and/or update.
 * @param value Value to set the register to. Defaults to 0xFFFF for update()
 */
void DAC7678::writeToChannel(uint8_t command, Channel chan, uint16_t value = 0xFFFF)
{
    if(chan == NONE) // leave if chan is invalid
        return;
    // See Table 17 in datasheet for why the command and data bytes are structured this way
    uint8_t command = 0 | (((uint8_t) command) << 4) | ((uint8_t) chan); // command byte
    uint16_t data = 0 | (value << 4);   // data bytes - shifted 4 bits - the first 4 bits are lost!
    write(command, data);
}

/**
 * @brief Performs a software reset on the DAC7678, resetting into low-speed or high-speed modes.
 * 
 * @param mode Speed mode to reset the device into - LOW_SPEED, HIGH_SPEED, or MAINTAIN_SPEED (keeps whatever speed was set previously)
 */
void DAC7678::reset(SpeedMode mode = LOW_SPEED)
{
    uint16_t data = (uint16_t) mode;        // make sure mode is 2 bytes
    write(DAC7678_RESET_COMMAND, data);  // send reset command with speed mode configuration
}

/**
 * @brief Sets the CLRCODE register, which determines the chip's response to an input from the CLR pin.
 * @details Pulling the CLR pin to GND will send the chip a signal to clear the DAC's outputs.
 *          If set to ZERO_SCALE, the CLR pin will set each channel to 0V.
 *          MID_SCALE will set each channel to VDD/2, and FULL_SCALE will set each channel to VDD.
 *          CLR_DISABLE will disable the CLR pin, and the chip won't respond to a CLR signal.
 * 
 * @param mode CLR setting to configure the DAC7678 chip with
 */
void DAC7678::setClrMode(ClrMode mode)
{
    uint16_t data = (uint16_t) mode;
    write(DAC7678_CLRCODE_COMMAND, data);
}

/**
 * @brief Sets contents of LDAC register. Channels with LDAC bits set to 1 will ignore input from the LDAC pin.
 *        Use getChannelCode() to generate chan_code if you're having trouble selecting channels yourself.
 * 
 * @param chan_code code to select channels - 8 bits, with each bit setting a particular channel.
 *                  Channel H is HSB, Channel A is LSB; e.g. bits go HGFEDCBA (they're backwards).
 *                  Example: chan_code = 11000000 would affect channels H and G.
 */
void DAC7678::setLDACMode(uint8_t chan_code)
{
    uint16_t data = ((uint8_t) chan_code) << 8; // shift code to correct location
    write(DAC7678_LDAC_COMMAND, data);
}

/**
 * @brief Sets the PDMode register, which powers pins on/off and selects pulldown resistor for OFF setting.
 *        Use getChannelCode() to generate chan_code if you're having trouble selecting channels yourself.
 * 
 * @param mode PDMode code - POWER_ON, PD_1K, PD_100K, or PD_HIGHZ
 * @param chan_code code to select channels - 8 bits, with each bit setting a particular channel.
 *                  Channel H is HSB, Channel A is LSB; e.g. bits go HGFEDCBA (they're backwards).
 *                  Example: chan_code = 11000000 would affect channels H and G.
 */
void DAC7678::setPDMode(PDMode mode, uint8_t chan_code)
{
    // bits shifted to match positions in Table 17 - Power Down Register in datasheet
    uint16_t pd_config = ((uint16_t) mode) << 13;       // PD1, PD0
    uint16_t chan_config = ((uint16_t) chan_code) << 5; // DACH ~ DACA
    
    uint16_t data = 0 | pd_config | chan_config; // combine PD code and channel code into one 16-bit message

    write(DAC7678_PDMODE_COMMAND, data);
}

/**
 * @brief Sets the internal reference mode to default on/off (ON/OFF_STATIC) or always on/off (ALWAYS_ON/OFF).
 * @details See comments by ReferenceMode enum or read the datasheet for an explanation of how the modes work.
 * 
 * @param mode Mode to set the internal reference to.
 */
void DAC7678::setReferenceMode(ReferenceMode mode)
{
    if(isFlexibleMode(ref_mode) && isStaticMode(mode)) // if switching from flexible mode to static mode
        setToStaticMode(); // switch to static mode

    uint8_t command;
    uint16_t data = (uint16_t) mode;
    
    // Choose between flexible or static mode command code
    if(isFlexibleMode(mode))
        command = DAC7678_FLEXIBLEREF_COMMAND;
    else // static mode
        command = DAC7678_STATICREF_COMMAND;
    
    write(command, data);

    ref_mode = mode; // set current reference mode tracker to new reference mode
}

/**
 * @brief Sends a command to switch the internal reference from flexible mode to static mode.
 * 
 */
void DAC7678::setToStaticMode()
{
    write(DAC7678_FLEXIBLEREF_COMMAND, DAC7678_STATICMODE_SW);
}

/**
 * @brief Read the input register for the specified channel (not necessarily the same as the channel's output).
 * 
 * @param chan Channel to read from
 * @return uint16_t - value read from input register
 */
uint16_t DAC7678::readChannelInput(Channel chan)
{
    uint8_t command = ((uint8_t) DAC7678_WRITE_CHANNEL) << 4 | (uint8_t) chan;
    uint16_t data = read(command) >> 4;
    return data;
}

/**
 * @brief Read the output register of the specified channel.
 * 
 * @param chan Channel to read from
 * @return uint16_t - value read from output register
 */
uint16_t DAC7678::readChannelOutput(Channel chan)
{
    uint8_t command = ((uint8_t) DAC7678_UPDATE_CHANNEL) << 4 | (uint8_t) chan;
    uint16_t data = read(command) >> 4;
    return data;
}

/**
 * @brief Read the Power Down register. Bit order: (fluff) PD1 PD0 H G F E D C B A
 *        PD1 PD0 correspond with enum PDMode - e.g. 00 indicates POWER_ON mode, 01 indicates 1k pulldown mode
 * 
 * @return uint16_t contents of PD register
 */
uint16_t DAC7678::readPDRegister()
{
    uint16_t data = read(DAC7678_PDMODE_COMMAND);
    return data;
}

/**
 * @brief Reads contents of ClrMode register. Only the two least significant bits matter.
 *        00 = zero scale, 01 = mid scale, 10 = full scale, 11 = clr pin disabled
 * 
 * @return uint16_t contents of ClrCode register
 */
uint16_t DAC7678::readClrModeRegister()
{
    uint16_t data = read(DAC7678_CLRCODE_COMMAND);
    return data;
}

/**
 * @brief Returns contents of LDAC register. Each bit corresponds with a channel.
 *        Bit order: H G F E D C B A
 * 
 * @return uint16_t contents of LDAC register
 */
uint16_t DAC7678::readLDACRegister()
{
    uint16_t data = read(DAC7678_LDAC_COMMAND);
    return data;
}

/**
 * @brief Returns contents of the static mode internal reference register. Only one bit actually matters.
 *        1 = internal reference enabled, 0 = internal reference disabled
 * 
 * @return uint16_t contents of static mode internal reference register
 */
uint16_t DAC7678::readStaticReference()
{
    uint16_t data = read(DAC7678_STATICREF_COMMAND);
    return data;
}

/**
 * @brief Returns contents of the flexible mode internal reference register. Only the three least significant bits matter.
 *
 * @return uint16_t contents of the flexible mode internal reference register. 
 */
uint16_t DAC7678::readFlexibleReference()
{
    uint16_t data = read(DAC7678_FLEXIBLEREF_COMMAND);
    return data;
}

/**
 * @brief Checks input address for validity (is it an address supported by DAC7678?)
 * @details Note that this function checks the entire possible address space - i.e. addresses set via ADDR1, ADDR0 pins.
 *          Some DAC7678 packages only support one address pin - ADDR0 - and therefore only have 3 possible addresses
 *          as opposed to the list of 8 addresses iterated through in this function.
 *          This library uses the Arduino library Wire, which supports 7-bit addresses only. As such, this function
 *          only checks 7-bit addresses (i.e. it doesn't check for the read/write bit)
 * 
 * @param address Address to check for validity
 * @return TRUE if match is found in list of valid addresses, FALSE if not
 */
bool DAC7678::isValidAddress(uint8_t address)
{
    for(int i = 0; i < 8; i++)
    {
        if(address == VALID_ADDRESSES[i])
            return true;
    }
    return false;
}


/**
 * @brief Returns 8-bit channel code for selecting which channels to write to. Takes anywhere from 1 to 8
 *        Channel enums as arguments.
 * @details Takes Channel enum arguments and returns 8 bits, with each bit representing one channel.
 *          Channel order: H G F E D C B A
 * 
 * @param chan Channel enum values - A, B, C, D, E, F, G, H, ALL, NONE
 * @return uint8_t 8-bit code that tells DAC7678 which channels to write to
 */
uint8_t DAC7678::getChannelCode(Channel chan1, Channel chan2 = NONE, Channel chan3 = NONE, Channel chan4 = NONE, 
                                Channel chan5 = NONE, Channel chan6 = NONE, Channel chan7 = NONE, Channel chan8 = NONE)
{
    uint8_t code = 0b00000000;
    // Get codes for each channel argument and logical OR them together to get final code representing all channels
    code |= chanCode(chan1) | chanCode(chan2) | chanCode(chan3) | chanCode(chan4) | 
            chanCode(chan5) | chanCode(chan6) | chanCode(chan7) | chanCode(chan8);
    return code;
}


/**
 * @brief (Private) Returns 8-bit channel code for selecting which channel to write to. 
 *        This function is only intended to be used in getChannelCode().
 * @details Takes Channel enum argument and returns 8 bits, with each bit representing one channel.
 *          Channel order: H G F E D C B A
 *          Logical OR channel codes together to set multiple channels at once.
 * 
 * @param chan Channel enum value - A, B, C, D, E, F, G, H, ALL, NONE
 * @return uint8_t 8-bit code that tells DAC7678 which channels to write to
 */
uint8_t DAC7678::chanCode(Channel chan)
{
    switch(chan)
    {
        case NONE:
            return 0b00000000;
        case A:
            return 0b00000001;
        case B:
            return 0b00000010;
        case C:
            return 0b00000100;
        case D:
            return 0b00001000;
        case E:
            return 0b00010000;
        case F:
            return 0b00100000;
        case G:
            return 0b01000000;
        case H:
            return 0b10000000;
        case ALL:
            return 0b11111111;
        default:
            return 0b00000000;
    }
}
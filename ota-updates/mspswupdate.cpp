// this code is not meant to compile properly, but rather base shell for how to receive
// msp software updates from a host connected through a spi bus, and manage the writing to flash
// and organization and setup of executing out of ram

#include "mspswupdate.h"

// needs to be synchronized with the memory map (linker.cmd)
#define FLASHSWU    0x5C00
#define RAMSWU      0x1C00
#define ISR_ADDR    0xFFD2
#define TOP_OF_RAM  0x5BFF
#define SWU_SIZE    0x0400

// start address of interrupt vector rable in RAM
#define IV_START_ADDR   (TOP_OF_RAM + 1 - sizeof(isrVect))

typedef void (*ivFn)();

// keep these in ram while we overwrite flash
swuaddr_t MspSwUpdate::address_ = 0;
uint32_t MspSwUpdate::count_ = 0;
Parser MspSwUpdate::parser_;

// internal pointer for the interrupt routine
MspSwUpdate* swuptr = nullptr;

/// default constructor
MspSwUpdate::MspSwUpdate()
{
    swuptr = this;
    isrVect_ = reinterpret_cast<isrVect*>(IV_START_ADDR);
    flashSwu_ = FLASHSWU;
    ramSwu_ = RAMSWU;
    parser_.init();
}

/// destructor
MspSwUpdate::~MspSwUpdate()
{
}

/// sets the addresses of various locations in memory - this should only be used for unit testing
/// @param[in] isr the address of the interrupt vectors
/// @param[in] flashSwu the address of the flash software update section
/// @param[in] ramSwu the address of the ram software update section
void MspSwUpdate::setAddr(void* isr, void* flashSwu, void* ramSwu)
{
    isrVect_ = reinterpret_cast<isrVect*>(isr);
    flashSwu_ = reinterpret_cast<uintptr_t>(flashSwu);
    ramSwu_ = reinterpret_cast<uintptr_t>(ramSwu);
}

/// for test purposes, we can test the isr vector against the address of the data reception function
/// @return the address of the rx data function
uintptr_t MspSwUpdate::usciAddr() const
{
    return reinterpret_cast<uintptr_t>(&MspSwUpdate::onRxData);
}

/// starts or finishes a software update
/// @return true if the update can be supported, otherwise false
bool MspSwUpdate::start()
{
    if (!loadRam(true))
        return false;

    address_ = 0;
    count_ = 0;
    return true;
}

/// loads the ram so we can move execution of critical functions during a software update
/// @param[in] iv flag to load the interrupt vector table to ram
/// @return success of the call
bool MspSwUpdate::loadRam(bool iv)
{
    uint8_t* ram = reinterpret_cast<uint8_t*>(ramSwu_);
    uint8_t* flashmem = reinterpret_cast<uint8_t*>(flashSwu_);
    memcpy(ram, flashmem, SWU_SIZE);

    if (iv)
    {
        // move interrupt vector table to ram, and assign interrupts we'll be using
        // since we are using a large code model (20 bit pointers), and the ISR vector must
        // be 16 bit pointers, we need to do this strange casting in order for the compiler to be OK
        ivFn fn = onRxData;
        isrVect_->usci_a2_rx_tx = (isr_type_t)((intptr_t)fn & 0xFFFF);

        fn = onHostInterrupt;
        isrVect_->io_p2 = (isr_type_t)((intptr_t)fn & 0xFFFF);

        // move!
        SYSCTL |= SYSRIVECT;
    }

    return true;
}

#pragma CODE_SECTION(".flashswu")
/// loops continuously while executing in ram
void MspSwUpdate::nop()
{
    while (1)
        __no_operation();
}

#pragma CODE_SECTION(".flashswu")
/// receives data from the spi bus
/// this is manually assigned to the uscia2 vector to operate in ram
__interrupt void MspSwUpdate::onRxData()
{
    switch (__even_in_range(SPI_IV, 4))
    {
    case 2:
        swuptr->parse(SPI_RXBUF);
        break;
    }
}

#pragma CODE_SECTION(".flashswu")
/// handles an interrupt from the host
/// this is manually assigned to the p2 vector to operate in ram
__interrupt void MspSwUpdate::onHostInterrupt()
{
    switch (P2IV)
    {
        case P2IV_P2IFG6:
            swuptr->parser_.reset();
            break;
    }
}

#pragma CODE_SECTION(".flashswu")
/// special parsing function when running a software update
/// @param[in] b the byte to fill
/// @return success of the call
bool MspSwUpdate::parse(uint8_t b)
{
    bool ret = true;

    if (parser_.parse(b))
    {
        spiEnum cmd = parser_.command();
        switch (cmd)
        {
            // sent by host to send a series of bytes for the software update
            case comm::swUpdateSetupData:
                ret = setupWrite(parser_.payload(), parser_.payloadSz());
                // only notify the host if the setup passed, otherwise we don't want to get into a state
                // where we potentially write bad data
                if (ret)
                    notifyHost();
                break;
            // sent by host to send a series of bytes for the software update
            case comm::swUpdateBytes:
                ret = writeData(parser_.payload(), parser_.payloadSz());
                notifyHost();
                break;
            // sent by host to complete a software update
            case spiEnum::swUpdateFinished:
                // reboot
                PMMCTL0 = PMMPW | PMMSWBOR;
                break;
            default:
                ret = false;
                break;
        }
    }

    return ret;
}

#pragma CODE_SECTION(".flashswu")
/// handles the setup (address and count) prior to a write of data
/// @param[in] data a pointer to the data
/// @param[in] sz the number of bytes pointed to
/// @return success of the call
bool MspSwUpdate::setupWrite(const uint8_t* data, uint32_t sz)
{
    // since the test program may use 64 bit addresses, we could have
    // 8 bytes with 32 bit address and 32 bit count, or 12 bytes with 64 bit address and 32 bit count
    if (sz != 8 && sz != 12)
        return false;

    address_ = ((swuaddr_t*)data)[0];
    if (sz == 12)
        count_ = (reinterpret_cast<const uint32_t*>(data))[2];
    else
        count_ = (reinterpret_cast<const uint32_t*>(data))[1];

    // safety check
    if (address_ < flashSwu_ || !count_)
        return false;

    return true;
}

#pragma CODE_SECTION(".flashswu")
/// handles incoming update data
/// @param[in] data a pointer to the data
/// @param[in] sz the number of bytes pointed to
/// @return success of the call
bool MspSwUpdate::writeData(const uint8_t* data, uint32_t sz)
{
    // safety check
    if (!address_ || !count_ || !sz || sz > count_)
        return false;

    // decrease the byte count
    count_ -= sz;

    // perform the actual write
    if (!writeToFlash(address_, data, sz))
        return false;

    // check if the address should be reset or incremented
    if (!count_)
        address_ = 0;
    else
        address_ += sz;

    return true;
}

#pragma CODE_SECTION(".flashswu")
/// writes data into flash for a software update
/// @param[in] addr the address to write to
/// @param[in] data the data to write
/// @param[in] sz the size of the data sent
/// @return success of the call
bool MspSwUpdate::writeToFlash(swuaddr_t addr, const uint8_t* data, uint32_t sz) const
{
    uint32_t i;
    bool blank = true, blockwrite = true;

    __disable_interrupt();
    // erase segment
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + ERASE;
    __data20_write_char(addr, 0);
    while (FCTL3 & BUSY) {}

    // check if the block is blank, if so, then skip the write, and just be happy with the erase
    for (i = 0; i < sz; i++)
    {
        if (data[i] != 0xFF)
        {
            blank = false;
            break;
        }
    }
    if (blank)
    {
        FCTL3 = FWKEY + LOCK;
        __enable_interrupt();
        return true;
    }

    // enable write
    FCTL1 = (blockwrite) ? FWKEY | BLKWRT : FWKEY | WRT;

    // write data
    if (blockwrite)
    {
        sz = sz >> 2;
        const uint32_t* wrdata = reinterpret_cast<const uint32_t*>(data);
        for (i = 0; i < sz; i++)
        {
            __data20_write_long(addr, wrdata[i]);
            addr += 4;
            while (!(FCTL3 & WAIT)) {}
        }
    }
    else
    {
        for (i = 0; i < sz; i++)
        {
            __data20_write_char(addr + i, data[i]);
            while (!(FCTL3 & WAIT)) {}
        }
    }

    // disable wrt
    FCTL1 = FWKEY;
    while (FCTL3 & BUSY) {}
    // set lock bit
    FCTL3 = FWKEY + LOCK;

    __enable_interrupt();
    return true;
}

/// notifies the host, since we're a spi slave and don't control the clock
void MspSwUpdate::notifyHost()
{
    // send gpio interrupt based on schematics, example on p1.1
    P1OUT |= BIT1;
    P1OUT &= ~BIT1;
}

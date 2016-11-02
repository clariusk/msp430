#pragma once

#include <parser.h>

#ifdef __COMPILE_FOR_UNIT_TESTING__
    #define swuaddr_t uint64_t
#else
    #define swuaddr_t uint32_t
#endif

namespace
{
    /// isr function pointer variable type
    typedef uint16_t* isr_type_t;

    /// interrupt vector table data type
    typedef struct _isrVect
    {
        isr_type_t reserved[41];    ///< reserved vector space
        isr_type_t rtc_a;           ///< rtc interrupt
        isr_type_t io_p2;           ///< p2 interrupt
        isr_type_t usci_b3_rx_tx;   ///< usci b3 interrupt
        isr_type_t usci_a3_rx_tx;   ///< usci a3 interrupt
        isr_type_t usci_b1_rx_tx;   ///< usci b1 interrupt
        isr_type_t usci_a1_rx_tx;   ///< usci a1 interrupt
        isr_type_t io_p1;           ///< p1 interrupt
        isr_type_t ta1_ta1iv;       ///< ta1 interrupt
        isr_type_t ta1_ccr0;        ///< ta1 ccr interrupt
        isr_type_t dma;             ///< dma interrupt
        isr_type_t usci_b2_rx_tx;   ///< usci b2 interrupt
        isr_type_t usci_a2_rx_tx;   ///< usci a2 interrupt
        isr_type_t ta0_ta0iv;       ///< ta0 interrupt
        isr_type_t ta0_ccr0;        ///< ta0 ccr interrupt
        isr_type_t adc12_a;         ///< adc12 interrupt
        isr_type_t usci_b0_rx_tx;   ///< usci b0 interrupt
        isr_type_t usci_a0_rx_tx;   ///< usci a0 interrupt
        isr_type_t wdt;             ///< watchdog interrupt
        isr_type_t tb_tbiv;         ///< timer b interrupt
        isr_type_t tb_ccr0;         ///< timer b ccr interrupt
        isr_type_t unmi;            ///< unmi interrupt
        isr_type_t snmi;            ///< snmi interrupt
        isr_type_t reset;           ///< reset interrupt
    } isrVect;

    /// this should be implemented to parse data coming from the spi bus
    /// to interpret specific commands and payload
    class Parser
    {
        // implement something
    };

    /// manages a software update of the msp
    class MspSwUpdate
    {
    public:
        MspSwUpdate();
        virtual ~MspSwUpdate();

        bool start();
        void nop();

        // following functions are for testing purposes only
        void setAddr(void* isr, void* flashSwu, void* ramSwu);
        uintptr_t usciAddr() const;

    private:
        bool loadRam(bool iv);
        bool setupWrite(const uint8_t* data, uint32_t sz);
        bool writeData(const uint8_t* data, uint32_t sz);
        bool writeToFlash(swuaddr_t addr, const uint8_t* data, uint32_t sz) const;
        bool parse(uint8_t b);
        void notifyHost();

        static __interrupt void onHostInterrupt();
        static __interrupt void onRxData();

    private:
        isrVect* isrVect_;              ///< isr vector for placement in ram
        uintptr_t flashSwu_;            ///< flash address for software update code
        uintptr_t ramSwu_;              ///< ram address for software update code
        Parser swuparser parser_;       ///< protocol parser
        static swuaddr_t address_;      ///< address to write data to
        static uint32_t count_;         ///< number of bytes to write to a new address
    };
}

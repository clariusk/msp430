#pragma once

namespace
{
    /// holds a block of flash data to write to the msp
    class FlashBlock
    {
    public:
        /// constructs a flash block
        /// @param[in] addr the address in flash
        /// @param[in] data the raw bytes to write
        FlashBlock(uint32_t addr, const std::vector<uint8_t>& data) : addr_(addr), data_(data) { }
        /// retrieves the number of bytes for a flash block
        /// @return the number of bytes for a flash block
        size_t size() const { return data_.size(); }

        uint32_t addr_;             ///< flash memory address
        std::vector<uint8_t> data_;  ///< data bytes
    };

    /// mock class that would send updates through the spi bus to an msp
    class Updater
    {
        public:
            Updater() : mspInterrupt_(-1), spi_(-1) {}

            bool sendUpdate(const std::string& hexfile);
            bool setupComm();

        private:
            bool loadMspFile(const std::string& file);
            std::vector<uint8_t> setupData(size_t index) const
            void waitForMspGpioInterrupt();
            bool spiMakeRequest(int req);
            bool spiSend(int cmd, const std::vector<uint8_t> msg);

        private:
            int mspInterrupt_;                  ///< gpio interrupt handle
            int spi_;                           ///< spi handle
            std::vector<FlashBlock> blocks_;    ///< holds blocks of flash data to write
            std::vector<uint8_t> readback_;     ///< readback buffer for receiving data over spi
    };
}

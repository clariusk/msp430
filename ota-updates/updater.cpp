// this code is not meant to compile properly, but rather base shell for how to parse and send
// msp software updates from a host connected through a spi bus

#include "updater.h"

#define FLASH_CHUNK         (512)   ///< must be 512 to align with the flash erases properly

namespace util
{
    /// calculates the checksum of a file
    /// @param[in] file the file to get the checksum for
    /// @param[in] sha1 flag for sha1 checksum (true), or md5 checksum (false)
    /// @return the checksum of the file
    std::string checksum(const std::string& file, bool sha1)
    {
        std::string sum, cmd = sha1 ? "sha1sum " : "md5sum ";
        std::istringstream f(logger::getTerminalResult(std::string(cmd + file), true));
        std::getline(f, sum, ' ');
        return sum;
    }
}

/// tries to load a software update file for the msp in ti-txt format
/// @param[in] file the path of the file to load
/// @return success of the call
bool Updater::loadMspFile(const std::string& file)
{
    // try and open the file
    std::ifstream f(file);
    if (!f.is_open())
        return false;

    blocks_.clear();
    std::stringstream ss;
    ss << f.rdbuf();
    f.close();

    uint32_t addr = 0, startAddr = 0;
    std::string str;
    std::vector<uint8_t> bytes;

    // read the file, one line at a time
    while (std::getline(ss, str, '\n'))
    {
        // check if there is a new address specified
        size_t pos = str.find('@');
        if (pos != std::string::npos)
        {
            str.erase(pos, 1);
            uint32_t newAddr = std::stoi(str, nullptr, 16);

            // pad instruction information that does not exist
            if (bytes.size())
            {
                for (auto i = 0u; i < newAddr - addr; i++)
                    bytes.push_back(0xFF);
            }

            addr = newAddr;
            if (startAddr == 0)
                startAddr = newAddr;
        }
        else
        {
            // check if we've hit the end of the file (denoted by a 'q' at the end)
            pos = str.find('q');
            if (pos != std::string::npos)
            {
                // pad the remaining flash
                for (auto i = 0u; i < 0x45C00 - addr; i++)
                    bytes.push_back(0xFF);

                blocks_.push_back(FlashBlock(startAddr, bytes));
                break;
            }

            // get individual bytes
            std::istringstream iss(str);
            do
            {
                std::string s;
                iss >> s;
                if (!s.empty())
                {
                    bytes.push_back(std::stoi(s, nullptr, 16));
                    addr++;
                }
            } while (iss);
        }
    }

    return blocks_.size() ? true : false;
}

/// converts the 32 bit address and size of a block into a byte array
/// @param[in] index the block index
/// @return the address and size in byte array format
std::vector<uint8_t> Updater::setupData(size_t index) const
{
    if (index >= blocks_.size() || !blocks_[index].addr_)
        return { };

    std::vector<uint8_t> v;
    v.push_back((blocks_[index].addr_ >> 0) & 0xFF);
    v.push_back((blocks_[index].addr_ >> 8) & 0xFF);
    v.push_back((blocks_[index].addr_ >> 16) & 0xFF);
    v.push_back((blocks_[index].addr_ >> 24) & 0xFF);

    uint32_t sz = blocks_[index].size();
    v.push_back((sz >> 0) & 0xFF);
    v.push_back((sz >> 8) & 0xFF);
    v.push_back((sz >> 16) & 0xFF);
    v.push_back((sz >> 24) & 0xFF);

    return v;
}

/// tries to perform a software update on the msp
/// this is a very critical step and could brick the device if not performed properly
/// @param[in] hexfile the file to parse for sending raw bytes to the msp
/// @return success of the call
bool Updater::sendUpdate(const std::string& hexfile)
{
    std::string sha1 = util::checksum(hexfile, true);
    if (!loadMspFile(hexfile) || !blocks_.size())
        return false;

    // makes a request over spi to the msp that we want to update firmware
    if (!spiMakeRequest(spiEnum::swUpdateReq))
        return false

    // tell msp to reconfigure ram, shut down interrupts, etc
    if (!spiSend(spiEnum::prepareSwUpdate, std::vector<uint8_t>()))
        return false;

    // wait until we hear back from the msp, meaning we're good to start sending the new firmware data
    waitForMspSpiInterrupt();
    
    printf("\nstarting firmware update");
    
    // send each flash block
    for (auto i = 0u; i < blocks_.size(); i++)
    {
        std::vector<uint8_t> setup = setupData(i);
        if (setup.size() == 8)
        {
            // send the setup info - address and size
            if (!spiSend(spiEnum::swUpdateSetupData, setup))
                return false;

            waitForMspGpioInterrupt();

            // send in chunks
            size_t data = blocks_[i].size(), offset = 0;
            while (data)
            {
                size_t sz = (data > FLASH_CHUNK) ? FLASH_CHUNK : data;
                // keep alignment so we don't end up performing 2 erases
                if (offset == 0 && blocks_[i].addr_ % FLASH_CHUNK)
                    sz = FLASH_CHUNK - (blocks_[i].addr_ % FLASH_CHUNK);

                std::vector<uint8_t> chunk(blocks_[i].data_.data() + offset, blocks_[i].data_.data() + offset + sz);
                if (!spiSend(spiEnum::swUpdateBytes, chunk))
                    return false;

                waitForMspGpioInterrupt();
                data -= sz;
                offset += sz;
            }
        }

        printf("\nfinished writing block @ %X with %uB", blocks_[i].addr_, blocks_[i].size()));
    }

    // send notification that it's time to reboot
    if (!spiSend(spiEnum::swUpdateFinished, std::vector<uint8_t>()))
        return false;

    return true;
}

// ****************************************************************
// * the following functions should be implemented based on the spi
// * connection setup between the updater host and the msp
// ****************************************************************

/// since the host in this case is setup as a spi master, it makes it more difficult
/// to have bi-directional communications with the msp, since the master is in charge of
/// the clock. to make the msp more intelligent a gpio was hooked up to the host which can be
/// setup to handle a notification from the msp
/// @return success of the call
bool Updater::setupComm()
{
    // in this case, the sysfs is used to open the gpio handle, note this should be mapped in the
    // device tree beforehand, XXX is the gpio number that was mapped
    mspInterrupt_ = open("/sys/class/gpio/gpioXXX/value", O_RDONLY | O_NONBLOCK);
    if (mspInterrupt_ < 0)
        return false;
        
    // open the spi controller
    spi_ = open("/dev/spidevXXXXX.0", O_RDWR);
    if (spi_ < 0)
        return false;

    return true;
}

/// waits for an inerrupt to occur on the gpio
/// ideally, this should be called in a separate thread, and then the waiting function
/// should setup a condition variable which gets triggered from this thread
/// for simplicity of this example, the 'bad' way is shown/implemented
void Updater::waitForMspGpioInterrupt()
{
    struct pollfd fds;
    uint32_t rd;

    memset(&fds, 0, sizeof(fds));
    fds.fd = mspInterrupt_;
    fds.events = POLLPRI;

    lseek(mspInterrupt_, 0, SEEK_SET);
    if (poll(&fds, 1, -1) > 0)
    {
        if (fds.revents & POLLPRI)
            read(mspInterrupt_, &rd, sizeof(rd));
    }

    return true;
}   

/// makes a request to the msp to retrieve information
/// @param[in] req the request command
/// @return success of the call
bool Updater::spiMakeRequest(int req)
{
    if (spi_ == -1)
       return false;

    size_t tries = SPI_TRIES;
    while (--tries)
    {
        readback_.clear();
        if (spiSend(req, std::vector<uint8_t>()) && spiRead())
            break;

        usleep(1000);
    }

    if (!tries)
        return false;

    return true;
}

/// sends data across the spi bus
/// @param[in] cmd the command to embed in the protocol
/// @param[in] msg the message payload to send
/// @return success of the call
bool Updater::spiSend(int cmd, const std::vector<uint8_t> msg)
{
    std::vector<uint8_t> data;

    // since there's likely a protocol to follow for comm over spi, assemble the data properly
    // data = parse(cmd, msg);
    
    if (write(spi_, data.data(), data.size()) != data.size())
        return false;

    return true;
}

/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe
 * Copyright (c) 2015-2017 Josh Blum

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyADSDR.hpp"
#include <iomanip>
#include <algorithm>

#define ADSDR_PRODUCT_ID        0x00f1
#define ADSDR_BOOT_ID           0x00f3

int SoapyADSDR::m_ad_count;
std::vector<SoapySDR::Kwargs> SoapyADSDR::m_ad_devices;
double SoapyADSDR::m_gainMin = 0.0;
double SoapyADSDR::m_gainMax = 84.0;

std::string& filterChars(std::string& s, const std::string& allowed) {
    s.erase(remove_if(s.begin(), s.end(), [&allowed](const char& c) {
        return allowed.find(c) == std::string::npos;
    }), s.end());
    return s;
}

std::string freqToStr(double _freq)
{
    double freqTemp = _freq;
    std::string suffix("");
    std::stringstream freqStr;

    if(freqTemp >= 1.0e9) {
        freqTemp /= 1.0e9;
        freqStr << std::setprecision(10);
        suffix = std::string("GHz");
    }else if(freqTemp >= 1.0e6) {
        freqTemp /= 1.0e6;
        freqStr << std::setprecision(7);
        suffix = std::string("MHz");
    }else if(freqTemp >= 1.0e3) {
        freqTemp /= 1.0e3;
        freqStr << std::setprecision(4);
        suffix = std::string("KHz");
    }

    freqStr << freqTemp;
    freqStr << suffix;

    return freqStr.str();
}

double strToFreq(std::string freqStr) {
    std::string filterStr = filterChars(freqStr, std::string("0123456789.GMKHgmkh"));
    size_t numLen = filterStr.find_first_not_of("0123456789.");

    if(numLen == std::string::npos) {
        numLen = freqStr.length();
    }

    std::string numPartStr = freqStr.substr(0, numLen);
    std::string suffixStr = freqStr.substr(numLen);

    std::stringstream numPartStream;
    numPartStream.str(numPartStr);

    double freqTemp = 0;
    numPartStream >> freqTemp;

    if(suffixStr.length()) {
        if(suffixStr.find_first_of("Gg") != std::string::npos) {
            freqTemp *= 1.0e9;
        } else if(suffixStr.find_first_of("Mm") != std::string::npos) {
            freqTemp *= 1.0e6;
        } else if(suffixStr.find_first_of("Kk") != std::string::npos) {
            freqTemp *= 1.0e3;
        } else if(suffixStr.find_first_of("Hh") != std::string::npos) {
            // ...
        }
    }

    return freqTemp;
}

SoapyADSDR::SoapyADSDR(const SoapySDR::Kwargs& args)
{
    if(!SoapyADSDR::m_ad_count)
    {
        throw std::runtime_error("AD-SDR device not found.");
    }

    m_DeviceId = -1;
    m_pDevice = NULL;

    m_rxFormat = AD_RX_FORMAT_CFLOAT32;
    tunerType = ADSDR_TUNER_UNKNOWN;

    m_sampleRate = SoapyADSDR::listSampleRates(SOAPY_SDR_RX, 0).front();
    m_bandWidth = SoapyADSDR::listBandwidths(SOAPY_SDR_RX, 0).front();
    m_centerFrequency = 100000000;

    m_numBuffers = DEFAULT_NUM_BUFFERS;
    m_bufferLength = DEFAULT_BUFFER_LENGTH;

    m_iqSwap = false;
    m_agcMode = false;

    m_bufferedElems = 0;
    m_resetBuffer = false;

    std::string fw_file = "./adsdrfx3.img";

    if(args.count("ad") != 0)
    {
        try
        {
            m_DeviceId = std::stoi(args.at("ad"));
        }
        catch (const std::invalid_argument &)
        {
        }
        if (m_DeviceId < 0 || m_DeviceId >= SoapyADSDR::m_ad_count)
        {
            throw std::runtime_error(
                    "device index 'ad' out of range [0 .. " + std::to_string(SoapyADSDR::m_ad_count) + "].");
        }

        SoapySDR_logf(SOAPY_SDR_INFO, "Found AD-SDR Device using device index parameter 'ad' = %d", m_DeviceId);
    }
    else if(args.count("serial") != 0)
    {
        std::string deviceSerialFind = args.at("serial");

        for(int i = 0; i < SoapyADSDR::m_ad_count; i++)
        {
            SoapySDR::Kwargs devInfo = SoapyADSDR::m_ad_devices[i];
            if(devInfo.at("serial") == deviceSerialFind)
            {
                SoapySDR_logf(SOAPY_SDR_INFO,
                        "Found AD-SDR Device #%d by serial %s -- Manufacturer: %s, Product Name: %s, Serial: %s", i,
                        deviceSerialFind.c_str(), devInfo.at("manufacturer").c_str(), devInfo.at("product").c_str(),
                        devInfo.at("serial").c_str());
                m_DeviceId = i;
                break;
            }
        }
    }
    else if(args.count("label") != 0)
    {
        std::string labelFind = args.at("label");
        for (int i = 0; i < SoapyADSDR::m_ad_count; i++)
        {
            SoapySDR::Kwargs devInfo = SoapyADSDR::m_ad_devices[i];
            if (devInfo.at("label") == labelFind)
            {
                SoapySDR_logf(SOAPY_SDR_INFO, "Found AD-SDR Device #%d by name: %s", devInfo.at("label").c_str());
                m_DeviceId = i;
                break;
            }
        }
    }

    if (m_DeviceId == -1)
    {
        throw std::runtime_error("Unable to find requested AD-SDR device.");
    }

    if (args.count("tuner") != 0)
    {
        tunerType = adStringToTuner(args.at("tuner"));
    }
    SoapySDR_logf(SOAPY_SDR_INFO, "AD-SDR Tuner type: %s", adTunerToString(tunerType).c_str());

    SoapySDR_logf(SOAPY_SDR_INFO, "AD-SDR opening device %d", m_DeviceId);



    // Смотрим тип устройства. Если boot, то прошиваем его.
    int prod_id = adsdr_find(m_DeviceId);

    if(prod_id == ADSDR_PRODUCT_ID) {
        SoapySDR_logf(SOAPY_SDR_INFO, "--- Reset ADSDR device --- \n");
        adsdr_reset(m_DeviceId);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        prod_id = adsdr_find(m_DeviceId);
    }

    if(prod_id == ADSDR_BOOT_ID) {
        SoapySDR_logf(SOAPY_SDR_INFO, "Flash image file: %s \n", fw_file.c_str());
        adsdr_flash(fw_file.c_str(), m_DeviceId);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        prod_id = adsdr_find(m_DeviceId);

        if(prod_id == ADSDR_PRODUCT_ID) {

            char manufact[256], product[256], serial[256];
            bool deviceAvailable = false;

            // Перезаписываем структуру с информацией.
            SoapySDR::Kwargs devInfo;

            std::string deviceManufacturer;
            std::string deviceProduct;
            std::string deviceTuner;
            std::string deviceSerial;

            if(adsdr_get_device_usb_strings(m_DeviceId, manufact, product, serial) == 0)
            {
                SoapySDR_logf(SOAPY_SDR_DEBUG, "\tManufacturer: %s, Product Name: %s, Serial: %s", manufact, product, serial);

                deviceSerial = serial;
                if(deviceSerial.empty())
                    deviceSerial = "XXXXXX";
                deviceProduct = product;
                deviceManufacturer = manufact;

                adsdr_dev_t* devTest;
                if(adsdr_open(&devTest, m_DeviceId) == 0)
                {
                    deviceAvailable = true;
                    deviceTuner = SoapyADSDR::adTunerToString(adsdr_get_tuner_type(devTest));
                    SoapySDR_logf(SOAPY_SDR_DEBUG, "\t Tuner type: %s", deviceTuner.c_str());
                    adsdr_close(devTest);
                }
            }

            std::string deviceLabel = std::string(adsdr_get_device_name(m_DeviceId)) + " :: " + deviceSerial;
            devInfo["ad"] = std::to_string(m_DeviceId);
            devInfo["label"] = deviceLabel;
            devInfo["available"] = deviceAvailable ? "Yes" : "No";
            devInfo["product"] = deviceProduct;
            devInfo["serial"] = deviceSerial;
            devInfo["manufacturer"] = deviceManufacturer;
            devInfo["tuner"] = deviceTuner;

            SoapySDR_logf(SOAPY_SDR_INFO, "Change devInfo after flashing. Size of dev: %d ", SoapyADSDR::m_ad_devices.size());
            std::vector<SoapySDR::Kwargs>::iterator Iter = SoapyADSDR::m_ad_devices.erase(SoapyADSDR::m_ad_devices.begin() + m_DeviceId);
            SoapyADSDR::m_ad_devices.insert(Iter, devInfo);

        }
    }

    if(prod_id != ADSDR_PRODUCT_ID)
        throw std::runtime_error("Product ID not  ADSDR_PRODUCT_ID. Index: " + std::to_string(m_DeviceId));

    SoapySDR_logf(SOAPY_SDR_INFO, "Found ADSDR device. index: %d. Open... \n", m_DeviceId);
    if(adsdr_open(&m_pDevice, m_DeviceId) < 0)
        throw std::runtime_error("Error open ADSDR. Index: " + std::to_string(m_DeviceId));

    if(adsdr_init(m_pDevice, adsdr_get_init_params(), adsdr_get_rx_fir_config(), adsdr_get_tx_fir_config()) < 0)
        throw std::runtime_error("Error init AD9361 device. Index: ");
}

SoapyADSDR::~SoapyADSDR(void)
{
    //cleanup device handles
    adsdr_close(m_pDevice);
}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyADSDR::getDriverKey(void) const
{
    return "ADSDR";
}

std::string SoapyADSDR::getHardwareKey(void) const
{
    return "ADSDR";
}

SoapySDR::Kwargs SoapyADSDR::getHardwareInfo(void) const
{
    //key/value pairs for any useful information
    //this also gets printed in --probe
    SoapySDR::Kwargs args;

    args["origin"] = "https://github.com/amungo/SoapyADSDR";
    args["ad"] = std::to_string(m_DeviceId);

    return args;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyADSDR::getNumChannels(const int dir) const
{
    return (dir == SOAPY_SDR_RX) ? 1 : 0;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyADSDR::listAntennas(const int direction, const size_t channel) const
{
    std::vector<std::string> antennas;
    antennas.push_back("RX");
    return antennas;
}

void SoapyADSDR::setAntenna(const int direction, const size_t channel, const std::string& name)
{
    if(direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("setAntena failed: AD-SDR only supports RX");
    }
}

std::string SoapyADSDR::getAntenna(const int direction, const size_t channel) const
{
    return "RX";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyADSDR::hasDCOffsetMode(const int direction, const size_t channel) const
{
    return false;
}

bool SoapyADSDR::hasFrequencyCorrection(const int direction, const size_t channel) const
{
    return false;
}

void SoapyADSDR::setFrequencyCorrection(const int direction, const size_t channel, const double value)
{
    SoapySDR::Device::setFrequencyCorrection(direction, channel, value);
}

double SoapyADSDR::getFrequencyCorrection(const int direction, const size_t channel) const
{
    return SoapySDR::Device::getFrequencyCorrection(direction, channel);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyADSDR::listGains(const int direction, const size_t channel) const
{
    //list available gain elements,
    //the functions below have a "name" parameter
    std::vector<std::string> gains;
    gains.push_back("TUNER");

    return gains;
}

bool SoapyADSDR::hasGainMode(const int direction, const size_t channel) const
{
    return true;
}

void SoapyADSDR::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    m_agcMode = automatic;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting AD-SDR AGC: %s", automatic ? "Automatic" : "Manual");
    if(adsdr_set_rx_gc_mode(m_pDevice, channel, automatic ? AD_GAIN_HYBRID_AGC : AD_GAIN_MGC) < 0)
        SoapySDR_logf(SOAPY_SDR_ERROR, "Error set AD-SDR RX Gain mode: %s. Channel %u.", automatic ? "Automatic" : "Manual", channel);
}

bool SoapyADSDR::getGainMode(const int direction, const size_t channel) const
{
    unsigned char mode = AD_GAIN_MGC;
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Getting AD-SDR RX Gain mode.");
    if(adsdr_get_rx_gc_mode(m_pDevice, channel, &mode) < 0)
        SoapySDR_logf(SOAPY_SDR_ERROR, "Unable get AD-SDR Gain mode. Channel: %u", channel);

    return mode != AD_GAIN_MGC;
}

void SoapyADSDR::setGain(const int direction, const size_t channel, const double value)
{
    //set the overall gain by distributing it across available gain elements
    //OR delete this function to use SoapySDR's default gain distribution algorithm...
    SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting AD-SDR RX Gain: %f", value);
    if(adsdr_set_rx_rf_gain(m_pDevice, channel, value) < 0)
        SoapySDR_logf(SOAPY_SDR_ERROR, "Error set AD-SDR RX Gain: %f, channel: %u", value, channel);
}

void SoapyADSDR::setGain(const int direction, const size_t channel, const std::string& name, const double value)
{
    if (name == "TUNER")
    {
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Setting AD-SDR RX Gain: %f", value);
        if(adsdr_set_rx_rf_gain(m_pDevice, channel, value) < 0)
            SoapySDR_logf(SOAPY_SDR_ERROR, "Error set AD-SDR RX Gain: %f, channel: %u", value, channel);
    }
}

double SoapyADSDR::getGain(const int direction, const size_t channel, const std::string& name) const
{
    int gain = 0;
    if (name == "TUNER")
    {
        if(adsdr_get_rx_rf_gain(m_pDevice, channel, &gain) < 0)
            SoapySDR_logf(SOAPY_SDR_ERROR, "Error get AD-SDR RX Gain, channel: %u ", channel);
    }

    return gain;
}

SoapySDR::Range SoapyADSDR::getGainRange(const int direction, const size_t channel, const std::string& name) const
{
    return SoapySDR::Range(SoapyADSDR::m_gainMin, SoapyADSDR::m_gainMax);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/
void SoapyADSDR::setFrequency( const int direction, const size_t channel, const std::string& name,
                               const double frequency, const SoapySDR::Kwargs& args)
{
    if(name == "RF")
    {
        m_centerFrequency = (uint32_t)frequency;
        m_resetBuffer = true;
        SoapySDR_logf(SOAPY_SDR_INFO, "Setting center freq: %d", m_centerFrequency);
        adsdr_set_rx_lo_freq(m_pDevice, m_centerFrequency);
    }
}

double SoapyADSDR::getFrequency(const int direction, const size_t channel, const std::string& name) const
{
    if (name == "RF")
    {
        return (double)m_centerFrequency;
    }

    return 0;
}

std::vector<std::string> SoapyADSDR::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> names;
    names.push_back("RF");

    return names;
}

SoapySDR::RangeList SoapyADSDR::getFrequencyRange( const int direction, const size_t channel, const std::string &name) const
{
    SoapySDR::RangeList results;
    if (name == "RF")
    {
        switch(tunerType) {
            case ADSDR_TUNER_UNKNOWN:
            default:
                results.push_back(SoapySDR::Range(70e6, 6000e6));
                break;
        }
    }

    return results;
}

SoapySDR::ArgInfoList SoapyADSDR::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;

    // TODO: frequency arguments

    return freqArgs;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyADSDR::setSampleRate(const int direction, const size_t channel, const double rate)
{
    m_sampleRate = rate;
    m_resetBuffer = true;
    SoapySDR_logf(SOAPY_SDR_INFO, "Setting sample rate: %d", m_sampleRate);

    if(adsdr_set_rx_samp_freq(m_pDevice, m_sampleRate) < 0) //@camry
        SoapySDR_logf(SOAPY_SDR_ERROR, "Error set sample rate: %d", m_sampleRate);
}

double SoapyADSDR::getSampleRate(const int direction, const size_t channel) const
{
    return m_sampleRate;
}

std::vector<double> SoapyADSDR::listSampleRates(const int direction, const size_t channel) const
{    
    std::vector<double> list;

    list.push_back(2.5e6);
    list.push_back(3e6);
    list.push_back(4e6);
    list.push_back(8e6);
    list.push_back(16e6);
    list.push_back(20e6);
    list.push_back(40e6);
    list.push_back(50e6);
    list.push_back(61.44e6);

    return list;
}

void SoapyADSDR::setBandwidth(const int direction, const size_t channel, const double bw)
{
    m_bandWidth = bw;
    SoapySDR_logf(SOAPY_SDR_INFO, "Setting bandWidth: %d", m_bandWidth);
    adsdr_set_rx_rf_bandwidth(m_pDevice, m_bandWidth);
}

double SoapyADSDR::getBandwidth(const int direction, const size_t channel) const
{
    return m_bandWidth;
}

std::vector<double> SoapyADSDR::listBandwidths(const int direction, const size_t channel) const
{
    std::vector<double> list;

    list.push_back(2.5e6);
    list.push_back(3e6);
    list.push_back(4e6);
    list.push_back(8e6);
    list.push_back(16e6);
    list.push_back(20e6);
    list.push_back(40e6);
    list.push_back(50e6);
    list.push_back(61.44e6);

    return list;
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyADSDR::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    SoapySDR::ArgInfo iqSwapArg;

    iqSwapArg.key = "iq_swap";
    iqSwapArg.value = "false";
    iqSwapArg.name = "I/Q Swap";
    iqSwapArg.description = "AD-SDR I/Q Swap Mode";
    iqSwapArg.type = SoapySDR::ArgInfo::BOOL;

    setArgs.push_back(iqSwapArg);

    SoapySDR::ArgInfo bandWidthArg;
    std::vector<double> ranges = SoapyADSDR::listBandwidths(SOAPY_SDR_RX, 0);
    for(double band : ranges) {
        bandWidthArg.options.push_back(std::to_string((int)band));
        bandWidthArg.optionNames.push_back(freqToStr(band));
    }
    int deevice_bandwidth = ranges.front();

    bandWidthArg.name = "Bandwidth";
    bandWidthArg.key = "band_width";
    bandWidthArg.description = "AD-SDR Bandwidth";
    bandWidthArg.type = SoapySDR::ArgInfo::STRING;
    bandWidthArg.units = "Hz";
    bandWidthArg.value = std::to_string(deevice_bandwidth);
    setArgs.push_back(bandWidthArg);

    SoapySDR_logf(SOAPY_SDR_DEBUG, "SETARGS?");

    return setArgs;
}

void SoapyADSDR::writeSetting(const std::string& key, const std::string& value)
{
    if(key == "band_width")
    {
        m_bandWidth = (uint32_t)strToFreq(value);
        SoapySDR_logf(SOAPY_SDR_INFO, "AD-SDR Bandwidth: %d [%s]", m_bandWidth, value);
        if(adsdr_set_rx_samp_freq(m_pDevice, m_bandWidth) < 0)
            SoapySDR_logf(SOAPY_SDR_ERROR, "Error set bandwidth: %d", m_bandWidth);
    }
}

std::string SoapyADSDR::readSetting(const std::string& key) const
{
    if(key == "iq_swap") {
        return m_iqSwap?"true":"false";
    } else if(key == "band_width") {
        return freqToStr((double)m_bandWidth);
    }

    SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
}

std::string SoapyADSDR::adTunerToString(adsdr_tuner tunerType)
{
    std::string deviceTuner;
    switch (tunerType)
    {
    case ADSDR_TUNER_UNKNOWN:
    default:
        deviceTuner = "Unknown";
        break;
    }
    return deviceTuner;
}

adsdr_tuner SoapyADSDR::adStringToTuner(std::string tunerType)
{
    adsdr_tuner deviceTuner = ADSDR_TUNER_UNKNOWN;

    return deviceTuner;
}


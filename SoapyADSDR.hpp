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
#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.h>
#include <SoapySDR/Types.h>
#include <adsdr.h>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

typedef enum
{
    AD_RX_FORMAT_CFLOAT32,
    AD_RX_FORMAT_CINT16
} adsdrRXFormat;

enum adsdr_gain_ctrl_mode {
    AD_GAIN_MGC,
    AD_GAIN_FASTATTACK_AGC,
    AD_GAIN_SLOWATTACK_AGC,
    AD_GAIN_HYBRID_AGC
};


#define DEFAULT_BUFFER_LENGTH (512*32*16)
#define DEFAULT_NUM_BUFFERS 16
#define BYTES_PER_SAMPLE 4

std::string& filterChars(std::string& s, const std::string& allowed);
std::string freqToStr(double _freq);
double strToFreq(std::string freqStr);



class SoapyADSDR: public SoapySDR::Device
{
public:
    SoapyADSDR(const SoapySDR::Kwargs &args);
    ~SoapyADSDR(void);

    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey(void) const;

    std::string getHardwareKey(void) const;

    SoapySDR::Kwargs getHardwareInfo(void) const;

    /*******************************************************************
     * Channels API
     ******************************************************************/
    size_t getNumChannels(const int) const;

    /*******************************************************************
     * Stream API
     ******************************************************************/
    std::vector<std::string> getStreamFormats(const int _direction, const size_t _channel) const;

    std::string getNativeStreamFormat(const int _direction, const size_t _channel, double& _fullScale) const;

    SoapySDR::ArgInfoList getStreamArgsInfo(const int _direction, const size_t _channel) const;

    SoapySDR::Stream *setupStream(const int _direction, const std::string& _format, const std::vector<size_t>& _channels =
            std::vector<size_t>(), const SoapySDR::Kwargs& args = SoapySDR::Kwargs());

    void closeStream(SoapySDR::Stream* _stream);

    size_t getStreamMTU(SoapySDR::Stream* _stream) const;

    int activateStream(SoapySDR::Stream* _stream, const int _flags = 0, const long long _timeNs = 0, const size_t _numElems = 0);
    int deactivateStream(SoapySDR::Stream* _stream, const int _flags = 0, const long long _timeNs = 0);
    int readStream(SoapySDR::Stream* _stream, void* const *_buffs, const size_t _numElems, int& _flags, long long& _timeNs, const long _timeoutUs = 100000);

    /*******************************************************************
     * Direct buffer access API
     ******************************************************************/
    size_t getNumDirectAccessBuffers(SoapySDR::Stream* _stream);
    int getDirectAccessBufferAddrs(SoapySDR::Stream* _stream, const size_t _handle, void** _buffs);

    int acquireReadBuffer(SoapySDR::Stream* _stream, size_t& _handle, const void** _buffs, int& _flags, long long& _timeNs, const long _timeoutUs = 100000);
    void releaseReadBuffer(SoapySDR::Stream* _stream, const size_t _handle);

    /*******************************************************************
     * Antenna API
     ******************************************************************/
    std::vector<std::string> listAntennas(const int _direction, const size_t _channel) const;
    void setAntenna(const int _direction, const size_t _channel, const std::string& _name);
    std::string getAntenna(const int _direction, const size_t _channel) const;

    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/
    bool hasDCOffsetMode(const int _direction, const size_t _channel) const;
    bool hasFrequencyCorrection(const int _direction, const size_t _channel) const;
    void setFrequencyCorrection(const int _direction, const size_t _channel, const double _value);
    double getFrequencyCorrection(const int _direction, const size_t _channel) const;

    /*******************************************************************
     * Gain API
     ******************************************************************/
    std::vector<std::string> listGains(const int _direction, const size_t _channel) const;
    bool hasGainMode(const int _direction, const size_t _channel) const;
    void setGainMode(const int _direction, const size_t _channel, const bool _automatic);
    bool getGainMode(const int _direction, const size_t _channel) const;
    void setGain(const int _direction, const size_t _channel, const double _value);
    void setGain(const int _direction, const size_t _channel, const std::string& _name, const double _value);
    double getGain(const int _direction, const size_t _channel, const std::string& _name) const;
    SoapySDR::Range getGainRange(const int _direction, const size_t _channel, const std::string& _name) const;

    /*******************************************************************
     * Frequency API
     ******************************************************************/
    void setFrequency(const int _direction, const size_t _channel, const std::string& _name, const double _frequency,
            const SoapySDR::Kwargs &args = SoapySDR::Kwargs());
    double getFrequency(const int _direction, const size_t _channel, const std::string& _name) const;
    std::vector<std::string> listFrequencies(const int _direction, const size_t _channel) const;
    SoapySDR::RangeList getFrequencyRange(const int _direction, const size_t _channel, const std::string& _name) const;
    SoapySDR::ArgInfoList getFrequencyArgsInfo(const int _direction, const size_t _channel) const;

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/
    void setSampleRate(const int _direction, const size_t _channel, const double _rate);
    double getSampleRate(const int _direction, const size_t _channel) const;
    std::vector<double> listSampleRates(const int _direction, const size_t _channel) const;
    void setBandwidth(const int _direction, const size_t _channel, const double _bw);
    double getBandwidth(const int _direction, const size_t _channel) const;
    std::vector<double> listBandwidths(const int _direction, const size_t _channel) const;

    /*******************************************************************
     * Utility
     ******************************************************************/
    static std::string adTunerToString(adsdr_tuner _tunerType);
    static adsdr_tuner adStringToTuner(std::string _tunerType);

    /*******************************************************************
     * Settings API
     ******************************************************************/
    SoapySDR::ArgInfoList getSettingInfo(void) const;
    void writeSetting(const std::string& _key, const std::string& _value);
    std::string readSetting(const std::string& _key) const;

private:
    //device handle
    int m_DeviceId;
    adsdr_dev_t* m_pDevice;

    //cached settings
    adsdrRXFormat m_rxFormat;
    adsdr_tuner tunerType;
    uint32_t m_sampleRate;
    uint32_t m_bandWidth;
    uint32_t m_centerFrequency;
    size_t m_numBuffers;
    size_t m_bufferLength;
    bool m_iqSwap;
    bool m_agcMode;

public:
    //async api usage
    std::thread m_rx_async_thread;
    void rx_async_operation(void);
    void rx_callback(unsigned char* buf, uint32_t len);

    std::mutex m_buf_mutex;
    std::condition_variable m_buf_cond;

    std::vector<std::vector<signed char> > m_buffs;
    size_t	m_buf_head;
    size_t	m_buf_tail;
    std::atomic<size_t>	m_buf_count;
    signed char* m_currentBuff;
    std::atomic<bool> m_overflowEvent;
    size_t m_currentHandle;
    size_t m_bufferedElems;
    std::atomic<bool> m_resetBuffer;

    static int m_ad_count;
    static std::vector<SoapySDR::Kwargs> m_ad_devices;
    static double m_gainMin;
    static double m_gainMax;
};


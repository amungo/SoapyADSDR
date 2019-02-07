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
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <algorithm> //min
#include <climits> //SHRT_MAX
#include <cstring> // memcpy


struct sample {
    sample() : i(0),q(0)
    {}
    int16_t i;
    int16_t q;
};


std::vector<std::string> SoapyADSDR::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;

    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);

    return formats;
}

std::string SoapyADSDR::getNativeStreamFormat(const int direction, const size_t channel, double& fullScale) const {
    //check that direction is SOAPY_SDR_RX
     if (direction != SOAPY_SDR_RX) {
         throw std::runtime_error("AD-SDR is RX only, use SOAPY_SDR_RX");
     }

     fullScale = 32768;
     return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyADSDR::getStreamArgsInfo(const int direction, const size_t channel)     const {
    //check that direction is SOAPY_SDR_RX
     if (direction != SOAPY_SDR_RX) {
         throw std::runtime_error("AD-SDR is RX only, use SOAPY_SDR_RX");
     }

    SoapySDR::ArgInfoList streamArgs;

    SoapySDR::ArgInfo bufflenArg;
    bufflenArg.key = "bufflen";
    bufflenArg.value = std::to_string(DEFAULT_BUFFER_LENGTH);
    bufflenArg.name = "Buffer Size";
    bufflenArg.description = "Number of bytes per buffer, multiples of 512 only.";
    bufflenArg.units = "bytes";
    bufflenArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(bufflenArg);

    SoapySDR::ArgInfo buffersArg;
    buffersArg.key = "buffers";
    buffersArg.value = std::to_string(DEFAULT_NUM_BUFFERS);
    buffersArg.name = "Ring buffers";
    buffersArg.description = "Number of buffers in the ring.";
    buffersArg.units = "buffers";
    buffersArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(buffersArg);

    return streamArgs;
}

/*******************************************************************
 * Async thread work
 ******************************************************************/

static void _rx_callback(unsigned char* buf, uint32_t len, void* ctx)
{
    //printf("_rx_callback\n");
    SoapyADSDR* self = (SoapyADSDR*)ctx;
    self->rx_callback(buf, len);
}

void SoapyADSDR::rx_async_operation(void)
{
    //printf("rx_async_operation\n");
    adsdr_read_async(m_pDevice, &_rx_callback, this, m_numBuffers, m_bufferLength);
    //printf("rx_async_operation done!\n");
}

void SoapyADSDR::rx_callback(unsigned char* buf, uint32_t len)
{
    //printf("_rx_callback %d _buf_head=%d, numBuffers=%d\n", len, m_buf_head, m_buf_tail);

    //overflow condition: the caller is not reading fast enough
    if(m_buf_count == m_numBuffers)
    {
        m_overflowEvent = true;
        return;
    }

    auto& buff = m_buffs[m_buf_tail];
    int samples_len = len / (sizeof(short) * 2);
    buff.resize(len);
    int16_t* pSamplesIn = (int16_t*)buf;
    sample* pSamplesOut = (sample*)buff.data();

    for(int i = 0; i < samples_len; i++)
    {
        pSamplesOut[i].i = pSamplesIn[2*i+0] >> 4;
        pSamplesOut[i].q = pSamplesIn[2*i+1] >> 4;
    }

    //increment the tail pointer
    m_buf_tail = (m_buf_tail + 1) % m_numBuffers;

    //increment buffers available under lock
    //to avoid race in acquireReadBuffer wait
    {
        std::lock_guard<std::mutex> lock(m_buf_mutex);
        m_buf_count++;
    }

    //notify readStream()
    m_buf_cond.notify_one();
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream* SoapyADSDR::setupStream(
        const int direction,
        const std::string& format,
        const std::vector<size_t>& channels,
        const SoapySDR::Kwargs& args)
{
    if (direction != SOAPY_SDR_RX)
    {
        throw std::runtime_error("AD-SDR is RX only, use SOAPY_SDR_RX");
    }

    //check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
    {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    //check the format
    if (format == SOAPY_SDR_CF32)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
        m_rxFormat = AD_RX_FORMAT_CFLOAT32;
    }
    else if (format == SOAPY_SDR_CS16)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
        m_rxFormat = AD_RX_FORMAT_CINT16;
    }
    else
    {
        throw std::runtime_error(
                "setupStream invalid format '" + format
                        + "' -- Only CS16 and CF32 are supported by SoapyADSDR module.");
    }

    m_bufferLength = DEFAULT_BUFFER_LENGTH;
    if (args.count("bufflen") != 0)
    {
        try
        {
            int bufferLength_in = std::stoi(args.at("bufflen"));
            if (bufferLength_in > 0)
            {
                m_bufferLength = bufferLength_in;
            }
        }
        catch (const std::invalid_argument &){}
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AD-SDR Using buffer length %d", m_bufferLength);

    m_numBuffers = DEFAULT_NUM_BUFFERS;
    if(args.count("buffers") != 0)
    {
        try
        {
            int numBuffers_in = std::stoi(args.at("buffers"));
            if (numBuffers_in > 0)
            {
                m_numBuffers = numBuffers_in;
            }
        }
        catch (const std::invalid_argument &){}
    }
    SoapySDR_logf(SOAPY_SDR_DEBUG, "AD-SDR Using %d buffers", m_numBuffers);

    //clear async fifo counts
    m_buf_tail = 0;
    m_buf_count = 0;
    m_buf_head = 0;

    //allocate buffers
    m_buffs.resize(m_numBuffers);
    for (auto &buff : m_buffs) buff.reserve(m_bufferLength);
    for (auto &buff : m_buffs) buff.resize(m_bufferLength);

    return (SoapySDR::Stream *)this;
}

void SoapyADSDR::closeStream(SoapySDR::Stream* stream)
{
    this->deactivateStream(stream, 0, 0);
    m_buffs.clear();
}

size_t SoapyADSDR::getStreamMTU(SoapySDR::Stream* stream) const
{
    return m_bufferLength / (BYTES_PER_SAMPLE);
}

int SoapyADSDR::activateStream(SoapySDR::Stream* stream, const int flags, const long long timeNs, const size_t numElems)
{
    if(flags != 0)
        return SOAPY_SDR_NOT_SUPPORTED;
    m_resetBuffer = true;
    m_bufferedElems = 0;

    adsdr_set_datapath_en(m_pDevice, 1);
    //start the async thread
    if(not m_rx_async_thread.joinable())
    {
        m_rx_async_thread = std::thread(&SoapyADSDR::rx_async_operation, this);
    }
    return 0;
}

int SoapyADSDR::deactivateStream(SoapySDR::Stream* stream, const int flags, const long long timeNs)
{
    if (flags != 0)
        return SOAPY_SDR_NOT_SUPPORTED;

    adsdr_set_datapath_en(m_pDevice, 0);
    if(m_rx_async_thread.joinable())
    {
        adsdr_cancel_async(m_pDevice);
        m_rx_async_thread.join();
    }
    return 0;
}

int SoapyADSDR::readStream(SoapySDR::Stream* stream, void* const* buffs, const size_t numElems,
                           int& flags, long long& timeNs, const long timeoutUs)
{
    //drop remainder buffer on reset
    if(m_resetBuffer and m_bufferedElems != 0)
    {
        m_bufferedElems = 0;
        this->releaseReadBuffer(stream, m_currentHandle);
    }

    //this is the user's buffer for channel 0
    void* buff0 = buffs[0];

    //are elements left in the buffer? if not, do a new read.
    if(m_bufferedElems == 0)
    {
        int ret = this->acquireReadBuffer(stream, m_currentHandle, (const void **)&m_currentBuff, flags, timeNs, timeoutUs);
        if(ret < 0)
            return ret;
        m_bufferedElems = ret;
    }

    size_t returnedElems = std::min(m_bufferedElems, numElems);
    int16_t* pSamplesIn = (int16_t*)m_currentBuff;

    //convert into user's buff0
    if(m_rxFormat == AD_RX_FORMAT_CFLOAT32)
    {
        float* ftarget = (float*)buff0;
        if(m_iqSwap)
        {
            for(size_t i = 0; i < returnedElems; i++)
            {
                ftarget[i*2] = ((float)pSamplesIn[i*2 + 1]) / 2048.0f;
                ftarget[i*2 + 1] = ((float)pSamplesIn[i*2]) / 2048.0f;
            }
        }
        else
        {
            for(size_t i = 0; i < returnedElems; i++)
            {
                ftarget[i*2] = ((float)pSamplesIn[i*2]) / 2048.0f;
                ftarget[i*2 + 1] = ((float)pSamplesIn[i*2 + 1]) / 2048.0f;
            }
        }

    }
    else if(m_rxFormat == AD_RX_FORMAT_CINT16)
    {
        int16_t* itarget = (int16_t*)buff0;
        if(m_iqSwap)
        {
            for(size_t i = 0; i < returnedElems; i++)
            {
                itarget[i*2] = pSamplesIn[i*2 + 1];
                itarget[i*2 + 1] = pSamplesIn[i*2];
            }
        }
        else
        {
            for(size_t i = 0; i < returnedElems; i++)
            {
                itarget[i*2] = pSamplesIn[i*2];
                itarget[i*2 + 1] = pSamplesIn[i*2 + 1];
            }
        }
    }

    //bump variables for next call into readStream
    m_bufferedElems -= returnedElems;
    m_currentBuff += returnedElems*BYTES_PER_SAMPLE;

    //return number of elements written to buff0
    if(m_bufferedElems != 0)
        flags |= SOAPY_SDR_MORE_FRAGMENTS;
    else
        this->releaseReadBuffer(stream, m_currentHandle);

    return returnedElems;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapyADSDR::getNumDirectAccessBuffers(SoapySDR::Stream* stream)
{
    return m_buffs.size();
}

int SoapyADSDR::getDirectAccessBufferAddrs(SoapySDR::Stream* stream, const size_t handle, void** buffs)
{
    buffs[0] = (void*)m_buffs[handle].data();
    return 0;
}

int SoapyADSDR::acquireReadBuffer(SoapySDR::Stream* stream, size_t& handle, const void** buffs,
                                  int& flags, long long& timeNs, const long timeoutUs)
{
    //reset is issued by various settings
    //to drain old data out of the queue
    if(m_resetBuffer)
    {
        //drain all buffers from the fifo
        m_buf_head = (m_buf_head + m_buf_count.exchange(0)) % m_numBuffers;
        m_resetBuffer = false;
        m_overflowEvent = false;
    }

    //handle overflow from the rx callback thread
    if(m_overflowEvent)
    {
        //drain the old buffers from the fifo
        m_buf_head = (m_buf_head + m_buf_count.exchange(0)) % m_numBuffers;
        m_overflowEvent = false;
        SoapySDR::log(SOAPY_SDR_SSI, "O");
        return SOAPY_SDR_OVERFLOW;
    }

    //wait for a buffer to become available
    if(m_buf_count == 0)
    {
        std::unique_lock <std::mutex> lock(m_buf_mutex);
        m_buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs), [this]{return m_buf_count != 0;});
        if(m_buf_count == 0)
            return SOAPY_SDR_TIMEOUT;
    }

    //extract handle and buffer
    handle = m_buf_head;
    m_buf_head = (m_buf_head + 1) % m_numBuffers;
    buffs[0] = (void*)m_buffs[handle].data();
    flags = 0;

    //return number available
    return m_buffs[handle].size() / BYTES_PER_SAMPLE;
}

void SoapyADSDR::releaseReadBuffer(SoapySDR::Stream* stream, const size_t handle)
{
    //TODO this wont handle out of order releases
    m_buf_count--;
}

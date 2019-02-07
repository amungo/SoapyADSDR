/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe

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
#include <SoapySDR/Registry.hpp>
#include <cstdlib> //malloc

static std::vector<SoapySDR::Kwargs> findADSDR(const SoapySDR::Kwargs &args)
{
    std::vector<SoapySDR::Kwargs> results;

    char manufact[256], product[256], serial[256];

    int this_count = adsdr_get_device_count();

    if (!SoapyADSDR::m_ad_devices.size() || SoapyADSDR::m_ad_count != this_count)
    {
        SoapyADSDR::m_ad_count = this_count;

        if(SoapyADSDR::m_ad_devices.size())
        {
            SoapyADSDR::m_ad_devices.erase(SoapyADSDR::m_ad_devices.begin(), SoapyADSDR::m_ad_devices.end());
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "AD-SDR Devices: %d", SoapyADSDR::m_ad_count);

        for(int i = 0; i < SoapyADSDR::m_ad_count; i++)
        {
            SoapySDR::Kwargs devInfo;

            std::string deviceName(adsdr_get_device_name(i));
            std::string deviceManufacturer;
            std::string deviceProduct;
            std::string deviceTuner;
            std::string deviceSerial;

            bool deviceAvailable = false;
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Device #%d: %s", i, deviceName.c_str());
            if(adsdr_get_device_usb_strings(i, manufact, product, serial) == 0)
            {
                SoapySDR_logf(SOAPY_SDR_DEBUG, "\tManufacturer: %s, Product Name: %s, Serial: %s", manufact, product, serial);

                deviceSerial = serial;
                if(deviceSerial.empty())
                    deviceSerial = "XXXXXX";
                deviceProduct = product;
                deviceManufacturer = manufact;

                adsdr_dev_t* devTest;
                if(adsdr_open(&devTest, i) == 0)
                {
                    deviceAvailable = true;
                    deviceTuner = SoapyADSDR::adTunerToString(adsdr_get_tuner_type(devTest));

                    SoapySDR_logf(SOAPY_SDR_DEBUG, "\t Tuner type: %s", deviceTuner.c_str());

                    adsdr_close(devTest);
                }
            }

            if (!deviceAvailable)
            {
                SoapySDR_logf(SOAPY_SDR_DEBUG, "\tUnable to access device #%d (in use?)", i);
            }

            std::string deviceLabel = std::string(adsdr_get_device_name(i)) + " :: " + deviceSerial;

            devInfo["ad"] = std::to_string(i);
            devInfo["label"] = deviceLabel;
            devInfo["available"] = deviceAvailable ? "Yes" : "No";
            devInfo["product"] = deviceProduct;
            devInfo["serial"] = deviceSerial;
            devInfo["manufacturer"] = deviceManufacturer;
            devInfo["tuner"] = deviceTuner;
            SoapyADSDR::m_ad_devices.push_back(devInfo);
        }
    }

    //filtering
    for (int i = 0; i < SoapyADSDR::m_ad_count; i++)
    {
        SoapySDR::Kwargs devInfo = SoapyADSDR::m_ad_devices[i];
        if (args.count("ad") != 0)
        {
            if (args.at("ad") != devInfo.at("ad"))
            {
                continue;
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Found device by index %s", devInfo.at("ad").c_str());
        }
        else if (args.count("serial") != 0)
        {
            if (devInfo.at("serial") != args.at("serial"))
            {
                continue;
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Found device by serial %s", args.at("serial").c_str());
        }
        else if (args.count("label") != 0)
        {
            if (devInfo.at("label") != args.at("label"))
            {
                continue;
            }
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Found device by label %s", args.at("label").c_str());
        }
        results.push_back(SoapyADSDR::m_ad_devices[i]);
    }
    return results;
}

static SoapySDR::Device* makeADSDR(const SoapySDR::Kwargs& args)
{
    return new SoapyADSDR(args);
}

static SoapySDR::Registry registerADSDR("adsdr", &findADSDR, &makeADSDR, SOAPY_SDR_ABI_VERSION);

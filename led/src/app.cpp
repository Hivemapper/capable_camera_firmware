/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * app.cpp - Main entry point for LED controller application
 */

#include <chrono>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>

#include <signal.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "app_options.hpp"
#include "i2c.hpp"
#include "inotify.hpp"
#include "is31fl3199.hpp"

using namespace std::string_literals;
using namespace std::chrono_literals;

bool doExit = false;
bool doUpdate = false;
AppOptions *gOptions = nullptr;

bool ValidateItem(json &object, std::string name, json::value_t type);
template <typename T>
bool GetValue(json &object, std::string name, json::value_t type, T &value);

void SigHandle(int sigNum)
{
    if ((gOptions != nullptr) && (gOptions->verbose == true))
    {
        std::cerr << "Received signal: " << sigNum << std::endl;
    }

    if (sigNum == SIGINT)
    {
        doExit = true;
    }
}

void InotifyCallback(std::string &fileName)
{
    if ((gOptions != nullptr) && (gOptions->verbose == true))
    {
        std::cerr << "File name: " << fileName << std::endl;
    }

    doUpdate = true;
}

int main(int argc, char *argv[])
{
    bool optionsValid = false;
    AppOptions options; 
            
    // Handle parsing input arguments
    try
    {
        if (options.Parse(argc, argv))
        {
            if (options.verbose)
            {
                options.Print();
            }
            optionsValid = true;
        }
    }
    catch (std::exception const &e)
    {
        std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
        return -1;
    }
    
    // If the options are valid, continue with the application
    if(optionsValid)
    {
        // Setup Sig handler
        gOptions = &options;
        signal(SIGINT, &SigHandle);

        // Setup I2C interface
        const I2c::Value reg = 0x64u;
        std::shared_ptr<Interface> ledI2c = 
            std::make_shared<I2c>("/dev/i2c-1", reg, options.verbose);

        // Setup LED controller 
        Is31fl3199 leds(ledI2c);
        Inotify::FileList files{options.fileName};

        // Setup Inotify infrastructure
        std::string fullPath = options.path + options.fileName;
        Inotify inotify(
            IN_CREATE | IN_MODIFY,
            options.path,
            files,
            milliseconds(options.refreshRate),
            options.verbose,
            options.debugLevel
        );
        inotify.SetChangeCallback(&InotifyCallback);
        inotify.Start();

        // Loop until we get a signal to exit
        while (doExit == false)
        {
            // Check to see if we got an inotify hit
            if (doUpdate == true)
            {
                if (options.verbose == true)
                {
                    std::cout << "Starting LED update..." << std::endl;
                }

                // Open a stream to the file
                std::fstream file(fullPath);
                if (file.is_open() == true)
                {
                    // Stream in the json data
                    json ledConfig;
                    file >> ledConfig;

                    // Check that json object has leds element and it's an array
                    if (ValidateItem(ledConfig, "leds", json::value_t::array) == true)
                    {
                        // Loop through all elements
                        for (json object : ledConfig["leds"])
                        {
                            int index = -1;
                            unsigned int red = 0;
                            unsigned int blue = 0;
                            unsigned int green = 0;
                            bool state = false;

                            // Get all elements that we expect
                            GetValue(object, "index", json::value_t::number_integer, index);
                            GetValue(object, "red", json::value_t::number_unsigned, red);
                            GetValue(object, "blue", json::value_t::number_unsigned, blue);
                            GetValue(object, "green", json::value_t::number_unsigned, green);
                            GetValue(object, "on", json::value_t::boolean, state);

                            // Update the LED if an index was provided
                            if (index != -1)
                            {
                                LedCtrlr::LedData newColor{
                                    (Interface::Value)(red & 0xFFu),
                                    (Interface::Value)(blue & 0xFFu),
                                    (Interface::Value)(green & 0xFFu)
                                };
                                leds.SetColor(index, newColor);
                                leds.SetState(index, state);
                                
                                if (options.verbose == true)
                                {
                                    std::cout << "Set LED values!" << std::endl;
                                }
                            };
                        }
                    }

                    // CLose the file
                    file.close();
                }
                else if (options.verbose == true)
                {
                    std::cout << "Unable to open file \"" << fullPath << "\""
                        << std::endl;
                }

                if (options.verbose == true)
                {
                    std::cout << "File handling complete!" << std::endl;
                }

                doUpdate = false;
            }

            if (options.verbose == true)
            {
                std::cout << "Looping..." << std::endl;
            }
            usleep(options.refreshRate * 1000);
        }

        if (options.verbose == true)
        {
            std::cout << "Stopping inotify thread..." << std::endl;
        }
        inotify.Stop();
    }
    
    return 0;
}

bool ValidateItem(json &object, std::string name, json::value_t type)
{
    bool status = false;

    if ((object.contains(name) == true) && (object[name].type() == type))
    {
        status = true;
    }

    return status;
}

template <typename T>
bool GetValue(json &object, std::string name, json::value_t type, T &value)
{
    bool status = false;

    if (object.contains(name) == true)
    {        
        json::value_t objectType = object[name].type();
        if (((typeid(T) == typeid(std::string)) && (objectType == json::value_t::string)) ||
            ((typeid(T) == typeid(bool)) && (objectType == json::value_t::boolean)) ||
            ((typeid(T) == typeid(int)) && ((objectType == json::value_t::number_integer) || (objectType == json::value_t::number_unsigned))) ||
            ((typeid(T) == typeid(unsigned int)) && (objectType == json::value_t::number_unsigned)) ||
            ((typeid(T) == typeid(double)) && (objectType == json::value_t::number_float)))
        {
            value = object[name];

            if ((gOptions != nullptr) && (gOptions->verbose == true))
            {
                std::cout << name << ": " << value << std::endl;
            }
        }
    }

    return status;
}
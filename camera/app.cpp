/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2021 Chris Osterwood for Capable Robot Components, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * Based on raspberrypi/libcamera-apps/libcamera_vid.cpp - libcamera video record app.
 */

#include <poll.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <sys/stat.h>

#include <iomanip>
#include <chrono>
#include <thread>

#include "core/libcamera_encoder.hpp"
#include "network/output.hpp"
#include "network/net_input.hpp"

using json = nlohmann::json;
using namespace std::placeholders;

// Some keypress/signal handling.

static int signal_received;
static void default_signal_handler(int signal_number)
{
  signal_received = signal_number;
  std::cout << "Received signal " << signal_number << std::endl;
}

static int get_key_or_signal(VideoOptions const *options, pollfd p[1])
{
  int key = 0;
  if (options->keypress)
  {
    poll(p, 1, 0);
    if (p[0].revents & POLLIN)
    {
      char *user_string = nullptr;
      size_t len;
      [[maybe_unused]] size_t r = getline(&user_string, &len, stdin);
      key = user_string[0];
    }
  }
  if (options->signal)
  {
    if (signal_received == SIGUSR1)
      key = '\n';
    else if (signal_received == SIGUSR2)
      key = 'x';
  }
  return key;
}

// The main even loop for the application.
static void execute_stream(LibcameraEncoder &app, VideoOptions *options)
{

  std::unique_ptr<Output> output = std::unique_ptr<Output>(Output::Create(options));
  app.SetEncodeOutputReadyCallback(std::bind(&Output::OutputReady, output.get(), _1, _2, _3, _4, _5, _6, _7, _8));
  app.StartEncoder();

  app.OpenCamera();
  app.ConfigureVideo();
  app.StartCamera();

  std::cout << "Starting Stream" << std::endl;

  // Monitoring for keypresses and signals.
  signal(SIGUSR1, default_signal_handler);
  signal(SIGUSR2, default_signal_handler);
  pollfd p[1] = { { STDIN_FILENO, POLLIN, 0 } };

  bool end_early = false;
  auto last_entry_time = std::chrono::high_resolution_clock::now();
  auto start_time      = last_entry_time;
  auto after_msg_time  = last_entry_time;
  auto real_wait  = last_entry_time;
  auto after_enc_time  = last_entry_time;

  for (unsigned int count = 0; !end_early; count++)
  {
    start_time = std::chrono::high_resolution_clock::now();
    LibcameraEncoder::Msg msg = app.Wait();
    real_wait = std::chrono::high_resolution_clock::now();
    if (msg.type == LibcameraEncoder::MsgType::Quit)
    {
      end_early = true;
      break;
    }
    else if (msg.type != LibcameraEncoder::MsgType::RequestComplete)
    {
      std::cout << "Unrecognized message!" << std::endl;
      end_early = true;
      break;
    }
    int key = get_key_or_signal(options, p);
    if (key == '\n')
      output->Signal();

    after_msg_time = std::chrono::high_resolution_clock::now();

    CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
    app.EncodeBuffer(completed_request, app.VideoStream());

    after_enc_time = std::chrono::high_resolution_clock::now();

//    if (options->verbose)
//    {
//      std::chrono::duration<double> diff1 = after_msg_time - start_time;
//      std::chrono::duration<double> diff2 = after_enc_time - after_msg_time;
//      std::chrono::duration<double> diff3 = real_wait -  start_time;
//      std::cout << "Frame # " << std::setw(6) << count << std::endl;
//      std::cout << "Wait Time: " << diff3.count() << " / / " << diff1.count() << std::endl;
//      std::cout << "Encode Time: " << diff2.count() << std::endl;
//
//    }
    last_entry_time = start_time;
  }
  
  app.StopCamera();
  app.StopEncoder();
  std::cout << "Stream destroyed" << std::endl;
}

int main(int argc, char *argv[])
{
  try
  {
    LibcameraEncoder app;
    VideoOptions *options = app.GetOptions();
    
    if (options->Parse(argc, argv))
    {
      if (options->verbose)
      {
        options->Print();
      }
    }
    execute_stream(app, options);
    app.Teardown();
    app.CloseCamera(); 
  }
  catch (std::exception const &e)
  {
    std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
    return -1;
  }
  return 0;
}

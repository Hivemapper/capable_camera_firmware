/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.cpp - mjpeg video encoder.
 */

#include <chrono>
#include <iostream>

#include <jpeglib.h>

#include "mjpeg_encoder.hpp"

#if JPEG_LIB_VERSION_MAJOR > 9 || (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
typedef size_t jpeg_mem_len_t;
#else
typedef unsigned long jpeg_mem_len_t;
#endif

MjpegEncoder::MjpegEncoder(VideoOptions const *options)
	: Encoder(options), abort_(false), index_(0)
{
    //for(int ii = 0; ii < NUM_OUT_THREADS; ii+=1)
    //{
    //    output_thread_[ii] = std::thread(&MjpegEncoder::outputThread, this);
    //}
    output_thread_ = std::thread(&MjpegEncoder::outputThread, this);
    for (int ii = 0; ii < NUM_ENC_THREADS; ii+=1)
    {
        encode_thread_[ii] = std::thread(std::bind(&MjpegEncoder::encodeThread, this, ii));
    }
    if (options_->verbose)
    {
        std::cerr << "Opened MjpegEncoder" << std::endl;
    }
}

MjpegEncoder::~MjpegEncoder()
{
	abort_ = true;
	for (int i = 0; i < NUM_ENC_THREADS; i++)
		encode_thread_[i].join();
	output_thread_.join();
	if (options_->verbose)
		std::cerr << "MjpegEncoder closed" << std::endl;
}

void MjpegEncoder::EncodeBuffer(int fd, size_t size, void *mem, unsigned int width, unsigned int height,
								unsigned int stride, int64_t timestamp_us)
{
  //2022-08-15 CNIESSL: Moved the EncodeItem encapsulation here
  EncodeItem item = { mem, width, height, stride, timestamp_us, index_++ };

  std::lock_guard<std::mutex> lock(encode_mutex_);
  //TODO: Move set up one queue per thread.
  //TODO: One conditional notification var per thread
  encode_queue_.push(item);
  encode_cond_var_.notify_all();
}

void MjpegEncoder::encodeJPEG(struct jpeg_compress_struct &cinfo, EncodeItem &item, uint8_t *&encoded_buffer,
							  size_t &buffer_len)
{
	// Copied from YUV420_to_JPEG_fast in jpeg.cpp.
	cinfo.image_width = item.width;
	cinfo.image_height = item.height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = 0;

	jpeg_set_defaults(&cinfo);
	cinfo.raw_data_in = TRUE;
	jpeg_set_quality(&cinfo, options_->quality, TRUE);
	encoded_buffer = nullptr;
	buffer_len = 0;
	jpeg_mem_len_t jpeg_mem_len;
	jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);
	jpeg_start_compress(&cinfo, TRUE);

	int stride2 = item.stride / 2;
	uint8_t *Y = (uint8_t *)item.mem;
	uint8_t *U = (uint8_t *)Y + item.stride * item.height;
	uint8_t *V = (uint8_t *)U + stride2 * (item.height / 2);
	uint8_t *Y_max = U - item.stride;
	uint8_t *U_max = V - stride2;
	uint8_t *V_max = U_max + stride2 * (item.height / 2);

	JSAMPROW y_rows[16];
	JSAMPROW u_rows[8];
	JSAMPROW v_rows[8];

	for (uint8_t *Y_row = Y, *U_row = U, *V_row = V; cinfo.next_scanline < item.height;)
	{
		for (int i = 0; i < 16; i++, Y_row += item.stride)
			y_rows[i] = std::min(Y_row, Y_max);
		for (int i = 0; i < 8; i++, U_row += stride2, V_row += stride2)
			u_rows[i] = std::min(U_row, U_max), v_rows[i] = std::min(V_row, V_max);

		JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
		jpeg_write_raw_data(&cinfo, rows, 16);
	}

	jpeg_finish_compress(&cinfo);
	buffer_len = jpeg_mem_len;
}

void MjpegEncoder::encodeThread(int num)
{
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  std::chrono::duration<double> encode_time(0);
  uint32_t frames = 0;

  EncodeItem encode_item;
  while (true)
  {
    {
      std::unique_lock<std::mutex> lock(encode_mutex_);
      while (true)
      {
        using namespace std::chrono_literals;
        if (abort_)
        {
          if (frames && options_->verbose)
          {
            std::cerr << "Encode " << frames << " frames, average time "
                      << encode_time.count() * 1000 / frames << std::endl;
          }
          jpeg_destroy_compress(&cinfo);
          return;
        }
        if (!encode_queue_.empty())
        {
          encode_item = encode_queue_.front();
          encode_queue_.pop();
          break;
        }
        else
	    {
	      encode_cond_var_.wait_for(lock, 200ms);
        }
      } 
    }
     
    // Encode the buffer.
    uint8_t *encoded_buffer = nullptr;
    size_t buffer_len = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    encodeJPEG(cinfo, encode_item, encoded_buffer, buffer_len);
    encode_time += (std::chrono::high_resolution_clock::now() - start_time);
    frames += 1;
     
    // Don't return buffers until the output thread as that's where they're
    // in order again.
    // We push this encoded buffer to another thread so that our
    // application can take its time with the data without blocking the
    // encode process.
     
    OutputItem output_item = { encoded_buffer, buffer_len, encode_item.timestamp_us, encode_item.index };
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        output_queue_[num].push(output_item);
        output_cond_var_.notify_one();
    }
  }
}

void MjpegEncoder::outputThread()
{
  OutputItem item;
  uint64_t index = 0;
  while (true)
  {
    {
      std::unique_lock<std::mutex> lock(output_mutex_);
      while (true)
      {
        using namespace std::chrono_literals;
        if (abort_)
        {
	      return;
	    }
	  
        // We look for the thread that's completed the frame we want next.
        // If we don't find it, we wait.
        for (auto &q : output_queue_)
        {
          if (!q.empty() && q.front().index == index)
          {
            item = q.front();
            q.pop();
            goto got_item;
          }
        }
        output_cond_var_.wait_for(lock, 200ms);
      }
    }
    got_item:
      input_done_callback_(nullptr);
      output_ready_callback_(item.mem, item.bytes_used, item.timestamp_us, true);
      free(item.mem);
      index+=1;
  }
}

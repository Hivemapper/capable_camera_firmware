/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * h264_encoder.hpp - h264 video encoder.
 */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "encoder.hpp"

class H264Encoder : public Encoder
{
public:
	H264Encoder(VideoOptions const *options);
	~H264Encoder();
	// Encode the given DMABUF.
	void EncodeBuffer(int fd, size_t size, void *mem, unsigned int width, unsigned int height, unsigned int stride,
					  int64_t timestamp_us, libcamera::ControlList metadata) override;

private:
	// We want at least as many output buffers as there are in the camera queue
	// (we always want to be able to queue them when they arrive). Make loads
	// of capture buffers, as this is our buffering mechanism in case of delays
	// dealing with the output bitstream.
	static constexpr int NUM_OUTPUT_BUFFERS = 6;
	static constexpr int NUM_CAPTURE_BUFFERS = 12;

	// This thread just sits waiting for the encoder to finish stuff. It will either:
	// * receive "output" buffers (codec inputs), which we must return to the caller
	// * receive encoded buffers, which we pass to the application.
	void pollThread();

	// Handle the output buffers in another thread so as not to block the encoder. The
	// application can take its time, after which we return this buffer to the encoder for
	// re-use.
	void outputThread();

	bool abort_;
	int fd_;
	struct BufferDescription
	{
		void *mem;
		size_t size;
	};
	BufferDescription buffers_[NUM_CAPTURE_BUFFERS];
	std::thread poll_thread_;
	std::mutex input_buffers_available_mutex_;
	std::queue<int> input_buffers_available_;
	struct OutputItem
	{
		void *mem;
		size_t bytes_used;
		size_t length;
		unsigned int index;
		bool keyframe;
		int64_t timestamp_us;
	};
	std::queue<OutputItem> output_queue_;
	std::mutex output_mutex_;
	std::condition_variable output_cond_var_;
	std::thread output_thread_;
};

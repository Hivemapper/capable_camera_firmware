/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.hpp - mjpeg video encoder.
 */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "encoder.hpp"

struct jpeg_compress_struct;

class MjpegEncoder : public Encoder
{
public:
	MjpegEncoder(VideoOptions const *options);
	~MjpegEncoder();
	// Encode the given buffer.
	void EncodeBuffer(int fd, size_t size, void *mem, unsigned int width, unsigned int height, unsigned int stride,
					  int64_t timestamp_us, libcamera::ControlList metadata) override;

private:
	// How many threads to use. Whichever thread is idle will pick up the next frame.
	static const int NUM_ENC_THREADS = 3;
	static const int NUM_OUT_THREADS = 4;

	// These threads do the actual encoding.
	void encodeThread(int num);

	// Handle the output buffers in another thread so as not to block the encoders. The
	// application can take its time, after which we return this buffer to the encoder for
	// re-use.
	void outputThread();

	bool abort_;
    bool doDownsample_;
	uint64_t index_;

	struct EncodeItem
	{
		void *mem;
        size_t size;
		unsigned int width;
		unsigned int height;
		unsigned int stride;
		int64_t timestamp_us;
		uint64_t index;
	};
	std::queue<EncodeItem> encode_queue_;
	std::mutex encode_mutex_;
	std::condition_variable encode_cond_var_;
	std::thread encode_thread_[NUM_ENC_THREADS];

    bool didInitDSI_;

    uint32_t oldHalfStride_;
    uint32_t newStride_;
    uint32_t newHeight_;
    uint32_t newSize_;
    uint8_t* newBuffer_[NUM_ENC_THREADS];

    void initDownSampleInfo(EncodeItem &source);

	void encodeJPEG(struct jpeg_compress_struct &cinfo,
                    EncodeItem &item,
                    uint8_t *&encoded_buffer,
                    size_t &buffer_len,
                    int num);
    void legacyEncodeJPEG(struct jpeg_compress_struct &cinfo, EncodeItem &item, uint8_t *&encoded_buffer,
                     size_t &buffer_len);
    void encodeDownsampleJPEG(struct jpeg_compress_struct &cinfo,
                              EncodeItem &source,
                              uint8_t *&encoded_buffer,
                              size_t &buffer_len,
                              int num);
    void CreateExifData(libcamera::ControlList metadata,
                        uint8_t *&exif_buffer,
                        unsigned int &exif_len);
	struct OutputItem
	{
		void *mem;
		size_t bytes_used;
        void *preview_mem;
        size_t preview_bytes_used;
        void *exif_mem;
        size_t exif_bytes_used;
		int64_t timestamp_us;
		uint64_t index;
	};
	std::queue<OutputItem> output_queue_[NUM_ENC_THREADS];
	std::mutex output_mutex_;
	std::condition_variable output_cond_var_;
	std::thread output_thread_;

	void print(std::string toPrint);
};

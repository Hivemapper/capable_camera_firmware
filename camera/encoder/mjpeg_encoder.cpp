/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.cpp - mjpeg video encoder.
 */

#include <chrono>
#include <iostream>

#include <jpeglib.h>
#include <libyuv.h>
#include <libexif/exif-data.h>
#include <boost/filesystem.hpp>
#include "mjpeg_encoder.hpp"

//#include <Magick++.h>

#if JPEG_LIB_VERSION_MAJOR > 9 || (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
typedef size_t jpeg_mem_len_t;
#else
typedef unsigned long jpeg_mem_len_t;
#endif

/*
 * EXIF data functions from libcamera-apps
 */
static const ExifByteOrder exif_byte_order = EXIF_BYTE_ORDER_INTEL;

struct ExifException
{
    ExifFormat format;
    unsigned int components; // can be zero for "variable/unknown"
};

typedef int (*ExifReadFunction)(char const *, unsigned char *);

static int exif_read_short(char const *str, unsigned char *mem);
static int exif_read_sshort(char const *str, unsigned char *mem);
static int exif_read_long(char const *str, unsigned char *mem);
static int exif_read_slong(char const *str, unsigned char *mem);
static int exif_read_rational(char const *str, unsigned char *mem);
static int exif_read_srational(char const *str, unsigned char *mem);

static ExifEntry *exif_create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag);
static void exif_set_string(ExifEntry *entry, char const *s);

static const unsigned char exif_header[] = { 0xff, 0xd8, 0xff, 0xe1 };

// libexif knows the formats of many tags, but not all (I mean, why not?!?).
// Exceptions can be listed here.
static std::map<ExifTag, ExifException> exif_exceptions =
        {
                { EXIF_TAG_YCBCR_COEFFICIENTS, { EXIF_FORMAT_RATIONAL, 3 } },
        };

static std::map<std::string, ExifIfd> exif_ifd_map =
        {
                { "EXIF", EXIF_IFD_EXIF },
                { "IFD0", EXIF_IFD_0 },
                { "IFD1", EXIF_IFD_1 },
                { "EINT", EXIF_IFD_INTEROPERABILITY },
                { "GPS",  EXIF_IFD_GPS }
        };

static ExifReadFunction const exif_read_functions[] =
        {
                // Same order as ExifFormat enum.
                nullptr, // dummy
                nullptr, // byte
                nullptr, // ascii
                exif_read_short,
                exif_read_long,
                exif_read_rational,
                nullptr, // sbyte
                nullptr, // undefined
                exif_read_sshort,
                exif_read_slong,
                exif_read_srational
        };

int exif_read_short(char const *str, unsigned char *mem)
{
  unsigned short value;
  int n;
  if (sscanf(str, "%hu%n", &value, &n) != 1)
    throw std::runtime_error("failed to read EXIF unsigned short");
  exif_set_short(mem, exif_byte_order, value);
  return n;
}

int exif_read_sshort(char const *str, unsigned char *mem)
{
  short value;
  int n;
  if (sscanf(str, "%hd%n", &value, &n) != 1)
    throw std::runtime_error("failed to read EXIF signed short");
  exif_set_sshort(mem, exif_byte_order, value);
  return n;
}

int exif_read_long(char const *str, unsigned char *mem)
{
  uint32_t value;
  int n;
  if (sscanf(str, "%u%n", &value, &n) != 1)
    throw std::runtime_error("failed to read EXIF unsigned short");
  exif_set_long(mem, exif_byte_order, value);
  return n;
}

int exif_read_slong(char const *str, unsigned char *mem)
{
  int32_t value;
  int n;
  if (sscanf(str, "%d%n", &value, &n) != 1)
    throw std::runtime_error("failed to read EXIF signed short");
  exif_set_slong(mem, exif_byte_order, value);
  return n;
}

int exif_read_rational(char const *str, unsigned char *mem)
{
  uint32_t num, denom;
  int n;
  if (sscanf(str, "%u/%u%n", &num, &denom, &n) != 2)
    throw std::runtime_error("failed to read EXIF unsigned rational");
  exif_set_rational(mem, exif_byte_order, { num, denom });
  return n;
}

int exif_read_srational(char const *str, unsigned char *mem)
{
  int32_t num, denom;
  int n;
  if (sscanf(str, "%d/%d%n", &num, &denom, &n) != 2)
    throw std::runtime_error("failed to read EXIF signed rational");
  exif_set_srational(mem, exif_byte_order, { num, denom });
  return n;
}

ExifEntry *exif_create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag)
{
  ExifEntry *entry = exif_content_get_entry(exif->ifd[ifd], tag);
  if (entry)
    return entry;
  entry = exif_entry_new();
  if (!entry)
    throw std::runtime_error("failed to allocate EXIF entry");
  entry->tag = tag;
  exif_content_add_entry(exif->ifd[ifd], entry);
  exif_entry_initialize(entry, entry->tag);
  exif_entry_unref(entry);
  return entry;
}

void exif_set_string(ExifEntry *entry, char const *s)
{
  if (entry->data)
    free(entry->data);
  entry->size = entry->components = strlen(s);
  entry->data = (unsigned char *)strdup(s);
  if (!entry->data)
    throw std::runtime_error("failed to copy exif string");
  entry->format = EXIF_FORMAT_ASCII;
}

MjpegEncoder::MjpegEncoder(VideoOptions const *options)
	: Encoder(options), abort_(false), index_(0)
{
    output_thread_ = std::thread(&MjpegEncoder::outputThread, this);
    for (int ii = 0; ii < NUM_ENC_THREADS; ii+=1)
    {
        encode_thread_[ii] = std::thread(std::bind(&MjpegEncoder::encodeThread, this, ii));
    }
    if (options_->verbose)
    {
        std::cerr << "Opened MjpegEncoder" << std::endl;
    }
    if (options_->downsampleStreamDir != "")
    {
      std::cerr << "Opening downsample stream at " << options_->downsampleStreamDir << std::endl;
      doDownsample_ = true;
    }
    else
    {
      doDownsample_ = false;
    }
    //If both primary and secondary/usb are not writing at main spec, then we turn off
    //Full resolution rendering.
    {
      bool writePrim = false;
      bool writeSec  = false;
      if(options_->output != "" && boost::filesystem::exists(options_->output))
      {
        writePrim = true;
      }
      if(options_->output_2nd != "" && boost::filesystem::exists(options_->output_2nd))
      {
        writeSec = true;
      }
      if(writePrim || writeSec)
      {
        doPrimsample_ = true;
      }
      else
      {
        doPrimsample_ = false;
      }
    }
    didInitDSI_ = false;
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
								unsigned int stride, int64_t timestamp_us, libcamera::ControlList metadata)
{
    int32_t newExpoTime = metadata.get(libcamera::controls::ExposureTime);
    float   newAlogGain = metadata.get(libcamera::controls::AnalogueGain);
    float   newDigiGain = metadata.get(libcamera::controls::DigitalGain);

    EncodeItem item = { mem,
                        size,
                        width,
                        height,
                        stride,
                        timestamp_us,
                        newExpoTime,
                        newAlogGain,
                        newDigiGain,
                        index_++ };
	std::lock_guard<std::mutex> lock(encode_mutex_);
    if(!didInitDSI_)
    {
      initDownSampleInfo(item);
    }
	encode_queue_.push(item);
	encode_cond_var_.notify_all();
}

void MjpegEncoder::initDownSampleInfo(EncodeItem &source)
{
  if(options_->verbose)
  {
    std::cout << "Initializing downsample structures" << std::endl;
  }

  oldHalfStride_ = source.stride / 2;
  newStride_     = oldHalfStride_ - (oldHalfStride_ % 8) + 8;
  newHeight_     = (source.height / 2);
  newSize_       = newStride_ * newHeight_;

  for(int ii = 0; ii < NUM_ENC_THREADS; ii+=1)
  {
    newBuffer_[ii] = (uint8_t*)malloc(newSize_);
  }

  didInitDSI_ = true;
}

void MjpegEncoder::CreateExifData(EncodeItem &source,
                                  uint8_t *&exif_buffer,
                                  unsigned int &exif_len)
{
    exif_buffer = nullptr;
    ExifData *exif = nullptr;

    try
    {
        exif = exif_data_new();
        if (!exif)
        {
          throw std::runtime_error("failed to allocate EXIF data");
        }

        exif_data_set_byte_order(exif, exif_byte_order);
        ExifEntry *entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_MAKE);
        exif_set_string(entry, "Raspberry Pi CM4");
        entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_MODEL);
        exif_set_string(entry, "IMX477");
        entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_SOFTWARE);
        exif_set_string(entry, "capable-camera bridge");
        entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME);

        std::time_t raw_time;
        std::time(&raw_time);
        std::tm *time_info;
        char time_string[32];
        time_info = std::localtime(&raw_time);
        std::strftime(time_string, sizeof(time_string), "%Y:%m:%d %H:%M:%S", time_info);
        exif_set_string(entry, time_string);
        entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME_ORIGINAL);
        exif_set_string(entry, time_string);
        entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_DATE_TIME_DIGITIZED);
        exif_set_string(entry, time_string);

        // Now add some tags filled in from the image metadata.
        auto exposure_time = metadata.get(libcamera::controls::ExposureTime);
        if (exposure_time)
        {
          entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_EXPOSURE_TIME);
          ExifRational exposure = { (ExifLong)exposure_time, 1000000 };
          exif_set_rational(entry->data, exif_byte_order, exposure);
        }

        auto ag = metadata.get(libcamera::controls::AnalogueGain);
        if (ag)
        {
            entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_ISO_SPEED_RATINGS);
            auto dg = metadata.get(libcamera::controls::DigitalGain);
            float gain;
            gain = ag * (dg ? dg : 1.0);
            exif_set_short(entry->data, exif_byte_order, 100 * gain);
        }

        // And create the EXIF data buffer *again*.
        exif_data_save_data(exif, &exif_buffer, &exif_len);
        exif_data_unref(exif);
        exif = nullptr;
    }
    catch (std::exception const &e)
    {
        if (exif)
            exif_data_unref(exif);
        //if (exif_buffer)
        //    free(exif_buffer);
        throw;
    }
}

void MjpegEncoder::encodeDownsampleJPEG(struct jpeg_compress_struct &cinfo,
                                        EncodeItem &source,
                                        uint8_t *&encoded_buffer,
                                        size_t &buffer_len,
                                        int num)
{
  (void)num;

  uint8_t *Y_src = (uint8_t *)source.mem;
  uint8_t *U_src = (uint8_t *)Y_src + source.stride * source.height;
  uint8_t *V_src = (uint8_t *)U_src + oldHalfStride_  * newHeight_;

  libyuv::ScalePlane(Y_src, source.stride, source.width, source.height, newBuffer_[num],
                     newStride_, source.width / 2, newHeight_, libyuv::kFilterBox);

  uint8_t *Y_max = newBuffer_[num] + (newStride_ * (newHeight_ - 1));
  uint8_t *U_max = V_src - oldHalfStride_;
  uint8_t *V_max = U_max + oldHalfStride_ * newHeight_;

  cinfo.image_width = source.width / 2;
  cinfo.image_height = newHeight_;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_YCbCr;
  cinfo.jpeg_color_space = JCS_YCbCr;
  cinfo.restart_interval = 0;

  jpeg_set_defaults(&cinfo);
  cinfo.raw_data_in = TRUE;
  cinfo.comp_info[0].h_samp_factor = 1;
  cinfo.comp_info[0].v_samp_factor = 1;
  cinfo.comp_info[1].h_samp_factor = 1;
  cinfo.comp_info[1].v_samp_factor = 1;
  cinfo.comp_info[2].h_samp_factor = 1;
  cinfo.comp_info[2].v_samp_factor = 1;

  jpeg_set_quality(&cinfo, options_->qualityDwn, TRUE);
  buffer_len = 0;
  jpeg_mem_len_t jpeg_mem_len;
  jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);

  JSAMPROW y_rows[8];
  JSAMPROW u_rows[8];
  JSAMPROW v_rows[8];

  jpeg_start_compress(&cinfo, TRUE);
  for (uint8_t *Y_row = newBuffer_[num], *U_row = U_src, *V_row = V_src; cinfo.next_scanline < newHeight_;)
  {
    unsigned int linesToWrite = 0;
    for (; linesToWrite < 8 && (linesToWrite + cinfo.next_scanline < newHeight_); linesToWrite+=1)
    {
      y_rows[linesToWrite] = std::min(Y_row, Y_max);
      u_rows[linesToWrite] = std::min(U_row, U_max);
      v_rows[linesToWrite] = std::min(V_row, V_max);
      Y_row += newStride_;
      U_row += oldHalfStride_;
      V_row += oldHalfStride_;
    }
    if (linesToWrite > 0)
    {
      JSAMPARRAY rows[] = {y_rows, u_rows, v_rows};
      jpeg_write_raw_data(&cinfo, rows, linesToWrite);
    }
  }
  jpeg_finish_compress(&cinfo);
  buffer_len = jpeg_mem_len;
}

void MjpegEncoder::encodeJPEG(struct jpeg_compress_struct &cinfo, EncodeItem &item, uint8_t *&encoded_buffer,
							  size_t &buffer_len, int num)
{
  (void)num;
	// Copied from YUV420_to_JPEG_fast in jpeg.cpp.
	cinfo.image_width = item.width;
	cinfo.image_height = item.height;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_YCbCr;
	cinfo.restart_interval = 0;

	jpeg_set_defaults(&cinfo);
	cinfo.raw_data_in = TRUE;
	jpeg_set_quality(&cinfo, options_->quality, TRUE);
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
  struct jpeg_compress_struct cinfoMain;
  struct jpeg_compress_struct cinfoPrev;
  struct jpeg_error_mgr jerr;

  cinfoMain.err = jpeg_std_error(&jerr);
  cinfoPrev.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfoMain);
  jpeg_create_compress(&cinfoPrev);

  std::chrono::duration<double> encode_time(0);
  uint32_t frames = 0;
  uint32_t index = 0;
  EncodeItem encode_item;

  // Preallocate buffers for better performance
  uint8_t *encoded_buffer[NUM_FRAMES];
  uint8_t *encoded_prev_buffer[NUM_FRAMES];
  uint8_t *exif_buffer[NUM_FRAMES];
  size_t buffer_len[NUM_FRAMES];
  size_t buffer_prev_len[NUM_FRAMES];
  size_t exif_buffer_len[NUM_FRAMES];
  for (int ii = 0; ii < NUM_FRAMES; ii+=1)
  {
    encoded_buffer[ii] = (uint8_t*)malloc(MAX_FRAME_MEMORY);
    encoded_prev_buffer[ii] = (uint8_t*)malloc(MAX_FRAME_MEMORY / 2);
    exif_buffer[ii] = nullptr;
    buffer_len[ii] = 0;
    buffer_prev_len[ii] = 0;
    exif_buffer_len[ii] = 0;
  }

  while (true) {
    {
      std::unique_lock <std::mutex> lock(encode_mutex_);
      while (true) {
        using namespace std::chrono_literals;
        if (abort_) {
          if (frames && options_->verbose) {
            std::cerr << "Encode " << frames << " frames, average time "
                      << encode_time.count() * 1000 / frames << std::endl;
          }
          jpeg_destroy_compress(&cinfoMain);
          jpeg_destroy_compress(&cinfoPrev);
          return;
        }
        if (!encode_queue_.empty()) {
          encode_item = encode_queue_.front();
          encode_queue_.pop();
          break;
        } else {
          encode_cond_var_.wait_for(lock, 200ms);
        }
      }
    }
    index = frames % NUM_FRAMES;
    {
      auto start_time = std::chrono::high_resolution_clock::now();
      CreateExifData(libcamera::ControlList metadata, exif_buffer, exif_buffer_len)
      if(doPrimsample_)
      {
          encodeJPEG(cinfoMain, encode_item, encoded_buffer[index], buffer_len[index], num);
      }
      if(doDownsample_)
      {
          encodeDownsampleJPEG(cinfoPrev, encode_item, encoded_prev_buffer[index], buffer_prev_len[index], num);
      }
      encode_time += (std::chrono::high_resolution_clock::now() - start_time);
      if(options_->verbose && frames > 1)
      {
        std::cout << "Thread # " << num << " average time " << encode_time.count() * 1000 / frames << std::endl;
      }
    }
    frames += 1;
     
    // Don't return buffers until the output thread as that's where they're
    // in order again.
    // We push this encoded buffer to another thread so that our
    // application can take its time with the data without blocking the
    // encode process.

    OutputItem output_item = { encoded_buffer[index],
                               buffer_len[index],
                               encoded_prev_buffer[index],
                               buffer_prev_len[index],
                               exif_buffer[index],
                               exif_buffer_len[index],
                               encode_item.timestamp_us,
                               encode_item.index };
    {
        std::lock_guard<std::mutex> lock(output_mutex_[num]);
        output_queue_[num].push(output_item);
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
      while (true)
      {
        using namespace std::chrono_literals;
        if (abort_)
        {
	      return;
	    }
	  
        // We look for the thread that's completed the frame we want next.
        // If we don't find it, we wait.
        for (uint32_t ii = 0; ii < NUM_ENC_THREADS; ii+=1)
        {
          std::unique_lock<std::mutex> lock(output_mutex_[ii]);
          if (!output_queue_[ii].empty() && output_queue_[ii].front().index == index)
          {
            item = output_queue_[ii].front();
            output_queue_[ii].pop();
            goto got_item;
          }
        }
        std::this_thread::sleep_for (50ms);
      }
    }
    got_item:
      input_done_callback_(nullptr);
      output_ready_callback_(item.mem,
                             item.bytes_used,
                             item.preview_mem,
                             item.preview_bytes_used,
                             item.timestamp_us,
                             true);
      index+=1;
  }
}

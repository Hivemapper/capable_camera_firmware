/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.cpp - mjpeg video encoder.
 */

#include <chrono>
#include <iostream>

#include <jpeglib.h>
#include <turbojpeg.h>
#include <libyuv.h>
#include <libexif/exif-data.h>
#include <map>
#include <cstring>
#include "mjpeg_encoder.hpp"
#include <fcntl.h>


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

struct ExifException {
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

// libexif knows the formats of many tags, but not all (I mean, why not?!?).
// Exceptions can be listed here.
static std::map<ExifTag, ExifException> exif_exceptions =
        {
                {EXIF_TAG_YCBCR_COEFFICIENTS, {EXIF_FORMAT_RATIONAL, 3}},
        };

static std::map<std::string, ExifIfd> exif_ifd_map =
        {
                {"EXIF", EXIF_IFD_EXIF},
                {"IFD0", EXIF_IFD_0},
                {"IFD1", EXIF_IFD_1},
                {"EINT", EXIF_IFD_INTEROPERABILITY},
                {"GPS",  EXIF_IFD_GPS}
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

int exif_read_short(char const *str, unsigned char *mem) {
    unsigned short value;
    int n;
    if (sscanf(str, "%hu%n", &value, &n) != 1)
        throw std::runtime_error("failed to read EXIF unsigned short");
    exif_set_short(mem, exif_byte_order, value);
    return n;
}

int exif_read_sshort(char const *str, unsigned char *mem) {
    short value;
    int n;
    if (sscanf(str, "%hd%n", &value, &n) != 1)
        throw std::runtime_error("failed to read EXIF signed short");
    exif_set_sshort(mem, exif_byte_order, value);
    return n;
}

int exif_read_long(char const *str, unsigned char *mem) {
    uint32_t value;
    int n;
    if (sscanf(str, "%u%n", &value, &n) != 1)
        throw std::runtime_error("failed to read EXIF unsigned short");
    exif_set_long(mem, exif_byte_order, value);
    return n;
}

int exif_read_slong(char const *str, unsigned char *mem) {
    int32_t value;
    int n;
    if (sscanf(str, "%d%n", &value, &n) != 1)
        throw std::runtime_error("failed to read EXIF signed short");
    exif_set_slong(mem, exif_byte_order, value);
    return n;
}

int exif_read_rational(char const *str, unsigned char *mem) {
    uint32_t num, denom;
    int n;
    if (sscanf(str, "%u/%u%n", &num, &denom, &n) != 2)
        throw std::runtime_error("failed to read EXIF unsigned rational");
    exif_set_rational(mem, exif_byte_order, {num, denom});
    return n;
}

int exif_read_srational(char const *str, unsigned char *mem) {
    int32_t num, denom;
    int n;
    if (sscanf(str, "%d/%d%n", &num, &denom, &n) != 2)
        throw std::runtime_error("failed to read EXIF signed rational");
    exif_set_srational(mem, exif_byte_order, {num, denom});
    return n;
}

ExifEntry *exif_create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag) {
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

void exif_set_string(ExifEntry *entry, char const *s) {
    if (entry->data)
        free(entry->data);
    entry->size = entry->components = strlen(s);
    entry->data = (unsigned char *) strdup(s);
    if (!entry->data)
        throw std::runtime_error("failed to copy exif string");
    entry->format = EXIF_FORMAT_ASCII;
}

MjpegEncoder::MjpegEncoder(VideoOptions const *options)
        : Encoder(options), abort_(false), index_(0) {
    if (options_->verbose) {
        output_thread_ = std::thread(&MjpegEncoder::outputThread, this);
    }

    for (int ii = 0; ii < NUM_ENC_THREADS; ii += 1) {
        encode_thread_[ii] = std::thread(std::bind(&MjpegEncoder::encodeThread, this, ii));
    }
    if (options_->verbose) {
        std::cerr << "Opened MjpegEncoder" << std::endl;
    }
    if (options_->downsampleStreamDir != "") {
        std::cerr << "Opening downsample stream at " << options_->downsampleStreamDir << std::endl;
        doDownsample_ = true;

    }
    didInitDSI_ = false;
}

MjpegEncoder::~MjpegEncoder() {
    abort_ = true;
    for (int i = 0; i < NUM_ENC_THREADS; i++)
        encode_thread_[i].join();
    output_thread_.join();
    if (options_->verbose)
        std::cerr << "MjpegEncoder closed" << std::endl;
}

void MjpegEncoder::EncodeBuffer(int fd, size_t size, void *mem, unsigned int width, unsigned int height,
                                unsigned int stride, int64_t timestamp_us, libcamera::ControlList metadata) {
    EncodeItem item = {mem,
                       size,
                       width,
                       height,
                       stride,
                       timestamp_us,
                       index_++};
    std::lock_guard<std::mutex> lock(encode_mutex_);
    if (!didInitDSI_) {
        initDownSampleInfo(item);
    }
    encode_queue_.push(item);
    encode_cond_var_.notify_all();
}

void MjpegEncoder::initDownSampleInfo(EncodeItem &source) {
    if (options_->verbose) {
        std::cout << "Initializing downsample structures" << std::endl;
    }

    crop_width_ = options_->crop_width;
    crop_height_ = options_->crop_height;

    crop_stride_ = crop_width_;

    crop_half_height_ = (crop_height_ + 1) / 2;
    crop_stride2_ = crop_width_ / 2;

    crop_y_size_ = crop_stride_ * crop_height_;
    crop_uv_size_ = crop_stride2_ * crop_half_height_;

    crop_size_ = crop_y_size_ + crop_uv_size_ * 2;

    for (int ii = 0; ii < NUM_ENC_THREADS; ii += 1) {
        cropBuffer_[ii] = (uint8_t *) malloc(crop_size_);
    }

    didInitDSI_ = true;
}

void MjpegEncoder::CreateExifData(libcamera::ControlList metadata,
                                  uint8_t *&exif_buffer,
                                  unsigned int &exif_len) {
    exif_buffer = nullptr;
    ExifData *exif = nullptr;

    try {
        exif = exif_data_new();
        if (!exif) {
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
        if (exposure_time) {
            entry = exif_create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_EXPOSURE_TIME);
            ExifRational exposure = {(ExifLong) exposure_time, 1000000};
            exif_set_rational(entry->data, exif_byte_order, exposure);
        }

        auto ag = metadata.get(libcamera::controls::AnalogueGain);
        if (ag) {
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
    catch (std::exception const &e) {
        if (exif)
            exif_data_unref(exif);
        if (exif_buffer)
            free(exif_buffer);
        throw;
    }
}

//void MjpegEncoder::encodeJPEG(struct jpeg_compress_struct &cinfo, EncodeItem &item, uint8_t *&encoded_buffer,
//                              size_t &buffer_len, int num)
//{
//    (void)num;
//
//    //----------------------------------------------
//    //----------------------------------------------
//    // SRC
//    //----------------------------------------------
//    //----------------------------------------------
//    uint8_t *src_i420 = (uint8_t *)item.mem;
//
//    unsigned int src_width = item.width;
//    unsigned int src_height = item.height;
//    unsigned int src_stride = item.stride;
//
//    if (options_->verbose) {
//        std::cout << "encodeJPEG source: " <<
//        "\n width:" << item.width <<
//        "\n height:" << item.height <<
//        std::endl;
//    }
//
//    unsigned int src_half_height = (src_height + 1) / 2;
//    unsigned int src_stride2 = item.stride / 2;
//
//    unsigned int src_y_size = src_stride * src_height;
//    unsigned int src_uv_size = src_stride2 * src_half_height ;
//
//    int src_U_stride = src_stride2;
//    int src_V_stride = src_stride2;
//
//    uint8_t *src_Y = (uint8_t *)src_i420;
//    uint8_t *src_U = (uint8_t *)src_Y + src_y_size;
//    uint8_t *src_V = (uint8_t *)src_U + src_uv_size;
//
//
//    //----------------------------------------------
//    //----------------------------------------------
//    // CROP
//    //----------------------------------------------
//    //----------------------------------------------
//    //720 x 1280 — HD is 720p
//    //1920 x 1080 — FHD is 1080p
//    //2560 x 1440 — QHD (Quad HD) is 2K 1440p
//    //3840 x 2160 — UHD (Ultra HD) is 4K 2160p
//    //4056 x 3040 - Cam Raw
//
//    int crop_U_stride = crop_stride2_;
//    int crop_V_stride = crop_stride2_;
//
//    uint8_t *crop_Y = (uint8_t *)cropBuffer_[num];
//    uint8_t *crop_U = (uint8_t *)crop_Y + crop_y_size_;
//    uint8_t *crop_V = (uint8_t *)crop_U + crop_uv_size_;
//
//    //----------------------------------------------
//    //----------------------------------------------
//    // OUT
//    //----------------------------------------------
//    //----------------------------------------------
//    uint8_t *out_Y = (uint8_t *)cropBuffer_[num];
//    unsigned int out_stride = crop_stride_;
//    int out_half_stride = crop_stride2_;
//
//    uint8_t *out_U = (uint8_t *)out_Y + crop_y_size_;
//    uint8_t *out_V = (uint8_t *)out_U + crop_uv_size_;
//
//    uint8_t *Y_max = out_Y + crop_y_size_ - 1;
//    uint8_t *U_max = out_U + crop_uv_size_ - 1;
//    uint8_t *V_max = out_V + crop_uv_size_ - 1;
//
//
////    As shown in the above image, the Y′, U and V components in Y′UV420 are encoded separately in sequential blocks.
////    A Y′ value is stored for every pixel, followed by a U value for each 2×2 square block of pixels,
////    and finally a V value for each 2×2 block.
////    Corresponding Y′, U and V values are shown using the same color in the diagram above.
////    Read line-by-line as a byte stream from a device,
////    the Y′ block would be found at position 0,
////    the U block at position x×y (6×4 = 24 in this example)
////    and the V block at position x×y + (x×y)/4 (here, 6×4 + (6×4)/4 = 30).
//
//    unsigned int lineToSkip = (src_height - crop_height_) / 2;
//    unsigned int skip_lines_offset = lineToSkip * src_stride;
//    unsigned int skip_lines_offset_UV = skip_lines_offset /4;
//    unsigned int crop_y_src_offset = (src_width - crop_width_) / 2;
//    unsigned int crop_uv_src_offset = crop_y_src_offset / 2;
//
////    if (options_->verbose) {
////        std::cout << "I420Rotate:"
////        << "\nsrc_width = " << src_width
////        << "\nsrc_stride = " << src_stride
////        << "\nsrc_y_size = " << src_y_size
////        << "\nsrc_uv_size = " << src_uv_size
////
////        << "\nlineToSkip = " << lineToSkip
////        << "\nskip_lines_offset = " << skip_lines_offset
////        << "\nskip_lines_offset_UV = " << skip_lines_offset_UV
////        << "\ncrop_y_src_offset =  " << crop_y_src_offset
////        << "\ncrop_uv_src_offset = " << crop_uv_src_offset
////        << std::endl;
////    }
//
//    libyuv::I420Rotate(
//            src_i420 + skip_lines_offset + crop_y_src_offset, src_stride,
//            src_U + skip_lines_offset_UV + crop_uv_src_offset, src_U_stride,
//            src_V + skip_lines_offset_UV + crop_uv_src_offset, src_V_stride,
//            cropBuffer_[num], crop_stride_,
//            crop_U, crop_U_stride,
//            crop_V, crop_V_stride,
//            crop_width_, crop_height_, libyuv::kRotate0);
//
//    cinfo.image_width = crop_width_;
//    cinfo.image_height = crop_height_;
//    cinfo.input_components = 3;
//    cinfo.in_color_space = JCS_YCbCr;
//    cinfo.restart_interval = 0;
//
//    jpeg_set_defaults(&cinfo);
//    cinfo.raw_data_in = TRUE;
//    jpeg_set_quality(&cinfo, options_->quality, TRUE);
//    encoded_buffer = nullptr;
//    buffer_len = 0;
//    jpeg_mem_len_t jpeg_mem_len;
//    jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);
//    jpeg_start_compress(&cinfo, TRUE);
//
//    JSAMPROW y_rows[16];
//    JSAMPROW u_rows[8];
//    JSAMPROW v_rows[8];
//
//
//    for (uint8_t *Y_row = out_Y, *U_row = out_U, *V_row = out_V; cinfo.next_scanline < crop_height_;)
//    {
//        for (int i = 0; i < 16; i++, Y_row += out_stride)
//            y_rows[i] = std::min(Y_row, Y_max);
//        for (int i = 0; i < 8; i++, U_row += out_half_stride, V_row += out_half_stride)
//            u_rows[i] = std::min(U_row, U_max), v_rows[i] = std::min(V_row, V_max);
//
//        JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
//        jpeg_write_raw_data(&cinfo, rows, 16);
//    }
//
//    jpeg_finish_compress(&cinfo);
//    buffer_len = jpeg_mem_len;
////    free(crop_i420_c);
//}
void MjpegEncoder::encodeJPEG(struct jpeg_compress_struct &cinfo, EncodeItem &item, uint8_t *&encoded_buffer,
                              size_t &buffer_len, int num)
{
    (void)num;

    //----------------------------------------------
    //----------------------------------------------
    // SRC
    //----------------------------------------------
    //----------------------------------------------
    uint8_t *src_i420 = (uint8_t *)item.mem;

    unsigned int src_width = item.width;
    unsigned int src_height = item.height;
    unsigned int src_stride = item.stride;

    if (options_->verbose) {
        std::cout << "encodeJPEG source: " <<
                  "\n width:" << item.width <<
                  "\n height:" << item.height <<
                  std::endl;
    }

    unsigned int src_half_height = (src_height + 1) / 2;
    unsigned int src_stride2 = item.stride / 2;

    unsigned int src_y_size = src_stride * src_height;
    unsigned int src_uv_size = src_stride2 * src_half_height ;

//    int src_U_stride = src_stride2;
//    int src_V_stride = src_stride2;

//    uint8_t *src_Y = (uint8_t *)src_i420;
//    uint8_t *src_U = (uint8_t *)src_Y + src_y_size;
//    uint8_t *src_V = (uint8_t *)src_U + src_uv_size;


//    //----------------------------------------------
//    //----------------------------------------------
//    // CROP
//    //----------------------------------------------
//    //----------------------------------------------
//    //720 x 1280 — HD is 720p
//    //1920 x 1080 — FHD is 1080p
//    //2560 x 1440 — QHD (Quad HD) is 2K 1440p
//    //3840 x 2160 — UHD (Ultra HD) is 4K 2160p
//    //4056 x 3040 - Cam Raw
//
//    int crop_U_stride = crop_stride2_;
//    int crop_V_stride = crop_stride2_;
//
//    uint8_t *crop_Y = (uint8_t *)cropBuffer_[num];
//    uint8_t *crop_U = (uint8_t *)crop_Y + crop_y_size_;
//    uint8_t *crop_V = (uint8_t *)crop_U + crop_uv_size_;

    //----------------------------------------------
    //----------------------------------------------
    // OUT
    //----------------------------------------------
    //----------------------------------------------
    uint8_t *out_Y = (uint8_t *)src_i420;
    unsigned int out_stride = src_stride;
    int out_half_stride = src_stride2;

    uint8_t *out_U = (uint8_t *)out_Y + src_y_size;
    uint8_t *out_V = (uint8_t *)out_U + src_uv_size;

    uint8_t *Y_max = out_Y + src_y_size - 1;
    uint8_t *U_max = out_U + src_uv_size - 1;
    uint8_t *V_max = out_V + src_uv_size - 1;


//    As shown in the above image, the Y′, U and V components in Y′UV420 are encoded separately in sequential blocks.
//    A Y′ value is stored for every pixel, followed by a U value for each 2×2 square block of pixels,
//    and finally a V value for each 2×2 block.
//    Corresponding Y′, U and V values are shown using the same color in the diagram above.
//    Read line-by-line as a byte stream from a device,
//    the Y′ block would be found at position 0,
//    the U block at position x×y (6×4 = 24 in this example)
//    and the V block at position x×y + (x×y)/4 (here, 6×4 + (6×4)/4 = 30).

//    unsigned int lineToSkip = (src_height - crop_height_) / 2;
//    unsigned int skip_lines_offset = lineToSkip * src_stride;
//    unsigned int skip_lines_offset_UV = skip_lines_offset /4;
//    unsigned int crop_y_src_offset = (src_width - crop_width_) / 2;
//    unsigned int crop_uv_src_offset = crop_y_src_offset / 2;

//    if (options_->verbose) {
//        std::cout << "I420Rotate:"
//        << "\nsrc_width = " << src_width
//        << "\nsrc_stride = " << src_stride
//        << "\nsrc_y_size = " << src_y_size
//        << "\nsrc_uv_size = " << src_uv_size
//
//        << "\nlineToSkip = " << lineToSkip
//        << "\nskip_lines_offset = " << skip_lines_offset
//        << "\nskip_lines_offset_UV = " << skip_lines_offset_UV
//        << "\ncrop_y_src_offset =  " << crop_y_src_offset
//        << "\ncrop_uv_src_offset = " << crop_uv_src_offset
//        << std::endl;
//    }

//    libyuv::I420Rotate(
//            src_i420 + skip_lines_offset + crop_y_src_offset, src_stride,
//            src_U + skip_lines_offset_UV + crop_uv_src_offset, src_U_stride,
//            src_V + skip_lines_offset_UV + crop_uv_src_offset, src_V_stride,
//            cropBuffer_[num], crop_stride_,
//            crop_U, crop_U_stride,
//            crop_V, crop_V_stride,
//            crop_width_, crop_height_, libyuv::kRotate0);

    cinfo.image_width = src_width;
    cinfo.image_height = src_height;
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

    JSAMPROW y_rows[16];
    JSAMPROW u_rows[8];
    JSAMPROW v_rows[8];


    for (uint8_t *Y_row = out_Y, *U_row = out_U, *V_row = out_V; cinfo.next_scanline < src_height;)
    {
        for (int i = 0; i < 16; i++, Y_row += out_stride)
            y_rows[i] = std::min(Y_row, Y_max);
        for (int i = 0; i < 8; i++, U_row += out_half_stride, V_row += out_half_stride)
            u_rows[i] = std::min(U_row, U_max), v_rows[i] = std::min(V_row, V_max);

        JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
        jpeg_write_raw_data(&cinfo, rows, 16);
    }

    jpeg_finish_compress(&cinfo);
    buffer_len = jpeg_mem_len;
//    free(crop_i420_c);
}


void MjpegEncoder::encodeDownsampleJPEG(struct jpeg_compress_struct &cinfo,
                                        EncodeItem &source,
                                        uint8_t *&encoded_buffer,
                                        size_t &buffer_len,
                                        int num)
{

    (void)num;

    uint8_t *crop_Y = (uint8_t *)cropBuffer_[num];
    uint8_t *crop_U = (uint8_t *)crop_Y + crop_y_size_;
    uint8_t *crop_V = (uint8_t *)crop_U + crop_uv_size_;

    unsigned int scale_width = options_->scale_width;
    unsigned int scale_height = options_->scale_height;

    unsigned int scale_y_stride =  scale_width;
    unsigned int scale_uv_stride =  scale_width/2;

    unsigned int scale_y_size = scale_y_stride * scale_height;
    unsigned int scale_uv_size = scale_uv_stride * scale_height;
    unsigned int scale_size = crop_y_size_ + (crop_uv_size_ * 2);

    uint8_t*  scaleBuffer = (uint8_t*)malloc(scale_size);

    uint8_t *scale_Y = (uint8_t *)scaleBuffer;
    uint8_t *scale_U = (uint8_t *)scale_Y + scale_y_size;
    uint8_t *scale_V = (uint8_t *)scale_U + scale_uv_size;

    uint8_t *scale_Y_max = scale_Y + scale_y_size - 1;
    uint8_t *scale_U_max = scale_U + scale_uv_size - 1;
    uint8_t *scale_V_max = scale_V + scale_uv_size - 1;

    libyuv::I420Scale(
        crop_Y, crop_stride_,
        crop_U, crop_stride2_,
        crop_V, crop_stride2_,
        crop_width_, crop_height_,
        scale_Y, scale_y_stride,
        scale_U, scale_uv_stride,
        scale_V, scale_uv_stride,
        scale_width, scale_height,
        libyuv::kFilterBilinear
        );

    cinfo.image_width = scale_width;
    cinfo.image_height = scale_height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_YCbCr;
    cinfo.restart_interval = 0;

    jpeg_set_defaults(&cinfo);
    cinfo.raw_data_in = TRUE;
    jpeg_set_quality(&cinfo, options_->scale_quality, TRUE);
    encoded_buffer = nullptr;
    buffer_len = 0;
    jpeg_mem_len_t jpeg_mem_len;
    jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);
    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW y_rows[16];
    JSAMPROW u_rows[8];
    JSAMPROW v_rows[8];

    for (uint8_t *Y_row = scale_Y, *U_row = scale_U, *V_row = scale_V; cinfo.next_scanline < scale_height;)
    {
        for (int i = 0; i < 16; i++, Y_row += scale_y_stride)
            y_rows[i] = std::min(Y_row, scale_Y_max);
        for (int i = 0; i < 8; i++, U_row += scale_uv_stride, V_row += scale_uv_stride){
            u_rows[i] = std::min(U_row, scale_U_max);
            v_rows[i] = std::min(V_row, scale_V_max);
        }

        JSAMPARRAY rows[] = { y_rows, u_rows, v_rows };
        jpeg_write_raw_data(&cinfo, rows, 16);
    }

    jpeg_finish_compress(&cinfo);
    buffer_len = jpeg_mem_len;
    free(scaleBuffer);
}


void MjpegEncoder::encodeThread(int num) {
    struct jpeg_compress_struct cinfoMain;
    struct jpeg_compress_struct cinfoPrev;

    struct jpeg_error_mgr jerr;

    cinfoMain.err = jpeg_std_error(&jerr);
    cinfoPrev.err = jpeg_std_error(&jerr);

    jpeg_create_compress(&cinfoMain);
    jpeg_create_compress(&cinfoPrev);
    typedef std::chrono::duration<float, std::milli> duration;

    duration encode_time(0);
    uint32_t frames = 0;

    EncodeItem encode_item;

    while (true) {
        {
            std::unique_lock<std::mutex> lock(encode_mutex_);
            while (true) {
                using namespace std::chrono_literals;
                if (abort_) {
                    if (frames && options_->verbose) {
                        std::cerr << "Encode " << frames << " frames, average time "
                                  << encode_time.count() * 1000 / frames << std::endl;
                    }
                    jpeg_destroy_compress(&cinfoMain);
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

        // Encode the buffers (and generate an accompanying preview and exif)
        uint8_t *encoded_buffer = nullptr;
        uint8_t *encoded_prev_buffer = nullptr;
        uint8_t *exif_buffer = nullptr;
        size_t buffer_len = 0;
        size_t buffer_prev_len = 0;
        size_t exif_buffer_len = 0;

        {
            auto start_time = std::chrono::high_resolution_clock::now();
            encodeJPEG(cinfoMain, encode_item, encoded_buffer, buffer_len, num);
//            encodeDownsampleJPEG(cinfoPrev, encode_item, encoded_prev_buffer, buffer_prev_len, num);
            encode_time = (std::chrono::high_resolution_clock::now() - start_time);

        }
        frames += 1;

        // Don't return buffers until the output thread as that's where they're
        // in order again.
        // We push this encoded buffer to another thread so that our
        // application can take its time with the data without blocking the
        // encode process.

        OutputItem output_item = {encoded_buffer,
                                  buffer_len,
                                  encoded_prev_buffer,
                                  buffer_prev_len,
                                  exif_buffer,
                                  exif_buffer_len,
                                  encode_item.timestamp_us,
                                  encode_item.index};

        input_done_callback_(nullptr);
        output_ready_callback_(output_item.mem,
                               output_item.bytes_used,
                               output_item.preview_mem,
                               output_item.preview_bytes_used,
                               output_item.timestamp_us,
                               true);


        free(output_item.mem);
        free(output_item.preview_mem);

//        stat_mutex_.lock();
//        std::cout << "stat_mutex_ lock in ++"  << std::endl;
        if (options_->verbose) {
            frame_second_ += 1;
        }
//        stat_mutex_.unlock();
//        std::cout << "stat_mutex_ unlock in ++"  << std::endl;
    }
}


void MjpegEncoder::outputThread() {
    if (options_->verbose) {
        while (true) {
            {
                std::this_thread::sleep_for (std::chrono::seconds(1));
                std::cout << "Frame / sec: " << frame_second_  << std::endl;
                frame_second_ = 0;
            }
        }
    }
}
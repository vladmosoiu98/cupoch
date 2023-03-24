/**
 * Copyright (c) 2020 Neka-Nat
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 **/
#include <thrust/tabulate.h>
#include "cupoch/camera/pinhole_camera_intrinsic.h"
#include "cupoch/geometry/image.h"
#include "cupoch/utility/console.h"
#include "cupoch/utility/platform.h"

using namespace cupoch;
using namespace cupoch::geometry;

namespace {

struct compute_camera_distance_functor {
    compute_camera_distance_functor(uint8_t *data,
                                    int width,
                                    const float *xx,
                                    const float *yy)
        : data_(data), width_(width), xx_(xx), yy_(yy){};
    uint8_t *data_;
    const int width_;
    const float *xx_;
    const float *yy_;
    __device__ void operator()(size_t idx) {
        int i = idx / width_;
        int j = idx % width_;
        float *fp = (float *)(data_ + idx * sizeof(float));
        *fp = sqrtf(xx_[j] * xx_[j] + yy_[i] * yy_[i] + 1.0f);
    }
};

__constant__ float grayscale_weights[2][3] = {{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0},
                                              {0.2990f, 0.5870f, 0.1140f}};

template <typename T>
struct make_gray_image_functor {
    make_gray_image_functor(const uint8_t *image,
                            int num_of_channels,
                            int bytes_per_channel,
                            Image::ColorToIntensityConversionType type,
                            uint8_t *fimage,
                            float denom)
        : image_(image),
          num_of_channels_(num_of_channels),
          bytes_per_channel_(bytes_per_channel),
          type_(type),
          fimage_(fimage),
          denom_(denom){};
    const uint8_t *image_;
    int num_of_channels_;
    int bytes_per_channel_;
    Image::ColorToIntensityConversionType type_;
    uint8_t *fimage_;
    float denom_;
    __device__ void operator()(size_t idx) {
        typedef float (*grayfn)(const uint8_t *);
        typedef float (*colorfn)(const uint8_t *, const float *);
        grayfn gf[4] = {
                [] __device__(const uint8_t *pi) { return (float)(*pi); },
                [] __device__(const uint8_t *pi) {
                    const uint16_t *pi16 = (const uint16_t *)pi;
                    return (float)(*pi16);
                },
                [] __device__(const uint8_t*) { return 0.0f; },
                [] __device__(const uint8_t *pi) {
                    const float *pf = (const float *)pi;
                    return *pf;
                }};
        colorfn cf[4] = {
                [] __device__(const uint8_t *pi, const float *weights) {
                    return (weights[0] * (float)(pi[0]) +
                            weights[1] * (float)(pi[1]) +
                            weights[2] * (float)(pi[2]));
                },
                [] __device__(const uint8_t *pi, const float *weights) {
                    const uint16_t *pi16 = (const uint16_t *)pi;
                    return weights[0] * (float)(pi16[0]) +
                           weights[1] * (float)(pi16[1]) +
                           weights[2] * (float)(pi16[2]);
                },
                [] __device__(const uint8_t*, const float*) {
                    return 0.0f;
                },
                [] __device__(const uint8_t *pi, const float *weights) {
                    const float *pf = (const float *)pi;
                    return weights[0] * pf[0] + weights[1] * pf[1] +
                           weights[2] * pf[2];
                }};
        T *p = (T *)(fimage_ + idx * sizeof(T));
        const uint8_t *pi =
                image_ + idx * num_of_channels_ * bytes_per_channel_;
        if (num_of_channels_ == 1) {
            // grayscale image
            *p = (T)(gf[bytes_per_channel_ - 1](pi) * denom_);
        } else if (num_of_channels_ == 3) {
            *p = (T)(cf[bytes_per_channel_ - 1](pi,
                                                grayscale_weights[(int)type_]) *
                     denom_);
        }
    }
};

template <typename T>
struct restore_from_float_image_functor {
    restore_from_float_image_functor(const float *src, uint8_t *dst)
        : src_(src), dst_(dst){};
    const float *src_;
    uint8_t *dst_;
    __device__ void operator()(size_t idx) {
        if (sizeof(T) == 1)
            *(dst_ + idx) = static_cast<T>(*(src_ + idx) * 255.0f);
        if (sizeof(T) == 2) *(dst_ + idx) = static_cast<T>(*(src_ + idx));
    }
};

}  // namespace

std::shared_ptr<Image> Image::CreateDepthToCameraDistanceMultiplierFloatImage(
        const camera::PinholeCameraIntrinsic &intrinsic) {
    auto fimage = std::make_shared<Image>();
    fimage->Prepare(intrinsic.width_, intrinsic.height_, 1, 4);
    float ffl_inv0 = 1.0f / (float)intrinsic.GetFocalLength().first;
    float ffl_inv1 = 1.0f / (float)intrinsic.GetFocalLength().second;
    float fpp0 = (float)intrinsic.GetPrincipalPoint().first;
    float fpp1 = (float)intrinsic.GetPrincipalPoint().second;
    utility::device_vector<float> xx(intrinsic.width_);
    utility::device_vector<float> yy(intrinsic.height_);
    thrust::tabulate(utility::exec_policy(utility::GetStream(0)),
                     xx.begin(), xx.end(), [=] __device__(int idx) {
                         return (idx - fpp0) * ffl_inv0;
                     });
    thrust::tabulate(utility::exec_policy(utility::GetStream(1)),
                     yy.begin(), yy.end(), [=] __device__(int idx) {
                         return (idx - fpp1) * ffl_inv1;
                     });
    cudaSafeCall(cudaDeviceSynchronize());
    compute_camera_distance_functor func(
            thrust::raw_pointer_cast(fimage->data_.data()), intrinsic.width_,
            thrust::raw_pointer_cast(xx.data()),
            thrust::raw_pointer_cast(yy.data()));
    for_each(thrust::make_counting_iterator<size_t>(0),
             thrust::make_counting_iterator<size_t>(intrinsic.height_ *
                                                    intrinsic.width_),
             func);
    return fimage;
}

std::shared_ptr<Image> Image::CreateGrayImage(
        Image::ColorToIntensityConversionType type /* = WEIGHTED*/) const {
    auto image = std::make_shared<Image>();
    if (IsEmpty()) {
        return image;
    }
    image->Prepare(width_, height_, 1, 1);
    if (bytes_per_channel_ == 1) {
        make_gray_image_functor<uint8_t> func(
                thrust::raw_pointer_cast(data_.data()), num_of_channels_,
                bytes_per_channel_, type,
                thrust::raw_pointer_cast(image->data_.data()), 1.0f);
        thrust::for_each(
                thrust::make_counting_iterator<size_t>(0),
                thrust::make_counting_iterator<size_t>(width_ * height_), func);
    } else if (bytes_per_channel_ == 2) {
        make_gray_image_functor<uint8_t> func(
                thrust::raw_pointer_cast(data_.data()), num_of_channels_,
                bytes_per_channel_, type,
                thrust::raw_pointer_cast(image->data_.data()), 255.0 / 65535.0);
        thrust::for_each(
                thrust::make_counting_iterator<size_t>(0),
                thrust::make_counting_iterator<size_t>(width_ * height_), func);
    } else if (bytes_per_channel_ == 4) {
        make_gray_image_functor<uint8_t> func(
                thrust::raw_pointer_cast(data_.data()), num_of_channels_,
                bytes_per_channel_, type,
                thrust::raw_pointer_cast(image->data_.data()), 255.0f);
        thrust::for_each(
                thrust::make_counting_iterator<size_t>(0),
                thrust::make_counting_iterator<size_t>(width_ * height_), func);
    }
    return image;
}

std::shared_ptr<Image> Image::CreateFloatImage(
        Image::ColorToIntensityConversionType type /* = WEIGHTED*/) const {
    auto fimage = std::make_shared<Image>();
    if (IsEmpty()) {
        return fimage;
    }
    fimage->Prepare(width_, height_, 1, 4);
    if (bytes_per_channel_ == 1) {
        make_gray_image_functor<float> func(
                thrust::raw_pointer_cast(data_.data()), num_of_channels_,
                bytes_per_channel_, type,
                thrust::raw_pointer_cast(fimage->data_.data()), 1.0 / 255.0);
        thrust::for_each(
                thrust::make_counting_iterator<size_t>(0),
                thrust::make_counting_iterator<size_t>(width_ * height_), func);
    } else {
        make_gray_image_functor<float> func(
                thrust::raw_pointer_cast(data_.data()), num_of_channels_,
                bytes_per_channel_, type,
                thrust::raw_pointer_cast(fimage->data_.data()), 1.0f);
        thrust::for_each(
                thrust::make_counting_iterator<size_t>(0),
                thrust::make_counting_iterator<size_t>(width_ * height_), func);
    }
    return fimage;
}

template <typename T>
std::shared_ptr<Image> Image::CreateImageFromFloatImage() const {
    auto output = std::make_shared<Image>();
    if (num_of_channels_ != 1 || bytes_per_channel_ != 4) {
        utility::LogError(
                "[CreateImageFromFloatImage] Unsupported image format.");
    }

    output->Prepare(width_, height_, num_of_channels_, sizeof(T));
    restore_from_float_image_functor<T> func(
            (const float *)thrust::raw_pointer_cast(data_.data()),
            thrust::raw_pointer_cast(output->data_.data()));
    thrust::for_each(thrust::make_counting_iterator<size_t>(0),
                     thrust::make_counting_iterator<size_t>(width_ * height_),
                     func);
    return output;
}

template std::shared_ptr<Image> Image::CreateImageFromFloatImage<uint8_t>()
        const;
template std::shared_ptr<Image> Image::CreateImageFromFloatImage<uint16_t>()
        const;

ImagePyramid Image::CreatePyramid(size_t num_of_levels,
                                  bool with_gaussian_filter /*= true*/) const {
    std::vector<std::shared_ptr<Image>> pyramid_image;
    if ((num_of_channels_ != 1 || bytes_per_channel_ != 4) &&
        (num_of_channels_ != 3 || bytes_per_channel_ != 1)) {
        utility::LogError("[CreateImagePyramid] Unsupported image format.");
    }

    for (size_t i = 0; i < num_of_levels; i++) {
        if (i == 0) {
            std::shared_ptr<Image> input_copy_ptr = std::make_shared<Image>();
            *input_copy_ptr = *this;
            pyramid_image.push_back(input_copy_ptr);
        } else {
            if (with_gaussian_filter && num_of_channels_ == 1) {
                // https://en.wikipedia.org/wiki/Pyramid_(image_processing)
                auto level_b = pyramid_image[i - 1]->Filter(
                        Image::FilterType::Gaussian3);
                auto level_bd = level_b->Downsample();
                pyramid_image.push_back(level_bd);
            } else {
                auto level_d = pyramid_image[i - 1]->Downsample();
                pyramid_image.push_back(level_d);
            }
        }
    }
    return pyramid_image;
}
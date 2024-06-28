#pragma once

#define PCL_NO_PRECOMPILE

#include "GaussianFrame.hpp"

class GaussianCodec
{
public:
    GaussianCodec() = delete;
    GaussianCodec(const GaussianCodec&) = delete;
    GaussianCodec& operator=(const GaussianCodec&) = delete;
    ~GaussianCodec() = default;

    static void encodeGaussian(GaussianFrame& frame);
    static void decodeGaussian(GaussianFrame& frame);
};

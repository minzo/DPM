//==============================================================================
//
// 深度画像向け 画像処理クラス群
//
//==============================================================================
#ifndef _MI_DEPTH_PROCESSING_H_
#define _MI_DEPTH_PROCESSING_H_

#include "miImageProcessing.h"
#include <vector>
#include <iostream>

namespace mi {
    
//------------------------------------------------------------------------------
// Logistic フィルタ
//------------------------------------------------------------------------------
class LogisticFilter : IImageProcessing {
public:
    LogisticFilter(Image& image, double paramA, double paramB);
    static void Process(Image& image, double paramA, double paramB) {
        LogisticFilter filter(image, paramA, paramB);
    }
};


//------------------------------------------------------------------------------
// MedianTS フィルタ 時間方向を含めたメディアンフィルタ
//------------------------------------------------------------------------------
class MedianTSFilter : IImageProcessing {
public:
    MedianTSFilter(Image& image, std::vector<Image> inputs, int filterSize);
    static void Process(Image& image, std::vector<Image> inputs, int filterSize) {
        MedianTSFilter filter(image, inputs, filterSize);
    }
};

    
//------------------------------------------------------------------------------
// Trilateral フィルタ
//------------------------------------------------------------------------------
class TrilateralFilter : IImageProcessing {
public:
    TrilateralFilter(Image& image, Image& reference,
                     int filterSize, double sigma, double sigma2);
    static void Process(Image& image, Image& reference,
                        int filterSize, double sigma, double sigma2) {
        TrilateralFilter filter(image, reference, filterSize, sigma, sigma2);
    }
};

    
//------------------------------------------------------------------------------
// Quadrilateral フィルタ
//------------------------------------------------------------------------------
class QuadrilateralFilter : IImageProcessing {
public:
    QuadrilateralFilter(Image& image, Image color, Image laser, Image camera, int filterSize);
    static void Process(Image& image, Image& color, Image& laser, Image& camera, int filterSize) {
        QuadrilateralFilter filter(image, color, laser, camera, filterSize);
    }
};
    
}

#endif
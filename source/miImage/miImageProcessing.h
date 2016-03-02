//==============================================================================
//
// 画像処理クラス群
//
//==============================================================================
#ifndef _MI_IMAGE_PROCESSING_H_
#define _MI_IMAGE_PROCESSING_H_

#include "miImage.h"
#include <functional>

namespace mi {

//------------------------------------------------------------------------------
// 画像処理インターフェイスクラス
//
// MEMO:
// フィルタを実装する場合はこのクラスを継承し、
// コンストラクタで Processing に画像処理の関数を代入する。その後 Run() を呼ぶ
//------------------------------------------------------------------------------
class IImageProcessing {
protected:
    // 継承を強制する
    IImageProcessing(){}
    
    // 画像処理関数
    // 第一引数に Image型メンバdataの開始番号, 第二引数に開始から終了までの長さが渡される
    std::function<void(int, int)> Processing;
    
    // 画像処理を分割実行する
    void Run(Image& image, int numThreads);
};

    
//------------------------------------------------------------------------------
// モノクロ処理
//------------------------------------------------------------------------------
class Monochrome : public IImageProcessing {
public:
    Monochrome(Image& image);
    static void Process(Image& image) {
        Monochrome filter(image);
    }
};

    
//------------------------------------------------------------------------------
// ディザ化処理
//------------------------------------------------------------------------------
class DitheringErrorDiffusion : public IImageProcessing {
public:
    DitheringErrorDiffusion(Image& image);
    static void Process(Image& image){
        DitheringErrorDiffusion filter(image);
    }
};

    
//------------------------------------------------------------------------------
// 2値化処理
//------------------------------------------------------------------------------
class Binarize : IImageProcessing {
public:
    Binarize(Image& image, int threshold);
    static void Process(Image& image, int threshold) {
        Binarize filter(image,threshold);
    }
};


//------------------------------------------------------------------------------
// メディアンフィルタ
//------------------------------------------------------------------------------
class MedianFilter : IImageProcessing {
public:
    MedianFilter(Image& image, int filterSize);
    static void Process(Image& image, int filterSize) {
    MedianFilter filter(image,filterSize);
    }
};

    
//------------------------------------------------------------------------------
// 平均化フィルタ
//------------------------------------------------------------------------------
class AverageFilter : IImageProcessing {
public:
    AverageFilter(Image& image, int filterSize);
    static void Process(Image& image, int filterSize) {
        AverageFilter filter(image,filterSize);
    }
};
    
    
//------------------------------------------------------------------------------
// Gaussian フィルタ
//------------------------------------------------------------------------------
class GaussianFilter : IImageProcessing {
public:
    GaussianFilter(Image& image, int filterSize, double sigma);
    static void Process(Image& image, int filterSize, double sigma) {
        GaussianFilter filter(image, filterSize, sigma);
    }
};
    
//------------------------------------------------------------------------------
// Bilateral フィルタ
//------------------------------------------------------------------------------
class BilateralFilter : IImageProcessing {
public:
    BilateralFilter(Image& image, int filterSize, double sigma, double sigma2);
    static void Process(Image& image, int filterSize, double sigma, double sigma2) {
        BilateralFilter filter(image, filterSize, sigma, sigma2);
    }
};
    
//------------------------------------------------------------------------------
// Sobel フィルタ
//------------------------------------------------------------------------------
class SobelFilter : IImageProcessing {
public:
    SobelFilter(Image& image);
    static void Process(Image& image) {
        SobelFilter filter(image);
    }
};

//------------------------------------------------------------------------------
// Laplacian フィルタ
//------------------------------------------------------------------------------
class LaplacianFilter : IImageProcessing {
public:
    LaplacianFilter(Image& image);
    static void Process(Image& image) {
        LaplacianFilter filter(image);
    }
};
    
//------------------------------------------------------------------------------
// ヒストグラムの均一化
//------------------------------------------------------------------------------
class HistgramEqualization : IImageProcessing {
public:
    HistgramEqualization(Image& image);
    static void Process(Image& image) {
        HistgramEqualization filter(image);
    }
};

//------------------------------------------------------------------------------
// ヒストグラム伸張
//------------------------------------------------------------------------------
class HistgramExtention : IImageProcessing {
public:
    HistgramExtention(Image& image);
    static void Process(Image& image) {
        HistgramExtention filter(image);
    }
};

//------------------------------------------------------------------------------
// アルファブレンド
//------------------------------------------------------------------------------
class AlphaBlend : IImageProcessing {
public:
    AlphaBlend(Image& image, const Image& blend, double alpha);
    static void Process(Image& image, const Image& blend, double alpha) {
        AlphaBlend filter(image, blend, alpha);
    }
};

//------------------------------------------------------------------------------
// ガンマ補正
//------------------------------------------------------------------------------
class GammaCollection : IImageProcessing {
public:
    GammaCollection(Image& image, double param);
    static void Process (Image& image, double param) {
        GammaCollection filter(image, param);
    }
};

}

#endif
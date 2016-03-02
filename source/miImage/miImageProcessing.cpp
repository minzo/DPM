//==============================================================================
//
// 画像処理クラス群
//
//==============================================================================
#include "miImageProcessing.h"

#include <thread>
#include <functional>
#include <algorithm>
#include <cmath>

namespace mi {

//------------------------------------------------------------------------------
// 画像処理を分割実行する
// image     : 処理する画像
// numThreads: スレッド数
//------------------------------------------------------------------------------
void IImageProcessing::Run(Image& image, int numThreads) {
    
    // データが指定スレッド数より少ない場合は1スレッドで処理する(処理が競合するのを防ぐ)
    if(numThreads > image.Size()) {
        numThreads = 1;
    }
    
    // スレッドオブジェクトを生成
    std::thread* threads = new std::thread[numThreads];
    
    // スレッドを生成して画像処理を実行する
    for(int i=0; i<numThreads; i++){
        int length = image.Size() / numThreads;
        int start  = i*length;
        threads[i] = std::thread(Processing,start,length);
    }
    
    // 各スレッドの終了処理
    for(int i=0; i<numThreads; i++){
        threads[i].join();
    }
    
    // スレッドオブジェクトを破棄
    delete[] threads;
}

//------------------------------------------------------------------------------
// モノクロ処理
//------------------------------------------------------------------------------
Monochrome::Monochrome(Image& image) {
    
    // 画像処理本体
    Processing = [&](int start, int length) {
        
        for(int i=start; i<start+length; i++) {
            int Y = (int)(0.299*image.data[i].r + 0.587*image.data[i].g + 0.114*image.data[i].b);
            image.data[i].r = (unsigned char)Y;
            image.data[i].g = (unsigned char)Y;
            image.data[i].b = (unsigned char)Y;
        }
    };
    
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}
    
//------------------------------------------------------------------------------
// ディザ化処理
//------------------------------------------------------------------------------
DitheringErrorDiffusion::DitheringErrorDiffusion(Image& image) {

    // 画像処理本体
    Processing = [&](int start, int length) {
        
        for(int i=start; i<start+length; i++) {
            int iX = i % image.Width();
            int iY = i / image.Width();

            int Y = (int)(0.299*image.data[i].r + 0.587*image.data[i].g + 0.114*image.data[i].b);
            
            if(Y > 127) image.pixel[iX][iY] = RGB(255,255,255);
            else        image.pixel[iX][iY] = RGB(0,0,0);
            
            double err =( Y - image.pixel[iX][iY].r ) / 16.0;
            
            if(iX+1 < image.Width()){
                image.pixel[iX+1][iY]+=err*5;
            }
            if(iX-1>=0 && iY+1<image.Height() ){
                image.pixel[iX-1][iY+1]+=err*3;
            }
            if(iY+1<image.Height()){
                image.pixel[iX][iY+1]+=err*5;
            }
            if(iX+1<image.Width() && iY+1<image.Height()){
                image.pixel[iX+1][iY+1]+=err*3;
            }
        }
    };
    
    // Processingの処理をおこなう
    Run(image, 1);
}

//------------------------------------------------------------------------------
// 2値化処理
//------------------------------------------------------------------------------
Binarize::Binarize(Image &image, int threshold) {

    // 画像処理本体
    Processing = [&](int start, int length) {
        
        for(int i=start; i<start+length; i++) {
            int Y = (int)(0.299*image.data[i].r + 0.587*image.data[i].g + 0.114*image.data[i].b);
            if(Y > threshold) {
                image.data[i] = RGB(255,255,255);
            }
            else {
                image.data[i] = RGB(0,0,0);
            }
        }
    };
    
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}

//------------------------------------------------------------------------------
// メディアンフィルタ
//------------------------------------------------------------------------------
MedianFilter::MedianFilter(Image& image, int filterSize) {
    
    // コピー
    Image copy = image;
    
    // 画像処理本体
    Processing = [&](int start, int length) {
        
        int halfSize = filterSize/2;
        int sqrSize  = filterSize*filterSize;
        
        int* R = new int[sqrSize];
        int* G = new int[sqrSize];
        int* B = new int[sqrSize];

        for(int i=start; i<start+length; i++) {
            int iX = i % image.Width();
            int iY = i / image.Width();

            int pixelCount = 0;

            for(int j=0; j<sqrSize; j++){
                int jX = iX + j%filterSize - halfSize;
                int jY = iY + j/filterSize - halfSize;
                    
                if(jX<0 || jX>=image.Width() || jY<0 || jY>=image.Height()) continue;
                
                R[pixelCount] = copy.pixel[jX][jY].r;
                G[pixelCount] = copy.pixel[jX][jY].g;
                B[pixelCount] = copy.pixel[jX][jY].b;
                pixelCount++;
            }
            
            std::sort(R, R+pixelCount);
            std::sort(G, G+pixelCount);
            std::sort(B, B+pixelCount);
            
            image.pixel[iX][iY].r = R[pixelCount/2];
            image.pixel[iX][iY].g = G[pixelCount/2];
            image.pixel[iX][iY].b = B[pixelCount/2];
        }
        
        delete[] R;
        delete[] G;
        delete[] B;
    };
    
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}
    
//------------------------------------------------------------------------------
// 平均化フィルタ
//------------------------------------------------------------------------------
AverageFilter::AverageFilter(Image& image, int filterSize) {

    int halfSize = filterSize/2;
    
    // 縦横の走査をわけるための値
    int horizontal = 0; // 横方向走査時には1, そうでなければ0
    int vertical   = 0; // 縦方向走査時には1, そうでなければ0
    
    // 宣言
    Image copy(image.Bit(), image.Width(), image.Height());
    
    // 処理本体
    Processing = [&](int start, int length){
        
        for(int i=start; i<start + length; i++) {
            int iX = i % image.Width();
            int iY = i / image.Width();
            
            double R=0, G=0, B=0;
            
            for(int j=0; j<filterSize; j++) {
                int jX = iX + (j-halfSize) * horizontal;
                int jY = iY + (j-halfSize) * vertical;
                
                if(jX<0 || jX>=image.Width() || jY<0 || jY>=image.Height()) continue;
                
                R += copy.pixel[jX][iY].r;
                G += copy.pixel[jX][iY].g;
                B += copy.pixel[jX][iY].b;
            }
            
            image.pixel[iX][iY] = RGB(R/filterSize, G/filterSize, B/filterSize);
        }
    };

    // 横方向 ----
    // コピー
    std::copy(image.data, image.data+image.Size(), copy.data);
    
    // 横方向
    horizontal = 1;
    vertical   = 0;
        
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
    
    
    // 縦方向 ----
    // コピー
    std::copy(image.data, image.data+image.Size(), copy.data);
    
    // 縦方向
    horizontal = 0;
    vertical   = 1;
        
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}

//------------------------------------------------------------------------------
// Gaussian フィルタ
//------------------------------------------------------------------------------
GaussianFilter::GaussianFilter(Image& image, int filterSize, double sigma) {
    
    int halfSize = filterSize/2;
    
    // マスクの生成
    double* LUT = new double[filterSize];
    double  DIV = 0;
    
    for(int i=0; i<filterSize; i++) {
        int j = i - halfSize;
        LUT[i] = exp( -j*j / sigma );
        DIV += LUT[i];
    }
    for(int i=0; i<filterSize; i++) {
        LUT[i] /= DIV;
    }
    
    // 縦横の走査をわけるための値
    int horizontal = 0; // 横方向走査時には1, そうでなければ0
    int vertical   = 0; // 縦方向走査時には1, そうでなければ0
    
    // 宣言
    Image copy(image.Bit(), image.Width(), image.Height());
    
    // 処理本体
    Processing = [&](int start, int length){
        
        for(int i=start; i<start + length; i++) {
            int iX = i % image.Width();
            int iY = i / image.Width();
            
            double R=0, G=0, B=0;
            
            for(int j=0; j<filterSize; j++) {
                int jX = iX + (j-halfSize)*horizontal;
                int jY = iY + (j-halfSize)*vertical;
                
                if(jX<0 || jX>=image.Width() || jY<0 || jY>=image.Height()) continue;
                
                R += copy.pixel[jX][jY].r * LUT[j];
                G += copy.pixel[jX][jY].g * LUT[j];
                B += copy.pixel[jX][jY].b * LUT[j];
            }
            
            image.pixel[iX][iY] = RGB(R, G, B);
        }
    };
    
    // コピー
    std::copy(image.data, image.data+image.Size(), copy.data);
        
    // 横方向
    horizontal = 1;
    vertical   = 0;

    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
    
    
    // コピー
    std::copy(image.data, image.data+image.Size(), copy.data);
        
    // 横方向
    horizontal = 0;
    vertical   = 1;
        
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
    
    delete[] LUT;
}
    
//------------------------------------------------------------------------------
// Bilateral フィルタ
//------------------------------------------------------------------------------
BilateralFilter::BilateralFilter(Image& image, int filterSize, double sigma, double sigma2) {
    
    struct dRGB { double r=0, g=0, b=0; };
    
    int halfSize = filterSize/2;
    
    // パラメータ
    double sig = 2 * sigma * sigma;
    double sig2= 2 * sigma2 * sigma2;
    
    // マスクの生成
    double* LUT = new double[filterSize*filterSize];
    
    for(int i=0; i<filterSize*filterSize; i++) {
        int iX = i%filterSize-halfSize;
        int iY = i/filterSize-halfSize;
        LUT[ i ] = exp(-(iX*iX+iY*iY)/sig);
    }
    
    // コピー
    Image copy = image;

    // 処理本体
    Processing = [&](int start, int length){
            
        for(int i=start; i<start + length; i++) {

            int iX = i % image.Width();
            int iY = i / image.Width();
                
            dRGB sum; // ピクセルとの計算結果合計値
            dRGB div; // 正規化用のフィルタ値合計
                
            for(int j=0; j<filterSize*filterSize; j++) {

                int jX = iX + j%filterSize - halfSize;
                int jY = iY + j/filterSize - halfSize;
                
                if(jX<0 || jX>=image.Width() || jY<0 || jY>=image.Height()) continue;
                    
                dRGB lateral, filter;
                    
                lateral.r = (copy.pixel[iX][iY].r - copy.pixel[jX][jY].r);
                lateral.g = (copy.pixel[iX][iY].g - copy.pixel[jX][jY].g);
                lateral.b = (copy.pixel[iX][iY].b - copy.pixel[jX][jY].b);
                    
                lateral.r = exp( -lateral.r*lateral.r / sig2 );
                lateral.g = exp( -lateral.g*lateral.g / sig2 );
                lateral.b = exp( -lateral.b*lateral.b / sig2 );
                        
                filter.r = LUT[j] * lateral.r;
                filter.g = LUT[j] * lateral.g;
                filter.b = LUT[j] * lateral.b;
                    
                sum.r += filter.r * copy.pixel[jX][jY].r;
                sum.g += filter.g * copy.pixel[jX][jY].g;
                sum.b += filter.b * copy.pixel[jX][jY].b;
                    
                div.r += filter.r;
                div.g += filter.g;
                div.b += filter.b;
            }
            
            image.pixel[iX][iY].r = (unsigned char)std::max(0.0,std::min(255.0,sum.r/div.r));
            image.pixel[iX][iY].g = (unsigned char)std::max(0.0,std::min(255.0,sum.g/div.g));
            image.pixel[iX][iY].b = (unsigned char)std::max(0.0,std::min(255.0,sum.b/div.b));
        }
    };
    
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
    
    delete[] LUT;
}
    
//------------------------------------------------------------------------------
// Sobel フィルタ
//------------------------------------------------------------------------------
SobelFilter::SobelFilter(Image& image) {
    
    // コピー
    Image copy = image;
    
    int horizontal_kernel[] = {
        -1, 0, 1,
        -2, 0, 2,
        -1, 0, 1
    };
    
    int vertical_kernel[] = {
        -1, -2, -1,
        0,  0,  0,
        1,  2,  1
    };
    
    int dx[] = {-1,0,1,-1,0,1,-1,0,1};
    int dy[] = {-1,-1,-1,0,0,0,1,1,1};

    
    // 画像処理本体
    Processing = [&](int start, int length) {
        
        for(int i=start; i<start+length; i++) {
            
            int iX = i%image.Width();
            int iY = i/image.Width();

            int rh=0, gh=0, bh=0;
            int rv=0, gv=0, bv=0;

            for(int j=0; j<9; j++) {
                int jX = iX + dx[j];
                int jY = iY + dy[j];

                if(jX<0 || jX>=image.Width() || jY<0 || jY>=image.Height()) continue;
                
                rh += copy.pixel[jX][jY].r * horizontal_kernel[j];
                gh += copy.pixel[jX][jY].g * horizontal_kernel[j];
                bh += copy.pixel[jX][jY].b * horizontal_kernel[j];

                rv += copy.pixel[jX][jY].r * vertical_kernel[j];
                gv += copy.pixel[jX][jY].g * vertical_kernel[j];
                bv += copy.pixel[jX][jY].b * vertical_kernel[j];
            }
            
            image.pixel[iX][iY].r = (unsigned char)sqrt((double)(rv*rv + rh*rh));
            image.pixel[iX][iY].g = (unsigned char)sqrt((double)(gv*gv + gh*gh));
            image.pixel[iX][iY].b = (unsigned char)sqrt((double)(bv*bv + bh*bh));
        }
    };
    
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}


//------------------------------------------------------------------------------
// Laplacian フィルタ
//------------------------------------------------------------------------------
LaplacianFilter::LaplacianFilter(Image& image) {

    // コピー
    Image copy = image;

    int kernel[] = {
        1,  1, 1,
        1, -8, 1,
        1,  1, 1
    };

    int dx[] = {-1,0,1,-1,0,1,-1,0,1};
    int dy[] = {-1,-1,-1,0,0,0,1,1,1};


    // 画像処理本体
    Processing = [&](int start, int length) {

        for(int i=start; i<start+length; i++) {

            int iX = i%image.Width();
            int iY = i/image.Width();

            int r=0, g=0, b=0;

            for(int j=0; j<9; j++) {
                int jX = iX + dx[j];
                int jY = iY + dy[j];

                if(jX<0 || jX>=image.Width() || jY<0 || jY>=image.Height()) continue;
                
                r += copy.pixel[jX][jY].r * kernel[j];
                g += copy.pixel[jX][jY].g * kernel[j];
                b += copy.pixel[jX][jY].b * kernel[j];
            }

            image.pixel[iX][iY].r = (unsigned char)std::min(std::max(r,0),255);
            image.pixel[iX][iY].g = (unsigned char)std::min(std::max(g,0),255);
            image.pixel[iX][iY].b = (unsigned char)std::min(std::max(b,0),255);
        }
    };

    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}

    
//------------------------------------------------------------------------------
// ヒストグラムの均一化
//------------------------------------------------------------------------------
HistgramEqualization::HistgramEqualization(Image& image) {
    
    int histgram[256] = {0};
    int LUT[256] =  {0};
    
    // ヒストグラム作成
    for(int i=0; i<image.Size(); i++){
        histgram[ image.data[i].r ]++;
    }

    // 均一化処理用のテーブルを生成
    double sum = 0;
    int min = histgram[0] / image.Size();
    for(int i=0; i<256; i++) {
        sum += histgram[i];
        LUT[i] = (int)(255.0 * (sum/image.Size() - min) / (1.0 - min));
    }
    
    // ヒストグラムの均一化処理
    Processing = [&](int start, int length) {
        for(int i=start; i<start+length; i++) {
            image.data[i].r = (unsigned char)LUT[image.data[i].r];
            image.data[i].g = image.data[i].r;
            image.data[i].b = image.data[i].r;
        }
    };

    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}

    
//------------------------------------------------------------------------------
// ヒストグラム伸張
//------------------------------------------------------------------------------
HistgramExtention::HistgramExtention(Image& image) {
    
    Image mono = image;
    
    mi::Monochrome::Process(mono);

    unsigned char LUT[256] = {0};
    unsigned char min = mono.data[0].r;
    unsigned char max = mono.data[0].r;
    
    // 最大値・最小値
    for(int i=1; i<image.Size(); i++) {
        min = std::min(min, mono.data[i].r);
        max = std::max(max, mono.data[i].r);
    }

    // 伸張処理用のテーブルを生成
    for(int i=0; i<256; i++) {
        LUT[i] = (unsigned char)(255.0 / (max-min) * (i-min));
    }
    
    // ヒストグラムの伸張処理
    Processing = [&](int start, int length) {
        for(int i=start; i<start+length; i++) {
            image.data[i].r = LUT[image.data[i].r];
            image.data[i].g = LUT[image.data[i].g];
            image.data[i].b = LUT[image.data[i].b];
        }
    };
    
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}


//------------------------------------------------------------------------------
// アルファブレンド
//------------------------------------------------------------------------------
AlphaBlend::AlphaBlend(Image& image, const Image& blend, double alpha) {
    
    double beta = 1.0-alpha;
    
    // アルファブレンド処理
    Processing = [&](int start, int length) {
        for(int i=start; i<start+length; i++) {
            image.data[i] = image.data[i] * beta + blend.data[i] * alpha;
        }
    };
    
    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}
    

//------------------------------------------------------------------------------
// ガンマ補正
//------------------------------------------------------------------------------
GammaCollection::GammaCollection(Image& image, double param) {
    
    unsigned char LUT[256];
    
    for(int i=0; i<256; i++) {
        LUT[i] = (unsigned char)(255*pow(i/255.0,1.0/param));
    }
    
    // ガンマ補正処理
    Processing = [&](int start, int length) {
        for(int i=start; i<start+length; i++) {
            
            image.data[i].r = LUT[image.data[i].r];
            image.data[i].g = LUT[image.data[i].g];
            image.data[i].b = LUT[image.data[i].b];
        }
    };

    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
}

}
//==============================================================================
//
// 深度画像向け 画像処理クラス群
//
//==============================================================================
#include "miDepthProcessing.h"

#include <vector>
#include <thread>
#include <algorithm>
#include <cmath>

namespace mi {

//------------------------------------------------------------------------------
// 実数RGB型
//------------------------------------------------------------------------------
struct dRGB {
    double r = 0, g = 0, b = 0;

    dRGB():r(0),g(0),b(0){}
    dRGB(const dRGB& obj) { r=obj.r; g=obj.g; b=obj.b; }
    dRGB(const  RGB& obj) { r=obj.r; g=obj.g; b=obj.b; }

    template<typename T> dRGB(T r, T g, T b) : r(r), g(g), b(b) {}

    template<typename T> dRGB operator+(const T a) { return dRGB(r+a,g+a,b+a); }
    template<typename T> dRGB operator-(const T a) { return dRGB(r-a,g-a,b-a); }
    template<typename T> dRGB operator*(const T a) { return dRGB(r*a,g*a,b*a); }
    template<typename T> dRGB operator/(const T a) { return dRGB(r/a,g/a,b/a); }
    template<typename T> dRGB operator%(const T a) { return dRGB(r%a,g%a,b%a); }
    template<typename T> dRGB& operator+=(const T a) { r+=a; g+=a; b+=a; return (*this); }
    template<typename T> dRGB& operator*=(const T a) { r-=a; g-=a; b-=a; return (*this); }
    template<typename T> dRGB& operator-=(const T a) { r*=a; g*=a; b*=a; return (*this); }
    template<typename T> dRGB& operator/=(const T a) { r/=a; g/=a; b/=a; return (*this); }

    dRGB operator+(const dRGB& a) { return dRGB(r+a.r, g+a.g, b+a.b); }
    dRGB operator-(const dRGB& a) { return dRGB(r-a.r, g-a.g, b-a.b); }
    dRGB operator*(const dRGB& a) { return dRGB(r*a.r, g*a.g, b*a.b); }
    dRGB operator/(const dRGB& a) { return dRGB(r/a.r, g/a.g, b/a.b); }
    dRGB& operator+=(dRGB a){ r+=a.r; g+=a.g; b+=a.b; return (*this); }
    dRGB& operator-=(dRGB a){ r-=a.r; g-=a.g; b-=a.b; return (*this); }
    dRGB& operator*=(dRGB a){ r*=a.r; g*=a.g; b*=a.b; return (*this); }
    dRGB& operator/=(dRGB a){ r/=a.r; g/=a.g; b/=a.b; return (*this); }

    dRGB operator+(const RGB& a) { return dRGB(r+a.r, g+a.g, b+a.b); }
    dRGB operator-(const RGB& a) { return dRGB(r-a.r, g-a.g, b-a.b); }
    dRGB operator*(const RGB& a) { return dRGB(r*a.r, g*a.g, b*a.b); }
    dRGB operator/(const RGB& a) { return dRGB(r/a.r, g/a.g, b/a.b); }
    dRGB& operator=(const RGB& a) { r=a.r; g=a.g; b=a.b; return (*this); }
    dRGB& operator+=(const RGB& a){ r+=a.r; g+=a.g; b+=a.b; return (*this); }
    dRGB& operator-=(const RGB& a){ r-=a.r; g-=a.g; b-=a.b; return (*this); }
    dRGB& operator*=(const RGB& a){ r*=a.r; g*=a.g; b*=a.b; return (*this); }
    dRGB& operator/=(const RGB& a){ r/=a.r; g/=a.g; b/=a.b; return (*this); }

    static dRGB exp(dRGB a){ return dRGB(std::exp(a.r),std::exp(a.g),std::exp(a.b)); };
    static dRGB abs(dRGB a){ return dRGB(std::abs(a.r),std::abs(a.g),std::abs(a.b)); };
    static dRGB max(dRGB a, dRGB b){ return dRGB(std::max(a.r,b.r),std::max(a.g,b.g),std::max(a.b,b.b)); };
    static dRGB min(dRGB a, dRGB b){ return dRGB(std::min(a.r,b.r),std::min(a.g,b.g),std::min(a.b,b.b)); };
};

//------------------------------------------------------------------------------
// Logistic フィルタ
//------------------------------------------------------------------------------
LogisticFilter::LogisticFilter(Image& image, double paramA, double paramB) {

    // 参照テーブル
    unsigned char LUT[256];

    // 参照テーブル作成
    for(int i=0; i<256; i++) {
        LUT[i] = (unsigned char)( 255/(1+exp(-paramA*(i-paramB))) );
    }

    // 処理本体
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
// MedianTS フィルタ 時間方向を含めたメディアンフィルタ
//------------------------------------------------------------------------------
MedianTSFilter::MedianTSFilter(Image& image,
                               std::vector<Image> inputs, int filterSize) {

    // 画像処理本体
    Processing = [&](int start, int length) {

        int halfSize = filterSize/2;
        int sqrSize  = filterSize*filterSize;

        unsigned char* R = new unsigned char[sqrSize*inputs.size()];
        unsigned char* G = new unsigned char[sqrSize*inputs.size()];
        unsigned char* B = new unsigned char[sqrSize*inputs.size()];

        for(int i=start; i<start+length; i++) {
            int iX = i % image.Width();
            int iY = i / image.Width();

            int pixelCount = 0;

            for(int j=0; j<sqrSize; j++){
                int jX = iX + j%filterSize - halfSize;
                int jY = iY + j/filterSize - halfSize;

                if(jX<0 || jX>=image.Width() || jY<0 || jY>=image.Height()) continue;

                for(auto& images : inputs) {
                    R[pixelCount] = images.pixel[jX][jY].r;
                    G[pixelCount] = images.pixel[jX][jY].g;
                    B[pixelCount] = images.pixel[jX][jY].b;
                    pixelCount++;
                }
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
// Trilateral フィルタ
//------------------------------------------------------------------------------
TrilateralFilter::TrilateralFilter(Image& image, Image& reference,
                            int filterSize, double sigma, double sigma2) {

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

                dRGB diff   = reference.pixel[iX][iY] - reference.pixel[jX][jY];
                dRGB weight = dRGB::exp( diff*diff / -sig2 );
                dRGB filter = weight * LUT[j];

                sum += filter * copy.pixel[jX][jY];
                div += filter;
            }

            image.pixel[iX][iY] = RGB(sum.r/div.r, sum.g/div.g, sum.b/div.b);
        }
    };

    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());

    delete[] LUT;
}

//------------------------------------------------------------------------------
// Quadrilateral フィルタ
//------------------------------------------------------------------------------
QuadrilateralFilter::QuadrilateralFilter(Image& image,
                Image color, Image laser, Image camera, int filterSize) {

    int halfSize = filterSize/2;

    // パラメータ
    double sigma  = 0.03;
    double sigma2 = 0.1;
    double sigma3 = 0.1;
    double sig = sigma * sigma;
    double sig2= sigma2 * sigma2;
    double sig3= sigma3 * sigma3;

    // カラー画像をモノクロ化
    mi::Monochrome::Process(color);
    
    // カメラ画像のノイズ除去
    mi::MedianFilter::Process(camera, 5);

    // ヒストグラム
//    mi::HistgramEqualization::Process(laser);
//    mi::HistgramEqualization::Process(camera);
//    laser.Save("histgram_laser.bmp");
//    camera.Save("histgram_camera.bmp");
    
    Image sub = image;
    
    
    // 処理本体
    Processing = [&](int start, int length) {

        for(int i=start; i<start+length; i++) {
            int iX = i % image.Width();
            int iY = i / image.Width();

            dRGB sumA; // ピクセルとの計算結果合計値
            dRGB sumB;
            dRGB div;  // 正規化用のフィルタ値合計
            
            dRGB suA;
            dRGB suB;

            for(int j=0; j<filterSize*filterSize; j++) {
                int jX = iX + j%filterSize - halfSize;
                int jY = iY + j/filterSize - halfSize;

                if( jX<0 || jX>=image.Width() || jY<0 || jY>=image.Height()) continue;
                
                dRGB one(1,1,1);

                dRGB laserDiff = dRGB::abs(laser.pixel[iX][iY] - laser.pixel[jX][jY]) / 255;
                dRGB colorDiff = dRGB::abs(color.pixel[iX][iY] - color.pixel[jX][jY]) / 255;
                dRGB cameraDiff= dRGB::abs(camera.pixel[iX][iY]- camera.pixel[jX][jY])/ 255;
                //dRGB camLsrDiff = dRGB::abs(camera.pixel[iX][iY]-laser.pixel[iX][iY]) / 255;

                
                // カラーとカメラのが両方ともエッジを検出しないと weight が小さくなる
                // 両方ともエッジを検出すると weight が大きくなる
                dRGB weight = one - dRGB::exp(cameraDiff*colorDiff/-sig);

                // レーザとカメラのエッジを検出しない画素からの色
                dRGB a = dRGB::exp(laserDiff*laserDiff/-sig2) * dRGB::exp(cameraDiff*colorDiff/-sig2);
                
                // レーザのエッジを検出し、カメラのエッジを検出しない画素からの色
                dRGB b = (one - dRGB::exp(laserDiff*laserDiff/-sig3)) * dRGB::exp(cameraDiff*colorDiff/-sig3);
                

                sumA += a * laser.pixel[jX][jY] * (one-weight);
                sumB += b * laser.pixel[jX][jY] * weight;
                div += a*(one-weight) + b*weight;
                
                suA += a * (one-weight);
                suB += b * weight;
            }
            
            dRGB sum = (sumA+sumB) / div;

            image.pixel[iX][iY].r = sum.r;
            image.pixel[iX][iY].g = sum.g;
            image.pixel[iX][iY].b = sum.b;

            dRGB sA = sumA / div;
            dRGB sB = sumB / div;

            //dRGB sA = suA/div*255.0;//sumA / div;
            //dRGB sB = suB/div*255.0;//sumB / div;
 
            sub.pixel[iX][iY].r = sA.r;
            sub.pixel[iX][iY].g = sB.g;
            sub.pixel[iX][iY].b = sB.b;
        }
    };
    

    // Processingの処理をおこなう
    Run(image, std::thread::hardware_concurrency());
    
    
    sub.Save("depth_output_sub.bmp");
}

}


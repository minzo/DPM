//==============================================================================
//
// DPMS (DP Matching Stereo)
//
//==============================================================================
#ifndef _DPMS_H_
#define _DPMS_H_

#include "DPM.h"

//------------------------------------------------------------------------------
//
// DP Matching Stereo
//
//------------------------------------------------------------------------------
class DPMS : public DPM
{
public:

    //--------------------------------------------------------------------------
    // @brief コンストラクタ
    // @param input     主画像(左カメラを想定)
    // @param reference 副画像(右カメラを想定)
    // @param threads   スレッド数
    //--------------------------------------------------------------------------
    DPMS(mi::Image& input, mi::Image& reference,
         int threads = std::thread::hardware_concurrency())
        : DPM(input, reference, threads),
          edge(input.Bit(), input.Width(), input.Height())
    {
    }


    //--------------------------------------------------------------------------
    // @brief DP マッチングによる対応付けをおこなう
    // @param skip         飛び越し量
    // @param weight       コストの重みパラメータ
    // @param rowRange     コスト計算時に参照する上下の画素数
    // @param threshold    コスト計算時に上下の画素の参照を打ち切る閾値
    // @param maxDisparity 想定する最大の視差
    //--------------------------------------------------------------------------
    virtual void dp(int skip, double weight, int rowRange, int threshold, int maxDisparity)
    {
        this->weight = weight;
        this->rowRange = rowRange;
        this->threshold = threshold;

        // 最大視差設定
        leftRange = maxDisparity;
        rightRange= 0;

        // エッジ抽出
        edge = input;

        int nThreads = threadPool.GetNumThread();
        int length   = input.Height() / nThreads;

        for(int i=0; i<nThreads; i++) {
            threadPool.Request([&,i,length](int id){ sobel(i*length, length);});
        }

        threadPool.Join();

        DPM::dp(skip);

        threadPool.Join();

        return;
    }

protected:

    //--------------------------------------------------------------------------
    // @brief コスト計算
    // @param x      DPテーブルX方向の系列の位置
    // @param y      DPテーブルY方向の系列の位置
    // @param column DPする走査線の位置
    // @param skip   マッチング済みの走査線までの距離
    //--------------------------------------------------------------------------
    virtual double calcCost(int x, int y, int column, int skip)
    {
        // 画素数
        int count = 1;

        // 対象画素
        double _d = norm(x, y, column);

        // 下方向
        for(int i=1; column+i<nScanlines && edge.pixel[x][column+i].g && i<rowRange; i++)
        {
            _d += norm(x, y, column+i);
            count++;
        }

        // 上方向
        for(int i=1; column-i>=0 && edge.pixel[x][column-i].g && i<rowRange; i++)
        {
            _d += norm(x, y, column-i);
            count++;
        }

        // 局所距離 d
        return _d/count;
    }

    //--------------------------------------------------------------------------
    // @brief 縦・横・斜, それぞれのパスに設定するコスト
    // @param x      DPテーブルX方向の系列の位置
    // @param y      DPテーブルY方向の系列の位置
    // @param column DPする走査線の位置
    // @param cost   CalcCost() で計算されたコスト
    //--------------------------------------------------------------------------
    virtual double diagonalCost(int x, int y, int column, double cost)
    {
        return weight * cost * cost;
    }

    //--------------------------------------------------------------------------
    // @brief ノルムの計算
    //--------------------------------------------------------------------------
    inline double norm(int x, int y, int column)
    {
        const mi::RGB& inputPixel = input.pixel[x][column];
        const mi::RGB& referPixel = refer.pixel[y][column];
        double r = inputPixel.r-referPixel.r;
        double g = inputPixel.g-referPixel.g;
        double b = inputPixel.b-referPixel.b;
        return sqrt(r*r+g*g+b*b) / 255.0;
    }

    //--------------------------------------------------------------------------
    // @brief エッジ抽出  G要素には閾値より大きければ 1 が入る
    //--------------------------------------------------------------------------
    inline void sobel(int start, int length)
    {
        const int w = input.Width() - 1;
        const int h = input.Height()- 1;

        if(start==0) start = 1;
        if(start+length>h-1) length = h - 1 - start;

        for(int iY=start; iY<start+length; iY++) {
            for(int iX=1; iX<w-1; iX++){

                const mi::RGB& rt = input.pixel[iX+1][iY-1]; // right top
                const mi::RGB& lt = input.pixel[iX-1][iY-1]; // left top
                const mi::RGB& rb = input.pixel[iX+1][iY+1]; // right bottom
                const mi::RGB& lb = input.pixel[iX-1][iY+1]; // left bottom

                const mi::RGB& rm = input.pixel[iX+1][iY]; // right middle
                const mi::RGB& lm = input.pixel[iX-1][iY]; // left middle
                const mi::RGB& ct = input.pixel[iX][iY-1]; // center top
                const mi::RGB& cb = input.pixel[iX][iY+1]; // center bottom

                int pxr = (rt.r - lt.r) + (rb.r - lb.r) + 2 * (rm.r - lm.r);
                int pxg = (rt.g - lt.g) + (rb.g - lb.g) + 2 * (rm.g - lm.g);
                int pxb = (rt.b - lt.b) + (rb.b - lb.b) + 2 * (rm.b - lm.b);

                int pyr = (lb.r - lt.r) + (rb.r - rt.r) + 2 * (cb.r - ct.r);
                int pyg = (lb.g - lt.g) + (rb.g - rt.g) + 2 * (cb.g - ct.g);
                int pyb = (lb.b - lt.b) + (rb.b - rt.b) + 2 * (cb.b - ct.b);

                int k = (pxr*pxr+pyr*pyr + pxg*pxg+pyg*pyg + pxb*pxb+pyb*pyb)/9;

                edge.pixel[iX][iY].r = (unsigned char)std::min(sqrt(k), 255.0);
                edge.pixel[iX][iY].g = edge.pixel[iX][iY].r > threshold ? 1 : 0;
            }
        }
    }

    // パラメータ
    double weight;
    int rowRange;
    int threshold;

    // エッジ画像
    mi::Image edge;
};


#endif
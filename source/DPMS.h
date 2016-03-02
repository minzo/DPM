//==============================================================================
//
// DPMS (DP Matching)
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
class DPMS : public DPM {
public:

    double weight = 13.0;
    int row_width = 4;
    int threshold = 80;

    //--------------------------------------------------------------------------
    // @brief コンストラクタ
    // @param input     主画像(左カメラを想定)
    // @param reference 副画像(右カメラを想定
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
    // @param maxDisparity 想定する最大の視差
    //--------------------------------------------------------------------------
    virtual void DP(int skip, int maxDisparity)
    {
        // 最大視差設定
        leftRange = maxDisparity;
        rightRange= 0;

        // エッジ抽出
        edge = input;
        
        int nThreads = threadPool.GetNumThread();
        int length   = input.Height() / nThreads;
        
        for(int i=0; i<nThreads; i++) {
            threadPool.Request([&,i,length](int id){ Sobel(i*length, length);});
        }
        
        threadPool.Join();
        
        DPM::DP(skip);
        
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
    virtual double CalcCost(int x, int y, int column, int skip)
    {
        auto norm = [&](int x, int y, int column) {
            double r = input.pixel[x][column].r-refer.pixel[y][column].r;
            double g = input.pixel[x][column].g-refer.pixel[y][column].g;
            double b = input.pixel[x][column].b-refer.pixel[y][column].b;
            return sqrt(r*r+g*g+b*b) / 255.0;
        };
        
        // 画素数
        int count = 1;

        // 対象画素
        double _d = norm(x, y, column);

        // 下方向
        for(int i=1; column+i<nScanlines && edge.pixel[x][column+i].g && i<row_width; i++)
        {
            _d += norm(x,y,column+i);
            count++;
        }

        // 上方向
        for(int i=1; column-i>=0 && edge.pixel[x][column-i].g && i<row_width; i++)
        {
            _d += norm(x,y,column-i);
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
    // @brief エッジ抽出
    //--------------------------------------------------------------------------
    void Sobel(int start, int length)
    {
        const int w = input.Width() - 1;
        const int h = input.Height()- 1;
        
        if(start==0) start = 1;
        if(start+length>h-1) length = h - 1 - start;
        
        for(int iY=start; iY<start+length; iY++) {
            for(int iX=1; iX<w-1; iX++){
                
                int pxr = (input.pixel[iX+1][iY-1].r - input.pixel[iX-1][iY-1].r)
                        + (input.pixel[iX+1][iY+1].r - input.pixel[iX-1][iY+1].r)
                        + 2 * (input.pixel[iX+1][iY].r - input.pixel[iX-1][iY].r);
                
                int pyr = (input.pixel[iX-1][iY+1].r - input.pixel[iX-1][iY-1].r)
                        + (input.pixel[iX+1][iY+1].r - input.pixel[iX+1][iY-1].r)
                        + 2 * (input.pixel[iX][iY+1].r - input.pixel[iX][iY-1].r);
                
                int pxg = (input.pixel[iX+1][iY-1].g - input.pixel[iX-1][iY-1].g)
                        + (input.pixel[iX+1][iY+1].g - input.pixel[iX-1][iY+1].g)
                        + 2 * (input.pixel[iX+1][iY].g - input.pixel[iX-1][iY].g);
                
                int pyg = (input.pixel[iX-1][iY+1].g - input.pixel[iX-1][iY-1].g)
                        + (input.pixel[iX+1][iY+1].g - input.pixel[iX+1][iY-1].g)
                        + 2 * (input.pixel[iX][iY+1].g - input.pixel[iX][iY-1].g);
                
                int pxb = (input.pixel[iX+1][iY-1].b - input.pixel[iX-1][iY-1].b)
                        + (input.pixel[iX+1][iY+1].b - input.pixel[iX-1][iY+1].b)
                        + 2 * (input.pixel[iX+1][iY].b - input.pixel[iX-1][iY].b);
                
                int pyb = (input.pixel[iX-1][iY+1].b - input.pixel[iX-1][iY-1].b)
                        + (input.pixel[iX+1][iY+1].b - input.pixel[iX+1][iY-1].b)
                        + 2 * (input.pixel[iX][iY+1].b - input.pixel[iX][iY-1].b);
                
                int k = (pxr*pxr+pyr*pyr + pxg*pxg+pyg*pyg + pxb*pxb+pyb*pyb)/9;
                
                edge.pixel[iX][iY].r = (unsigned char)std::min(sqrt(k), 255.0);
                edge.pixel[iX][iY].g = edge.pixel[iX][iY].r > threshold ? 1 : 0;
            }
        }
    }
    
    void NormTable() {
        
    }

    // エッジ画像
    mi::Image edge;
};


#endif
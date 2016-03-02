//==============================================================================
//
// DPMF (DP Matching Fusion)
//
//==============================================================================
#ifndef _DPMF_H_
#define _DPMF_H_

#include "DPM.h"

//------------------------------------------------------------------------------
//
// DP Matching Fusion
//
//------------------------------------------------------------------------------
class DPMF : public DPM {
public:
    
    // パラメタ
    double CostSigmaC = 0.01; //
    double CostSigmaG = 0.1;  //


    //--------------------------------------------------------------------------
    // @brief コンストラクタ
    // @param input     入力画像
    // @param reference 正確な距離情報の参照画像
    // @param threads   スレッド数
    //--------------------------------------------------------------------------
    DPMF(mi::Image& input, mi::Image& reference,
         int threads = std::thread::hardware_concurrency())
        : DPM(input, reference, threads)
    {
    }


    //--------------------------------------------------------------------------
    // @brief DP マッチングによる対応付けをおこなう
    // @param skip 飛び越し量
    // @param sigmaC パラメータ
    // @param sigmaG パラメータ
    //--------------------------------------------------------------------------
    virtual void DP(int skip, double sigmaC, double sigmaG)
    {
        CostSigmaC = sigmaC;
        CostSigmaG = sigmaG;

        DPM::DP(skip);

        threadPool.Join();
    }

protected:
    //--------------------------------------------------------------------------
    // @brief コスト計算
    // @param x      DPテーブルX方向の系列の位置
    // @param y      DPテーブルY方向の系列の位置
    // @param column DPする走査線の位置
    // @param skip   参考にする走査線までの距離(マッチング済みの走査線までの距離)
    //--------------------------------------------------------------------------
    virtual double CalcCost(int x, int y, int column, int skip)
    {
        const auto i = 1;
        const auto sig = 2*CostSigmaC*CostSigmaC;
        const auto sig2= 2*CostSigmaG*CostSigmaG;

        auto cA = 0.0;
        auto cB = 0.0;
        auto cC = 0.0;

        if(x-i<0) cA = (input.pixel[x][column].r-input.pixel[x+i][column].r) / 255.0;
        else      cA = (input.pixel[x][column].r-input.pixel[x-i][column].r) / 255.0;
            
            
        if(y-i<0) cB = (refer.pixel[y][column].r-refer.pixel[y+i][column].r) / 255.0;
        else      cB = (refer.pixel[y][column].r-refer.pixel[y-i][column].r) / 255.0;


        // 粘性計算
        double gluey=0.0;

        if(column-skip >=0 && column+skip<nScanlines)
        {
            std::vector<int>& matchPrev = matchPatterns[column-skip];
            std::vector<int>& matchNext = matchPatterns[column+skip];

            double prev    = refer.pixel[y][column-skip].r;
            double current = refer.pixel[y][column+0].r;
            double next    = refer.pixel[y][column+skip].r;
            

            // 上下の列の対応付けの結果における移動(伸縮)距離
            // 対応付け結果との距離が大きいほど g が大きくなり exp(g*g)が小さくなる
            // つまり, 1-exp(g*g) が大きなるためこの対応付けは回避される
            double distPrev = 1.0 * (matchPrev[y]-y) / length;
            double distNext = 1.0 * (matchNext[y]-y) / length;
            
            // 上下のピクセルとの類似度(値が1に近いほど似ている)
            double simPrev = 1.0 - std::abs(prev-current) / 255.0;
            double simNext = 1.0 - std::abs(next-current) / 255.0;

            // 上下のピクセルとの非類似度が低い(=類似度が高い)ほど
            // 上の列の対応付けの結果に近い対応のノードになるほどglueyの値が大きくなる
            gluey = std::abs(distPrev*simPrev) / 1.0;
        }


        // 上の列の対応付けの結果から大きく離れる(gが大きい)ほど exp() は小さくなる
        // 結果 1-exp() コストは, 大きくなる.
 
        double g = gluey;

        // 隣接するピクセルとの勾配が、レーザーとステレオで差(f)が大きいほど exp() は小さくなる
        // 結果 1-exp() コストは, 大きくなる.
        double f = std::abs(cA-cB);


        return (1.0 - std::exp(-f*f/sig)) + (1.0 - std::exp(-g*g/sig2));
    }

    //--------------------------------------------------------------------------
    // @brief 縦・横・斜, それぞれのパスに設定するコスト
    // @param x      DPテーブルX方向の系列の位置
    // @param y      DPテーブルY方向の系列の位置
    // @param column DPする走査線の位置
    // @param cost   CalcCost() で計算されたコスト
    //--------------------------------------------------------------------------
    virtual double verticalCost(int x, int y, int column, double cost)
    {
        double bias = (x-y)/X;
        return cost + bias*bias;
    }

    virtual double horizontalCost(int x, int y, int column, double cost)
    {
        double bias = (x-y)/X;
        return cost + bias*bias;
    }
};


#endif
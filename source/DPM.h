//==============================================================================
//
// DPM (DP Matching)
//
//  走査線の飛び越しをおこなうDPマッチング
//
//==============================================================================
#ifndef _DPM_H_
#define _DPM_H_

#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

#include "ThreadPool.h"
#include "miImage/miImage.h"

//------------------------------------------------------------------------------
//
// DP Matching
//
//  このクラスを継承して CalcCost() を実装することで用途にあわせて拡張できる.
//  * 追加でパラメータを与えたい場合は,
//    継承したクラスで DP() をオーバーライドして, その中でこのクラスの DP() を呼ぶこと.
//  * パスによってコストに偏らせたい場合は,
//    verticalBias(), horizontalBias(), diagonalBias() をオーバーライドすること.
//------------------------------------------------------------------------------
class DPM {
public:
    // パラメタ
    int leftRange = 40; // 対象画素から見た対応点の探索範囲左限界までの画素数
    int rightRange= 40; // 対象画素から見た対応点の探索範囲右限界までの画素数
    
    //--------------------------------------------------------------------------
    // @brief コンストラクタ
    // @param input     入力画像
    // @param reference 正確な距離情報の参照画像
    // @param threads   スレッド数
    //--------------------------------------------------------------------------
    DPM(mi::Image& input, mi::Image& reference,
        int threads = std::thread::hardware_concurrency())
        : input(input),
          refer(reference),
          threadPool(threads) {

        length    = input.Width() * refer.Width();
        nScanlines= input.Height();
        
        X = input.Width();
        Y = refer.Width();
        
        // ノードの確保
        nodes.resize(threadPool.GetNumThread());
        for(int i=0; i<nodes.size(); i++)
        {
            nodes[i].resize(length);
        }

        // マッチング結果の格納場所を確保
        matchPatterns.resize(nScanlines);
        for(int i=0; i<nScanlines; i++)
        {
            matchPatterns[i].resize(X);
            std::fill(matchPatterns[i].begin(),matchPatterns[i].end(), -1);
        }
    }

    //--------------------------------------------------------------------------
    // @brief マッチングしたパターンを取得
    // @param column 取得するスキャンライン
    //--------------------------------------------------------------------------
    std::vector<int>& GetMatchPattern(unsigned int column)
    {
        return matchPatterns[column];
    }


    //--------------------------------------------------------------------------
    // @brief DP マッチングによる対応付けをおこなう
    // @param skip 飛び越し量
    //--------------------------------------------------------------------------
    virtual void DP(int skip)
    {
        for(int i=0; i<nScanlines; i+=skip)
        {
            threadPool.Request([&,i,skip](int id){
                Matching(0, 0, X-1, Y-1, i, skip, id);
            });
        }
        
        SkipDP(skip/2);
    }

protected:
    //--------------------------------------------------------------------------
    // Node
    //--------------------------------------------------------------------------
    struct Node {

        enum PathDir : char {
            HORIZONTAL = 'h',
            VERTICAL   = 'v',
            DIAGONAL   = 'd',
            NONE       = 'n',
        };

        const double MAX_COST = 1.7976931348623158e+308;

        double cost = MAX_COST;

        double verticalPathCost  = MAX_COST;
        double horizontalPathCost= MAX_COST;
        double diagonalPathCost  = MAX_COST;

        char selectedPathDir = NONE;
    };
    

    // 画像
    mi::Image& input;
    mi::Image& refer;

    // スレッドプール
    ThreadPool threadPool;

    // DPテーブルに関する値
    int nScanlines; // スキャンラインの数
    int length;     // DPテーブルの長さ(幅x高さ)
    int X, Y;       // DPテーブルの横縦の長さ

    // DPテーブル
    std::vector<std::vector<Node> > nodes;

    // 各走査線のマッチング結果
    std::vector<std::vector<int> > matchPatterns;

    //--------------------------------------------------------------------------
    // @brief 補完する飛び越しつき DP マッチングによる対応付けをおこなう
    // @param skip 飛び越し量
    //--------------------------------------------------------------------------
    void SkipDP(int skip)
    {
        if(skip == 0)
        {
            return;
        }

        for(int i=skip; i<nScanlines; i+=(skip*2))
        {
            threadPool.Request([&,i,skip](int id){
                
                std::vector<int>& prev    = matchPatterns[std::max(i-skip,0)];
                std::vector<int>& next    = matchPatterns[std::min(i+skip,nScanlines-1)];
                std::vector<int>& current = matchPatterns[i];

                for(int iX=0; iX<X; iX++)
                {
                    // 補間
                    if(std::abs(std::abs(prev[iX]-iX) - std::abs(next[iX]-iX)) < 5)
                    {
                        current[iX] = prev[iX];
                    }
                    // 計算
                    else
                    {
                        int sx = std::max(0,iX-1);
                        int ex = [&]{
                            for(int jX=iX+1; jX<X; jX++)
                            {
                                if(prev[jX] == next[jX])
                                {
                                    return jX;
                                }
                            }
                            return X-1;
                        }();

                        Matching(sx, 0, ex, Y-1, i, skip, id);
                        iX = ex;
                    }
                }
            });
        }
        
        SkipDP(skip/2);
    }

    //--------------------------------------------------------------------------
    // @brief マッチングしてパターンを格納
    // @param sx     DPテーブルの始点 X 座標
    // @param sy     DPテーブルの始点 Y 座標
    // @param ex     DPテーブルの終点 X 座標
    // @param ex     DPテーブルの終点 Y 座標
    // @param column DPする走査線の位置
    // @param skip   飛び越した量(マッチング済みの走査線までの距離)
    // @param id     スレッド番号
    //--------------------------------------------------------------------------
    void Matching(int sx, int sy, int ex, int ey, int column, int skip, int id)
    {
        std::vector<Node>& node = nodes[id];

        // 探索範囲を考慮した値に更新
        sy = std::min(sx+rightRange, std::max(sx-leftRange, sy));
        ey = std::min(ex+rightRange, std::max(ex-leftRange, ey));

        
        // ノードの初期化 --------------------------------------------------------
        for(int iY=sy; iY<=ey; iY++)
        {
            int start = std::max(sx,iY-rightRange);
            int end   = std::min(ex,iY+leftRange);

            for(int iX=start; iX<=end; iX++)
            {
                int i = iX + iY*X;

                // コスト計算
                double cost = CalcCost(iX,iY,column,skip);

               // std::cout << iX << " " << iY << std::endl;

                node[i].verticalPathCost  = verticalCost(iX, iY, column, cost);
                node[i].horizontalPathCost= horizontalCost(iX, iY, column, cost);
                node[i].diagonalPathCost  = diagonalCost(iX, iY, column, cost);
            }
        }
        
        // DPM による最短経路探索 -------------------------------------------------
        // 始点の計算
        node[sx+sy*X].cost = 0;

        // 下端の計算
        for(int iX=sx+1; iX<=leftRange; iX++)
        {
            node[iX].cost = node[iX].horizontalPathCost + node[iX-1].cost;
            node[iX].selectedPathDir = Node::HORIZONTAL;
        }
        
        // 左端の計算
        for(int iY=sy+1; iY<=rightRange; iY++)
        {
            node[sx+iY*X].cost = node[sx+iY*X].verticalPathCost + node[sx+(iY-1)*X].cost;
            node[sx+iY*X].selectedPathDir = Node::VERTICAL;
        }

        // 経路探索
        for(int iY=sy+1; iY<=ey; iY++)
        {
            int start = std::max(sx+1,iY-rightRange);
            int end   = std::min(ex,iY+leftRange);
            
            for(int iX=start; iX<=end; iX++)
            {
                // ノードのインデックスを計算 (現在,縦,横,斜め)
                int ni = iX + iY * X;
                int nv = iX + (iY-1)*X;
                int nh = (iX-1) + iY*X;
                int nd = (iX-1) + (iY-1)*X;
                
                // コスト計算
                double vCost = node[ni].verticalPathCost   + node[nv].cost;
                double hCost = node[ni].horizontalPathCost + node[nh].cost;
                double dCost = node[ni].diagonalPathCost   + node[nd].cost;
                
                // 最小コスト計算
                node[ni].cost = std::min({vCost,hCost,dCost});
                
                // 選んだパスを記録
                //   double型だが計算はしていないので bit が一致する
                if(node[ni].cost == dCost) {
                    node[ni].selectedPathDir = Node::DIAGONAL;
                }else
                if(node[ni].cost == vCost) {
                    node[ni].selectedPathDir = Node::VERTICAL;
                }else
                if(node[ni].cost == hCost) {
                    node[ni].selectedPathDir = Node::HORIZONTAL;
                }
                else {
                    throw "NaN";
                }
            }
        }


        // Backtrace -----------------------------------------------------------
        int iX = ex;
        int iY = ey;
        
        std::vector<int>& matchPattern = matchPatterns[column];
        
        while( iX>sx || iY>sy )
        {
            matchPattern[iX] = iY;
            
            switch( node[iX+iY*X].selectedPathDir )
            {
                case Node::VERTICAL  : iY--; break;
                case Node::HORIZONTAL: iX--; break;
                case Node::DIAGONAL  : iX--; iY--; break;
                case Node::NONE :
                    std::cout<<sx<<" "<<sy<<" "<<ex<<" "<<ey<<" ";
                    std::cout<<iX<<" "<<iY<<" "<<node[iX+iY*X].selectedPathDir<<std::endl;
                default:
                    if(iX<=sx && iY>sy) iY--;
                    if(iY<=sy && iX>sx) iX--;
                    break;
            }
        }
    }

    //--------------------------------------------------------------------------
    // @brief コスト計算
    // @param x      DPテーブルX方向の系列の位置
    // @param y      DPテーブルY方向の系列の位置
    // @param column DPする走査線の位置
    // @param skip   マッチング済みの走査線までの距離
    //--------------------------------------------------------------------------
    virtual double CalcCost(int x, int y, int column, int skip) = 0;

    //--------------------------------------------------------------------------
    // @brief 縦・横・斜, それぞれのパスに設定するコスト
    // @param x      DPテーブルX方向の系列の位置
    // @param y      DPテーブルY方向の系列の位置
    // @param column DPする走査線の位置
    // @param cost   CalcCost() で計算されたコスト
    //--------------------------------------------------------------------------
    virtual double verticalCost(int x, int y, int column, double cost)  { return cost; }
    virtual double horizontalCost(int x, int y, int column, double cost){ return cost; }
    virtual double diagonalCost(int x, int y, int column,  double cost) { return cost; }
};


#endif
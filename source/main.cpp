#include <iostream>
#include <string>

#include "miImage/miImage.h"
#include "miImage/miImageProcessing.h"
#include "miImage/miDepthProcessing.h"

#include "DPMS.h"
#include "DPMF.h"

void Stereo(int i = 1)
{
    mi::Image left("input/tsukuba/color_left.bmp");
    mi::Image right("input/tsukuba/color_right.bmp");
    mi::Image stereo(left.Bit(), left.Width(), left.Height());

    DPMS dpms(left, right, 8);

    int disparity = 40;

    while(i-->0)
    {
        auto start = std::chrono::system_clock::now();

        dpms.DP(8, 13, 4, 80, disparity);

        // 視差画像の生成
        for(int iY=0; iY<stereo.Height(); iY++) {
            std::vector<int>& match = dpms.GetMatchPattern(iY);
            for(int iX=0; iX<stereo.Width(); iX++) {
                stereo.pixel[iX][iY].r = std::min(std::abs(match[iX] - iX) * 255.0/ disparity, 255.0);
                stereo.pixel[iX][iY].b = stereo.pixel[iX][iY].g = stereo.pixel[iX][iY].r;
            }
        }


        auto end = std::chrono::system_clock::now();

        std::cout << "elapsed time = ";
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
        std::cout << " msec." << std::endl;
    }

    // 保存
    stereo.Save("depth_stereo.bmp");
}

void Fusion(int i = 1)
{
    mi::Image laser("input/depth_laser.bmp");
    mi::Image camera("depth_stereo.bmp");
    mi::Image fusion(laser.Bit(), laser.Width(), laser.Height());

    DPMF dpmf(camera,laser);

    while(i-->0)
    {
        auto start = std::chrono::system_clock::now();

        // マッチング
        dpmf.DP(8, 0.30, 0.03);

        // ピクセルの置き換え
        for(int iY=0; iY<laser.Height(); iY++) {
            std::vector<int>& match = dpmf.GetMatchPattern(iY);
            for(int iX=0; iX<laser.Width(); iX++) {
                fusion.pixel[iX][iY] = laser.pixel[ match[iX] ][iY];
            }
        }

        auto end = std::chrono::system_clock::now();

        std::cout << "elapsed time = ";
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
        std::cout << " msec." << std::endl;
    }


    // 保存
    fusion.Save("depth_fusion.bmp");
}




int main(int argc, char* argv[])
{
    Stereo();
//    Fusion();

    return 0;
}
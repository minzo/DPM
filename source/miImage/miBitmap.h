//==============================================================================
//
// Windows Bitmap を読み書きするクラス
//
//==============================================================================
#ifndef _MI_BITMAP_H_
#define _MI_BITMAP_H_

#include <fstream>
#include "miImage.h"
#include "IImageReaderWriter.h"

namespace mi {

//------------------------------------------------------------------------------
// Windows Bitmap を読み書きするクラス (8,24,32bitのみ)
//------------------------------------------------------------------------------
class Bitmap : public IImageReaderWriter {
public:

    //--------------------------------------------------------------------------
    // コンストラクタ
    //--------------------------------------------------------------------------
    Bitmap(int bit, int width, int height);
    Bitmap(const char* fileName);
    Bitmap(mi::Image& image);
    ~Bitmap();


    //--------------------------------------------------------------------------
    // Getter
    //--------------------------------------------------------------------------
    int Bit()   { return _header.bitmapInfoHeader.biBitCount; }
    int Width() { return _header.bitmapInfoHeader.biWidth; }
    int Height(){ return _header.bitmapInfoHeader.biHeight; }
    int Size()  { return Width() * Height(); }


    //--------------------------------------------------------------------------
    // Bitmapファイルの読み込み・書き込み
    //--------------------------------------------------------------------------
    void Read(const char* fileName);
    void Write(const char* fileName);


    //--------------------------------------------------------------------------
    // IImageReaderWriter
    //--------------------------------------------------------------------------
    void CopyToImage(Image& image);
    void CopyFromImage(Image& image);

private:
    // 使えないようにする
    Bitmap(){};

    // BitmapFileHeader (Bitmapファイル共通のヘッダ情報)
    struct BITMAPFILEHEADER {
        unsigned short bfType      = ('B'|('M'<<8));
        unsigned int   bfSize      = 54;
        unsigned short bfReserved1 = 0;
        unsigned short bfReserved2 = 0;
        unsigned int   bfOffBits   = 54;
    };

    // BitmapInfoHeader (Windows Bitmapのヘッダ情報)
    struct BITMAPINFOHEADER{
        unsigned int   biSize         = 0;
        int            biWidth        = 0;
        int            biHeight       = 0;
        unsigned short biPlanes       = 1;
        unsigned short biBitCount     = 24;
        unsigned int   biCompression  = 0;
        unsigned int   biSizeImage    = 0;
        int            biXPixPerMeter = 3780;
        int            biYPixPerMeter = 3780;
        unsigned int   biClrUsed      = 0;
        unsigned int   biClrImportant = 0;
    };

    // ピクセル型
    struct RGBQUAD {
        unsigned char b = 0;
        unsigned char g = 0;
        unsigned char r = 0;
        unsigned char reserved = 0;
    };

    // Bitmap Header をまとめた型
    struct WindowsBitmapHeader {
        BITMAPFILEHEADER bitmapFileHeader;
        BITMAPINFOHEADER bitmapInfoHeader;
    };

    // Bitmap Header
    WindowsBitmapHeader _header;

    // パレットデータ
    RGBQUAD _palette[256];
    
    // 画素データ
    RGBQUAD* _pixels = nullptr;

    //--------------------------------------------------------------------------
    // WindowsBitmapheader の読み込み・書き込み
    //--------------------------------------------------------------------------
    void ReadWindowsBitmapHeader(std::ifstream& file);
    void WriteWindowsBitmapHeader(std::ofstream& file);
    
    //--------------------------------------------------------------------------
    // BitmapPalette の読み込み・書き込み
    //--------------------------------------------------------------------------
    void ReadBitmapPalette(std::ifstream& file);
    void WriteBitmapPalette(std::ofstream& file);

    //--------------------------------------------------------------------------
    // WindowsBitmap 画像の読み込み・書き込み
    //--------------------------------------------------------------------------
    void ReadBitmapImage(std::ifstream& file);
    void WriteBitmapImage(std::ofstream& file);
};

}

#endif
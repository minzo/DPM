//==============================================================================
//
// Windows Bitmap を読み書きするクラス
//
//==============================================================================
#include "miBitmap.h"

#include <iostream>
#include <fstream>

namespace mi {

//------------------------------------------------------------------------------
// コンストラクタ
//------------------------------------------------------------------------------

// 空のBitmapを作成する
Bitmap::Bitmap(int bit, int width, int height) {

    _header.bitmapInfoHeader.biBitCount = bit;
    _header.bitmapInfoHeader.biWidth    = width;
    _header.bitmapInfoHeader.biHeight   = height;

    _pixels = new RGBQUAD[width * height];
    
    // パレットをグレースケールで生成
    for(auto i=0; i<sizeof(_palette)/sizeof(_palette[0]); i++) {
        _palette[i].r = _palette[i].g = _palette[i].b = i;
    }
}

// ファイルからBitmapを作成する
Bitmap::Bitmap(const char* fileName) {

    // Readで再確保するためのダミー
    _pixels = new RGBQUAD[1];

    Read(fileName);
}


//------------------------------------------------------------------------------
// デストラクタ
//------------------------------------------------------------------------------
Bitmap::~Bitmap() {
    delete[] _pixels;
}

//------------------------------------------------------------------------------
// Bitmapを読み込む
//------------------------------------------------------------------------------
void Bitmap::Read(const char* fileName) {

    // ファイルを開く
    std::ifstream readFile(fileName, std::ios::binary);
    if(!readFile.is_open()) {
        std::cerr<<"Error: Cant File Open"<<std::endl;
        throw "File Open Error";
    }

    // ヘッダを読み込む
    ReadWindowsBitmapHeader(readFile);

    // Bitmapファイルかチェック
    if(_header.bitmapFileHeader.bfType != ('B'|('M'<<8))){
        std::cerr<<"Error: This is not Bitmap Image"<<std::endl;
        throw "File Open Error";
    }

    // Windows Bitmap がチェック (biSizeが40ならWindowsBitmap)
    if(_header.bitmapInfoHeader.biSize != 40 &&
       _header.bitmapInfoHeader.biSize !=108 &&
       _header.bitmapInfoHeader.biSize !=124 ) {
        std::cerr<<"Error: This is not Windows Bitmap"<<std::endl;
        throw "File Open Error";
    }
    
    // 8,24,32bitではない場合
    if( Bit()!=8 && Bit()!=24 && Bit()!=32 ) {
        std::cerr<<"Error: Not Supported Bitmap"<<std::endl;
        throw "File Open Error";
    }

    // パレットの読み込み
    ReadBitmapPalette(readFile);

    // 領域の再確保
    int width = _header.bitmapInfoHeader.biWidth;
    int height= _header.bitmapInfoHeader.biHeight;
    delete[] _pixels;

    _pixels = new RGBQUAD[width*abs(height)];

    // 画像の読み込み
    ReadBitmapImage(readFile);

    // ファイルを閉じる
    readFile.close();
    
    // 高さを正の数にする
    _header.bitmapInfoHeader.biHeight = abs(height);
}


//------------------------------------------------------------------------------
// Bitmapで書き込む
//------------------------------------------------------------------------------
void Bitmap::Write(const char* fileName) {

    // ファイルを開く
    std::ofstream writeFile(fileName, std::ios::binary | std::ios::trunc | std::ios::out);

    // ヘッダを書き込む
    WriteWindowsBitmapHeader(writeFile);

    // パレットの書き込み
    WriteBitmapPalette(writeFile);

    // 画素の書き込み
    WriteBitmapImage(writeFile);

    // ファイルを閉じる
    writeFile.close();
}


//------------------------------------------------------------------------------
// 自身のピクセルデータをmiImage型へ変換してコピーする
//------------------------------------------------------------------------------
void Bitmap::CopyToImage(Image& image){
    for(int i=0; i<image.Size(); i++) {
        image.data[i].r = _pixels[i].r;
        image.data[i].g = _pixels[i].g;
        image.data[i].b = _pixels[i].b;
    }
}


//------------------------------------------------------------------------------
// 自身のピクセルデータにmiImage型から変換してコピーする
//------------------------------------------------------------------------------
void Bitmap::CopyFromImage(Image& image){
    for(int i=0; i<image.Size(); i++) {
        _pixels[i].r = image.data[i].r;
        _pixels[i].g = image.data[i].g;
        _pixels[i].b = image.data[i].b;
    }
}


//------------------------------------------------------------------------------
// ヘッダを読み込む
//------------------------------------------------------------------------------
void Bitmap::ReadWindowsBitmapHeader(std::ifstream& file) {

    // 各型のサイズや構造体のアラインを考慮して各項目の読み込みサイズを直接指定している

    // BITMAPFILEHEADER
    file.read((char*)&_header.bitmapFileHeader.bfType,      2);
    file.read((char*)&_header.bitmapFileHeader.bfSize,      4);
    file.read((char*)&_header.bitmapFileHeader.bfReserved1, 2);
    file.read((char*)&_header.bitmapFileHeader.bfReserved2, 2);
    file.read((char*)&_header.bitmapFileHeader.bfOffBits,   4);

    // BITMAPINFOHEADER
    file.read((char*)&_header.bitmapInfoHeader.biSize,         4);
    file.read((char*)&_header.bitmapInfoHeader.biWidth,        4);
    file.read((char*)&_header.bitmapInfoHeader.biHeight,       4);
    file.read((char*)&_header.bitmapInfoHeader.biPlanes,       2);
    file.read((char*)&_header.bitmapInfoHeader.biBitCount,     2);
    file.read((char*)&_header.bitmapInfoHeader.biCompression,  4);
    file.read((char*)&_header.bitmapInfoHeader.biSizeImage,    4);
    file.read((char*)&_header.bitmapInfoHeader.biXPixPerMeter, 4);
    file.read((char*)&_header.bitmapInfoHeader.biYPixPerMeter, 4);
    file.read((char*)&_header.bitmapInfoHeader.biClrUsed,      4);
    file.read((char*)&_header.bitmapInfoHeader.biClrImportant, 4);

    // V4タイプ対応のための読み飛ばし
    if(_header.bitmapInfoHeader.biSize == 108) {
        char dummy[36];
        file.read(dummy, 4); // bV4RedMask
        file.read(dummy, 4); // bV4GreenMask
        file.read(dummy, 4); // bV4BlueMask
        file.read(dummy, 4); // bV4AlphaMask

        file.read(dummy, 4); // bV4CSType
        file.read(dummy,36); // bV4Endpoints;

        file.read(dummy, 4); // bV4GammaRed;
        file.read(dummy, 4); // bV4GammaGreen;
        file.read(dummy, 4); // bV4GammaBlue;
    }

    // V5タイプ対応のための読み飛ばし
    if(_header.bitmapInfoHeader.biSize == 124) {
        char dummy[36];
        file.read(dummy, 4); // bV5RedMask
        file.read(dummy, 4); // bV5GreenMask
        file.read(dummy, 4); // bV5BlueMask
        file.read(dummy, 4); // bV5AlphaMask

        file.read(dummy, 4); // bV5CSType
        file.read(dummy,36); // bV5Endpoints;

        file.read(dummy, 4); // bV5GammaRed;
        file.read(dummy, 4); // bV5GammaGreen;
        file.read(dummy, 4); // bV5GammaBlue;

        file.read(dummy, 4); // bV5Intent;
        file.read(dummy, 4); // bV5ProfileData;
        file.read(dummy, 4); // bV5ProfileSize;
        file.read(dummy, 4); // bV5Reserved;
    }
}


//------------------------------------------------------------------------------
// ヘッダを書き込む
//------------------------------------------------------------------------------
void Bitmap::WriteWindowsBitmapHeader(std::ofstream& file) {

    auto byte  = Bit()/8;  // 1画素のByte数
    auto width = Width();  // 画像の幅
    auto height= Height(); // 画像の高さ

    auto imageSize = (byte*width + (byte*Width())%4) * height;

    // ヘッダに書き込む値を設定
    _header.bitmapFileHeader.bfType = 'B'|('M'<<8);
    _header.bitmapFileHeader.bfSize = 54 + imageSize;
    _header.bitmapFileHeader.bfReserved1 = 0;
    _header.bitmapFileHeader.bfReserved2 = 0;
    _header.bitmapFileHeader.bfSize = 54 + imageSize;

    _header.bitmapInfoHeader.biSize         = 40;
    _header.bitmapInfoHeader.biWidth        = width;
    _header.bitmapInfoHeader.biHeight       = height;
    _header.bitmapInfoHeader.biPlanes       = 1;
    _header.bitmapInfoHeader.biCompression  = 0;
    _header.bitmapInfoHeader.biSizeImage    = imageSize;
    _header.bitmapInfoHeader.biXPixPerMeter = 3780;
    _header.bitmapInfoHeader.biYPixPerMeter = 3780;
    _header.bitmapInfoHeader.biClrUsed      = 0x01ff & (0x01 << Bit());
    _header.bitmapInfoHeader.biClrImportant = 0;


    // 各型のサイズや構造体のアラインを考慮して各項目の書き込みサイズを直接指定している

    // BITMAPFILEHEADER
    file.write((char*)&_header.bitmapFileHeader.bfType,      2);
    file.write((char*)&_header.bitmapFileHeader.bfSize,      4);
    file.write((char*)&_header.bitmapFileHeader.bfReserved1, 2);
    file.write((char*)&_header.bitmapFileHeader.bfReserved2, 2);
    file.write((char*)&_header.bitmapFileHeader.bfOffBits,   4);

    // BITMAPINFOHEADER
    file.write((char*)&_header.bitmapInfoHeader.biSize,         4);
    file.write((char*)&_header.bitmapInfoHeader.biWidth,        4);
    file.write((char*)&_header.bitmapInfoHeader.biHeight,       4);
    file.write((char*)&_header.bitmapInfoHeader.biPlanes,       2);
    file.write((char*)&_header.bitmapInfoHeader.biBitCount,     2);
    file.write((char*)&_header.bitmapInfoHeader.biCompression,  4);
    file.write((char*)&_header.bitmapInfoHeader.biSizeImage,    4);
    file.write((char*)&_header.bitmapInfoHeader.biXPixPerMeter, 4);
    file.write((char*)&_header.bitmapInfoHeader.biYPixPerMeter, 4);
    file.write((char*)&_header.bitmapInfoHeader.biClrUsed,      4);
    file.write((char*)&_header.bitmapInfoHeader.biClrImportant, 4);
}

//------------------------------------------------------------------------------
// パレットを読み込む
//------------------------------------------------------------------------------
void Bitmap::ReadBitmapPalette(std::ifstream& file) {
     // 24,32bitのときは biClrUsed=0 なので実際は読み込まないのと同じ
     file.read((char*)_palette, _header.bitmapInfoHeader.biClrUsed*sizeof(RGBQUAD));
}

//------------------------------------------------------------------------------
// パレットを書き込む
//------------------------------------------------------------------------------
void Bitmap::WriteBitmapPalette(std::ofstream& file) {
    // 24,32bitのときは biClrUsed=0 なので実際は書き込まないのと同じ
    file.write((char*)_palette, _header.bitmapInfoHeader.biClrUsed*sizeof(RGBQUAD));
}

//------------------------------------------------------------------------------
// ピクセルを読み込む
//------------------------------------------------------------------------------
void Bitmap::ReadBitmapImage(std::ifstream& file) {

    auto byte  = Bit()/8;              // 1画素のByte数
    auto width = Width();              // 画像の幅
    auto height= Height();             // 画像の高さ

    const int readWidth   = byte * width;      // 横1ラインのサイズ
    const int paddingSize = (4-readWidth%4)%4; // 横のサイズ4byteに揃えるための大きさ

    int iX, iY, padding = 0;

    // height が 正の数なら左下から読み込む
    if(height > 0) {
        // 読み込み
        for(iY=height-1; iY>=0; iY--){

            // 1行分読み込み
            for(iX=0; iX<width; iX++){
                file.read((char*)&_pixels[width*iY + iX], byte);
            }

            // 4byteに揃えるための部分をから読みして飛ばす
            file.read((char*)&padding, paddingSize);
        }
    }
    //  そうでなければ左上から読み込む
    else {
        // 読み込み
        for(iY=0; iY<-height; iY++){

            // 1行分読み込み
            for(iX=0; iX<width; iX++){
                file.read((char*)&_pixels[width*iY + iX], byte);
            }

            // 4byteに揃えるための部分をから読みして飛ばす
            file.read((char*)&padding, paddingSize);
        }        
    }
}


//------------------------------------------------------------------------------
// ピクセルを書き込む
//------------------------------------------------------------------------------
void Bitmap::WriteBitmapImage(std::ofstream& file) {

    auto byte  = Bit()/8;              // 1画素のByte数
    auto height= Height();             // 画像の高さ 
    auto width = Width();              // 画像の幅

    auto writeWidth  = byte * width;      // 横1ラインのサイズ
    auto paddingSize = (4-writeWidth%4)%4; // 横のサイズ4byteに揃えるための大きさ

    int iX, iY, padding = 0;

    // 書き込み
    for(iY=height-1; iY>=0; iY--){

        // 1行分書き込み
        for(iX=0; iX<width; iX++){
            file.write((char*)&_pixels[width*iY + iX], byte);
        }

        // 4byteに揃えるため0で埋める
        file.write((char*)&padding, paddingSize);
    }
}

}

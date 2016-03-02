//==============================================================================
//
// 画像を扱うファイル
//
//==============================================================================
#include "miImage.h"

#include "miBitmap.h"

namespace mi {

//------------------------------------------------------------------------------
// コンストラクタ / デストラクタ
//------------------------------------------------------------------------------
Image::Image(const char* fileName) {
    Load(fileName);
}

Image::Image(int bit, int width, int height) {
    Initialize(bit, width, height);
}

Image::Image() {
    Initialize(bit, width, height);
}

Image::~Image() {
    delete[] data;
}
    
Image::Image(const Image& copied) {
    Initialize(copied.Bit(), copied.Width(), copied.Height());
    std::copy(copied.data, copied.data+copied.Size(), data);

}
    
Image& Image::operator=(const Image& copied) {
    Initialize(copied.Bit(), copied.Width(), copied.Height());
    std::copy(copied.data, copied.data+copied.Size(), data);
    return *this;
}


//--------------------------------------------------------------------------
// 読み込み
//--------------------------------------------------------------------------
void Image::Load(const char* fileName) {

    Bitmap bitmap(fileName);

    // 画像のサイズが違ったら再確保
    if(width != bitmap.Width() || height != bitmap.Height()) {
        Initialize(bitmap.Bit(), bitmap.Width(), bitmap.Height());
    }
    bitmap.CopyToImage(*this);
}


//--------------------------------------------------------------------------
// 書き込み
//--------------------------------------------------------------------------
void Image::Save(const char* fileName) {

    Bitmap bitmap(bit, width, height);
    bitmap.CopyFromImage(*this);
    bitmap.Write(fileName);
}
    
    
//--------------------------------------------------------------------------
// サイズ変更
//--------------------------------------------------------------------------
void Image::Resize(int width, int height) {

    // コピー
    Image copy = *this;
    
    // 大きさ変更
    Initialize(Bit(), width, height);
    
    // 縮小処理
    double scaleX = (double)(width - 1) / ( copy.Width() - 1);
    double scaleY = (double)(height- 1) / ( copy.Height()- 1);
    
    for(int i=0; i<Size(); i++) {
        int iX = i % Width();
        int iY = i / Width();
        int jX = (int)(iX / scaleX);
        int jY = (int)(iY / scaleY);
        pixel[iX][iY] = copy.pixel[jX][jY];
    }
}
    

//--------------------------------------------------------------------------
// 切り抜き
//--------------------------------------------------------------------------
void Image::Clip(int x, int y, int width, int height) {
    
    // コピー
    Image copy = *this;
    
    // 大きさ変更
    Initialize(Bit(), width, height);

    // コピー作業
    for(int i=0; i<Height(); i++) {
        int iImage = i*Width();
        int iCopy  = (i+y)*copy.Width() + x;
        std::copy(&copy.data[iCopy], &copy.data[iCopy]+width, &data[iImage]);
    }
}

//--------------------------------------------------------------------------
// 初期化する
//
//   MEMO: data がすでに確保された領域がある場合リークするので自分で解放すること
//--------------------------------------------------------------------------
void Image::Initialize(int bit, int width, int height) {
    
    // すでにデータがあったら削除しておく
    if(data!=nullptr) {
        delete[] data;
        data = nullptr;
    }

    // 初期化処理
    this->bit   = bit;
    this->width = width;
    this->height= height;

    size = width * height;
    data = new RGB[size];

    pixel.sizeX = width;
    pixel.sizeY = height;
    pixel.data  = data;
}

}

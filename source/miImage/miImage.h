//==============================================================================
//
// 画像を扱うファイル
//
//==============================================================================
#ifndef _MI_IMAGE_H_
#define _MI_IMAGE_H_

namespace mi {

//------------------------------------------------------------------------------
// 汎用ピクセル型
//------------------------------------------------------------------------------
struct RGB {
    unsigned char r = 0;
    unsigned char g = 0;
    unsigned char b = 0;

    RGB():r(0),g(0),b(0){}
    RGB(const RGB& obj) { r=obj.r; g=obj.g; b=obj.b; }
    template<typename T> RGB(T r, T g, T b)
        : r((unsigned char)r), g((unsigned char)g), b((unsigned char)b) {}

    RGB operator+(const RGB& a) { return RGB(r+a.r, g+a.g, b+a.b); }
    RGB operator-(const RGB& a) { return RGB(r-a.r, g-a.g, b-a.b); }
    RGB operator*(const RGB& a) { return RGB(r*a.r, g*a.g, b*a.b); }
    RGB operator/(const RGB& a) { return RGB(r/a.r, g/a.g, b/a.b); }
    RGB& operator+=(RGB a){ r+=a.r; g+=a.g; b+=a.b; return (*this); }
    RGB& operator-=(RGB a){ r-=a.r; g-=a.g; b-=a.b; return (*this); }
    RGB& operator*=(RGB a){ r*=a.r; g*=a.g; b*=a.b; return (*this); }
    RGB& operator/=(RGB a){ r/=a.r; g/=a.g; b/=a.b; return (*this); }

    template<typename T> RGB operator+(const T a) { return RGB(r+a,g+a,b+a); }
    template<typename T> RGB operator-(const T a) { return RGB(r-a,g-a,b-a); }
    template<typename T> RGB operator*(const T a) { return RGB(r*a,g*a,b*a); }
    template<typename T> RGB operator/(const T a) { return RGB(r/a,g/a,b/a); }
    template<typename T> RGB operator%(const T a) { return RGB(r%a,g%a,b%a); }
    template<typename T> RGB& operator+=(const T a) {
        r+=(unsigned char)a;
        g+=(unsigned char)a;
        b+=(unsigned char)a;
        return (*this);
    }
    template<typename T> RGB& operator*=(const T a) {
        r-=(unsigned char)a;
        g-=(unsigned char)a;
        b-=(unsigned char)a;
        return (*this);
    }
    template<typename T> RGB& operator-=(const T a) {
        r*=(unsigned char)a;
        g*=(unsigned char)a;
        b*=(unsigned char)a;
        return (*this);
    }
    template<typename T> RGB& operator/=(const T a) {
        r/=(unsigned char)a;
        g/=(unsigned char)a;
        b/=(unsigned char)a;
        return (*this);
    }
};


//------------------------------------------------------------------------------
// 画素に[X座標][Y座標]でアクセスするためのクラス
//------------------------------------------------------------------------------
template<class T> class PixelArray2D {
private:
    class Proxy {
    public:
        Proxy(PixelArray2D<T>& array, int index) : array(array), indexX(index) {}
        T& operator[](int index)
        {
            return array.data[array.sizeX * index + indexX];
        }
    private:
        int indexX;
        PixelArray2D<T>& array;
    };

public:
    int sizeX;
    int sizeY;
    T* data;

    Proxy operator[](int index)
    {
        return Proxy(*this,index);
    }
};


//------------------------------------------------------------------------------
// 汎用画像型
//------------------------------------------------------------------------------
class Image {
public:
    int bit;                 // bit数
    RGB* data = nullptr;     // 画素データ
    PixelArray2D<RGB> pixel; // 画素データを2次元配列でアクセス

    //--------------------------------------------------------------------------
    // コンストラクタ / デストラクタ / コピーコンストラクタ
    //--------------------------------------------------------------------------
    Image(const char* filename);
    Image(int bit, int width, int height);
    Image();
    ~Image();
    Image(const Image& copied);
    Image& operator=(const Image& copied);

    //--------------------------------------------------------------------------
    // 読み込み / 書き込み
    //--------------------------------------------------------------------------
    void Load(const char* fileName);
    void Save(const char* fileName);
    
    //--------------------------------------------------------------------------
    // サイズ変更
    //--------------------------------------------------------------------------
    void Resize(int width, int height);
    void Clip(int x, int y, int width, int height);

    //--------------------------------------------------------------------------
    // Getter
    //--------------------------------------------------------------------------
    int Bit()    const { return bit;}
    int Width()  const { return width; }
    int Height() const { return height; }
    int Size()   const { return size; }

    RGB* Data() { return data; }
    PixelArray2D<RGB>& Pixel() { return pixel; };

private:

    // 初期化する
    void Initialize(int bit, int width, int height);

    int width  = 0;  // 幅
    int height = 0;  // 高さ
    int size   = 0;  // 画素総数
};

}

#endif

#ifndef _I_IMAGE_READER_WRITER_H_
#define _I_IMAGE_READER_WRITER_H_

namespace mi {
    class Image;
}

class IImageReaderWriter {

public:
    // 自身のピクセルデータをmiImage型へ変換してコピーする
    virtual void CopyToImage(mi::Image& image) = 0;

    // 自身のピクセルデータにmiImage型から変換してコピーする
    virtual void CopyFromImage(mi::Image& image) = 0;
};

#endif

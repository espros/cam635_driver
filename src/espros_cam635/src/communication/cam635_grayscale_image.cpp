#include "cam635_grayscale_image.h"

namespace com_lib
{

TofCam635Image::TofCam635ImageType_e TofCam635GrayscaleImage::getType()
{
  return TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_GRAYSCALE;
}

}

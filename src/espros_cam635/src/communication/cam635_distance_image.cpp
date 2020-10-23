#include "cam635_distance_image.h"

namespace com_lib
{

void TofCam635DistanceImage::changePixel(TofCam635Image &imageSource, const unsigned int index)
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

   //Make a pointer to the distance
   uint16_t *distanceTarget =  (uint16_t *)(&data.data()[dataOffset]);

   uint16_t *distanceSource =  (uint16_t *)(&imageSource.getData().data()[dataOffset]);

   distanceTarget[index] = distanceSource[index];
}

TofCam635Image::TofCam635ImageType_e TofCam635DistanceImage::getType()
{
  return TofCam635Image::TofCam635ImageType_e::TOFCAM635IMAGE_DISTANCE;
}

unsigned int TofCam635DistanceImage::getDistanceOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  //Make a pointer to the distance
  uint16_t *distance =  (uint16_t *)(&data.data()[dataOffset]);

  return (distance[index] & CommunicationConstants::PixelTofCam635::MASK_OUT_CONFIDENCE);
}

}

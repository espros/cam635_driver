#include "cam635_distance_amplitude_image.h"

namespace com_lib
{

void TofCam635DistanceAmplitudeImage::changePixel(TofCam635Image &imageSource, const unsigned int index)
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  //Make a pointer to the distance
  uint32_t *distanceAmplitudeTarget =  (uint32_t *)(&data.data()[dataOffset]);

  uint32_t *distanceAmplitudeSource =  (uint32_t *)(&imageSource.getData().data()[dataOffset]);

  distanceAmplitudeTarget[index] = distanceAmplitudeSource[index];
}

TofCam635Image::TofCam635ImageType_e TofCam635DistanceAmplitudeImage::getType()
{
  return TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_DISTANCE_AMPLITUDE;
}

unsigned int TofCam635DistanceAmplitudeImage::getDistanceOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  //Make a pointer to the distance
  uint32_t *distanceAmplitude =  (uint32_t *)(&data.data()[dataOffset]);

  return (distanceAmplitude[index] & 0xFFFF);
}

unsigned int TofCam635DistanceAmplitudeImage::getAmplitudeOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  //Make a pointer to the distance
  uint32_t *distanceAmplitude =  (uint32_t *)(&data.data()[dataOffset]);

  return (distanceAmplitude[index] >> 16);
}

}

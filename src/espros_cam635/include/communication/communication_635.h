/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication_epc635 EPC635
 * @brief Implementation for Cam635
 * @ingroup communication
 *
 * @{
 */
#ifndef COMMUNICATION_635_H
#define COMMUNICATION_635_H

#include "communication.h"
#include "cam635_image.h"
#include "cam635_distance_image.h"
#include "cam635_distance_amplitude_image.h"
#include "cam635_distance_grayscale_image.h"
#include "cam635_grayscale_image.h"


namespace com_lib
{
//! Communication Implementation for TofCam635
/*!
 * This class implements the specific functionality for TofCam635 device.
 */
class Communication635: public Communication
{
public:
    Communication635();
    ErrorNumber_e setMode(const unsigned int mode);

    //signals
    Gallant::Signal1<std::shared_ptr<TofCam635Image>> sigReceivedDistance;
    Gallant::Signal1<std::shared_ptr<TofCam635Image>> sigReceivedGrayscale;    
    Gallant::Signal1<std::shared_ptr<TofCam635Image>> sigReceivedDistanceGrayscale;   
    Gallant::Signal1<std::shared_ptr<TofCam635Image>> sigReceivedDistanceAmplitude;    
    Gallant::Signal1<std::vector<uint8_t>>  sigReceivedLensCalibrationData;

private:

    unsigned int getBaudRate();    
    bool setupDevice(const Device_e device);
    void processDistanceAmplitude(const std::vector<uint8_t> &array);
    void processDistance(const std::vector<uint8_t> &array);
    void processGrayscale(const std::vector<uint8_t> &array);
    void processDistanceGrayscale(const std::vector<uint8_t> &array);
    void processLensCalibrationData(const std::vector<uint8_t> &array);

    void processTemporalHdr(std::shared_ptr<com_lib::TofCam635Image> &image);
    void patchImage(std::shared_ptr<com_lib::TofCam635Image> &imageTarget, std::shared_ptr<com_lib::TofCam635Image> imageCompare);
    bool allImagesFilled(const unsigned int numImage);

    static const unsigned int NUM_HDR_IMAGE = 3;
    std::shared_ptr<com_lib::TofCam635Image> hdrImage[NUM_HDR_IMAGE];
    std::shared_ptr<com_lib::TofCam635Image> resultImage;
    unsigned int indexIntegrationTime;    

};
}

#endif // COMMUNICATION_635_H

/** @} */

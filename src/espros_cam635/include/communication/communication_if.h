/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication Communication
 * @brief Communication to the camera
 * @defgroup communication_if Interface
 * @brief Communication Interface
 * @ingroup communication
 *
 * @{
 */
#ifndef COMMUNICATION_IF_H
#define COMMUNICATION_IF_H

#include <list>
#include <string>
#include <vector>

#include "cam_signal.h"

namespace com_lib
{

///Enum to define the modes
enum TofCam635Mode_e
{
  MODE_BEAM_A = 0,                                        ///<Normal operation with illumination beam A
  MODE_BEAM_B_MANUAL = 1,                                 ///<Normal operation with illumination beam B (all settings by user, same as)
  MODE_BEAM_B_RESULT = 2,                                 ///<Beam B with calibrated ROI, only one distance as result
  MODE_BEAM_B_RESULT_DATA = 3,                            ///<Beam B with calibrated ROI, one distance and the pixels as result
  MODE_BEAM_AB_RESULT = 4,                                ///<Beam A and B operating with calibrated ROI and only one distance as result
  MODE_BEAM_AB_AUTO_RESULT = 5,                           ///<Beam A and B with automatic selection
  MODE_BEAM_AB_INTERLEAVED_DATA = 6                       ///<Beam A and B interleaved output
};

///Enum to define the modulation frequencies
enum ModulationFrequency_e
{
  MODULATION_FREQUENCY_10MHZ = 0,
  MODULATION_FREQUENCY_20MHZ = 1
};

///Enum to define the modes
enum Mode_e
{
  MODE_TIM = 0,
  MODE_ULN = 1,
  MODE_UFS = 2
};

//These error numbers are given in the signal "error"
enum ErrorNumber_e
{
  ERROR_NUMMBER_NO_ERROR = 0,
  ERROR_NUMBER_TIMEOUT = 32768,
  ERROR_NUMBER_NOT_ACKNOWLEDGE = 32769,
  ERROR_NUMBER_INVALID_PARAMETER = 32770,
  ERROR_NUMBER_SERIAL_PORT_ERROR = 32771,
  ERROR_NUMBER_INVALID_DATA = 32772
};

//Device list. The device is read with the command "getIdentification"
enum Device_e
{
  DEVICE_UNKNOWN = 0,
  DEVICE_TOFRANGE611 = 1,
  DEVICE_TOFFRAME611 = 2,
  DEVICE_TOFCAM635 = 3,
  DEVICE_TOFCC635 = 4
};

//! Communication Interface
/*!
 * This abstract class is the interface for the communication. All access must be done using this
 * interface.
 */
/* --Remarks--
 *
 * Bit width
 * Where the number of bits is important, the stdint types are used. If the number of bits is not important, the
 * standard types are used, so they could be 32Bit or 64Bit without affect. In any case they are as big as in the
 * sensor module or bigger. Like this the compiler on the PC does not need to do unneeded masking (for example
 * for a uint32_t on a 64Bit machine).
 *
 * Blocking/NonBlocking
 * Settings commands are done blocking. This makes it easier to send a list of commands.
 * Acquisition commands are done non blocking
 */
class Communication_IF
{

  public:                                                                                                           //Command type
    Communication_IF(){}
    virtual ~Communication_IF(){}

    virtual bool open(std::string &portName) = 0;
    virtual void close() = 0;
	
	//General commands
    virtual ErrorNumber_e setPower(const bool enabled) = 0;                                                         //blocking command
	
	//Information commands
    virtual ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader) = 0;                              //blocking command
    virtual ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader, unsigned int &version) = 0;       //blocking command
    virtual ErrorNumber_e getChipInformation(uint16_t &chipId, uint16_t &waferId) = 0;                              //blocking command
    virtual ErrorNumber_e getFirmwareRelease(unsigned int &major, unsigned int &minor) = 0;                         //blocking command
    virtual std::string getDeviceName() = 0;                                                                        //blocking command
    virtual Device_e    getDevice() = 0;
    virtual ErrorNumber_e getProductionInfo(unsigned int &year, unsigned int &week) = 0;                            //blocking command
    	
	//Setup commands
    virtual ErrorNumber_e setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime) = 0;   //blocking command
    virtual ErrorNumber_e setIntegrationTimeGrayscale(const unsigned int integrationTime) = 0;                      //blocking command
    virtual ErrorNumber_e setModulationFrequency(const ModulationFrequency_e modulationFrequency) = 0;              //blocking command
    virtual ErrorNumber_e setMode(const unsigned int mode) = 0;                                                     //blocking command
    virtual ErrorNumber_e setFilter(const unsigned int threshold, const unsigned int factor) = 0;                   //blocking command
    virtual ErrorNumber_e setFilterSpot(const unsigned int threshold, const unsigned int factor) = 0;               //blocking command
    virtual ErrorNumber_e setDcsFilter(const bool enabled) = 0;                                    // --> no signal in case of success
    virtual ErrorNumber_e setGaussianFilter(const bool enabled) = 0;                               // --> no signal in case of success
    virtual ErrorNumber_e setCalibrationMode(const bool enabled) = 0;                                               //blocking command
    virtual ErrorNumber_e setOffset(const int offset) = 0;                                                                                 // --> no signal in case of success
    virtual ErrorNumber_e setMinimalAmplitude(const unsigned int index, const unsigned int amplitude) = 0;                                 // --> no signal in case of success
    virtual ErrorNumber_e setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax) = 0;  // --> no signal in case of success                                                                                      // --> no signal in case of success
    virtual ErrorNumber_e setBinning(const int binning) = 0;                                                                               // --> no signal in case of success
    virtual ErrorNumber_e setFrameRate(const unsigned int FrameTime) = 0;                                                                  // --> no signal in case of success
    virtual ErrorNumber_e setHdr(const uint hdr) = 0;
    virtual ErrorNumber_e setInterferenceDetection(const bool enabled, const bool useLastValue, const int limit) = 0; // --> no signal in case of success
    virtual ErrorNumber_e setIlluminationPower(const bool lowPower) = 0;

	
  //Update commands
    virtual void updateFirmware(const std::vector<uint8_t> &updateFile) = 0;                                                  // --> firmwareUpdateProgress

  //public slots
    //Information commands
    virtual void getTemperature() = 0;                                                                              // --> receivedTemperature

    //Acquisition commands
    virtual void getGrayscale(unsigned int acquisitionMode = 0, unsigned int opMode = 0, unsigned int hdr = 0) = 0;                       // --> receivedGrayscale
    virtual void getDistanceGrayscale(unsigned int acquisitionMode = 0, unsigned int opMode = 0, unsigned int hdr = 0) = 0;               // --> receivedDistanceGrayscale
    virtual void getDistanceAmplitude(unsigned int acquisitionMode = 0, unsigned int opMode = 0, unsigned int hdr = 0) = 0;               // --> receivedDistanceAmplitude
    virtual void getDistance(unsigned int acquisitionMode = 0, unsigned int opMode = 0, unsigned int hdr = 0) = 0;  // --> receivedDistance
    virtual void getDcsDistanceAmplitude() = 0;                                                                     // --> receivedDcsDistanceAmplitude
    virtual void getDcs() = 0;                                                                                      // --> receivedDcs
    virtual void getLensCalibrationData() = 0;                                                                      // --> receivedLensCalibrationData
    virtual void stopStream() = 0;
    virtual ErrorNumber_e getIntegrationTime3d(unsigned int &integrationTime) = 0;

  //signals    
    Gallant::Signal1<int16_t> sigReceivedTemperature;
    Gallant::Signal1<unsigned int> sigFirmwareUpdateProgress;
    Gallant::Signal1<ErrorNumber_e> sigError;

};
} //end namespace com_lib

#endif // COMMUNICATION_IF_H

/** @} */

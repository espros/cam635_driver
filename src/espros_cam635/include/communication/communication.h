/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication_implementation Base Implementation
 * @brief Abstract base implementation
 * @ingroup communication
 *
 * @{
 */
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "communication_if.h"
#include "serial_connection.h"
#include "update_controller.h"
#include "epc_timer.h"
#include <exception>
#include <list>
#include <string>
#include <iostream>

#include "cam_signal.h"



namespace com_lib
{

//! Communication Base Implementation
/*!
 * This class implements the communication common for different kinds of sensor/camera systems.
 * For concrete devices, an own class must be implemented with some functions containing device
 * specific tasks.
 */
class Communication  // : public Communication_IF
{

  public:
    Communication();
    virtual ~Communication();    
    bool open(std::string &portName);
    void close();

    //General commands
    ErrorNumber_e setPower(const bool enabled);

    //Information commands
    Device_e      getDevice();
    std::string   getDeviceName();
    ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader);
    ErrorNumber_e getIdentification(Device_e &device, bool &isBootloader, unsigned int &version);
    ErrorNumber_e getChipInformation(uint16_t &chipId, uint16_t &waferId);
    ErrorNumber_e getFirmwareRelease(unsigned int &major, unsigned int &minor);
    ErrorNumber_e getProductionInfo(unsigned int &year, unsigned int &week);

    //Setup commands
    ErrorNumber_e setIntegrationTime3d(const unsigned int index, const unsigned int integrationTime);
    ErrorNumber_e setIntegrationTimeGrayscale(const unsigned int integrationTime);
    ErrorNumber_e setModulationFrequency(const ModulationFrequency_e modulationFrequency);
    ErrorNumber_e setModulationChannel(const int  channel);
    ErrorNumber_e setMode(const unsigned int mode __attribute__((unused))){ return ERROR_NUMMBER_NO_ERROR; } 
    ErrorNumber_e setFilter(const unsigned int threshold, const unsigned int factor);
    ErrorNumber_e setFilterSpot(const unsigned int threshold, const unsigned int factor);
    ErrorNumber_e setDcsFilter(const bool enabled);
    ErrorNumber_e setGaussianFilter(const bool enabled);
    ErrorNumber_e setCalibrationMode(const bool enabled);   
    ErrorNumber_e setRoi(const unsigned int xMin, const unsigned int yMin, const unsigned int xMax, const unsigned int yMax);
    ErrorNumber_e setOffset(const int offset);
    ErrorNumber_e setMinimalAmplitude(const unsigned index, const unsigned int amplitude);
    ErrorNumber_e setBinning(const int binning);
    ErrorNumber_e setFrameRate(const unsigned int FrameTime);
    ErrorNumber_e setHdr(const unsigned int hdr);
    ErrorNumber_e setInterferenceDetection(const bool enabled, const bool useLastValue, const int limit);
    ErrorNumber_e setIlluminationPower(const bool lowPower);

    //Update commands 
    void updateFirmware(const std::vector<uint8_t> &updateFile);
    
    //Internal update commands--> these commands are not in the interface, because they are used only internally
    void sendCommandJumpToBootloader();
    void sendCommandFirmwareUpdateStart(const unsigned int fileSize);
    void sendCommandFirmwareUpdateWriteData(const uint8_t *dataToWrite, const uint32_t index, const unsigned int bytesToWrite);
    void sendCommandFirmwareUpdateFinished();


    //public slots:
    //Information commands
    void getTemperature();

    //Acquisition commands
    void getDistanceGrayscale(unsigned int acquisitionMode = 0, unsigned int opMode = 0, unsigned int hdr = 0);
    void getDistanceAmplitude(unsigned int acquisitionMode = 0, unsigned int opMode = 0, unsigned int hdr = 0);
    void getDistance(unsigned int acquisitionMode = 0, unsigned int opMode =0, unsigned int hdr = 0);
    void getDcsDistanceAmplitude();
    void getDcs();
    void getGrayscale(unsigned int acquisitionMode = 0, unsigned int opMode = 0, unsigned int hdr = 0);
    void getLensCalibrationData();    
    void stopStream();
    void startStream();
    ErrorNumber_e getIntegrationTime3d(unsigned int &integrationTime);

    //private signals:       
    Gallant::Signal1<ErrorNumber_e &> sigErrorInternal;
    Gallant::Signal1<uint32_t> sigReceivedIdentification;
    Gallant::Signal1<uint16_t> sigReceivedIntegrationTime;
    Gallant::Signal2<uint16_t, uint16_t> sigReceivedChipInformation;
    Gallant::Signal1<uint32_t> sigReceivedFirmwareRelease;
    Gallant::Signal2<uint8_t, uint8_t> sigReceivedProductionInfo;
    Gallant::Signal0<void> sigReceivedAnswer;
    Gallant::Signal0<void> sigReceivedAck;

    Gallant::Signal1<int16_t> sigReceivedTemperature; //TODO...
    Gallant::Signal1<unsigned int> sigFirmwareUpdateProgress;
    Gallant::Signal1<ErrorNumber_e> sigError;


    //private slots:
    void onReceivedData(std::vector<uint8_t> &array, uint8_t type);
    void onFirmwareUpdateProgress(const unsigned int progress);
    void onFirmwareUpdateFinished();
    void onTimeout();


  protected:
    ErrorNumber_e sendCommand(uint8_t *data, int size, bool streamMode = false);
    ErrorNumber_e sendCommandWithoutData(const uint8_t command, int size, bool streamMode = false);
    ErrorNumber_e sendCommandSingleByte(const uint8_t command, const uint8_t payload, int size = CommunicationConstants::Command::SIZE_PAYLOAD, bool streaming = false);
    ErrorNumber_e sendCommandUint16(const uint8_t command, const uint16_t payload);
    ErrorNumber_e sendCommandInt16(const uint8_t command, const int16_t payload);
    ErrorNumber_e sendCommand2xUint16(const uint8_t command, const uint16_t payload0, const uint16_t payload1);
    ErrorNumber_e sendCommand2xUint8(const uint8_t command, const uint8_t payload0, const uint8_t payload1);

    Device_e connectedDevice;            ///<Stores the device type that is connected
    int hdrMode;

  private:
    void sendErrorSignal(ErrorNumber_e errorNumber);
    bool openInternal(std::string &portName, Device_e &device, bool &isBootloader);
    void processIdentification(const std::vector<uint8_t> &array);
    void processChipInformation(const std::vector<uint8_t> &array);
    void processTemperature(const std::vector<uint8_t> &array);
    void processFirmwareRelease(const std::vector<uint8_t> &array);
    void processIntegrationTime(const std::vector<uint8_t> &array);
    void processProductionInfo(const std::vector<uint8_t>  &array);
    std::string createDeviceString(const Device_e device);

    //These functions must be implemented by device specific classes
    virtual bool setupDevice(const Device_e device) = 0;
    virtual void processDistanceAmplitude(const std::vector<uint8_t> &array) = 0;
    virtual void processDistance(const std::vector<uint8_t> &array) = 0;
    virtual void processDistanceGrayscale(const std::vector<uint8_t> &array) = 0;
    virtual void processGrayscale(const std::vector<uint8_t> &array) = 0;    
    virtual void processLensCalibrationData(const std::vector<uint8_t> &array) = 0;
    virtual unsigned int getBaudRate() = 0;

    enum CommunicationState_e
    {
      COMMUNICATION_STATE_UNCONNECTED,
      COMMUNICATION_STATE_NORMAL,
      COMMUNICATION_STATE_UPDATE
    };

    bool startStreamMode;
    SerialConnection *serialConnection;     ///<SerialConnection instance   
    EpcTimer *timeoutTimer;
    CommunicationState_e state;             ///<Defines the state of the communication
    UpdateController updateController;      ///<Update controller instance
    unsigned int timeout;                   ///<Stores the timeout time to use    

    unsigned int xMin_;
    unsigned int yMin_;
    unsigned int xMax_;
    unsigned int yMax_;


};
}

#endif // COMMUNICATION_H

/** @} */

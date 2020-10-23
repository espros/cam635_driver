
#include "camera635_driver.h"


using namespace com_lib;
using namespace std;

Settings *Camera635Driver::gSettings;

Camera635Driver::Camera635Driver(const ros::Publisher &imagePublisher1_, const ros::Publisher &imagePublisher2_,
                                 const ros::Publisher &imageHeaderPublisher_, const ros::Publisher &pointCloud2Publisher_, Settings &set_):
imagePublisher1(imagePublisher1_),
imagePublisher2(imagePublisher2_),
imageHeaderPublisher(imageHeaderPublisher_),
pointCloud2Publisher(pointCloud2Publisher_)
{    
    hdr_last = 0;
    leftX_last = 0;
    topY_last = 0;
    rightX_last = 0;
    bootomY_last = 0;

    frameSeq = 0;
    imageSize8 = 0;
    imageSize16_1 = 0;
    imageSize16_2 = 0;    
    gSettings = &set_;    
    gSettings->runVideo = false;
    gSettings->updateParam = false;
    lastSingleShot = false;
    lastStreaming = false;
    strFrameID = "sensor_frame";    

    initCommunication();

    communication.sigReceivedGrayscale.connect(boost::bind(&Camera635Driver::updateGrayscaleFrame, this, _1));
    communication.sigReceivedDistance.connect(boost::bind(&Camera635Driver::updateDistanceFrame, this, _1));
    communication.sigReceivedDistanceAmplitude.connect(boost::bind(&Camera635Driver::updateDistanceAmplitudeFrame, this, _1));
    communication.sigReceivedDistanceGrayscale.connect(boost::bind(&Camera635Driver::updateDistanceGrayscaleFrame, this, _1));    

    imageHeaderMsg.data.resize(42);

    gSettings->updateParam = true;        
    timeLast = ros::Time::now();

    setParameters();

    if(gSettings->startStream)
      gSettings->runVideo = true;

    initLensTransform(0.02, 160, 60, gSettings->lensCenterOffsetX, gSettings->lensCenterOffsetY);
    oldLensCenterOffsetX = gSettings->lensCenterOffsetX;
    oldLensCenterOffsetY = gSettings->lensCenterOffsetY;

}


Camera635Driver::~Camera635Driver()
{
    communication.close();
}

void Camera635Driver::update()
{
    if(gSettings->runVideo && !gSettings->updateParam){

        updateData(); //streaming

    }else if(gSettings->updateParam){
        setParameters(); //update parameters

        if(gSettings->triggerSingleShot && gSettings->triggerSingleShot != lastSingleShot)
            updateData(); //trigger single shot

        lastSingleShot = gSettings->triggerSingleShot;
    }
}


void Camera635Driver::setParameters()
{
    if(gSettings->updateParam)
    {
        gSettings->updateParam = false;
        ROS_INFO("update parameters");

        framePeriod = 1.0 / gSettings->frameRate;

        if(communication.getDevice() == Device_e::DEVICE_TOFCAM635){
            communication.setMode(gSettings->mode);
        }else{
            gSettings->mode = CommunicationConstants::ModeTofCam635::MODE_BEAM_A;
        }

        communication.setBinning(0);

        if(gSettings->automaticIntegrationTime){
            communication.setIntegrationTime3d(0xff, 0);
        }else{
            communication.setIntegrationTime3d(0, gSettings->integrationTimeATOF1);
            communication.setIntegrationTime3d(1, gSettings->integrationTimeATOF2);
            communication.setIntegrationTime3d(2, gSettings->integrationTimeATOF3);
            communication.setIntegrationTime3d(3, gSettings->integrationTimeATOF4);
            
            if(communication.getDevice() == Device_e::DEVICE_TOFCAM635){
	         communication.setIntegrationTime3d(4, gSettings->integrationTimeBTOF1);
             communication.setIntegrationTime3d(5, gSettings->integrationTimeBTOF2);
            }
        }

        communication.setIntegrationTimeGrayscale(gSettings->integrationTimeGray);
        communication.setInterferenceDetection(gSettings->useInterferenceDetection, gSettings->useInterferenceDetectionLastValue, gSettings->interferenceDetectionThreshold);

        communication.setMinimalAmplitude(0, gSettings->minAmplitude1);
        communication.setMinimalAmplitude(1, gSettings->minAmplitude2);
        communication.setMinimalAmplitude(2, gSettings->minAmplitude3);
        communication.setMinimalAmplitude(3, gSettings->minAmplitude4);
        
        if(communication.getDevice() == Device_e::DEVICE_TOFCAM635){
            communication.setMinimalAmplitude(4, gSettings->minAmplitude5);
        }
        
        communication.setOffset(gSettings->offsetDistance);                                        
        communication.setFilter(gSettings->kalmanThreshold, static_cast<uint>(gSettings->kalmanFactor*1000.0));
        
        if(communication.getDevice() == Device_e::DEVICE_TOFCAM635){            
            communication.setFilterSpot(gSettings->kalmanThresholdSpot, static_cast<uint>(gSettings->kalmanFactorSpot*1000.0));
            communication.setIlluminationPower(gSettings->lowPowerIllumination);            
        }
        
        communication.setModulationFrequency(MODULATION_FREQUENCY_20MHZ);

        communication.setRoi(gSettings->roi_leftX, gSettings->roi_topY, gSettings->roi_rightX, gSettings->roi_bottomY);         

        if(leftX_last != gSettings->roi_leftX || topY_last != gSettings->roi_topY || rightX_last != gSettings->roi_rightX || bootomY_last != gSettings->roi_bottomY)
        {
            leftX_last   = gSettings->roi_leftX;
            topY_last    = gSettings->roi_topY;
            rightX_last  = gSettings->roi_rightX;
            bootomY_last = gSettings->roi_bottomY;
            usleep(400000); //defined empyric
        }


        if(oldLensCenterOffsetX != gSettings->lensCenterOffsetX  || oldLensCenterOffsetY != gSettings->lensCenterOffsetY){
            initLensTransform(0.02, 160, 60, gSettings->lensCenterOffsetX, gSettings->lensCenterOffsetY); //0.02 - sensor pixel size mm
            oldLensCenterOffsetX = gSettings->lensCenterOffsetX;
            oldLensCenterOffsetY = gSettings->lensCenterOffsetY;
        }

        communication.setDcsFilter(gSettings->averageFilter);

        if((gSettings->hdr == HDR_SPATIAL) || (hdr_last == HDR_SPATIAL && gSettings->hdr != HDR_SPATIAL)){
            usleep(400000);
        }

        hdr_last = gSettings->hdr;


        communication.setHdr(gSettings->hdr);


    } //END if(gSettings->updateParam)

}


void Camera635Driver::updateData()
{      
    ros::Time timeNow = ros::Time::now();
    double elapsed_time = timeNow.toSec() - timeLast.toSec();

    if(elapsed_time >= framePeriod){

        timeLast = timeNow;

        ROS_DEBUG("FRAME RATE HZ: %2.4f", 1.0/elapsed_time);

        switch(gSettings->iType)
        {
            case TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_GRAYSCALE:
                  communication.getGrayscale(AUTO_REPEAT, gSettings->mode, gSettings->hdr);
                  break;
            case TofCam635Image::TofCam635ImageType_e::TOFCAM635IMAGE_DISTANCE:
                  communication.getDistance(AUTO_REPEAT, gSettings->mode, gSettings->hdr);
                  break;
            case TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_DISTANCE_AMPLITUDE:
                  communication.getDistanceAmplitude(AUTO_REPEAT, gSettings->mode, gSettings->hdr);
                  break;
            case TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_DISTANCE_GRAYSCALE:
                  communication.getDistanceGrayscale(AUTO_REPEAT, gSettings->mode, gSettings->hdr);
                  break;            
        }

    } //end if elapsed_time

}

void Camera635Driver::initCommunication(){

  communication.open(gSettings->port_name); //open serial port

  unsigned int minor, major;
  communication.getFirmwareRelease(major, minor);
  ROS_INFO("Firmware release:  major= %d  minor= %d", major, minor);

  uint16_t chipID, waferID;
  communication.getChipInformation(chipID, waferID);
  ROS_INFO("Chip ID= %d   Wafer ID= %d", chipID, waferID);

}


void Camera635Driver::publishImageHeader(std::shared_ptr<TofCam635Image> image)
{
    if(gSettings->enableImageHeader)
    {
        imageHeaderMsg.data.at(0)  = static_cast<int32_t>(image->getHeaderVersion());
        imageHeaderMsg.data.at(1)  = static_cast<int32_t>(image->getTimestamp());
        imageHeaderMsg.data.at(3)  = static_cast<int32_t>(image->getFirmwareVersion());
        imageHeaderMsg.data.at(4)  = static_cast<int32_t>(image->getHardwareVersion());
        imageHeaderMsg.data.at(5)  = static_cast<int32_t>(image->getChipID());
        imageHeaderMsg.data.at(6)  = static_cast<int32_t>(image->getWidth());
        imageHeaderMsg.data.at(7)  = static_cast<int32_t>(image->getHeight());
        imageHeaderMsg.data.at(8)  = static_cast<int32_t>(image->getOriginX());
        imageHeaderMsg.data.at(9)  = static_cast<int32_t>(image->getOriginY());
        imageHeaderMsg.data.at(10) = static_cast<int32_t>(image->getCurrentIntegrationTime3DWF());
        imageHeaderMsg.data.at(11) = static_cast<int32_t>(image->getCurrentIntegrationTime3DNF());
        imageHeaderMsg.data.at(12) = static_cast<int32_t>(image->getCurrentIntegrationTimeGrayscale());
        imageHeaderMsg.data.at(13) = static_cast<int32_t>(image->getIntegrationTimeGrayscale());
        imageHeaderMsg.data.at(14) = static_cast<int32_t>(image->getIntegrationTime3d(0));
        imageHeaderMsg.data.at(15) = static_cast<int32_t>(image->getIntegrationTime3d(1));
        imageHeaderMsg.data.at(16) = static_cast<int32_t>(image->getIntegrationTime3d(2));
        imageHeaderMsg.data.at(17) = static_cast<int32_t>(image->getIntegrationTime3d(3));
        imageHeaderMsg.data.at(18) = static_cast<int32_t>(image->getIntegrationTime3d(4));
        imageHeaderMsg.data.at(19) = static_cast<int32_t>(image->getIntegrationTime3d(5));
        imageHeaderMsg.data.at(20) = static_cast<int32_t>(image->getIntegrationTime3d(6));
        imageHeaderMsg.data.at(21) = static_cast<int32_t>(image->getIntegrationTime3d(7));
        imageHeaderMsg.data.at(22) = static_cast<int32_t>(image->getAmplitudeLimit(0));
        imageHeaderMsg.data.at(23) = static_cast<int32_t>(image->getAmplitudeLimit(1));
        imageHeaderMsg.data.at(24) = static_cast<int32_t>(image->getAmplitudeLimit(2));
        imageHeaderMsg.data.at(25) = static_cast<int32_t>(image->getAmplitudeLimit(3));
        imageHeaderMsg.data.at(26) = static_cast<int32_t>(image->getAmplitudeLimit(4));
        imageHeaderMsg.data.at(27) = static_cast<int32_t>(image->getOffset());
        imageHeaderMsg.data.at(28) = static_cast<int32_t>(image->getBinningType());
        imageHeaderMsg.data.at(29) = static_cast<int32_t>(image->getTemporalFilterDistance().getFactor());
        imageHeaderMsg.data.at(30) = static_cast<int32_t>(image->getTemporalFilterDistance().getThreshold());

        imageHeaderMsg.data.at(31) = static_cast<int32_t>(image->getTemporalFilterSingleValue().getFactor());
        imageHeaderMsg.data.at(32) = static_cast<int32_t>(image->getTemporalFilterSingleValue().getThreshold());
        imageHeaderMsg.data.at(33) = static_cast<int32_t>(image->getModulation().getFrequencyMhz());
        imageHeaderMsg.data.at(34) = static_cast<int32_t>(image->getModulation().getChannel());
        imageHeaderMsg.data.at(35) = static_cast<int32_t>(image->getHeaderFlags().getFlags());
        imageHeaderMsg.data.at(36) = static_cast<int32_t>(image->getTemperature());
        imageHeaderMsg.data.at(37) = static_cast<int32_t>(image->getBeamType());
        imageHeaderMsg.data.at(38) = static_cast<int32_t>(image->getSingleValueDistance());
        imageHeaderMsg.data.at(39) = static_cast<int32_t>(image->getSingleValueAmplitude());
        imageHeaderMsg.data.at(40) = static_cast<int32_t>(image->getSingleValue(0));
        imageHeaderMsg.data.at(41) = static_cast<int32_t>(image->getSingleValue(1));

        imageHeaderPublisher.publish(imageHeaderMsg);
    }

}


void Camera635Driver::updateGrayscaleFrame(std::shared_ptr<com_lib::TofCam635Image> image){

    publishImageHeader(image);

    if(gSettings->enableImages)
    {
        img8.header.seq = frameSeq++;
        img8.header.stamp = ros::Time::now();
        img8.header.frame_id = strFrameID;
        img8.height = static_cast<uint32_t>(image->getHeight());
        img8.width = static_cast<uint32_t>(image->getWidth());
        img8.encoding = sensor_msgs::image_encodings::MONO8;
        img8.step = img8.width;
        img8.is_bigendian = 0;
        uint numPix = static_cast<uint>(img8.width) * static_cast<uint>(img8.height);

        if(imageSize8 != numPix){
            imageSize8 = numPix;
            img8.data.resize(static_cast<unsigned long>(numPix));
        }


        for(uint i=0; i< numPix; i++ )
            img8.data[i] = static_cast<uint8_t>(image->getGrayscaleOfPixel(i));


        imagePublisher1.publish(img8);

    } //end if enableImages

}


void Camera635Driver::updateDistanceFrame(std::shared_ptr<com_lib::TofCam635Image> image)
{
    uint k, val;
    uint16_t x, y, i, l;

    publishImageHeader(image);    

    if(gSettings->enableImages)
    {
        img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = ros::Time::now();
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());        
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        uint numPix = img16_1.width * img16_1.height;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(static_cast<ulong>(numPix) * 2);
        }

        uint16_t value = 0;


        for(i=0, l=0; l< numPix; l++, i+=2){
            value = static_cast<uint16_t>(image->getDistanceOfPixel(l));
            img16_1.data[i] =  value & 0xff;
            img16_1.data[i+1] = (value>>8) & 0xff;
        }

        imagePublisher1.publish(img16_1);
    }


    if(gSettings->enablePointCloud)
    {
        static int sz_pc = 0;
        const uint nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;

        if(sz_pc != static_cast<int>(nPixel)){
            cloud->points.resize(static_cast<ulong>(nPixel));
            sz_pc = static_cast<int>(nPixel);
        }
                
        double px, pz, py;
        uint16_t leftX   = static_cast<uint16_t>(image->getOriginX());
        uint16_t topY    = static_cast<uint16_t>(image->getOriginY());
        uint16_t rightX  = static_cast<uint16_t>(image->getWidth()  + leftX);
        uint16_t bottomY = static_cast<uint16_t>(image->getHeight() + topY);


        for(k=0, y = topY; y < bottomY; y++){
            for(x = leftX; x < rightX; x++, k++){

                val = image->getDistanceOfPixel(k);
                pcl::PointXYZI &p = cloud->points[k];

                if(val < CommunicationConstants::PixelTofCam635::LIMIT_VALID_PIXEL){

                    if(gSettings->enableCartesian){
                        transformPixel(x, y, val, px, py, pz);
                        p.x = static_cast<float>(px / 1000.0); //mm -> m
                        p.y = static_cast<float>(py / 1000.0);
                        p.z = static_cast<float>(pz / 1000.0);
                        p.intensity = static_cast<float>(pz / 1000.0);
                    }else{

                        p.x = static_cast<float>(x / 1000.0);
                        p.y = static_cast<float>(y / 1000.0);
                        p.z = static_cast<float>(val / 1000.0);
                        p.intensity = static_cast<float>(val / 1000.0);
                    }

                }else{
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                    p.intensity = std::numeric_limits<float>::quiet_NaN();
                }

            } //ensd for x
        } //end for y

        pointCloud2Publisher.publish(cloud);

    } //end if enablePointCloud

}

void Camera635Driver::updateDistanceAmplitudeFrame(std::shared_ptr<TofCam635Image> image)
{
    uint32_t i;
    uint16_t val;
    uint l;
    int x,y;

    publishImageHeader(image);

    if(gSettings->enableImages){

        img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = ros::Time::now();
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        uint numPix = img16_1.width * img16_1.height;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(static_cast<ulong>(numPix) * 2);
        }


        for(i=0, l=0; l< static_cast<uint>(numPix); l++, i+=2 ){
            val = static_cast<uint16_t>(image->getAmplitudeOfPixel(l));
            img16_1.data[i] =  val & 0xff;
            img16_1.data[i+1] = (val>>8) & 0xff;
        }


        imagePublisher1.publish(img16_1);


        img16_2.header.seq = frameSeq++;
        img16_2.header.stamp = ros::Time::now();
        img16_2.header.frame_id = strFrameID;
        img16_2.height = static_cast<uint32_t>(image->getHeight());
        img16_2.width = static_cast<uint32_t>(image->getWidth());
        img16_2.encoding = sensor_msgs::image_encodings::MONO16;
        img16_2.step = img16_2.width * 2;
        img16_2.is_bigendian = 0;
        numPix = img16_2.width * img16_2.height;

        if(imageSize16_2 != numPix){
            imageSize16_2 = numPix;
            img16_2.data.resize(static_cast<ulong>(numPix) * 2);
        }



        for(i=0, l=0; l< static_cast<uint>(numPix); l++, i+=2 ){
            uint16_t val = static_cast<uint16_t>(image->getDistanceOfPixel(l));
            img16_2.data[i] =  val & 0xff;
            img16_2.data[i+1] = (val>>8) & 0xff;
        }

        imagePublisher2.publish(img16_2);
    }


    if(gSettings->enablePointCloud)
    {
        const uint nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;

        static uint szAmp= 0;
        if(szAmp != nPixel){
            szAmp = nPixel;
            cloud->points.resize(nPixel);
        }

        uint x, y, k;
        uint16_t val;        
        double px, pz, py;

        uint16_t leftX   = static_cast<uint16_t>(image->getOriginX());
        uint16_t topY    = static_cast<uint16_t>(image->getOriginY());
        uint16_t rightX  = static_cast<uint16_t>(image->getWidth()  + leftX);
        uint16_t bottomY = static_cast<uint16_t>(image->getHeight() + topY);

        for(k=0, y = topY; y < bottomY; y++){
            for(x = leftX; x < rightX; x++, k++){

                double dist = image->getDistanceOfPixel(k);
                float ampl = static_cast<float>(image->getAmplitudeOfPixel(k));
                pcl::PointXYZI &p = cloud->points[k];


                if(val < CommunicationConstants::PixelTofCam635::LIMIT_VALID_PIXEL && ampl>= gSettings->minAmplitude1){

                    if(gSettings->enableCartesian){
                        transformPixel(x, y, dist, px, py, pz);
                        p.x = static_cast<float>(px / 1000.0); //mm -> m
                        p.y = static_cast<float>(py / 1000.0);
                        p.z = static_cast<float>(pz / 1000.0);
                    }else{
                        p.x = static_cast<float>(x / 1000.0);
                        p.y = static_cast<float>(y / 1000.0);
                        p.z = static_cast<float>(val / 1000.0);
                    }

                    p.intensity = ampl;

                }else{
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                    p.intensity = std::numeric_limits<float>::quiet_NaN();
                }

            } //ensd for x
        } //end for y

        pointCloud2Publisher.publish(cloud);

      } //end if enablePointCloud

}


void Camera635Driver::updateDistanceGrayscaleFrame(std::shared_ptr<TofCam635Image> image)
{
    publishImageHeader(image);

    if(gSettings->enableImages)
    {
        img8.header.seq = frameSeq;
        img8.header.stamp = ros::Time::now();
        img8.header.frame_id = strFrameID; //std::to_string(0);
        img8.height = static_cast<uint32_t>(image->getHeight());
        img8.width = static_cast<uint32_t>(image->getWidth());
        img8.encoding = sensor_msgs::image_encodings::MONO8;
        img8.step = img8.width;
        img8.is_bigendian = 0;
        uint numPix = img8.width * img8.height;
        uint i, l;

        if(imageSize8 != numPix){
            imageSize8 = numPix;
            img8.data.resize(static_cast<ulong>(numPix));
        }

        for(l=0; l< numPix; l++)
            img8.data[l] = static_cast<uint8_t>(image->getGrayscaleOfPixel(l));

        imagePublisher1.publish(img8);

        img16_2.header.seq = frameSeq++;
        img16_2.header.stamp = ros::Time::now();
        img16_2.header.frame_id = strFrameID;  //std::to_string(0);
        img16_2.height = static_cast<uint32_t>(image->getHeight());
        img16_2.width = static_cast<uint32_t>(image->getWidth());
        img16_2.encoding = sensor_msgs::image_encodings::MONO16;
        img16_2.step = img16_2.width * 2; //f->px_size;
        img16_2.is_bigendian = 0;
        numPix = img16_2.width * img16_2.height;
        uint16_t val;

        if(imageSize16_2 != numPix){
            imageSize16_2 = numPix;
            img16_2.data.resize(static_cast<ulong>(numPix) * 2);
        }


        for(i=0, l=0; l< numPix; l++, i+=2){
            val = static_cast<uint16_t>(image->getDistanceOfPixel(l));
            img16_2.data[i] =  val & 0xff;
            img16_2.data[i+1] = (val>>8) & 0xff;
        }

        imagePublisher2.publish(img16_2);
    }

    if(gSettings->enablePointCloud)
    {
        const size_t nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;

        static int szAmp= 0;
        if(szAmp != static_cast<int>(nPixel)){
            szAmp = static_cast<int>(nPixel);
            cloud->points.resize(nPixel);
        }

        uint x,y,k;       
        uint8_t gray;
        double px, pz, py;

        uint16_t leftX   = static_cast<uint16_t>(image->getOriginX());
        uint16_t topY    = static_cast<uint16_t>(image->getOriginY());
        uint16_t rightX  = static_cast<uint16_t>(image->getWidth()  + leftX);
        uint16_t bottomY = static_cast<uint16_t>(image->getHeight() + topY);


        for(k=0, y = topY; y < bottomY; y++){
            for(x = leftX; x < rightX; x++, k++){

                double dist = image->getDistanceOfPixel(k);                
                gray = static_cast<uint8_t>(image->getGrayscaleOfPixel(k));
                pcl::PointXYZI &p = cloud->points[k];

                if(dist < CommunicationConstants::PixelTofCam635::LIMIT_VALID_PIXEL){

                    if(gSettings->enableCartesian){
                        transformPixel(x, y, dist, px, py, pz);
                        p.x = static_cast<float>(px / 1000.0);
                        p.y = static_cast<float>(py / 1000.0);
                        p.z = static_cast<float>(pz / 1000.0);
                    }else{
                        p.x = static_cast<float>(x / 1000.0);
                        p.y = static_cast<float>(y / 1000.0);
                        p.z = static_cast<float>(dist / 1000.0);
                    }

                    p.intensity = gray;

                }else {
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                    p.intensity = std::numeric_limits<float>::quiet_NaN();
                }

            } //ensd for x
        } //end for y

        pointCloud2Publisher.publish(cloud);

      } //end if enablePointCloud
}



void Camera635Driver::transformPixel(uint srcX, uint srcY, double srcZ, double &destX, double &destY, double &destZ)
{
    destX = srcZ * xUA[srcX][srcY];
    destY = srcZ * yUA[srcX][srcY];
    destZ = srcZ * zUA[srcX][srcY];
}

void Camera635Driver::initLensDistortionTable(bool isEpc635CC)
{
    distortionTableSize = 101;

    if(isEpc635CC){

        lensAngle[0] = 0.0;
        lensAngle[1] = 0.625;
        lensAngle[2] = 1.25;
        lensAngle[3] = 1.875;
        lensAngle[4] = 2.5;
        lensAngle[5] = 3.125;
        lensAngle[6] = 3.75;
        lensAngle[7] = 4.375;
        lensAngle[8] = 5;
        lensAngle[9] = 5.625;
        lensAngle[10] = 6.25;
        lensAngle[11] = 6.875;
        lensAngle[12] = 7.5;
        lensAngle[13] = 8.125;
        lensAngle[14] = 8.75;
        lensAngle[15] = 9.375;
        lensAngle[16] = 10;
        lensAngle[17] = 10.625;
        lensAngle[18] = 11.25;
        lensAngle[19] = 11.875;
        lensAngle[20] = 12.5;
        lensAngle[21] = 13.125;
        lensAngle[22] = 13.75;
        lensAngle[23] = 14.375;
        lensAngle[24] = 15;
        lensAngle[25] = 15.625;
        lensAngle[26] = 16.25;
        lensAngle[27] = 16.875;
        lensAngle[28] = 17.5;
        lensAngle[29] = 18.125;
        lensAngle[30] = 18.75;
        lensAngle[31] = 19.375;
        lensAngle[32] = 20;
        lensAngle[33] = 20.625;
        lensAngle[34] = 21.25;
        lensAngle[35] = 21.875;
        lensAngle[36] = 22.5;
        lensAngle[37] = 23.125;
        lensAngle[38] = 23.75;
        lensAngle[39] = 24.375;
        lensAngle[40] = 25;
        lensAngle[41] = 25.625;
        lensAngle[42] = 26.25;
        lensAngle[43] = 26.875;
        lensAngle[44] = 27.5;
        lensAngle[45] = 28.125;
        lensAngle[46] = 28.75;
        lensAngle[47] = 29.375;
        lensAngle[48] = 30;
        lensAngle[49] = 30.625;
        lensAngle[50] = 31.25;
        lensAngle[51] = 31.875;
        lensAngle[52] = 32.5;
        lensAngle[53] = 33.125;
        lensAngle[54] = 33.75;
        lensAngle[55] = 34.375;
        lensAngle[56] = 35;
        lensAngle[57] = 35.625;
        lensAngle[58] = 36.25;
        lensAngle[59] = 36.875;
        lensAngle[60] = 37.5;
        lensAngle[61] = 38.125;
        lensAngle[62] = 38.75;
        lensAngle[63] = 39.375;
        lensAngle[64] = 40;
        lensAngle[65] = 40.625;
        lensAngle[66] = 41.25;
        lensAngle[67] = 41.875;
        lensAngle[68] = 42.5;
        lensAngle[69] = 43.125;
        lensAngle[70] = 43.75;
        lensAngle[71] = 44.375;
        lensAngle[72] = 45;
        lensAngle[73] = 45.625;
        lensAngle[74] = 46.25;
        lensAngle[75] = 46.875;
        lensAngle[76] = 47.5;
        lensAngle[77] = 48.125;
        lensAngle[78] = 48.75;
        lensAngle[79] = 49.375;
        lensAngle[80] = 50;
        lensAngle[81] = 50.625;
        lensAngle[82] = 51.25;
        lensAngle[83] = 51.875;
        lensAngle[84] = 52.5;
        lensAngle[85] = 53.125;
        lensAngle[86] = 53.75;
        lensAngle[87] = 54.375;
        lensAngle[88] = 55;
        lensAngle[89] = 55.625;
        lensAngle[90] = 56.25;
        lensAngle[91] = 56.875;
        lensAngle[92] = 57.5;
        lensAngle[93] = 58.125;
        lensAngle[94] = 58.75;
        lensAngle[95] = 59.375;
        lensAngle[96] = 60;
        lensAngle[97] = 60.625;
        lensAngle[98] = 61.25;
        lensAngle[99] = 61.875;
        lensAngle[100] = 62.5;

        //new 850 nm
        /*rp[0] = 0;
        rp[1] = 0.01682277;
        rp[2] = 0.03364954;
        rp[3] = 0.05048433;
        rp[4] = 0.06733114;
        rp[5] = 0.08419401;
        rp[6] = 0.10107698;
        rp[7] = 0.1179841;
        rp[8] = 0.13491948;
        rp[9] = 0.1518872;
        rp[10] = 0.16889143;
        rp[11] = 0.18593634;
        rp[12] = 0.20302614;
        rp[13] = 0.22016511;
        rp[14] = 0.23735753;
        rp[15] = 0.25460779;
        rp[16] = 0.2719203;
        rp[17] = 0.28929954;
        rp[18] = 0.30675006;
        rp[19] = 0.32427647;
        rp[20] = 0.34188347;
        rp[21] = 0.35957584;
        rp[22] = 0.37735844;
        rp[23] = 0.39523623;
        rp[24] = 0.41321427;
        rp[25] = 0.43129772;
        rp[26] = 0.44949185;
        rp[27] = 0.46780204;
        rp[28] = 0.48623382;
        rp[29] = 0.50479283;
        rp[30] = 0.52348486;
        rp[31] = 0.54231583;
        rp[32] = 0.56129184;
        rp[33] = 0.58041914;
        rp[34] = 0.59970415;
        rp[35] = 0.61915348;
        rp[36] = 0.63877392;
        rp[37] = 0.65857248;
        rp[38] = 0.67855637;
        rp[39] = 0.69873303;
        rp[40] = 0.71911014;
        rp[41] = 0.73969561;
        rp[42] = 0.76049764;
        rp[43] = 0.7815247;
        rp[44] = 0.80278554;
        rp[45] = 0.82428922;
        rp[46] = 0.84604515;
        rp[47] = 0.86806305;
        rp[48] = 0.89035302;
        rp[49] = 0.91292554;
        rp[50] = 0.9357915;
        rp[51] = 0.9589622;
        rp[52] = 0.98244941;
        rp[53] = 1.00626535;
        rp[54] = 1.03042278;
        rp[55] = 1.05493495;
        rp[56] = 1.07981572;
        rp[57] = 1.10507951;
        rp[58] = 1.13074139;
        rp[59] = 1.15681709;
        rp[60] = 1.18332308;
        rp[61] = 1.21027656;
        rp[62] = 1.23769553;
        rp[63] = 1.26559885;
        rp[64] = 1.29400631;
        rp[65] = 1.32293862;
        rp[66] = 1.35241756;
        rp[67] = 1.38246598;
        rp[68] = 1.4131079;
        rp[69] = 1.44436861;
        rp[70] = 1.4762747;
        rp[71] = 1.5088542;
        rp[72] = 1.54213667;
        rp[73] = 1.57615328;
        rp[74] = 1.61093698;
        rp[75] = 1.64652256;
        rp[76] = 1.68294685;
        rp[77] = 1.72024884;
        rp[78] = 1.75846985;
        rp[79] = 1.79765369;
        rp[80] = 1.83784691;
        rp[81] = 1.87909896;
        rp[82] = 1.92146246;
        rp[83] = 1.96499344;
        rp[84] = 2.00975163;
        rp[85] = 2.05580079;
        rp[86] = 2.10320903;
        rp[87] = 2.15204922;
        rp[88] = 2.20239941;
        rp[89] = 2.25434325;
        rp[90] = 2.30797062;
        rp[91] = 2.3633781;
        rp[92] = 2.42066968;
        rp[93] = 2.47995749;
        rp[94] = 2.54136257;
        rp[95] = 2.60501584;
        rp[96] = 2.67105906;
        rp[97] = 2.73964605;
        rp[98] = 2.81094396;
        rp[99] = 2.88513477;
        rp[100] = 2.96241697;*/


        rp[0] = 0.0; //our old table
        rp[1] = 0.01682211;
        rp[2] = 0.03364430;
        rp[3] = 0.05046662;
        rp[4] = 0.06728914;
        rp[5] = 0.08411191;
        rp[6] = 0.10093495;
        rp[7] = 0.11775828;
        rp[8] = 0.13458188;
        rp[9] = 0.15140572;
        rp[10] = 0.16822973;
        rp[11] = 0.18505381;
        rp[12] = 0.20187782;
        rp[13] = 0.21870158;
        rp[14] = 0.23552489;
        rp[15] = 0.25234747;
        rp[16] = 0.26916903;
        rp[17] = 0.28598922;
        rp[18] = 0.30280766;
        rp[19] = 0.31962388;
        rp[20] = 0.33643741;
        rp[21] = 0.35324772;
        rp[22] = 0.37005421;
        rp[23] = 0.38685626;
        rp[24] = 0.40365318;
        rp[25] = 0.42044427;
        rp[26] = 0.43722875;
        rp[27] = 0.45400582;
        rp[28] = 0.47077463;
        rp[29] = 0.48753431;
        rp[30] = 0.50428394;
        rp[31] = 0.52102256;
        rp[32] = 0.53774921;
        rp[33] = 0.55446287;
        rp[34] = 0.57116254;
        rp[35] = 0.58784716;
        rp[36] = 0.60451568;
        rp[37] = 0.62116703;
        rp[38] = 0.63780014;
        rp[39] = 0.65441393;
        rp[40] = 0.67100732;
        rp[41] = 0.68757925;
        rp[42] = 0.70412865;
        rp[43] = 0.72065447;
        rp[44] = 0.73715569;
        rp[45] = 0.75363128;
        rp[46] = 0.77008028;
        rp[47] = 0.78650170;
        rp[48] = 0.80289464;
        rp[49] = 0.81925819;
        rp[50] = 0.83559150;
        rp[51] = 0.85189374;
        rp[52] = 0.86816414;
        rp[53] = 0.88440197;
        rp[54] = 0.90060653;
        rp[55] = 0.91677717;
        rp[56] = 0.93291330;
        rp[57] = 0.94901436;
        rp[58] = 0.96507983;
        rp[59] = 0.98110924;
        rp[60] = 0.99710217;
        rp[61] = 1.01305823;
        rp[62] = 1.02897708;
        rp[63] = 1.04485840;
        rp[64] = 1.0607019;
        rp[65] = 1.07650735;
        rp[66] = 1.09227451;
        rp[67] = 1.10800317;
        rp[68] = 1.12369315;
        rp[69] = 1.13934427;
        rp[70] = 1.15495637;
        rp[71] = 1.17052926;
        rp[72] = 1.18606278;
        rp[73] = 1.20155675;
        rp[74] = 1.21701098;
        rp[75] = 1.23242524;
        rp[76] = 1.24779929;
        rp[77] = 1.26313287;
        rp[78] = 1.27842567;
        rp[79] = 1.29367732;
        rp[80] = 1.30888744;
        rp[81] = 1.32405556;
        rp[82] = 1.33918117;
        rp[83] = 1.35426369;
        rp[84] = 1.36930246;
        rp[85] = 1.38429676;
        rp[86] = 1.39924574;
        rp[87] = 1.41414852;
        rp[88] = 1.42900406;
        rp[89] = 1.44381123;
        rp[90] = 1.45856880;
        rp[91] = 1.47327539;
        rp[92] = 1.48792947;
        rp[93] = 1.50252937;
        rp[94] = 1.51707324;
        rp[95] = 1.53155905;
        rp[96] = 1.54598457;
        rp[97] = 1.56034732;
        rp[98] = 1.57464460;
        rp[99] = 1.58887343;
        rp[100] = 1.60303051;

    }else{

        lensAngle[0]  = 0.0;
        lensAngle[1]  = 0.742;
        lensAngle[2]  = 1.483;
        lensAngle[3]  = 2.225;
        lensAngle[4]  = 2.967;
        lensAngle[5]  = 3.708;
        lensAngle[6]  = 4.45;
        lensAngle[7]  = 5.192;
        lensAngle[8]  = 5.933;
        lensAngle[9]  = 6.675;
        lensAngle[10] = 7.417;
        lensAngle[11] = 8.158;
        lensAngle[12] = 8.9;
        lensAngle[13] = 9.642;
        lensAngle[14] = 10.384;
        lensAngle[15] = 11.125;
        lensAngle[16] = 11.867;
        lensAngle[17] = 12.609;
        lensAngle[18] = 13.35;
        lensAngle[19] = 14.092;
        lensAngle[20] = 14.834;
        lensAngle[21] = 15.575;
        lensAngle[22] = 16.317;
        lensAngle[23] = 17.059;
        lensAngle[24] = 17.8;
        lensAngle[25] = 18.542;
        lensAngle[26] = 19.284;
        lensAngle[27] = 20.025;
        lensAngle[28] = 20.767;
        lensAngle[29] = 21.509;
        lensAngle[30] = 22.25;
        lensAngle[31] = 22.992;
        lensAngle[32] = 23.734;
        lensAngle[33] = 24.475;
        lensAngle[34] = 25.217;
        lensAngle[35] = 25.959;
        lensAngle[36] = 26.701;
        lensAngle[37] = 27.442;
        lensAngle[38] = 28.184;
        lensAngle[39] = 28.926;
        lensAngle[40] = 29.667;
        lensAngle[41] = 30.409;
        lensAngle[42] = 31.151;
        lensAngle[43] = 31.892;
        lensAngle[44] = 32.634;
        lensAngle[45] = 33.376;
        lensAngle[46] = 34.117;
        lensAngle[47] = 34.859;
        lensAngle[48] = 35.601;
        lensAngle[49] = 36.342;
        lensAngle[50] = 37.084;
        lensAngle[51] = 37.826;
        lensAngle[52] = 38.567;
        lensAngle[53] = 39.309;
        lensAngle[54] = 40.051;
        lensAngle[55] = 40.792;
        lensAngle[56] = 41.534;
        lensAngle[57] = 42.276;
        lensAngle[58] = 43.018;
        lensAngle[59] = 43.759;
        lensAngle[60] = 44.501;
        lensAngle[61] = 45.243;
        lensAngle[62] = 45.984;
        lensAngle[63] = 46.726;
        lensAngle[64] = 47.468;
        lensAngle[65] = 48.209;
        lensAngle[66] = 48.951;
        lensAngle[67] = 49.693;
        lensAngle[68] = 50.434;
        lensAngle[69] = 51.176;
        lensAngle[70] = 51.918;
        lensAngle[71] = 52.659;
        lensAngle[72] = 53.401;
        lensAngle[73] = 54.143;
        lensAngle[74] = 54.884;
        lensAngle[75] = 55.626;
        lensAngle[76] = 56.368;
        lensAngle[77] = 57.109;
        lensAngle[78] = 57.851;
        lensAngle[79] = 58.593;
        lensAngle[80] = 59.335;
        lensAngle[81] = 60.076;
        lensAngle[82] = 60.818;
        lensAngle[83] = 61.56;
        lensAngle[84] = 62.301;
        lensAngle[85] = 63.043;
        lensAngle[86] = 63.785;
        lensAngle[87] = 64.526;
        lensAngle[88] = 65.268;
        lensAngle[89] = 66.01;
        lensAngle[90] = 66.751;
        lensAngle[91] = 67.493;
        lensAngle[92] = 68.235;
        lensAngle[93] = 68.976;
        lensAngle[94] = 69.718;
        lensAngle[95] = 70.46;
        lensAngle[96] = 71.201;
        lensAngle[97] = 71.943;
        lensAngle[98] = 72.685;
        lensAngle[99] = 73.426;
        lensAngle[100] = 74.168;

        //size mm
        rp[0] = 0.0;
        rp[1] = 0.048;
        rp[2] = 0.095;
        rp[3] = 0.143;
        rp[4] = 0.19;
        rp[5] = 0.238;
        rp[6] = 0.286;
        rp[7] = 0.333;
        rp[8] = 0.381;
        rp[9] = 0.428;
        rp[10] = 0.476;
        rp[11] = 0.523;
        rp[12] = 0.571;
        rp[13] = 0.618;
        rp[14] = 0.665;
        rp[15] = 0.713;
        rp[16] = 0.76;
        rp[17] = 0.807;
        rp[18] = 0.854;
        rp[19] = 0.901;
        rp[20] = 0.948;
        rp[21] = 0.995;
        rp[22] = 1.042;
        rp[23] = 1.089;
        rp[24] = 1.135;
        rp[25] = 1.182;
        rp[26] = 1.228;
        rp[27] = 1.275;
        rp[28] = 1.321;
        rp[29] = 1.367;
        rp[30] = 1.413;
        rp[31] = 1.459;
        rp[32] = 1.505;
        rp[33] = 1.551;
        rp[34] = 1.596;
        rp[35] = 1.641;
        rp[36] = 1.687;
        rp[37] = 1.732;
        rp[38] = 1.777;
        rp[39] = 1.822;
        rp[40] = 1.866;
        rp[41] = 1.911;
        rp[42] = 1.955;
        rp[43] = 1.999;
        rp[44] = 2.043;
        rp[45] = 2.087;
        rp[46] = 2.13;
        rp[47] = 2.173;
        rp[48] = 2.216;
        rp[49] = 2.259;
        rp[50] = 2.302;
        rp[51] = 2.344;
        rp[52] = 2.386;
        rp[53] = 2.428;
        rp[54] = 2.47;
        rp[55] = 2.511;
        rp[56] = 2.552;
        rp[57] = 2.593;
        rp[58] = 2.634;
        rp[59] = 2.674;
        rp[60] = 2.714;
        rp[61] = 2.754;
        rp[62] = 2.793;
        rp[63] = 2.832;
        rp[64] = 2.871;
        rp[65] = 2.909;
        rp[66] = 2.948;
        rp[67] = 2.985;
        rp[68] = 3.023;
        rp[69] = 3.06;
        rp[70] = 3.096;
        rp[71] = 3.132;
        rp[72] = 3.168;
        rp[73] = 3.204;
        rp[74] = 3.239;
        rp[75] = 3.273;
        rp[76] = 3.308;
        rp[77] = 3.341;
        rp[78] = 3.375;
        rp[79] = 3.408;
        rp[80] = 3.44;
        rp[81] = 3.472;
        rp[82] = 3.504;
        rp[83] = 3.535;
        rp[84] = 3.565;
        rp[85] = 3.595;
        rp[86] = 3.625;
        rp[87] = 3.654;
        rp[88] = 3.682;
        rp[89] = 3.71;
        rp[90] = 3.738;
        rp[91] = 3.765;
        rp[92] = 3.791;
        rp[93] = 3.817;
        rp[94] = 3.842;
        rp[95] = 3.866;
        rp[96] = 3.89;
        rp[97] = 3.914;
        rp[98] = 3.936;
        rp[99] = 3.959;
        rp[100] = 3.98;
    }
}

double Camera635Driver::interpolate(double x_in, double x0, double y0, double x1, double y1){

    if(fabs(x1 - x0) < std::numeric_limits<double>::epsilon())  return y0;
    else return ((x_in-x0)*(y1-y0)/(x1-x0) + y0);
}

double Camera635Driver::getAngle(double x, double y, double sensorPointSizeMM)
{
    double radius = sensorPointSizeMM * sqrt(x*x + y*y);
    double alfaGrad = 0;

    for(int i=1; i<distortionTableSize; i++)
    {
        if(radius >= rp[i-1] && radius <= rp[i]){

            alfaGrad = interpolate(radius, rp[i-1], lensAngle[i-1], rp[i], lensAngle[i]);
        }
    }

    return alfaGrad;
}


void Camera635Driver::initLensTransform(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY)
{
    int x, y, row, col;
    numCols = width;
    numRows = height;

    initLensDistortionTable(communication.getDevice() == Device_e::DEVICE_TOFCC635);

    int r0 = 1 - numRows/2 + offsetY; //lens center offset
    int c0 = 1 - numCols/2 + offsetX;

    for(y=0, row = r0; y < numRows; row++, y++){
        for(x=0, col = c0; x < numCols; col++, x++){

            double c = col - 0.5;
            double r = row - 0.5;

            double angleGrad = getAngle(c, r, sensorPointSizeMM);
            double angleRad =  angleGrad * 3.14159265 / 180.0;

            double rp = sqrt(c * c + r * r);
            double rUA = sin(angleRad);

            xUA[x][y] = c * rUA / rp;
            yUA[x][y] = r * rUA / rp;
            zUA[x][y] = cos(angleRad);
        }
    }

}

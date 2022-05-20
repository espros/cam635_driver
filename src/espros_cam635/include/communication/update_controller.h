/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup update_controller Update Controller
 * @brief Helper calss for the update procedure
 * @ingroup communication
 *
 * @{
 */
#ifndef UPDATECONTROLLER_H
#define UPDATECONTROLLER_H

#include <cstdint>
#include <vector>
#include "cam_signal.h"

namespace com_lib
{
//! Helper class for the update procedure
/*!
 * This class controls the update procedure when a firmware update has to be loaded to the device.
 */
class UpdateController
{
  public:
    UpdateController(class Communication *commuication);
    void startUpdate(const std::vector<uint8_t> &updateFile);
    void cancel();

    //signals:
    Gallant::Signal1<unsigned int> sigUpdateProgress;
    Gallant::Signal0<void> sigUpdateFinished;

    //public slots:
    void onReceivedAck();

  private:
    enum UpdateState_e
    {
      UPDATE_STATE_IDLE,
      UPDATE_STATE_WAIT_BOOTLOADER,
      UPDATE_STATE_WRITE,
      UPDATE_STATE_FINISH,
      UPDATE_STATE_WAIT_FINISHED
    };

    //QByteArray updateFile;
    std::vector<uint8_t> updateFile;
    unsigned int index;
    unsigned int sizeRemaining;
    unsigned int sizeTotal;
    UpdateState_e state;
    class Communication *communication;
};
}

#endif // UPDATECONTROLLER_H

/** @} */

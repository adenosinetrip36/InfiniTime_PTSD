#include "components/ble/HeartRateService.h"
#include "components/heartrate/HeartRateController.h"
#include "components/ble/NimbleController.h"
#include <nrf_log.h>

using namespace Pinetime::Controllers;

constexpr ble_uuid16_t HeartRateService::heartRateServiceUuid;
constexpr ble_uuid16_t HeartRateService::heartRateMeasurementUuid;

uint8_t ptsdTrig = 0;

namespace {
  int HeartRateServiceCallback(uint16_t /*conn_handle*/, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    auto* heartRateService = static_cast<HeartRateService*>(arg);
    return heartRateService->OnHeartRateRequested(attr_handle, ctxt);
  }
}

// TODO Refactoring - remove dependency to SystemTask
HeartRateService::HeartRateService(NimbleController& nimble, Controllers::HeartRateController& heartRateController)
  : nimble {nimble},
    heartRateController {heartRateController},
    characteristicDefinition {{.uuid = &heartRateMeasurementUuid.u,
                               .access_cb = HeartRateServiceCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                               .val_handle = &heartRateMeasurementHandle},
                              {0}},
    serviceDefinition {
      {/* Device Information Service */
       .type = BLE_GATT_SVC_TYPE_PRIMARY,
       .uuid = &heartRateServiceUuid.u,
       .characteristics = characteristicDefinition},
      {0},
    } {
  // TODO refactor to prevent this loop dependency (service depends on controller and controller depends on service)
  heartRateController.SetService(this);
}

void HeartRateService::Init() {
  int res = 0;
  res = ble_gatts_count_cfg(serviceDefinition);
  ASSERT(res == 0);

  res = ble_gatts_add_svcs(serviceDefinition);
  ASSERT(res == 0);
}

int HeartRateService::OnHeartRateRequested(uint16_t attributeHandle, ble_gatt_access_ctxt* context) {
  if (attributeHandle == heartRateMeasurementHandle) {
    NRF_LOG_INFO("HEARTRATE : handle = %d", heartRateMeasurementHandle);
    uint8_t buffer[2] = {0, heartRateController.HeartRate()}; // [0] = flags, [1] = hr value

    int res = os_mbuf_append(context->om, buffer, 2);
    return (res == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  return 0;
}

void HeartRateService::OnNewHeartRateValue(uint8_t heartRateValue) {

  if (!heartRateMeasurementNotificationEnable)
    return;
/*------------------------------------------------------------*/
  //Array and for loop added for a test procedure of Dog 
  //  vibration device to simulate multiple activations of HR 
  //  protocol within a period of around 400 - 800 seconds
  //  or 6 - 12 minutes
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
//Original call of the ptsdTrigger function for the actual processing of HR data.
uint8_t ptsdTrig = ptsdTrigger(heartRateValue);
/*------------------------------------------------------------*/
  uint8_t PTSDbuffer[2] = {0, ptsdTrig}; // [0] = flags, [1] = hr value

  auto* pd = ble_hs_mbuf_from_flat(PTSDbuffer, 2);

  uint16_t connectionHandle = nimble.connHandle();

  if (connectionHandle == 0 || connectionHandle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }
  ble_gattc_notify_custom(connectionHandle, heartRateMeasurementHandle, pd);
}

void HeartRateService::SubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == heartRateMeasurementHandle)
    heartRateMeasurementNotificationEnable = true;
}

void HeartRateService::UnsubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == heartRateMeasurementHandle)
    heartRateMeasurementNotificationEnable = false;
}
/*######################################################################
|This algorithm was written by Curtis Daniel and Catriona Huber for    |                                                                   
| for the Binghamton University ECD302 PTSD Detection Project.         |
| The goal of this adaptation is to detect a PTSD episode within       |
| 2 minutes of the symptoms building within the user.  This is         |
| accomplished by taking the standard deviation of a moving average of |
| the last 7 heart rate values, typically obtained every 8 seconds.    |
| When the SD is above 6, the watch triggers the vibration device.     |
######################################################################*/

bool HeartRateService::ptsdTrigger(uint8_t heartRateValue){
  uint8_t lastSevenHR[7];
  uint16_t sum = 0, sum_temp = 0;
  float HRmean, standDev, sumSquares, trigger = 0;

  //**This portion of code creates a moving window of 7 heart rate values 
  // for calculations to be done on**

  for(uint8_t cnt = 0; cnt < 7; cnt++)
      lastSevenHR[cnt] = heartRateValue;

  //**This portion of code calculates the Standard Deviation of the data**

  if(lastSevenHR[6] != 0){
    for(uint8_t i = 0; i < 7; i++){
      sum += lastSevenHR[i];
    }
    HRmean = (float)sum / 7;

    for(uint8_t i = 0; i < 7; i++){
      sum_temp += pow(lastSevenHR[i] - HRmean, 2);
    }
    sumSquares = (float)sum_temp / 7;
    standDev = ceil(sqrt(sumSquares));

  //**This portion of code calcuates a weighted trigger value based on heart rate SD**
  
    if(standDev > 6)
      trigger = 0.5;
    else if(standDev > 3)
      trigger = 0.3;
    else
      trigger = trigger;
  }

  //**This portion of code calculates a weighted trigger value based on the heart rate, as you can see 
  // bradycardia and tachycardia carries less weight that heart rate variability**

  if ((heartRateValue < 70) || (heartRateValue > 101))
      trigger = 0.2;
      
  //**This portion of code logically determines whether a PTSD episode is indicated, and outputs the 
  // according bool value to the vibration device to receive**

  if(trigger >= 0.5)
    return 1;
  else
    return 0;
}

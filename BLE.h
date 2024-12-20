#ifndef _BLE_H
#define _BLE_H

BLEService BLEService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  //custom 128-bit UUID, read and writable by central

void initializeBLE() {
  //BLE initialization
  if (!BLE.begin()) Serial.println("starting Bluetooth® Low Energy module failed!");

  // set advertised local name and service UUID:
  BLE.setLocalName("Wei Quadcopter");
  BLE.setAdvertisedService(BLEService);

  // add the characteristic to the service
  BLEService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(BLEService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();
}

#endif /* BLE_H */
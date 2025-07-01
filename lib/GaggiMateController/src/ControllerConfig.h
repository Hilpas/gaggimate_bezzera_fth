#ifndef CONTROLLERCONFIG_H
#define CONTROLLERCONFIG_H
#include <string>

struct Capabilities {
    bool dimming;
    bool pressure;
    bool ssrPump;
};

struct ControllerConfig {
    std::string name;

    // The autodetect value that is measured through a PCB voltage divider.
    // The detected value in milli volts is divided by 100 and rounded.
    uint16_t autodetectValue;

    uint8_t heaterPin;
    uint8_t pumpPin;
    uint8_t pumpSensePin = 0;
    uint8_t pumpOn;
    uint8_t valvePin;
    uint8_t valveOn;
    uint8_t altPin;
    uint8_t altOn;

    uint8_t pressureScl = 0;
    uint8_t pressureSda = 0;

    uint8_t maxSckPin;
    uint8_t maxCsPin;
    uint8_t maxMisoPin;

    uint8_t brewButtonPin;
    uint8_t steamButtonPin;

    uint8_t scaleSclPin;
    uint8_t scaleSdaPin;
    uint8_t scaleSda1Pin;

    uint8_t ext1Pin;
    uint8_t ext2Pin;
    uint8_t ext3Pin;
    uint8_t ext4Pin;
    uint8_t ext5Pin;

    Capabilities capabilites;
};

// Base on original ControllerConfig
struct ControllerConfigBezzera {
    std::string name;

    // The autodetect value that is measured through a PCB voltage divider.
    // The detected value in milli volts is divided by 100 and rounded.
    uint16_t autodetectValue;

    uint8_t heaterPin;
    uint8_t pumpPin;
    uint8_t pumpSensePin = 0;
    uint8_t pumpOn;
    uint8_t valvePin;
    uint8_t valveOn;
    uint8_t altPin;
    uint8_t altOn;

    uint8_t pressureScl = 0;
    uint8_t pressureSda = 0;

    uint8_t maxSckPin;
    uint8_t maxCsPin;
    uint8_t maxMisoPin;

    uint8_t brewButtonPin;
    uint8_t steamButtonPin;

    uint8_t scaleSclPin;
    uint8_t scaleSdaPin;
    uint8_t scaleSda1Pin;

    uint8_t ext1Pin;
    uint8_t ext2Pin;
    uint8_t ext3Pin;
    uint8_t ext4Pin;
    uint8_t ext5Pin;

    uint8_t temperaturePin;

    Capabilities capabilites;
};

const ControllerConfig GM_STANDARD_REV_1X = {.name = "GaggiMate Standard Rev 1.x",
                                             .autodetectValue = 0, // Voltage divider was missing in Rev 1.0 so it's 0
                                             .heaterPin = 14,
                                             .pumpPin = 9,
                                             .pumpOn = 1,
                                             .valvePin = 10,
                                             .valveOn = 1,
                                             .altPin = 11,
                                             .altOn = 1,
                                             .maxSckPin = 6,
                                             .maxCsPin = 7,
                                             .maxMisoPin = 4,
                                             .brewButtonPin = 38,
                                             .steamButtonPin = 48,
                                             .scaleSclPin = 17,
                                             .scaleSdaPin = 18,
                                             .scaleSda1Pin = 39,
                                             .ext1Pin = 1,
                                             .ext2Pin = 2,
                                             .ext3Pin = 8,
                                             .ext4Pin = 12,
                                             .ext5Pin = 13,
                                             .capabilites = {
                                                 .dimming = false,
                                                 .pressure = false,
                                                 .ssrPump = false,
                                             }};

const ControllerConfig GM_STANDARD_REV_2X = {.name = "GaggiMate Standard Rev 2.x",
                                             .autodetectValue = 1, // Voltage divider was missing in Rev 1.0 so it's 0
                                             .heaterPin = 14,
                                             .pumpPin = 9,
                                             .pumpOn = 1,
                                             .valvePin = 10,
                                             .valveOn = 1,
                                             .altPin = 47,
                                             .altOn = 1,
                                             .maxSckPin = 6,
                                             .maxCsPin = 7,
                                             .maxMisoPin = 4,
                                             .brewButtonPin = 38,
                                             .steamButtonPin = 48,
                                             .scaleSclPin = 17,
                                             .scaleSdaPin = 18,
                                             .scaleSda1Pin = 39,
                                             .ext1Pin = 1,
                                             .ext2Pin = 2,
                                             .ext3Pin = 8,
                                             .ext4Pin = 12,
                                             .ext5Pin = 13,
                                             .capabilites = {
                                                 .dimming = false,
                                                 .pressure = false,
                                                 .ssrPump = true,
                                             }};

const ControllerConfig GM_PRO_REV_1x = {.name = "GaggiMate Pro Rev 1.x",
                                        .autodetectValue = 2, // Voltage divider was missing in Rev 1.0 so it's 0
                                        .heaterPin = 14,
                                        .pumpPin = 9,
                                        .pumpSensePin = 21,
                                        .pumpOn = 1,
                                        .valvePin = 10,
                                        .valveOn = 1,
                                        .altPin = 47,
                                        .altOn = 1,
                                        .pressureScl = 41,
                                        .pressureSda = 42,
                                        .maxSckPin = 6,
                                        .maxCsPin = 7,
                                        .maxMisoPin = 4,
                                        .brewButtonPin = 38,
                                        .steamButtonPin = 48,
                                        .scaleSclPin = 17,
                                        .scaleSdaPin = 18,
                                        .scaleSda1Pin = 39,
                                        .ext1Pin = 1,
                                        .ext2Pin = 2,
                                        .ext3Pin = 8,
                                        .ext4Pin = 12,
                                        .ext5Pin = 13,
                                        .capabilites = {
                                            .dimming = true,
                                            .pressure = true,
                                            .ssrPump = false,
                                        }};

const ControllerConfig GM_PRO_LEGO = {.name = "GaggiMate Pro Lego Build",
                                      .autodetectValue = 3,
                                      .heaterPin = 14,
                                      .pumpPin = 9,
                                      .pumpSensePin = 21,
                                      .pumpOn = 1,
                                      .valvePin = 10,
                                      .valveOn = 1,
                                      .altPin = 47,
                                      .altOn = 1,
                                      .pressureScl = 41,
                                      .pressureSda = 42,
                                      .maxSckPin = 6,
                                      .maxCsPin = 7,
                                      .maxMisoPin = 4,
                                      .brewButtonPin = 38,
                                      .steamButtonPin = 48,
                                      .scaleSclPin = 17,
                                      .scaleSdaPin = 18,
                                      .scaleSda1Pin = 39,
                                      .ext1Pin = 1,
                                      .ext2Pin = 2,
                                      .ext3Pin = 8,
                                      .ext4Pin = 12,
                                      .ext5Pin = 13,
                                      .capabilites = {
                                          .dimming = true,
                                          .pressure = true,
                                          .ssrPump = false,
                                      }};

                                      // PINs from Testbench Project
const ControllerConfigBezzera GM_BEZZERA = {.name = "GaggiMate Bezzera Build",
                                      .autodetectValue = 0,
                                      .heaterPin = 10,
                                      .pumpPin = 9,
                                      .pumpSensePin = 39,
                                      .pumpOn = 1,          // TODO
                                      .valvePin = 7,
                                      .valveOn = 1,         // TODO
                                      .altPin = 0,          // NC
                                      .altOn = 0,           // NC
                                      .pressureScl = 0,     // capability currently disabled // TODO Analog Sensor
                                      .pressureSda = 0,     // capability currently disabled // TODO Analog Sensor
                                      .maxSckPin = 0,       // not needed due to NTC
                                      .maxCsPin = 0,        // not needed due to NTC
                                      .maxMisoPin = 0,      // not needed due to NTC                                   
                                      .brewButtonPin = 15,
                                      .steamButtonPin = 0,  // TODO
                                      .scaleSclPin = 0,     // NC
                                      .scaleSdaPin = 0,     // NC
                                      .scaleSda1Pin = 0,    // NC
                                      .ext1Pin = 0,
                                      .ext2Pin = 0,
                                      .ext3Pin = 0,
                                      .ext4Pin = 0,
                                      .ext5Pin = 0,
                                      .temperaturePin = 4,  
                                      .capabilites = {
                                          .dimming = false,
                                          .pressure = false,
                                          .ssrPump = false,
                                      }};

#endif // CONTROLLERCONFIG_H

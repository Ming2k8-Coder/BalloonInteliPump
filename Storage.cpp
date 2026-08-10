#include "Storage.h"
#include <EEPROM.h>

// EEPROM Layout
// Slot Size: 
//   Magic (2) + 
//   SystemConfig + 
//   Total ~ 140 bytes. 
//   EEPROM_ROW_SIZE 150 is sufficient.
static_assert(sizeof(SystemConfig) <= (EEPROM_ROW_SIZE - 2), "SystemConfig too large for EEPROM slot!");

int findActiveSlot() {
    uint16_t magicTest;
    int bestSlot = -1;
    uint32_t bestVersion = 0;
    
    for (int i = 0; i < EEPROM_SLOTS; i++) {
        int addr = i * EEPROM_ROW_SIZE;
        EEPROM.get(addr, magicTest);
        if (magicTest == EEPROM_MAGIC_VAL) {
            SystemConfig cfg;
            EEPROM.get(addr + 2, cfg);
            if (bestSlot == -1 || cfg.version > bestVersion) {
                bestSlot = i;
                bestVersion = cfg.version;
            }
        }
    }
    return bestSlot;
}

void initStorage() {
    loadSystemConfig();
}

void saveSystemConfig() {
    SystemConfig cfg;
    
    // Gather Data
    cfg.pressureCal = pressureSensor.getCalibration();
    cfg.voltageCal = voltageSensor.getCalibration();
    cfg.currentCal = currentSensor.getCalibration();
    cfg.motorSafety = pumpMotor.getSafetySettings();
    
    // Calculate Slot
    int currentSlot = findActiveSlot();
    int targetSlot = 0;
    
    if (currentSlot >= 0) {
        SystemConfig oldCfg;
        EEPROM.get((currentSlot * EEPROM_ROW_SIZE) + 2, oldCfg);
        cfg.version = oldCfg.version + 1;
        targetSlot = (currentSlot + 1) % EEPROM_SLOTS;
    } else {
        cfg.version = 1;
    }
    
    int baseAddr = targetSlot * EEPROM_ROW_SIZE;
    int addr = baseAddr + 2; // Skip magic
    
    // Write Data
    EEPROM.put(addr, cfg);
    
    // Write New Magic
    uint16_t m = EEPROM_MAGIC_VAL;
    EEPROM.put(baseAddr, m);
    
    // Invalidate Old Magic
    if (currentSlot >= 0 && currentSlot != targetSlot) {
        int oldAddr = currentSlot * EEPROM_ROW_SIZE;
        uint16_t dead = 0x0000;
        EEPROM.put(oldAddr, dead);
    }
}

void loadSystemConfig() {
    int slot = findActiveSlot();
    
    if (slot < 0) {
        factoryReset();
        return;
    }
    
    int addr = (slot * EEPROM_ROW_SIZE) + 2;
    SystemConfig cfg;
    EEPROM.get(addr, cfg);
    
    // Distribute Data
    pressureSensor.setCalibration(cfg.pressureCal);
    voltageSensor.setCalibration(cfg.voltageCal);
    currentSensor.setCalibration(cfg.currentCal);
    pumpMotor.setSafetySettings(cfg.motorSafety);
}

void factoryReset() {
    // Defaults are handled by individual modules if Cal/Settings are empty/invalid.
    // But we can force sane defaults here too.
    
    // Sensors will check if(cal.numPoints==0) and load their own defaults.
    pressureSensor.resetCalibration();
    voltageSensor.resetCalibration();
    currentSensor.resetCalibration();
    
    // Motor Safety needs defaults if loaded is garbage, logic inside Motor::loadMotorConfig handled that.
    // Now we must handle it here or let Motor.cpp logic persist?
    // Let's set defaults manually here.
    SafetySettings s;
    s.maxCurrent = SAFETY_OC_DEFAULT;
    s.minVoltage = SAFETY_UV_DEFAULT;
    s.sagLimitPercent = SAFETY_SAG_DEFAULT;
    pumpMotor.setSafetySettings(s);
    
    // Force a save to initialize EEPROM
    pressureSensor.begin(); // Re-init to get defaults if we rely on init logic? 
    // Actually, initSensors() in .ino calls .begin(), which sets defaults if empty.
    // So if we just return here, initSensors will see empty cal and set defaults.
}

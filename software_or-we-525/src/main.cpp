/**
 * @file        main.cpp
 * @brief       Entry point and main program to read data from OR-WE-525 meter
 * @author      Copyright (C) Peter Ivanov, 2023
 *
 * Created      2023-12-22 20:30:53
 * Last modify: 2023-12-29 08:45:59 ivanovp {Time-stamp}
 * Licence:     GPL
 */

#include <Arduino.h>
#include "ModbusMaster.h"

#define LED_PIN                 PC13

/* Default baud rate: 9600 */
/* Available baud rates: 9600, 19200, 38400, 115200 */
#define RS485_BAUD_RATE         9600
#define RS485_DIR_PIN           PA5
#define MODBUS_SLAVE_ADDRESS    1   /* Meter ID of OR-WE-525 */

/* DO NOT CHANGE VALUE OF THE FOLLOWING MACROS! */
#define OR_WE_525_REG_VOLTAGE                       0x0100
#define OR_WE_525_REG_CURRENT                       0x0102
#define OR_WE_525_REG_FREQUENCY                     0x010A
#define OR_WE_525_REG_ACTIVE_POWER                  0x0104
#define OR_WE_525_REG_REACTIVE_POWER                0x0108
#define OR_WE_525_REG_APPARENT_POWER                0x0106
#define OR_WE_525_REG_POWER_FACTOR                  0x010B

#define OR_WE_525_REG_TOTAL_FORWARD_ACTIVE_ENERGY   0x010E
#define OR_WE_525_REG_TOTAL_REVERSE_ACTIVE_ENERGY   0x0118
#define OR_WE_525_REG_TOTAL_FORWARD_REACTIVE_ENERGY 0x012C
#define OR_WE_525_REG_TOTAL_REVERSE_REACTIVE_ENERGY 0x0136

#define OR_WE_525_REG_METER_SERIAL_NUMBER           0x1000
#define OR_WE_525_REG_METER_ID                      0x1003
#define OR_WE_525_REG_BAUD_RATE                     0x100C

#define OR_WE_525_REG_BAUD_RATE_9600                6
#define OR_WE_525_REG_BAUD_RATE_19200               7
#define OR_WE_525_REG_BAUD_RATE_38400               8
#define OR_WE_525_REG_BAUD_RATE_115200              9

#if RS485_BAUD_RATE == 9600
#define OR_WE_525_REG_BAUD_RATE_REG_CONTENT OR_WE_525_REG_BAUD_RATE_9600
#elif RS485_BAUD_RATE == 19200
#define OR_WE_525_REG_BAUD_RATE_REG_CONTENT OR_WE_525_REG_BAUD_RATE_19200
#elif RS485_BAUD_RATE == 38400
#define OR_WE_525_REG_BAUD_RATE_REG_CONTENT OR_WE_525_REG_BAUD_RATE_38400
#elif RS485_BAUD_RATE == 115200
#define OR_WE_525_REG_BAUD_RATE_REG_CONTENT OR_WE_525_REG_BAUD_RATE_115200
#else
#error Unsupported baud rate!
#endif

ModbusMaster ModbusMasterRS485;

//                         RX    TX
HardwareSerial SerialRS485(PA3, PA2);

/**
 * @brief Measured values from energy meter.
 *
 * @see or_we_525_read_values() function.
 */
typedef struct
{
    float voltage_V;
    float current_A;
    float freq_Hz;
    int32_t activePower_W;
    int32_t reactivePower_var;
    int32_t apparentPower_VA;
    float powerFactor;
    float totalForwardActiveEnergy_kWh;
    float totalReverseActiveEnergy_kWh;
    float totalForwardReactiveEnergy_kWh;
    float totalReverseReactiveEnergy_kWh;
} powerTotalEnergy_t;

powerTotalEnergy_t power;

uint8_t or_we_525_auto_detect_baud_rate();
uint8_t or_we_525_set_baud_rate(uint16_t a_baud_rate);

/* Set direction pin to TX before RS-485 transmission */
void preTransmission()
{
    digitalWrite(RS485_DIR_PIN, HIGH);
}

/* Set direction pin to RX after RS-485 transmission */
void postTransmission()
{
    digitalWrite(RS485_DIR_PIN, LOW);
}

void setup()
{
    /* Setup LED pin to output */
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    /* Setup direction pin to output */
    pinMode(RS485_DIR_PIN, OUTPUT);
    digitalWrite(RS485_DIR_PIN, HIGH);

    Serial.begin(115200);
    Serial.printf("\n\n\n");
    Serial.printf("Blue Pill OR-WE-525 reader started\n");
    Serial.printf("Compiled on " __DATE__ " " __TIME__ "\n");
    /* Configure RS-485 communication */
    SerialRS485.begin(RS485_BAUD_RATE, SERIAL_8N1);
    /* Set ModBus protocol over RS-485 */
    ModbusMasterRS485.begin(MODBUS_SLAVE_ADDRESS, SerialRS485);
    ModbusMasterRS485.preTransmission(preTransmission);
    ModbusMasterRS485.postTransmission(postTransmission);
    /* Uncomment the following function to auto detect baud rate of OR-WE-525 */
    // or_we_525_auto_detect_baud_rate();
    /* Uncomment following lines to change baud rate */
    // or_we_525_set_baud_rate(OR_WE_525_REG_BAUD_RATE_115200);
    // or_we_525_set_baud_rate(OR_WE_525_REG_BAUD_RATE_9600);
    // or_we_525_auto_detect_baud_rate();
    Serial.printf("Initialized\n");
    delay(500);
}


/* Convert modbus error code to human readable string */
const char * modbusErrorStr(uint8_t error)
{
    const char * errorStr = "unknown error";

    switch (error)
    {
    case ModbusMaster::ku8MBSuccess:
        errorStr = "success";
        break;

    case ModbusMaster::ku8MBInvalidCRC:
        errorStr = "invalid CRC";
        break;

    case ModbusMaster::ku8MBResponseTimedOut:
        errorStr = "response timeout";
        break;

    case ModbusMaster::ku8MBIllegalFunction:
        errorStr = "illegal function";
        break;

    case ModbusMaster::ku8MBIllegalDataAddress:
        errorStr = "illegal data address";
        break;

    case ModbusMaster::ku8MBIllegalDataValue:
        errorStr = "illegal data value";
        break;

    case ModbusMaster::ku8MBSlaveDeviceFailure:
        errorStr = "slave device failure";
        break;

    case ModbusMaster::ku8MBInvalidSlaveID:
        errorStr = "invalid slave ID";
        break;
    }

    return errorStr;
}


/**
 * @brief Read uint16 register via ModBus.
 *
 * @param a_reg_addr Address of register to read.
 * @param a_int16 Pointer to 16-bit variable to fill.
 * @return uint8_t Modbus error code. 0 if success.
 */
uint8_t modbus_read_int16(uint16_t a_reg_addr, int16_t * a_int16)
{
    uint8_t error = ModbusMaster::ku8MBSuccess;
    uint16_t buffer;

    error = ModbusMasterRS485.readHoldingRegisters(a_reg_addr, 1);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        buffer = ModbusMasterRS485.getResponseBuffer(0);
        *a_int16 = buffer;
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }

    return error;
}


/**
 * @brief Read uint32 register via ModBus.
 *
 * @param a_reg_addr Address of register to read.
 * @param a_int32 Pointer to 32-bit variable to fill.
 * @return uint8_t Modbus error code. 0 if success.
 */
uint8_t modbus_read_int32(uint16_t a_reg_addr, int32_t * a_int32)
{
    uint8_t error;
    uint32_t response32;
    uint16_t buffer;

    error = ModbusMasterRS485.readHoldingRegisters(a_reg_addr, 2);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        buffer = ModbusMasterRS485.getResponseBuffer(0);
        response32 = buffer;
        buffer = ModbusMasterRS485.getResponseBuffer(1);
        response32 <<= 16;
        response32 |= buffer;
        *a_int32 = response32;
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }

    return error;
}


/**
 * @brief Test OR-WE-525 by reading baud rate register and check its value.
 */
uint8_t or_we_525_test()
{
    uint8_t error;
    uint16_t buffer;

    Serial.printf("Trying to read baud rate register from OR-WE-525\n");
    error = ModbusMasterRS485.readHoldingRegisters(OR_WE_525_REG_BAUD_RATE, 1);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        buffer = ModbusMasterRS485.getResponseBuffer(0);
        if (buffer == OR_WE_525_REG_BAUD_RATE_REG_CONTENT)
        {
            Serial.printf("Baud rate is %i! Test passed!\n", buffer);
        }
        else
        {
            Serial.printf("Test failed! Buffer: %i\n", buffer);
            error = ModbusMaster::ku8MBSlaveDeviceFailure;
        }
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }

    return error;
}


/**
 * @brief Detect baud rate of OR-WE-525.
 */
uint8_t or_we_525_auto_detect_baud_rate()
{
    uint8_t error;
    uint16_t buffer;

    Serial.printf("Auto detecting baud rate of OR-WE-525\n");

    Serial.printf("Trying baud rate 9600...\n");
    SerialRS485.begin(9600, SERIAL_8N1);
    error = ModbusMasterRS485.readHoldingRegisters(OR_WE_525_REG_BAUD_RATE, 1);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        buffer = ModbusMasterRS485.getResponseBuffer(0);
        if (buffer == OR_WE_525_REG_BAUD_RATE_9600)
        {
            Serial.printf("Baud rate is 9600! Autodetect succeeded!\n", buffer);
        }
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }
    if (error != ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Trying baud rate 19200...\n");
        SerialRS485.begin(19200, SERIAL_8N1);
        error = ModbusMasterRS485.readHoldingRegisters(OR_WE_525_REG_BAUD_RATE, 1);
        if (error == ModbusMaster::ku8MBSuccess)
        {
            buffer = ModbusMasterRS485.getResponseBuffer(0);
            if (buffer == OR_WE_525_REG_BAUD_RATE_19200)
            {
                Serial.printf("Baud rate is 19200! Autodetect succeeded!\n", buffer);
            }
        }
        else
        {
            Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
        }
    }
    if (error != ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Trying baud rate 38400...\n");
        SerialRS485.begin(38400, SERIAL_8N1);
        error = ModbusMasterRS485.readHoldingRegisters(OR_WE_525_REG_BAUD_RATE, 1);
        if (error == ModbusMaster::ku8MBSuccess)
        {
            buffer = ModbusMasterRS485.getResponseBuffer(0);
            if (buffer == OR_WE_525_REG_BAUD_RATE_38400)
            {
                Serial.printf("Baud rate is 38400! Autodetect succeeded!\n", buffer);
            }
        }
        else
        {
            Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
        }
    }
    if (error != ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Trying baud rate 115200...\n");
        SerialRS485.begin(115200, SERIAL_8N1);
        error = ModbusMasterRS485.readHoldingRegisters(OR_WE_525_REG_BAUD_RATE, 1);
        if (error == ModbusMaster::ku8MBSuccess)
        {
            buffer = ModbusMasterRS485.getResponseBuffer(0);
            if (buffer == OR_WE_525_REG_BAUD_RATE_115200)
            {
                Serial.printf("Baud rate is 115200! Autodetect succeeded!\n", buffer);
            }
        }
        else
        {
            Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
        }
    }

    return error;
}


/**
 * @brief Set baud rate of OR-WE-525.
 */
uint8_t or_we_525_set_baud_rate(uint16_t a_baud_rate)
{
    uint8_t error;
    uint16_t buffer;

    Serial.printf("Setting baud rate...\n");
    error = ModbusMasterRS485.writeSingleRegister(OR_WE_525_REG_BAUD_RATE, a_baud_rate);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Done.\n");
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }

    return error;
}


/**
 * @brief Read registers from OR-WE-525 and process them.
 *
 * @param a_power Pointer of structure to fill.
 * @return uint8_t  Modbus error code. 0 if success.
 */
uint8_t or_we_525_read_values(powerTotalEnergy_t * a_power)
{
    uint8_t error = 0;
    int16_t reg16_value = 0;
    int32_t reg32_value = 0;

    error = modbus_read_int32(OR_WE_525_REG_VOLTAGE, &reg32_value);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->voltage_V = reg32_value * 0.001f;
        error = modbus_read_int32(OR_WE_525_REG_CURRENT, &reg32_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->current_A = reg32_value * 0.001f;
        error = modbus_read_int16(OR_WE_525_REG_FREQUENCY, &reg16_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->freq_Hz = reg16_value * 0.1f;
        error = modbus_read_int32(OR_WE_525_REG_ACTIVE_POWER, &(a_power->activePower_W));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_read_int32(OR_WE_525_REG_REACTIVE_POWER, &(a_power->reactivePower_var));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_read_int32(OR_WE_525_REG_APPARENT_POWER, &(a_power->apparentPower_VA));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_read_int16(OR_WE_525_REG_POWER_FACTOR, &reg16_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->powerFactor = reg16_value * 0.001f;
        error = modbus_read_int32(OR_WE_525_REG_TOTAL_FORWARD_ACTIVE_ENERGY, &reg32_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->totalForwardActiveEnergy_kWh = reg32_value * 0.01f;
        error = modbus_read_int32(OR_WE_525_REG_TOTAL_REVERSE_ACTIVE_ENERGY, &reg32_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->totalReverseActiveEnergy_kWh = reg32_value * 0.01f;
        error = modbus_read_int32(OR_WE_525_REG_TOTAL_FORWARD_REACTIVE_ENERGY, &reg32_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->totalForwardReactiveEnergy_kWh = reg32_value * 0.01f;
        error = modbus_read_int32(OR_WE_525_REG_TOTAL_REVERSE_REACTIVE_ENERGY, &reg32_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->totalReverseReactiveEnergy_kWh = reg32_value * 0.01f;
    }

    return error;
}

/**
 * @brief Print members of powerTotalEnergy structure to serial port.
 *
 * @param a_power Pointer of structure to print.
 */
void print_power(powerTotalEnergy_t * a_power)
{
    Serial.printf("Voltage: %.3f V\n", a_power->voltage_V);
    Serial.printf("Current: %.3f A\n", a_power->current_A);
    Serial.printf("Frequency: %.1f Hz\n", a_power->freq_Hz);
    Serial.printf("Active power: %d W\n", (unsigned int)a_power->activePower_W);
    Serial.printf("Reactive power: %d var\n", (unsigned int)a_power->reactivePower_var);
    Serial.printf("Power factor: %.3f\n", a_power->powerFactor);
    Serial.printf("Apparent power: %d VA\n", (unsigned int)a_power->apparentPower_VA);
    Serial.printf("Total forward active energy: %.2f kWh\n", a_power->totalForwardActiveEnergy_kWh);
    Serial.printf("Total reverse active energy: %.2f kWh\n", a_power->totalReverseActiveEnergy_kWh);
    Serial.printf("Total forward reactive energy: %.2f kWh\n", a_power->totalForwardReactiveEnergy_kWh);
    Serial.printf("Total reverse reactive energy: %.2f kWh\n", a_power->totalReverseReactiveEnergy_kWh);
}


void loop()
{
    Serial.printf("\n");
    Serial.printf("---------------------------------------------------------\n");
    // or_we_525_test();

    uint32_t start_timestamp = millis();
    uint8_t error = or_we_525_read_values(&power);
    uint32_t stop_timestamp = millis();

    Serial.printf("Time: %i ms\n", stop_timestamp - start_timestamp);

    if (error == ModbusMaster::ku8MBSuccess)
    {
        print_power(&power);
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }

    delay(5000);
}

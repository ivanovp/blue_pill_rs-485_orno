/**
 * @file        main.cpp
 * @brief       Entry point and main program to read data from OR-WE-516 meter
 * @author      Copyright (C) Peter Ivanov, 2023
 *
 * Created      2023-12-16 11:30:53
 * Last modify: 2023-12-22 20:36:42 ivanovp {Time-stamp}
 * Licence:     GPL
 */
#include <Arduino.h>
#include "ModbusMaster.h"

#define LED_PIN                 PC13

#define RS485_BAUD_RATE         9600
#define RS485_DIR_PIN           PA5
#define MODBUS_SLAVE_ADDRESS    1   /* Meter ID of OR-WE-516 */

/* DO NOT CHANGE THE VALUE OF FOLLOWING MACROS */
#define OR_WE_516_REG_METER_ID          0x0002
#define OR_WE_516_REG_BAUD_RATE         0x0003
#define OR_WE_516_REG_SOFTWARE_VERSION  0x0004
#define OR_WE_516_REG_HARDWARE_VERSION  0x0006
#define OR_WE_516_REG_CT_RATE           0x0008
#define OR_WE_516_REG_S0_OUTPUT_RATE    0x0009
#define OR_WE_516_REG_A3                0x000B
#define OR_WE_516_REG_UNKNOWN           0x000C
#define OR_WE_516_REG_CYCLE_TIME        0x000D

ModbusMaster ModbusMasterRS485;

//                         RX    TX
HardwareSerial SerialRS485(PA3, PA2);

/* Created using OR-WE-516_MODBUS_Registers_List.pdf */
/* First register's address is 0xE */
/* NOTE: DON'T CHANGE ORDER! */
typedef struct
{
    float L1_V;                         /* Register 0x000E */
    float L2_V;                         /* Register 0x0010 */
    float L3_V;                         /* Register 0x0012 */
    float freq_Hz;                      /* Register 0x0014 */
    float L1_A;                         /* Register 0x0016 */
    float L2_A;                         /* Register 0x0018 */
    float L3_A;                         /* Register 0x001A */
    float totalActivePower;             /* Register 0x001C */
    float L1ActivePower;                /* Register 0x001E */
    float L2ActivePower;                /* Register 0x0020 */
    float L3ActivePower;                /* Register 0x0022 */
    float totalReactivePower;           /* Register 0x0024 */
    float L1ReactivePower;              /* Register 0x0026 */
    float L2ReactivePower;              /* Register 0x0028 */
    float L3ReactivePower;              /* Register 0x002A */
    float totalApparentPower;           /* Register 0x002C */
    float L1ApparentPower;              /* Register 0x002E */
    float L2ApparentPower;              /* Register 0x0030 */
    float L3ApparentPower;              /* Register 0x0032 */
    float totalPowerFactor;             /* Register 0x0034 */
    float L1PowerFactor;                /* Register 0x0036 */
    float L2PowerFactor;                /* Register 0x0038 */
    float L3PowerFactor;                /* Register 0x003A */
} power_t;

typedef struct
{
    float totalActiveEnergy;            /* Register 0x0100 */
    float L1TotalActiveEnergy;          /* Register 0x0102 */
    float L2TotalActiveEnergy;          /* Register 0x0104 */
    float L3TotalActiveEnergy;          /* Register 0x0106 */
    float totalForwardActiveEnergy;     /* Register 0x0108 */
    float L1ForwardActiveEnergy;        /* Register 0x010a */
    float L2ForwardActiveEnergy;        /* Register 0x010c */
    float L3ForwardActiveEnergy;        /* Register 0x010E */
    float totalReverseActiveEnergy;     /* Register 0x0110 */
    float L1ReverseActiveEnergy;        /* Register 0x0112 */
    float L2ReverseActiveEnergy;        /* Register 0x0114 */
    float L3ReverseActiveEnergy;        /* Register 0x0116 */
    float totalReactiveEnergy;          /* Register 0x0118 */
    float L1ReactiveEnergy;             /* Register 0x011A */
    float L2ReactiveEnergy;             /* Register 0x011C */
    float L3ReactiveEnergy;             /* Register 0x011E */
    float totalForwardReactiveEnergy;   /* Register 0x0120 */
    float L1ForwardReactiveEnergy;      /* Register 0x0122 */
    float L2ForwardReactiveEnergy;      /* Register 0x0124 */
    float L3ForwardReactiveEnergy;      /* Register 0x0126 */
    float totalReverseReactiveEnergy;   /* Register 0x0128 */
    float L1ReverseReactiveEnergy;      /* Register 0x012A */
    float L2ReverseReactiveEnergy;      /* Register 0x012C */
    float L3ReverseReactiveEnergy;      /* Register 0x012E */
} totalEnergy_t;

power_t power;
totalEnergy_t totalEnergy;

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
    Serial.printf("Blue Pill OR-WE-516 reader started\n");
    Serial.printf("Compiled on " __DATE__ " " __TIME__ "\n");
    /* Configure RS-485 communication */
    SerialRS485.begin(RS485_BAUD_RATE, SERIAL_8E1);
    /* Set ModBus protocol over RS-485 */
    ModbusMasterRS485.begin(MODBUS_SLAVE_ADDRESS, SerialRS485);
    ModbusMasterRS485.preTransmission(preTransmission);
    ModbusMasterRS485.postTransmission(postTransmission);
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
 * @param a_uint16 Pointer to 16-bit variable to fill.
 * @return uint8_t Modbus error code. 0 if success.
 */
uint8_t modbus_read_uint16(uint16_t a_reg_addr, uint16_t * a_uint16)
{
    uint8_t error;
    uint16_t buffer;
    error = ModbusMasterRS485.readHoldingRegisters(a_reg_addr, 1);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        buffer = ModbusMasterRS485.getResponseBuffer(0);
        *a_uint16 = buffer;
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }

    return error;
}

/**
 * @brief Read float register via ModBus.
 *
 * @param a_reg_addr Address of register to read.
 * @param a_float Pointer to float variable to fill.
 * @return uint8_t Modbus error code. 0 if success.
 */
uint8_t modbus_read_float(uint16_t a_reg_addr, float * a_float)
{
    uint8_t error;
    uint16_t buffer;
    uint32_t response32;
    float * response = reinterpret_cast<float*>(&response32);
    error = ModbusMasterRS485.readHoldingRegisters(a_reg_addr, 2);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        buffer = ModbusMasterRS485.getResponseBuffer(0);
        response32 = buffer;
        buffer = ModbusMasterRS485.getResponseBuffer(1);
        response32 <<= 16;
        response32 |= buffer;
        // Serial.printf("Response raw: 0x%08X\n", response32);
        // Serial.printf("Response float: %f\n", *response);
        *a_float = *response;
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }

    return error;
}

/**
 * @brief Fill structure or array of float values.
 *
 * @param a_first_reg_addr Address of first register to read.
 * @param a_float_struct Pointer to array/structure of float values.
 * @param a_reg_count Number of registers to read.
 * @return uint8_t Modbus error code. 0 if success.
 */
uint8_t modbus_read_float_struct(uint16_t a_first_reg_addr, float * a_float_struct, uint16_t a_reg_count)
{
    uint8_t error = 0;
    uint16_t i;
    uint8_t retry_cnt;

    for (i = 0; i < a_reg_count && error == ModbusMaster::ku8MBSuccess; i++)
    {
        retry_cnt = 3;
        do
        {
            error = modbus_read_float(a_first_reg_addr + i * 2, &a_float_struct[i]);
        } while (error && retry_cnt--);
    }

    return error;
}

/**
 * @brief Test OR-WE-516 by reading baud rate register and check its value.
 */
void or_we_516_test()
{
    uint8_t error;
    uint16_t buffer;

    Serial.printf("Trying to read baud rate register from OR-WE-516\n");
    error = ModbusMasterRS485.readHoldingRegisters(OR_WE_516_REG_BAUD_RATE, 1);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        buffer = ModbusMasterRS485.getResponseBuffer(0);
        if (buffer == RS485_BAUD_RATE)
        {
            Serial.printf("Baud rate is %i! Test passed!\n", buffer);
        }
        else
        {
            Serial.printf("Test failed! Buffer: %i\n", buffer);
        }
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }
}

/**
 * @brief Retrieve and print miscellaneous values from OR-WE-516.
 */
void print_misc()
{
    uint8_t error;
    float software_version;
    float hardware_version;
    uint16_t ct_rate;
    float s0_output_rate;
    uint16_t a3;
    uint16_t cycle_time;
    uint16_t unknown;

    error = modbus_read_float(OR_WE_516_REG_SOFTWARE_VERSION, &software_version);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Software version: %.2f\n", software_version);
    }
    error = modbus_read_float(OR_WE_516_REG_HARDWARE_VERSION, &hardware_version);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Hardware version: %.2f\n", hardware_version);
    }
    error = modbus_read_uint16(OR_WE_516_REG_CT_RATE, &ct_rate);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("CT rate: %i\n", ct_rate);
    }
    error = modbus_read_float(OR_WE_516_REG_S0_OUTPUT_RATE, &s0_output_rate);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("S0 output rate: %.2f\n", s0_output_rate);
    }
    error = modbus_read_uint16(OR_WE_516_REG_A3, &a3);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("A3: %i\n", a3);
    }
    error = modbus_read_uint16(OR_WE_516_REG_UNKNOWN, &unknown);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Unknown: %i\n", unknown);
    }
    error = modbus_read_uint16(OR_WE_516_REG_CYCLE_TIME, &cycle_time);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Cycle time: %i\n", cycle_time);
    }
    Serial.printf("\n");
}

/**
 * @brief Retrieve actual values of voltage, current, power and print.
 */
void print_power()
{
    uint8_t error;

    Serial.printf("Getting power struct\n");
    uint32_t start_timestamp = millis();
    error = modbus_read_float_struct(0xe,
                                     reinterpret_cast<float*>(&power),
                                     sizeof(power) / sizeof(float));
    uint32_t stop_timestamp = millis();

    Serial.printf("Time: %i ms\n", stop_timestamp - start_timestamp);

    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Frequency: %.2f Hz\n", power.freq_Hz);
        Serial.printf("L1 voltage: %.2f V\n", power.L1_V);
        Serial.printf("L2 voltage: %.2f V\n", power.L2_V);
        Serial.printf("L3 voltage: %.2f V\n", power.L3_V);
        Serial.printf("L1 current: %.2f A\n", power.L1_A);
        Serial.printf("L2 current: %.2f A\n", power.L2_A);
        Serial.printf("L3 current: %.2f A\n", power.L3_A);
        Serial.printf("Total active power: %.2f kW\n", power.totalActivePower);
        Serial.printf("L1 active power: %.2f kW\n", power.L1ActivePower);
        Serial.printf("L2 active power: %.2f kW\n", power.L2ActivePower);
        Serial.printf("L3 active power: %.2f kW\n", power.L3ActivePower);
        Serial.printf("Total reactive power: %.2f kvar\n", power.totalReactivePower);
        Serial.printf("L1 reactive power: %.2f kvar\n", power.L1ReactivePower);
        Serial.printf("L2 reactive power: %.2f kvar\n", power.L2ReactivePower);
        Serial.printf("L3 reactive power: %.2f kvar\n", power.L3ReactivePower);
        Serial.printf("Total apparent power: %.2f kVA\n", power.totalApparentPower);
        Serial.printf("L1 apparent power: %.2f kVA\n", power.L1ApparentPower);
        Serial.printf("L2 apparent power: %.2f kVA\n", power.L2ApparentPower);
        Serial.printf("L3 apparent power: %.2f kVA\n", power.L3ApparentPower);
        Serial.printf("Total power factor: %.2f\n", power.totalPowerFactor);
        Serial.printf("L1 power factor: %.2f\n", power.L1PowerFactor);
        Serial.printf("L2 power factor: %.2f\n", power.L2PowerFactor);
        Serial.printf("L3 power factor: %.2f\n", power.L3PowerFactor);
    }
}

/**
 * @brief Retrieve energy values and print.
 */
void print_total_energy()
{
    uint8_t error;

    Serial.printf("\nGetting totalEnergy struct\n");
    uint32_t start_timestamp = millis();
    error = modbus_read_float_struct(0x100,
                                     reinterpret_cast<float*>(&totalEnergy),
                                     sizeof(totalEnergy) / sizeof(float));
    uint32_t stop_timestamp = millis();

    Serial.printf("Time: %i ms\n", stop_timestamp - start_timestamp);

    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Total active energy: %.2f kWh\n", totalEnergy.totalActiveEnergy);
        Serial.printf("L1 active energy: %.2f kWh\n", totalEnergy.L1TotalActiveEnergy);
        Serial.printf("L2 active energy: %.2f kWh\n", totalEnergy.L2TotalActiveEnergy);
        Serial.printf("L3 active energy: %.2f kWh\n", totalEnergy.L3TotalActiveEnergy);
        Serial.printf("Total forward active energy: %.2f kWh\n", totalEnergy.totalForwardActiveEnergy);
        Serial.printf("L1 forward active energy: %.2f kWh\n", totalEnergy.L1ForwardActiveEnergy);
        Serial.printf("L2 forward active energy: %.2f kWh\n", totalEnergy.L2ForwardActiveEnergy);
        Serial.printf("L3 forward active energy: %.2f kWh\n", totalEnergy.L3ForwardActiveEnergy);
        Serial.printf("Total reverse active energy: %.2f kWh\n", totalEnergy.totalReverseActiveEnergy);
        Serial.printf("L1 reverse active energy: %.2f kWh\n", totalEnergy.L1ReverseActiveEnergy);
        Serial.printf("L2 reverse active energy: %.2f kWh\n", totalEnergy.L2ReverseActiveEnergy);
        Serial.printf("L3 reverse active energy: %.2f kWh\n", totalEnergy.L3ReverseActiveEnergy);
        Serial.printf("Total reactive energy: %.2f kvarh\n", totalEnergy.totalReactiveEnergy);
        Serial.printf("L1 reactive energy: %.2f kvarh\n", totalEnergy.L1ReactiveEnergy);
        Serial.printf("L2 reactive energy: %.2f kvarh\n", totalEnergy.L2ReactiveEnergy);
        Serial.printf("L3 reactive energy: %.2f kvarh\n", totalEnergy.L3ReactiveEnergy);
        Serial.printf("Total forward reactive energy: %.2f kvarh\n", totalEnergy.totalForwardReactiveEnergy);
        Serial.printf("L1 forward reactive energy: %.2f kvarh\n", totalEnergy.L1ForwardReactiveEnergy);
        Serial.printf("L2 forward reactive energy: %.2f kvarh\n", totalEnergy.L2ForwardReactiveEnergy);
        Serial.printf("L3 forward reactive energy: %.2f kvarh\n", totalEnergy.L3ForwardReactiveEnergy);
        Serial.printf("Total reverse reactive energy: %.2f kvarh\n", totalEnergy.totalReverseReactiveEnergy);
        Serial.printf("L1 reverse reactive energy: %.2f kvarh\n", totalEnergy.L1ReverseReactiveEnergy);
        Serial.printf("L2 reverse reactive energy: %.2f kvarh\n", totalEnergy.L2ReverseReactiveEnergy);
        Serial.printf("L3 reverse reactive energy: %.2f kvarh\n", totalEnergy.L3ReverseReactiveEnergy);
    }
}

void loop()
{
    Serial.printf("\n");
    Serial.printf("---------------------------------------------------------\n");
    // or_we_516_test();
    print_misc();
    print_power();
    print_total_energy();
    delay(5000);
}

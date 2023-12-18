#include <Arduino.h>
#include "ModbusMaster.h"

#define LED_PIN                 PC13

#define RS485_BAUD_RATE         9600
#define RS485_DIR_PIN           PA5
#define MODBUS_SLAVE_ADDRESS    1   /* Meter ID of OR-WE-504 */

#define OR_WE_504_READ_DELAY_MS 12  /* Empiric value, necessary delay between register reads */

/* DO NOT CHANGE VALUE OF THE FOLLOWING MACROS! */
#define OR_WE_504_REG_VOLTAGE           0x0000
#define OR_WE_504_REG_CURRENT           0x0001
#define OR_WE_504_REG_FREQUENCY         0x0002
#define OR_WE_504_REG_ACTIVE_POWER      0x0003
#define OR_WE_504_REG_REACTIVE_POWER    0x0004
#define OR_WE_504_REG_APPARENT_POWER    0x0005
#define OR_WE_504_REG_POWER_FACTOR      0x0006
#define OR_WE_504_REG_ACTIVE_ENERGY     0x0007
#define OR_WE_504_REG_REACTIVE_ENERGY   0x0009
#define OR_WE_504_REG_BAUD_RATE         0x000E
#define OR_WE_504_REG_METER_ID          0x000F

#define OR_WE_504_REG_BAUD_RATE_1200    1
#define OR_WE_504_REG_BAUD_RATE_2400    2
#define OR_WE_504_REG_BAUD_RATE_4800    3
#define OR_WE_504_REG_BAUD_RATE_9600    4

#if RS485_BAUD_RATE == 9600
#define OR_WE_504_REG_BAUD_RATE_REG_CONTENT OR_WE_504_REG_BAUD_RATE_9600
#elif RS485_BAUD_RATE == 4800
#define OR_WE_504_REG_BAUD_RATE_REG_CONTENT OR_WE_504_REG_BAUD_RATE_4800
#elif RS485_BAUD_RATE == 2400
#define OR_WE_504_REG_BAUD_RATE_REG_CONTENT OR_WE_504_REG_BAUD_RATE_2400
#elif RS485_BAUD_RATE == 1200
#define OR_WE_504_REG_BAUD_RATE_REG_CONTENT OR_WE_504_REG_BAUD_RATE_1200
#else
#error Unsupported baud rate!
#endif

ModbusMaster ModbusMasterRS485;

//                         RX    TX
HardwareSerial SerialRS485(PA3, PA2);

typedef struct
{
    float voltage_V;
    float current_A;
    float freq_Hz;
    uint16_t activePower;
    uint16_t reactivePower;
    uint16_t apparentPower;
    float powerFactor;
    uint32_t activeEnergy;
    uint32_t reactiveEnergy;
} powerTotalEnergy_t;

powerTotalEnergy_t power;
uint16_t modbusRegisters[16];

/* Set direction pin to TX before RS-485 transmission */
void preTransmission()
{
    digitalWrite(RS485_DIR_PIN, HIGH);
}

/* Set direction pin to RX before RS-485 transmission */
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
    Serial.printf("Blue Pill OR-WE-504 reader started\n");
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
 * @brief Read multiple registers via ModBus.
 *
 * @return uint8_t Modbus error code. 0 if success.
 */
uint8_t modbus_read_registers(uint16_t a_reg_addr, uint16_t a_reg_count, uint16_t * a_uint16)
{
    uint8_t error;
    uint16_t i;

    delay(OR_WE_504_READ_DELAY_MS); /* Give some time for the device */
    error = ModbusMasterRS485.readHoldingRegisters(a_reg_addr, a_reg_count);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        for (i = 0u; i < a_reg_count; i++)
        {
            a_uint16[i] = ModbusMasterRS485.getResponseBuffer(i);
        }
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }

    return error;
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
    uint8_t error = ModbusMaster::ku8MBSuccess;
    uint16_t buffer;

    delay(OR_WE_504_READ_DELAY_MS); /* Give some time for the device */
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
 * @brief Read uint32 register via ModBus.
 *
 * @param a_reg_addr Address of register to read.
 * @param a_uint32 Pointer to 32-bit variable to fill.
 * @return uint8_t Modbus error code. 0 if success.
 */
uint8_t modbus_read_uint32(uint16_t a_reg_addr, uint32_t * a_uint32)
{
    uint8_t error;
    uint32_t response32;
    uint16_t buffer;

    delay(OR_WE_504_READ_DELAY_MS); /* Give some time for the device */
    error = ModbusMasterRS485.readHoldingRegisters(a_reg_addr, 2);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        buffer = ModbusMasterRS485.getResponseBuffer(0);
        response32 = buffer;
        buffer = ModbusMasterRS485.getResponseBuffer(1);
        response32 <<= 16;
        response32 |= buffer;
        *a_uint32 = response32;
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }

    return error;
}

/**
 * @brief Get uint16 register from modbusRegisters.
 *
 * @param a_reg_addr Address of register to read.
 * @param a_uint16 Pointer to 16-bit variable to fill.
 * @return uint8_t Modbus error code. 0 if success.
 */
uint8_t modbus_get_uint16(uint16_t a_reg_addr, uint16_t * a_uint16)
{
    uint8_t error = ModbusMaster::ku8MBSuccess;

    if (a_reg_addr < sizeof(modbusRegisters) / sizeof(modbusRegisters[0]))
    {
        *a_uint16 = modbusRegisters[a_reg_addr];
    }
    else
    {
        error = ModbusMaster::ku8MBInvalidFunction;
    }

    return error;
}

/**
 * @brief Get uint32 register from modbusRegisters.
 *
 * @param a_reg_addr Address of register to read.
 * @param a_uint32 Pointer to 32-bit variable to fill.
 * @return uint8_t Modbus error code. 0 if success.
 */
uint8_t modbus_get_uint32(uint16_t a_reg_addr, uint32_t * a_uint32)
{
    uint8_t error;
    uint32_t response32;
    uint16_t buffer;

    error = modbus_get_uint16(a_reg_addr, &buffer);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        response32 = buffer;
        error = modbus_get_uint16(a_reg_addr + 1, &buffer);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        response32 <<= 16;
        response32 |= buffer;
        *a_uint32 = response32;
    }

    return error;
}

/**
 * @brief Test OR-WE-504 by reading baud rate register and check its value.
 */
void or_we_504_test()
{
    uint8_t error;
    uint16_t buffer;

    Serial.printf("Trying to read baud rate register from OR-WE-504\n");
    delay(OR_WE_504_READ_DELAY_MS); /* Give some time for the device */
    error = ModbusMasterRS485.readHoldingRegisters(OR_WE_504_REG_BAUD_RATE, 1);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        buffer = ModbusMasterRS485.getResponseBuffer(0);
        if (buffer == OR_WE_504_REG_BAUD_RATE_REG_CONTENT)
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
 * @brief Read registers from OR-WE-504.
 *
 * @param a_power Pointer of structure to fill.
 * @return uint8_t  Modbus error code. 0 if success.
 */
uint8_t or_we_504_get_values(powerTotalEnergy_t * a_power)
{
    uint8_t error = 0;
    uint16_t reg_value = 0;

    error = modbus_get_uint16(OR_WE_504_REG_VOLTAGE, &reg_value);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->voltage_V = reg_value * 0.1f;
        error = modbus_get_uint16(OR_WE_504_REG_CURRENT, &reg_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->current_A = reg_value * 0.1f;
        error = modbus_get_uint16(OR_WE_504_REG_FREQUENCY, &reg_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->freq_Hz = reg_value * 0.1f;
        error = modbus_get_uint16(OR_WE_504_REG_ACTIVE_POWER, &(a_power->activePower));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_get_uint16(OR_WE_504_REG_REACTIVE_POWER, &(a_power->reactivePower));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_get_uint16(OR_WE_504_REG_APPARENT_POWER, &(a_power->apparentPower));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_get_uint16(OR_WE_504_REG_POWER_FACTOR, &reg_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->powerFactor = reg_value * 0.001f;
        error = modbus_get_uint32(OR_WE_504_REG_ACTIVE_ENERGY, &(a_power->activeEnergy));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_get_uint32(OR_WE_504_REG_REACTIVE_ENERGY, &(a_power->reactiveEnergy));
    }

    return error;
}


/**
 * @brief Process registers of modbusRegisters array.
 *
 * @param a_power Pointer of structure to fill.
 * @return uint8_t  Modbus error code. 0 if success.
 */
uint8_t or_we_504_read_values(powerTotalEnergy_t * a_power)
{
    uint8_t error = 0;
    uint16_t reg_value = 0;

    error = modbus_read_uint16(OR_WE_504_REG_VOLTAGE, &reg_value);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->voltage_V = reg_value * 0.1f;
        error = modbus_read_uint16(OR_WE_504_REG_CURRENT, &reg_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->current_A = reg_value * 0.1f;
        error = modbus_read_uint16(OR_WE_504_REG_FREQUENCY, &reg_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->freq_Hz = reg_value * 0.1f;
        error = modbus_read_uint16(OR_WE_504_REG_ACTIVE_POWER, &(a_power->activePower));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_read_uint16(OR_WE_504_REG_REACTIVE_POWER, &(a_power->reactivePower));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_read_uint16(OR_WE_504_REG_APPARENT_POWER, &(a_power->apparentPower));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_read_uint16(OR_WE_504_REG_POWER_FACTOR, &reg_value);
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        a_power->powerFactor = reg_value * 0.001f;
        error = modbus_read_uint32(OR_WE_504_REG_ACTIVE_ENERGY, &(a_power->activeEnergy));
    }
    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = modbus_read_uint32(OR_WE_504_REG_REACTIVE_ENERGY, &(a_power->reactiveEnergy));
    }

    return error;
}

/**
 * @brief Print memebrs of powerTotalEnergy structure to serial port.
 *
 * @param a_power Pointer of structure to print.
 */
void print_power(powerTotalEnergy_t * a_power)
{
    Serial.printf("Voltage: %.1f V\n", a_power->voltage_V);
    Serial.printf("Current: %.1f A\n", a_power->current_A);
    Serial.printf("Frequency: %.1f Hz\n", a_power->freq_Hz);
    Serial.printf("Active power: %d W\n", (unsigned int)a_power->activePower);
    Serial.printf("Reactive power: %d var\n", (unsigned int)a_power->reactivePower);
    Serial.printf("Power factor: %.3f\n", a_power->powerFactor);
    Serial.printf("Apparent power: %d VA\n", (unsigned int)a_power->apparentPower);
    Serial.printf("Active energy: %d Wh\n", a_power->activeEnergy);
    Serial.printf("Reactive energy: %d varh\n", a_power->reactiveEnergy);
}

void loop()
{
    Serial.printf("\n");
    Serial.printf("---------------------------------------------------------\n");
    // or_we_504_test();
#if 1
    /* Read all registers at once */
    uint32_t start_timestamp = millis();
    uint8_t error = modbus_read_registers(0,
                                  sizeof(modbusRegisters) / sizeof(modbusRegisters[0]),
                                  modbusRegisters);
    uint32_t stop_timestamp = millis();

    Serial.printf("Time: %i ms\n", stop_timestamp - start_timestamp);

    if (error == ModbusMaster::ku8MBSuccess)
    {
        error = or_we_504_get_values(&power);
    }

    if (error == ModbusMaster::ku8MBSuccess)
    {
        print_power(&power);
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }
#elif 0
    uint32_t start_timestamp = millis();
    uint8_t error = or_we_504_read_values(&power);
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
#elif 0
    uint32_t reg32;
    uint8_t error = modbus_read_uint32(OR_WE_504_REG_ACTIVE_ENERGY, &reg32);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Active energy: %d Wh\n", reg32);
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }
    error = modbus_read_uint32(OR_WE_504_REG_REACTIVE_ENERGY, &reg32);
    if (error == ModbusMaster::ku8MBSuccess)
    {
        Serial.printf("Reactive energy: %d varh\n", reg32);
    }
    else
    {
        Serial.printf("Modbus error: 0x%X %s\n", error, modbusErrorStr(error));
    }
#endif

    delay(5000);
}

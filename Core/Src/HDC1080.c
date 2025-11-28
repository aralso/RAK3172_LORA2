#include "HDC1080.h"


void HDC1080_init(void)
{
	uint16_t config = HDC1080_HEATER_OFF_CMD | HDC1080_SEQUENCE_MODE | HDC1080_HUM_RES_14_BIT | HDC1080_TEMP_RES_14_BIT;
	HDC1080_write_configuration_register(config);
	HAL_Delay(10);

	/*HDC1080_write_configuration_register(HDC1080_SW_RST_CMD);
	HAL_Delay(10);
	HDC1080_write_configuration_register(HDC1080_NORMAL_CMD);
	HAL_Delay(10);
	HDC1080_write_configuration_register(HDC1080_HEATER_ON_CMD);
	HAL_Delay(100);
	HDC1080_write_configuration_register(HDC1080_HEATER_OFF_CMD \
										| HDC1080_INDEPENDENT_MODE \
										| HDC1080_TEMP_RES_14_BIT \
										| HDC1080_HUM_RES_14_BIT);
	HAL_Delay(10);*/
}


void HDC1080_write_configuration_register(uint8_t value)
{
	uint8_t wr_buffer[3] = {HDC1080_Configuration_REG, 0x00, 0x00};

	wr_buffer[1] = value;  //((value & 0xFF00) >> 8);

	HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, wr_buffer, 3, 1000);
}


/*uint16_t HDC1080_read_configuration_register(void)
{
	uint8_t data_buffer[2] = {HDC1080_Configuration_REG, 0x00};
	uint16_t retval = 0x0000;

	HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

	retval = ((uint16_t)data_buffer[0]);
	retval <<= 8;
	retval |= ((uint16_t)data_buffer[1]);

	return retval;
}*/

uint16_t HDC1080_read_configuration_register(void)
{
    //uint8_t reg = HDC1080_Configuration_REG;
    uint8_t data_buffer[2] = {0};
    uint16_t retval = 0x0000;

    HAL_I2C_Mem_Read(&hi2c2,
                     HDC1080_I2C_Address,
                     HDC1080_Configuration_REG,
                     I2C_MEMADD_SIZE_8BIT,
                     data_buffer,
                     2,
                     1000);

    //HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, &reg, 1, 1000);
    //HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

    retval = (data_buffer[0] << 8) | data_buffer[1];

    return retval;
}

uint16_t HDC1080_read_manufacturer_ID(void)
{

	uint8_t data_buffer[2] = {HDC1080_Manufacturer_ID_REG, 0x00};
	uint16_t retval = 0x0000;

	HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

	retval = ((uint16_t)data_buffer[0]);
	retval <<= 8;
	retval |= ((uint16_t)data_buffer[1]);

	return retval;
}


uint16_t HDC1080_read_device_ID(void)
{
	uint8_t data_buffer[2] = {HDC1080_Device_ID_REG, 0x00};
	uint16_t retval = 0x0000;

	HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

	retval = ((uint16_t)data_buffer[0]);
	retval <<= 8;
	retval |= ((uint16_t)data_buffer[1]);

	return retval;
}


uint16_t HDC1080_read_serial_ID_H(void)
{
	uint8_t data_buffer[2] = {HDC1080_ID1_REG, 0x00};
	uint16_t retval = 0x0000;

	HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

	retval = ((uint16_t)data_buffer[0]);
	retval <<= 8;
	retval |= ((uint16_t)data_buffer[1]);

	return retval;
}


uint16_t HDC1080_read_serial_ID_M(void)
{
	uint8_t data_buffer[2] = {HDC1080_ID2_REG, 0x00};
	uint16_t retval = 0x0000;

	HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

	retval = ((uint16_t)data_buffer[0]);
	retval <<= 8;
	retval |= ((uint16_t)data_buffer[1]);

	return retval;
}


uint8_t HDC1080_read_serial_ID_L(void)
{
	uint8_t data_buffer[2] = {HDC1080_ID3_REG, 0x00};
	uint16_t retval = 0x0000;

	HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

	retval = ((uint16_t)data_buffer[0]);
	retval <<= 8;
	retval |= ((uint16_t)data_buffer[1]);

	return ((uint8_t)(retval >> 8));
}


float HDC1080_read_temperature(void)
{
	uint8_t data_buffer[2] = {HDC1080_Temperature_REG, 0x00};
	uint16_t temp = 0x0000;
	float retval = 0.0;

	HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	HAL_Delay(HDC1080_conversion_delay);
	HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

	temp = ((uint16_t)data_buffer[0]);
	temp <<= 8;
	temp |= ((uint16_t)data_buffer[1]);

	retval = ((((float)temp) * T_Coff) - 40.0);

	return retval;
}


float HDC1080_read_humidity(void)
{
	uint8_t data_buffer[2] = {HDC1080_Humidity_REG, 0x00};
	uint16_t temp = 0x0000;
	float retval = 0.0;

	HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	HAL_Delay(HDC1080_conversion_delay);
	HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

	temp = ((uint16_t)data_buffer[0]);
	temp <<= 8;
	temp |= ((uint16_t)data_buffer[1]);

	retval = (((float)temp) * RH_Coff);

	return retval;
}

// lecture en sequence temp et humid
// return : 0:ok 1:erreur
/*uint8_t HDC1080_read_tempe_humid(uint16_t* tempe, uint8_t* humid)
{
	uint8_t ret = 1;
	uint8_t status1, status2;
	uint8_t data_buffer[2] = {HDC1080_Temperature_REG, 0x00};
	uint16_t temp = 0x0000;
	float temp_f = 0.0;

	status1 = HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	HAL_Delay(HDC1080_conversion_delay);
	status2 = HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

	temp = ((uint16_t)data_buffer[0]);
	temp <<= 8;
	temp |= ((uint16_t)data_buffer[1]);

	temp_f = ((((float)temp) * T_Coff) - 40.0);
	*tempe = (uint16_t)((temp_f+100)*100);
	ret = status1 + status2;

	*humid=0;
	return ret;
}*/

uint8_t HDC1080_read_tempe_humid1(uint16_t* temperature, uint8_t* humidity)
{
    uint8_t ret = 1;
    uint8_t data_buffer[4];  // Buffer pour la température et l'humidité
    uint16_t temp = 0x0000;
    uint16_t humid = 0x0000;
    float temp_f = 0.0;

    // 1. Demande de conversion de température (Registre 0x00)
    //data_buffer[0] = HDC1080_Temperature_REG;
    ret = HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, HDC1080_Temperature_REG, 1, 1000);
    if (ret != HAL_OK) {
        return 2;  // Si une erreur survient dans la transmission pour la température
    }

	//HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
	//HAL_Delay(HDC1080_conversion_delay);
	//HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 2, 1000);

    // 2. Attendre 8 ms pour que la conversion de température soit terminée
    HAL_Delay(HDC1080_conversion_delay*2);  // Attente de 8 ms pour la conversion de température

    // 3. Lire la température et humidite (Registre 0x00)
	ret = HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 4, 1000);
    //ret = HAL_I2C_Mem_Read(&hi2c2, HDC1080_I2C_Address, HDC1080_Temperature_REG, I2C_MEMADD_SIZE_8BIT, data_buffer, 2, 1000);
    if (ret != HAL_OK) {
        return 1;  // Si une erreur survient dans la lecture de la température
    }

    // Conversion des données de température en °C
    temp = ((uint16_t)data_buffer[0] << 8) | data_buffer[1];
    temp_f = (((float)temp) * T_Coff) - 40.0;  // Application de la conversion en float
    *temperature = (uint16_t)((temp_f + 100.0) * 100);  // Conversion en centi-degrés

    // 4. Demande de conversion d'humidité (Registre 0x01)
    /*data_buffer[0] = HDC1080_Humidity_REG;
    ret = HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, data_buffer, 1, 1000);
    if (ret != HAL_OK) {
        return 0;  // Si une erreur survient dans la transmission pour l'humidité
    }*/

    // 5. Attendre 8 ms pour que la conversion d'humidité soit terminée
    //HAL_Delay(8);  // Attente de 8 ms pour la conversion d'humidité

    // 6. Lire l'humidité (Registre 0x01)
    /*ret = HAL_I2C_Mem_Read(&hi2c2, HDC1080_I2C_Address, HDC1080_Humidity_REG, I2C_MEMADD_SIZE_8BIT, data_buffer, 2, 1000);
    if (ret != HAL_OK) {
        return 0;  // Si une erreur survient dans la lecture de l'humidité
    }*/

    // Conversion des données d'humidité en pourcentage
    humid = ((uint16_t)data_buffer[2] << 8) | data_buffer[3];
    *humidity = humid;  // Humidité déjà en pourcentage

    return 0;  // Lecture réussie
}

uint8_t HDC1080_read_tempe_humid(uint16_t* temperature, uint8_t* humidity)
{
    uint8_t ret = 1;
    uint8_t data_buffer[4];  // Buffer pour la température et l'humidité

    *temperature = 0;
    *humidity = 0;

    // 1. Demande de conversion de température (Registre 0x00)
    ret = HAL_I2C_Master_Transmit(&hi2c2, HDC1080_I2C_Address, HDC1080_Temperature_REG, 1, 1000);
    if (ret != HAL_OK)  return 2;  // Si une erreur survient dans la transmission pour la température

    osDelay(HDC1080_conversion_delay*2);
    HAL_Delay(HDC1080_conversion_delay*2);  // Attente de 20 ms pour la conversion de température

    // 3. Lire la température et humidite (Registre 0x00)
	ret = HAL_I2C_Master_Receive(&hi2c2, HDC1080_I2C_Address, data_buffer, 4, 1000);
    if (ret != HAL_OK)  return 3;  // Si une erreur survient dans la lecture de la température

    // Conversion des données de température en °C
    uint16_t temp16 = ((uint16_t)data_buffer[0] << 8) | data_buffer[1];
    float temp_f = (((float)temp16) * T_Coff) - 40.0;  // Application de la conversion en float : en degré
    *temperature = (uint16_t)((temp_f + 100.0) * 100);  // Conversion en centi-degrés -10°:9000 0°:10000 25°:12500
    uint16_t humid = ((uint16_t)data_buffer[2] << 8) | data_buffer[3];
    float humid_f = ((float)humid)*RH_Coff;
    *humidity = (uint16_t)(humid_f);  // Humidité déjà en pourcentage

    return ret;  // Lecture réussie
}

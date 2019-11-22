#include "config/config.h"
#include "config/iic.h"
#include "bme680.h"
#include "bme680_defs.h"

#define BME680_DEFAULT_ADDRESS       (0x76)     ///< The default I2C address
#define SEALEVELPRESSURE_HPA (1013.25)

XIic iic;

struct bme680_dev gas_sensor;

int main(void)
{
	printf ("Start BME680\r\n");

	XIic_Initialize (&iic, XPAR_AXI_IIC_0_DEVICE_ID);

	BME680_begin(BME680_DEFAULT_ADDRESS, true);

	BME680_setTemperatureOversampling(BME680_OS_8X);
	BME680_setHumidityOversampling(BME680_OS_2X);
	BME680_setPressureOversampling(BME680_OS_4X);
	BME680_setIIRFilterSize(BME680_FILTER_SIZE_3);
	BME680_setGasHeater(320, 150); // 320*C for 150 ms

	while(1)
	{
      if (! BME680_performReading())
	  {
		printf("Failed to perform reading :(");
		return 0;
	  }

	  printf("Temperature = ");
	  printf("%f", BME680_readTemperature());
	  printf(" *C");
	  printf("\r\n");

	  printf("Pressure = ");
	  printf("%f",BME680_readPressure() / 100.0);
	  printf(" hPa");
	  printf("\r\n");

	  printf("Humidity = ");
	  printf("%f",BME680_readHumidity());
	  printf(" %%");
	  printf("\r\n");

	  printf("Gas = ");
	  printf("%f",BME680_readGas()/ 1000.0);
	  printf(" KOhms");
	  printf("\r\n");

	  printf("Approx. Altitude = ");
	  printf("%f",BME680_readAltitude(SEALEVELPRESSURE_HPA));
	  printf(" m");
	  printf("\r\n");

	  printf("\r\n");
	  usleep(2000);
	}

	return 0;
}

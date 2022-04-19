#include "main.h"

#define SPI1EN					(1U<<12)
#define	GPIOAEN					(1U<<0)
#define GYRO_CONFIG				0x1B
#define ACCEL_CONFIG			0x1C
#define ACCEL_CONFIG2			0x1D
#define ACCEL_XOUT_H_REG		0x43
#define	TEMP_OUT_H_REG			0x41
#define	GYRO_XOUT_H_REG 		0x3B
#define PWR_MGMT_1				0x6B
#define USER_CTRL				0x6A
#define WHO_AM_I_REG			0x75
#define SMPLRT_DIV_REG			0x19

uint8_t Rx_data[2];

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int16_t Temperature_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz, Temperature;

void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
}

void TIM5Config (void){
	RCC->APB1ENR |= (1<<3);
	TIM5->PSC = 90-1;
	TIM5->ARR = 0xffff;
	TIM5->CR1 |= (1<<0);
	while (!(TIM5->SR & (1<<0)));
}

void Delay_us (uint16_t us){
	TIM5->CNT = 0;
	while (TIM5->CNT < us);
}

void Delay_ms (uint16_t ms){
	for (uint16_t i=0; i<ms; i++){
		Delay_us (1000);
	}
}

void SPI_Init(void){
	RCC->AHB1ENR  |= GPIOAEN;
	RCC->APB2ENR  |= SPI1EN;

	// Pins PA4 as NSS, PA5 as SCK, PA6 as MISO, and PA7 as MOSI
	GPIOA->MODER |= (1<<8);
	GPIOA->MODER &=~(1<<9);
	GPIOA->MODER |= (2<<10) | (2<<12) | (2<<14);

	GPIOA->OTYPER |= (1<<4);

	GPIOA->PUPDR |= (1<<8);
	GPIOA->PUPDR &=~(1<<9);

	GPIOA->OSPEEDR |= (3<<10)|(3<<12)|(3<<14);

	// Select AF5
	GPIOA->AFR[0] |= (5<<20) | (5<<24) | (5<<28);


	SPI1->CR1 |= (1<<0);
	SPI1->CR1 |= (1<<1);   // CPOL=1, CPHA=1
	SPI1->CR1 |= (1<<2);  // Master Mode
	SPI1->CR1 |= (4<<3);  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz
 	SPI1->CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first
 	SPI1->CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 -> Software Slave Management
 	SPI1->CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex
 	SPI1->CR1 &= ~(1<<11);  // DFF=0, 8 bit data
 	SPI1->CR2 = 0;

}

void SPI_Transmit (uint8_t *data, int size){
	int i=0;
	uint8_t temp;
	while (i<size){
	   while (!((SPI1->SR)&(1<<1)));
	   SPI1->DR = data[i];
	   i++;
	}
	while (!((SPI1->SR)&(1<<1)));
	while (((SPI1->SR)&(1<<7)));
	temp = SPI1->DR | SPI1->SR;
}

void SPI_Receive (uint8_t *data, int size){
	while (size){
		while (((SPI1->SR)&(1<<7)));
		SPI1->DR = 0;
		while (!((SPI1->SR) &(1<<0)));
		*data++ = (SPI1->DR);
		size--;
	}
}


void SPI_Enable(void){
	SPI1->CR1 |= (1<<6);
}

void SPI_Disable(void){
	SPI1->CR1 &=~(1<<6);
}

void CS_Enable(void){
	GPIOA->BSRR |= (1<<4) << 16;
}

void CS_Disable(void){
	GPIOA->BSRR |= (1<<4);
}


void MPU9250_Write(uint8_t address, uint8_t value){
	uint8_t data[2];
	data[0] = address;
	data[1] = value;
	CS_Enable();
	SPI_Transmit(data,2);
	CS_Disable();
}

void MPU9250_Read(uint8_t address, int size){
	address |= 0x80;
	CS_Enable();
	SPI_Transmit(&address,1);
	SPI_Receive(Rx_data, size);
	CS_Disable();
}

void MPU9250_Init (void){
		MPU9250_Write(GYRO_CONFIG, 0x00);
		MPU9250_Write(ACCEL_CONFIG, 0x00);
		MPU9250_Write(PWR_MGMT_1,0x00);
		MPU9250_Write(USER_CTRL,0x10);
		MPU9250_Write(SMPLRT_DIV_REG, 0x00);
}


void MPU9250_Read_Accel(int16_t *Accel_X_RAW, int16_t *Accel_Y_RAW, int16_t *Accel_Z_RAW, float *Ax, float *Ay, float *Az){
	MPU9250_Read(ACCEL_XOUT_H_REG, 2);
	*Accel_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
	MPU9250_Read(ACCEL_XOUT_H_REG+2, 2);
	*Accel_Y_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
	MPU9250_Read(ACCEL_XOUT_H_REG+4, 2);
	*Accel_Z_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);

	*Ax = *Accel_X_RAW/16384.0;
	*Ay = *Accel_Y_RAW/16384.0;
	*Az = *Accel_Z_RAW/16384.0;
}

void MPU9250_Read_Gyro(int16_t *Gyro_X_RAW, int16_t *Gyro_Y_RAW, int16_t *Gyro_Z_RAW, float *Gx, float *Gy, float *Gz){
	MPU9250_Read(GYRO_XOUT_H_REG, 2);
	*Gyro_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
	MPU9250_Read(GYRO_XOUT_H_REG+2, 2);
	*Gyro_Y_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
	MPU9250_Read(GYRO_XOUT_H_REG+4, 2);
	*Gyro_Z_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);

	*Gx = *Gyro_X_RAW / 178;
	*Gy = *Gyro_Y_RAW / 178;
	*Gz = *Gyro_Z_RAW / 178;
}

void MPU9250_Read_Temp(int16_t *Temperature_RAW, float *Temperature){
	MPU9250_Read(TEMP_OUT_H_REG, 2);
	*Temperature_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);

	*Temperature = (*Temperature_RAW-21)/333.87 + 17;
}

int main(void){
  SystemClock_Config();
  TIM5Config();
  SPI_Init();
  SPI_Enable();
  MPU9250_Init();
  while (1){
	  MPU9250_Read_Accel(&Accel_X_RAW, &Accel_Y_RAW, &Accel_Z_RAW, &Ax, &Ay, &Az);
	  MPU9250_Read_Gyro(&Gyro_X_RAW, &Gyro_Y_RAW, &Gyro_Z_RAW, &Gx, &Gy, &Gz);
	  MPU9250_Read_Temp(&Temperature_RAW, &Temperature);
	  Delay_ms(100);
  }
}



#include "main.h"

// LIS3DH I2C address
#define LIS3DH_ADDR 0x32

// LIS3DH registers
#define LIS3DH_REG_CTRL1 0x20
#define LIS3DH_REG_OUT_X_L 0x28

I2C_HandleTypeDef hi2c1;

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void LIS3DH_Init(void);
void LIS3DH_Read_Accel(int16_t* x, int16_t* y, int16_t* z);
uint8_t LIS3DH_Read_Register(uint8_t reg);
void LIS3DH_Write_Register(uint8_t reg, uint8_t value);

int main(void)
{
  // Initialization
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  // Initialize LIS3DH
  LIS3DH_Init();

  // Variables to store accelerometer data
  int16_t x, y, z;

  while (1)
  {
    // Read accelerometer data
    LIS3DH_Read_Accel(&x, &y, &z);

    // Simple movement detection based on threshold
    if (abs(x) > 1000 || abs(y) > 1000 || abs(z) > 1000)
    {
      // Turn on LED
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
    else
    {
      // Turn off LED
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    // Small delay
    HAL_Delay(100);
  }
}

void LIS3DH_Init(void)
{
  // Write to CTRL_REG1 to enable the accelerometer
  LIS3DH_Write_Register(LIS3DH_REG_CTRL1, 0x47);
}

void LIS3DH_Read_Accel(int16_t* x, int16_t* y, int16_t* z)
{
  uint8_t buffer[6];

  // Read 6 bytes from the accelerometer starting from OUT_X_L
  HAL_I2C_Mem_Read(&hi2c1, LIS3DH_ADDR, LIS3DH_REG_OUT_X_L | 0x80, 1, buffer, 6, HAL_MAX_DELAY);

  // Combine high and low bytes
  *x = (int16_t)(buffer[1] << 8 | buffer[0]);
  *y = (int16_t)(buffer[3] << 8 | buffer[2]);
  *z = (int16_t)(buffer[5] << 8 | buffer[4]);
}

uint8_t LIS3DH_Read_Register(uint8_t reg)
{
  uint8_t value;
  HAL_I2C_Mem_Read(&hi2c1, LIS3DH_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
  return value;
}

void LIS3DH_Write_Register(uint8_t reg, uint8_t value)
{
  HAL_I2C_Mem_Write(&hi2c1, LIS3DH_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
}

// I2C1 initialization function
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  //hi2c1.Init.ClockSpeed = 100000;
  //hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

// GPIO initialization function
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure LED GPIO pin
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// System Clock Configuration
void SystemClock_Config(void)
{
  // Configuration code generated by STM32CubeMX
}

// Error Handler
void Error_Handler(void)
{
  while (1)
  {
    // Error handling code
  }
}
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <ICM_20948.h>
#include <stm32f4xx_hal.h>

#define MOSI1 PA7 //black
#define MISO1 PA6 //yellow
#define SCLK1 PA5 //red
#define CS1 PA4 //white
#define IMU_CS PB12

#define USART_TX_Pin GPIO_PIN_2
#define USART_RX_Pin GPIO_PIN_3

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

// DMA buffer and completion flag
volatile bool dma_tx_complete = true;

#undef Error_Handler

extern "C" void Error_Handler(void)
{
  __disable_irq();
  while(1) { }
}
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 24;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

extern "C" void HAL_MspInit(void)
{

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

}

extern "C" void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmatx, hdma_usart2_tx);

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

extern "C" void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);
    HAL_DMA_DeInit(huart->hdmatx);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

extern "C" void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

extern "C" void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

extern "C" void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	dma_tx_complete = true;
}

MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, CS1);
SPIClass SPI_1(MOSI1, MISO1, SCLK1);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PC9);

float target_angle = 0;
volatile float derivative = 0;
volatile float imu_angle_z = 0;  // shared between ISR and main loop
volatile float imu_angle_y = 0;
volatile float filtered_imu_angle_z = 0;  // filtered IMU value
const float alpha = 0.9;  // Low-pass filter coefficient (0.1 for smoothing)

// PID gains
const float pid_Kp = 0.025;
const float pid_Ki = 12;
const float pid_Kd = 0.0;

// PID state
volatile float pid_output = 0;
volatile float pid_integral = 0;
volatile float last_error = 0;
volatile float rate_error = 0;
static float tx_value = 0.0f;

static uint8_t tx_buf[5];

Commander command = Commander(Serial);

SPIClass SPI_2(PB15, PB14, PB13); // MOSI, MISO, SCK
ICM_20948_SPI myimu;
SPISettings imuSPISettings(7000000, MSBFIRST, SPI_MODE3);

void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {

  SystemClock_Config();
  MX_DMA_Init();
  MX_USART2_UART_Init();
 
  Serial.begin(1000000);

  HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);

  SPI_2.begin();

  ICM_20948_fss_t fs;   // full scale settings struct
  fs.a = gpm2;          // accelerometer +-2g
  fs.g = dps2000;       // gyroscope +-2000 deg/s

  if (myimu.begin(IMU_CS, SPI_2) != ICM_20948_Stat_Ok) {
    Serial.println("IMU init failed!");
    while (1) {}
  }

  myimu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fs);
  Serial.println("IMU initialized successfully");

  motor.useMonitoring(Serial);

  Serial.println(F("Motor ready."));

  SimpleFOCDebug::enable(&Serial);
  sensor.init(&SPI_1);
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 5;
  
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL;
  motor.monitor_downsample = 1000;

  motor.init();
  motor.initFOC();

  command.add('T', doTarget, "target angle");
  command.add('M', doMotor, "motor");
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));

  HardwareTimer* timer = new HardwareTimer(TIM5);
  timer->setOverflow(10000, HERTZ_FORMAT); 
  timer->attachInterrupt([&](){

    rate_error = target_angle - imu_angle_z;

    if (fabs(rate_error) < 6.0) {
        rate_error = 0.0;  // Ignore small errors within the deadband range
    }

    pid_integral += rate_error * 0.0001;

        if (pid_integral > 10.0) {
        pid_integral = 10.0;  // Limit the integral term
    } else if (pid_integral < -10.0) {
        pid_integral = -10.0;  // Limit the integral term
    }

    derivative = (rate_error - last_error) / 0.0001;
    pid_output = (pid_Kp*rate_error) + (pid_Ki*pid_integral) + (pid_Kd*derivative);

    // Limit the target_angle between -2 and 2
    if (pid_output > 3.5) {
        pid_output = 3.5;
    } else if (pid_output < -3.5) {
        pid_output = -3.5;
    }
    
    motor.loopFOC();
    motor.move(pid_output);
    last_error = rate_error;
  });

  timer->resume();
  HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);

  HardwareTimer* imuTimer = new HardwareTimer(TIM4); // Use TIM4 for IMU
  imuTimer->setOverflow(1200, HERTZ_FORMAT);
  imuTimer->attachInterrupt([]() {
      if (myimu.dataReady()) {
          myimu.getAGMT();             // read accel, gyro, mag, temp
          imu_angle_z = myimu.gyrZ();  // store Z gyro value in volatile
          imu_angle_y = myimu.gyrY();
      }
  });
  imuTimer->resume();
  HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
  
  _delay(1000);

}

void loop() {


  if (dma_tx_complete && huart2.gState == HAL_UART_STATE_READY)
  {
    tx_value = imu_angle_y;
    tx_buf[0] = 0xAB; // sync byte
    memcpy(&tx_buf[1], &tx_value, 4);
    HAL_UART_Transmit_DMA(&huart2, tx_buf, 5);
    dma_tx_complete = false;

  }

  // motor.monitor();
  // command.run(); 

}
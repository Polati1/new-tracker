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
DMA_HandleTypeDef hdma_usart2_rx;

// DMA RX buffer
#define UART_RX_BUFFER_SIZE 5  // 1 sync byte + 4 float bytes
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile bool dma_rx_complete = false;
extern volatile float imu_angle_y;

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
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ===== RX DMA CONFIGURATION =====
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

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
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

extern "C" void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

extern "C" void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

extern "C" void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (uart_rx_buffer[0] == 0xAB)
  {
    memcpy((void*)&imu_angle_y, &uart_rx_buffer[1], 4);
  }

}

MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, CS1);
SPIClass SPI_1(MOSI1, MISO1, SCLK1);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PC9);

float target_angle = 0;
volatile float derivative = 0;
volatile float imu_angle_y = 0;
const float alpha = 0.9;

const float pid_Kp = 0.015;
const float pid_Ki = 2.15;
const float pid_Kd = 0.0;

volatile float pid_output = 0;
volatile float pid_integral = 0;
volatile float last_error = 0;
volatile float rate_error = 0;

Commander command = Commander(Serial);

void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {

  SystemClock_Config();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  Serial.begin(115200);

  HAL_UART_Receive_DMA(&huart2, uart_rx_buffer, UART_RX_BUFFER_SIZE);

  HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);

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
  timer->setOverflow(20000, HERTZ_FORMAT);
  timer->attachInterrupt([&](){

    rate_error = 0.0 - imu_angle_y;

    if (fabs(rate_error) < 2.3) {
        rate_error = 0.0;
    }

    pid_integral += rate_error * 0.00005f;

    if (pid_integral > 1.2) pid_integral = 1.2;
    else if (pid_integral < -1.2) pid_integral = -1.2;

    derivative = (rate_error - last_error) / 0.00005f;
    pid_output = (pid_Kp*rate_error) + (pid_Ki*pid_integral) + (pid_Kd*derivative);

    if (pid_output > 2.0) {
        pid_output = 2.0;
    } else if (pid_output < -2.0) {
        pid_output = -2.0;
    }

    motor.loopFOC();
    motor.move(pid_output);
    last_error = rate_error;
  });

  timer->resume();
  HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);

  _delay(1000);
}

void loop() {


  if (dma_rx_complete) {
    
    dma_rx_complete = false;
  
  }

  // motor.monitor();
  // command.run();

}

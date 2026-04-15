#include <Arduino.h>
#include <SimpleFOC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stm32f4xx_hal.h>

#define MOSI1 PA7 //black
#define MISO1 PA6 //yellow
#define SCLK1 PA5 //red
#define CS1 PA4 //white

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



MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, CS1);
SPIClass SPI_1(MOSI1, MISO1, SCLK1);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PC9);


Commander command = Commander(Serial);

float target_angle = 0;
bool sequenceActive = false;
unsigned long sequenceStartTime = 0;

void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void startPulse(char* cmd) {
    sequenceActive = true;
    sequenceStartTime = millis();
}

void setup() {

  SystemClock_Config();

  Serial.begin(115200);

  HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);

  motor.useMonitoring(Serial);

  Serial.println(F("Motor ready."));

  SimpleFOCDebug::enable(&Serial);
  sensor.init(&SPI_1);
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12; 
  motor.voltage_limit = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.PID_velocity.output_ramp = 0;
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
  command.add('F', startPulse, "kick sequence");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));

  HardwareTimer* timer = new HardwareTimer(TIM5);
  timer->setOverflow(10000, HERTZ_FORMAT); 
  timer->attachInterrupt([&](){

    motor.loopFOC();

  });

  timer->resume();
  HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);

  _delay(1000);
}

void loop() {

    // 2. The Kick State Machine
    if (sequenceActive) {
        unsigned long now = millis();
        unsigned long elapsed = now - sequenceStartTime;

        if (elapsed < 30) {
            // Stage 1: Positive kick (5ms)
            motor.move(50000);
        } 
        else if (elapsed < 60) {
            // Stage 2: Negative kick (Next 5ms)
            motor.move(-50000);
        } 
        else {
            // Stage 3: Sequence finished, return to 0 and wait for next 'F'
            motor.move(0);
            sequenceActive = false;
            Serial.println(F("Sequence complete."));
        }
    } else {
        // Normal operation (uses the 'T' command target)
        motor.move(target_angle);
    }

    // 3. Keep communication alive
    command.run();
    motor.monitor();
}
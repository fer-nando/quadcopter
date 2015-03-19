/*
 ===============================================================================
 Name        : main.c
 Author      : Fernando Padilha Ferreira
 Version     : 1.0
 Copyright   : (c) Fernando Padilha Ferreira
 Description :
 ===============================================================================
 */

#include "inc/lm4f120h5qr.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include "driverlib/fpu.h"
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/eeprom.h>
#include <utils/uartstdio.h>
#include <drivers/buttons.h>
#include "driverlib/timer.h"

#include "utils/spi_functions.h"
#include "utils/i2c_functions.h"
#include "utils/delay.h"
#include "utils/linalg.h"
#include "sensors.h"
#include "motor.h"
#include "rc.h"
#include "RF24.h"



//#define SERIAL_DEBUG
#define RADIO_LOG
//#define ESC_CALIBRATION


#define PID_I_LIMIT      5.0
#define MOTOR_FREQUENCY  400      // 400 Hz
#define MOTOR_TIME_US    2500     // 2,500 us = 2.5 ms
#define LOG_TIME_US      10000    // 10,000 us = 10 ms

#define RED_LED          GPIO_PIN_1
#define BLUE_LED         GPIO_PIN_2
#define GREEN_LED        GPIO_PIN_3

#define ledOn(x)         (GPIOPinWrite(GPIO_PORTF_BASE, x, x))
#define ledOff(x)        (GPIOPinWrite(GPIO_PORTF_BASE, x, 0))
#define readButton(x)    (GPIOPinRead(GPIO_PORTF_BASE, x))

enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDCONTROLS
};

static struct {
  int      defaults;
  float    Kp[PIDCONTROLS];
  float    Ki[PIDCONTROLS];
  float    Kd[PIDCONTROLS];
  int32_t  accZero[3];
  int32_t  magZero[3];
  uint32_t checksum;
} config;

uint64_t currentTime, previousTime, cycleTime, uartTime, rcTime;

float roll, pitch, yaw, rollRate, pitchRate, altitude;
extern float     gyroValue[3], accValue[3], magValue[3];

int rc[4];

float mtr[4];

float lastError[PIDCONTROLS], errorSum[PIDCONTROLS];

tBoolean armed = false;

char str[8][10];

uint8_t buffer[PAYLOAD_SIZE];

int rf_state;
const uint64_t addr = 0x5958575655;

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void InitConsole(void) {
  //
  // Enable GPIO port A which is used for UART0 pins.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  //
  // Configure the pin muxing for UART0 functions on port A0 and A1.
  // This step is not necessary if your part does not support pin muxing.
  //
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);

  //
  // Select the alternate (UART) function for these pins.
  //
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //
  // Initialize the UART for console I/O.
  //
  UARTStdioInit(0);
}


int CalculateChecksum(uint32_t * p, int size) {
  uint32_t sum = 0;
  int i;
  for (i = 0; i < size - 1; i++) {
    sum += p[i];
  }
  return sum;
}

void LoadDefaults() {
  config.defaults = true;

  config.Kp[PIDROLL]  = 0.15;  config.Ki[PIDROLL]  = 0.001;  config.Kd[PIDROLL]  = 0.05;
  config.Kp[PIDPITCH] = 0.15;  config.Ki[PIDPITCH] = 0.001;  config.Kd[PIDPITCH] = 0.05;
  config.Kp[PIDYAW]   = 0.0;  config.Ki[PIDYAW]   = 0.0;  config.Kd[PIDYAW] = 0.0;
  config.Kp[PIDALT]   = 0.0;  config.Ki[PIDALT]   = 0.0;  config.Kd[PIDALT] = 0.0;

  config.accZero[0] = 0;  config.accZero[1] = 0;  config.accZero[2] = 0;
  config.magZero[0] = 0;  config.magZero[1] = 0;  config.magZero[2] = 0;

  config.checksum = CalculateChecksum((uint32_t *) &config, sizeof(config)/sizeof(uint32_t));
}

void ReadConfig() {
  // read the configuration stored on eeprom
  EEPROMRead((unsigned long *) &config, 0, sizeof(config));

  // invalid configuration, load default values
  if (CalculateChecksum((uint32_t *) &config, sizeof(config)/sizeof(uint32_t)) != config.checksum) {
#ifdef SERIAL_DEBUG
  UARTprintf("\tInvalid configuration, loading default values.\n");
#endif
    LoadDefaults();
  }
#ifdef SERIAL_DEBUG
  else {
    UARTprintf("\tConfiguration read.\n");
  }
#endif
}

void WriteConfig() {
  // change default values flag and calculate checksum
  config.defaults = false;
  config.checksum = CalculateChecksum((uint32_t *) &config, sizeof(config)/sizeof(uint32_t));

  // write the configuration on eeprom
  EEPROMProgram((unsigned long *) &config, 0, sizeof(config));
}


void PIDInit() {
  lastError[PIDROLL]  = 0.0;
  lastError[PIDPITCH] = 0.0;
  lastError[PIDYAW]   = 0.0;
  lastError[PIDALT]   = 0.0;
  errorSum[PIDROLL]  = 0.0;
  errorSum[PIDPITCH] = 0.0;
  errorSum[PIDYAW]   = 0.0;
  errorSum[PIDALT]   = 0.0;
}


void AttitudeControl() {
  int i;
  float throttle;
  float error, errorDiff;
  float pid[PIDCONTROLS];

  // PITCH PID
  error = pitch;
  errorDiff = pitchRate;//(error - lastError[PIDPITCH]); // pitchRate
  lastError[PIDPITCH] = error;
  errorSum[PIDPITCH] += pitchRate * config.Ki[PIDPITCH];

  if(errorSum[PIDPITCH] > PID_I_LIMIT) {
    errorSum[PIDPITCH] = PID_I_LIMIT;
  } else if (errorSum[PIDPITCH] < -PID_I_LIMIT) {
    errorSum[PIDPITCH] = -PID_I_LIMIT;
  }

  pid[PIDPITCH] = config.Kp[PIDPITCH]*error
                + config.Kd[PIDPITCH]*errorDiff
                + errorSum[PIDPITCH];

  // ROLL PID
  error = roll;
  errorDiff = rollRate;//(error - lastError[PIDROLL]); // rollRate
  lastError[PIDROLL] = error;
  errorSum[PIDROLL] += error * config.Ki[PIDROLL];

  if(errorSum[PIDROLL] > PID_I_LIMIT) {
    errorSum[PIDROLL] = PID_I_LIMIT;
  } else if (errorSum[PIDROLL] < -PID_I_LIMIT) {
    errorSum[PIDROLL] = -PID_I_LIMIT;
  }

  pid[PIDROLL] = config.Kp[PIDROLL]*error
                + config.Kd[PIDROLL]*errorDiff
                + errorSum[PIDROLL];

  /*         [0 1]
   *           ^ x              0   1
   *       y   |                 \ /       frente:   pitch/rate > 0
   * [0 3] <---+--- [1 2]         X        tras:     pitch/rate < 0
   *           |                 / \       esquerda: roll/rate  > 0
   *         [2 3]              3   2      direita:  roll/rate  < 0
   */

  if(rc[THROTTLE] > 1100) {
    throttle = (float)(rc[THROTTLE] - 1000) / 10.0;
    mtr[0] = throttle + pid[PIDPITCH];// + pid[PIDROLL];
    mtr[1] = throttle + pid[PIDPITCH];// - pid[PIDROLL];
    mtr[2] = .95*throttle - pid[PIDPITCH];// - pid[PIDROLL];
    mtr[3] = throttle - pid[PIDPITCH];// + pid[PIDROLL];
  } else {
    mtr[0] = 0.0;
    mtr[1] = 0.0;
    mtr[2] = 0.0;
    mtr[3] = 0.0;
  }


  for (i = 0; i < 4; i++) {
    if(mtr[i] < 0.0) {
      mtr[i] = 0.0;
    } else if (mtr[i] > 100.0) {
      mtr[i] = 100.0;
    }
  }

}


int SendPID() {
  uint64_t startTime;
  const uint64_t timeout = 50000; // 50 ms
  int16_t * tmp = (int16_t *)(buffer+1);

  rf24StopListening();

  buffer[0] = 'P';
  tmp[0] = (int16_t)(config.Kp[PIDPITCH] * 100.0);
  tmp[1] = (int16_t)(config.Kd[PIDPITCH] * 100.0);
  tmp[2] = (int16_t)(config.Ki[PIDPITCH] * 10000.0);

  rf24Send(buffer, PAYLOAD_SIZE);

  startTime = micros();

  while ((rf24Busy() == RF_SENDING) && (micros() - startTime < timeout))
    DelayMs(1);

  return rf24Busy();
}

void SendState() {
  int16_t * tmp = (int16_t *)(buffer+1);

  rf24StopListening();

  buffer[0] = 'S';
  tmp[0] = (int16_t)(pitch * 100.0);
  tmp[1] = (int16_t)(roll * 100.0);
  tmp[2] = (int16_t)(yaw * 100.0);
  tmp[3] = (int16_t)(altitude * 100.0);

  rf24Send(buffer, PAYLOAD_SIZE);
}

void SendRawData() {
  int16_t * tmp = (int16_t *)(buffer+1);

  rf24StopListening();

  buffer[0] = 'R';

  tmp[0] = (int16_t)(gyroValue[0] * 100.0);
  tmp[1] = (int16_t)(gyroValue[1] * 100.0);
  tmp[2] = (int16_t)(gyroValue[2] * 100.0);
  tmp[3] = (int16_t)(accValue[0]  * 1000.0);
  tmp[4] = (int16_t)(accValue[1]  * 1000.0);
  tmp[5] = (int16_t)(accValue[2]  * 1000.0);
  tmp[6] = (int16_t)(rc[0]);
  tmp[7] = (int16_t)(rc[1]);
  tmp[8] = (int16_t)(rc[3]);

  tmp[9] = (int16_t)(pitch * 100.0);
  tmp[10] = (int16_t)(roll * 100.0);
  tmp[11] = (int16_t)(yaw * 100.0);
  tmp[12] = (int16_t)(altitude * 100.0);

  rf24Send(buffer, PAYLOAD_SIZE);
}


//*****************************************************************************
//
// Main function
//
//*****************************************************************************
void main(void) {
  int flags, buttons;

  //
  // Enable stacking for interrupt handlers.  This allows floating-point
  // instructions to be used within interrupt handlers, but at the expense of
  // extra stack usage.
  //
  FPUEnable();
  FPULazyStackingEnable();

  //
  // Set the clocking to run directly from the external crystal/oscillator.
  //
  SysCtlClockSet(
      SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  //
  // Enable interrupts to the processor.
  //
  IntMasterEnable();

  //
  // Initialize RGB leds
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED);
  ledOn(GREEN_LED);
  ledOff(RED_LED | BLUE_LED);

  //
  // Initialize buttons
  //
  ButtonsInit();

  //
  // Init delay library
  //
  DelayInit();

  //
  // Set up the serial console to use for displaying messages.
  //
  InitConsole();

#ifdef SERIAL_DEBUG
  UARTprintf("Console initialized.\n");
  UARTprintf("Reading EEPROM...\n");
#endif

  //
  // Initialize eeprom and load configuration
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
  EEPROMInit();
  ReadConfig();

#ifdef SERIAL_DEBUG
  UARTprintf("Initializing motors...\n");
#endif

  //
  // Set up motors
  //
  mtr[0] = mtr[1] = mtr[2] = mtr[3] = 0.0;
  MotorInit();
  MotorSetFrequency(MOTOR_FREQUENCY);
  MotorSetDutyCycle(mtr);
  MotorEnable();

#ifdef SERIAL_DEBUG
  UARTprintf("Initializing RC receiver...\n");
#endif

  //
  // Set up RC receiver
  //
  RCInit();
#ifdef ESC_CALIBRATION
  DelayMs(50);
  RCGetValues(rc);
  if (rc[THROTTLE] > 1500) {
    MotorSetMicroseconds(2000);
    DelayMs(2000);
    MotorSetMicroseconds(1000);
    ledOff(GREEN_LED);
    while (1) {
      ledOn(BLUE_LED);
      DelayMs(500);
      ledOff(BLUE_LED);
      DelayMs(500);
    }
    exit(0);
  }
#endif


#ifdef DEBUG
  UARTprintf("Initializing I2C and SPI communications...\n");
#endif

  //
  // Set up I2C and SPI communications
  //
  I2CInit();
  SPIInit();

#ifdef SERIAL_DEBUG
  UARTprintf("Initializing sensors...\n");
#endif

  //
  // Set up sensors
  //
  buttons = readButton(ALL_BUTTONS);
  flags = GYRO_CALIBRATE;

  if (config.defaults == true) {
    flags |= ACC_CALIBRATE | MAG_CALIBRATE;
  } else {
#ifdef SERIAL_DEBUG
    UARTprintf("\tLoading ACC+MAG calibration.\n");
#endif
    SensorsSetCalibration(config.accZero, config.magZero);
  }

  if (~buttons & ALL_BUTTONS) {
    config.defaults = true; // forces a WriteConfig()
    if (~buttons & RIGHT_BUTTON)
      flags |= ACC_CALIBRATE;
    if (~buttons & LEFT_BUTTON)
      flags |= MAG_CALIBRATE;
  }

#ifdef SERIAL_DEBUG
  UARTprintf("\tCalibrating: ");
  if (flags & GYRO_CALIBRATE)
    UARTprintf("GYRO ");
  if (flags & ACC_CALIBRATE)
    UARTprintf("ACC ");
  if (flags & MAG_CALIBRATE)
    UARTprintf("MAG ");
  UARTprintf("\n");
#endif

  SensorsInit(flags);

  if (config.defaults == true) {
#ifdef SERIAL_DEBUG
    UARTprintf("\tSaving ACC+MAG calibration.\n");
#endif
    SensorsGetCalibration(config.accZero, config.magZero);
    WriteConfig();
  }

#ifdef SERIAL_DEBUG
  UARTprintf("Initializing nRF24 transceiver...\n");
#endif

  //
  // Set up nRF24L01 transceiver
  //
  rf24Init();
  rf24Setup(addr);

  // Send the current PID parameters
  rf_state = SendPID();

  // Start listening
  rf24StartListening();


  ledOff(GREEN_LED);

#ifdef SERIAL_DEBUG
  UARTprintf("End of initializations.\n\n");
#endif

  // if bad I2C communication, blink the red led
  // if rf sent failed, blink the green led
  // continue if both buttons are pressed
  if (I2CGetErrorsCount() > 0 || rf_state != RF_SENT) {
    int leds = (I2CGetErrorsCount() > 0) ? RED_LED : 0;
    leds += (rf_state != RF_SENT) ? GREEN_LED : 0;
    while (readButton(ALL_BUTTONS)) {
      ledOn(leds);
      DelayMs(250);
      ledOff(leds);
      DelayMs(250);
    }
  }

  previousTime = uartTime = rcTime = micros();
  armed = false;
  rf_state = RF_LISTENING;

  /*
  int cycleCounter = 0;
  int rfTryCounter = 0;
  uint64_t lastTxCounter = 0;
  */

  // loop forever
  while (1) {

    currentTime = micros();
    //cycleTime = currentTime - previousTime;

    if (I2CGetErrorsCount() > 0)
      ledOn(RED_LED);

    if (rf_state == RF_SENDING && rf24Busy() != RF_SENDING) {
      rf24StartListening();
      rf_state = RF_LISTENING;
    }

    /*
    if (currentTime - previousTime >= SAMPLE_TIME_US) { // 400 Hz
      SensorsUpdate();
      AttitudeEstimation(&roll, &rollRate, &pitch, &pitchRate, &yaw, &altitude);
      previousTime = currentTime;
      cycleCounter++;

    }

    if (currentTime - rcTime >= MOTOR_TIME_US) {
      if (rf_state != RF_SENDING) {
        SendRawData();
        rf_state = RF_SENDING;
        rfTryCounter++;
      }
      rcTime = currentTime;
    }

    if(currentTime - uartTime >= 4000000) {
      unsigned long rfSentCounter = rf24GetTxCounter() - lastTxCounter;
      lastTxCounter = rf24GetTxCounter();
      unsigned long meanCycleTime = (unsigned long)(4000000 / cycleCounter);
      unsigned long meanCycleRate = (unsigned long)(cycleCounter >> 2);
      unsigned long meanSendTime = (unsigned long)(4000000 / rfSentCounter);
      unsigned long meanSendRate = (unsigned long)(rfSentCounter >> 2);
      unsigned long failRate = 1000 - rfSentCounter * 1000 / rfTryCounter;
      UARTprintf("Cycle: time = %d us, rate = %d Hz\n", meanCycleTime, meanCycleRate);
      UARTprintf("Send: time = %d us, rate = %d Hz, fail = %d %%o\n", meanSendTime, meanSendRate, failRate);
      cycleCounter = 0;
      rfTryCounter = 0;
      uartTime = currentTime;
    }
    */

    if (rf24Available()) {
      rf24Receive(buffer, PAYLOAD_SIZE);
#ifdef SERIAL_DEBUG
      UARTprintf("Message received.\n");
      UARTprintf("\tBuffer = [%d, %d, %d, %d, %u, %u]\n", ((int16_t *)buffer)[0],
          ((int16_t *)buffer)[1], ((int16_t *)buffer)[2], ((int16_t *)buffer)[3],
          buffer[8], buffer[9]);
#endif
      config.Kp[PIDROLL] = ((int16_t *)buffer)[0] / 100.0;
      config.Kd[PIDROLL] = ((int16_t *)buffer)[1] / 100.0;
      config.Ki[PIDROLL] = ((int16_t *)buffer)[2] / 10000.0;
      config.Kp[PIDPITCH] = ((int16_t *)buffer)[0] / 100.0;
      config.Kd[PIDPITCH] = ((int16_t *)buffer)[1] / 100.0;
      config.Ki[PIDPITCH] = ((int16_t *)buffer)[2] / 10000.0;
#ifdef SERIAL_DEBUG
      snprintf(str[0], 10, "%2.3f", config.Kp[PIDPITCH]);
      snprintf(str[1], 10, "%2.3f", config.Kd[PIDPITCH]);
      snprintf(str[2], 10, "%2.3f", config.Ki[PIDPITCH]);
      UARTprintf("\tPID ROLL+PITCH: Kp = %s, Kd = %s, Ki = %s\n", str[0], str[1], str[2]);
#endif
      WriteConfig();
    }


    if (currentTime - previousTime >= SAMPLE_TIME_US) { // 400 Hz
      SensorsUpdate();
      AttitudeEstimation(&pitch, &pitchRate, &roll, &rollRate, &yaw, &altitude);
      previousTime = currentTime;
    }

    if (currentTime - rcTime >= MOTOR_TIME_US) {
      RCGetValues(rc);

      if (rc[THROTTLE] < 1200) {
        if (rc[YAW] < 1200) {  // disarm command
          armed = false;
          MotorSetMicroseconds(1000);
          //MotorDisable();
          ledOff(BLUE_LED);
        } else if (rc[YAW] > 1800) { // arm command
          armed = true;
          //MotorEnable();
          PIDInit();
          ledOn(BLUE_LED);
        }
      }

      if (armed) {
        AttitudeControl();
        MotorSetDutyCycle(mtr);
#ifdef RADIO_LOG
        if (rf_state != RF_SENDING && currentTime - uartTime >= LOG_TIME_US) {
          //SendState();
          SendRawData();
          rf_state = RF_SENDING;
          uartTime = currentTime;
        }
#endif
      }
      rcTime = currentTime;
    }

#ifdef SERIAL_DEBUG
    if (currentTime - uartTime >= 500000) { // 5 Hz


      unsigned long loadValue = TimerLoadGet(WTIMER2_BASE, TIMER_A);
      unsigned long matchValue = loadValue - (unsigned long) ((mtr[0] / 100.0 + 1.0) * SysCtlClockGet() / 1000);

      snprintf(str[0], 10, "%5.2f", roll);
      snprintf(str[1], 10, "%5.2f", pitch);
      snprintf(str[2], 10, "%5.2f", yaw);
      snprintf(str[3], 10, "%5.2f", altitude);
      snprintf(str[4], 10, "%5.2f", mtr[0]);
      snprintf(str[5], 10, "%5.2f", mtr[1]);
      snprintf(str[6], 10, "%5.2f", mtr[2]);
      snprintf(str[7], 10, "%5.2f", mtr[3]);

      UARTprintf("Armed: %d\n", armed);
      UARTprintf("Attitude: x=%s deg  , y=%s deg, z=%s deg\n", str[0], str[1],
          str[2]);
      UARTprintf("Altitude: %s m\n", str[3]);
      UARTprintf("RC in: roll:%5d, pitch: %5d, yaw:%5d, thr:%5d\n", rc[0], rc[1], rc[2],
          rc[3]);
      UARTprintf("Motors out: 1:%s%%, 2:%s%%, 3:%s%%, 4:%s%%\n", str[4], str[5], str[6],
          str[7]);
      UARTprintf("RF tx: %u\n", (unsigned long)rf24GetTxCounter());
      UARTprintf("\n");

      uartTime = currentTime;
    }
#endif


  }

}

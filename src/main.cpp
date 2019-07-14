//#define USBCON 1
//#define USBD_USE_CDC 1

#include <Arduino.h>
#include "Weather.h"
#include <SFE_BMP180.h>

// Includes enegyShield2 library files
#include <NS_energyShield2.h>


// Defines NS_energyShield2 class object "Es2"
NS_energyShield2 Es2;
SFE_BMP180 Bmp180;
TwoWire Wire3(PIN_A4, PIN_A5);
//void SystemClock_Config(void);

HardwareSerial Esp8266(USART1);

double baseline;

// Blinks LED
void blink()
{
  digitalWrite(13, HIGH);
  delay(166);
  digitalWrite(13, LOW);
  delay(166);
}

void EspReset()
{
  pinMode(PA10, OUTPUT);    // reset pin
  pinMode(PA8, OUTPUT);     // esp enable pin
  
  digitalWrite(PA10, HIGH); // reset pin high
  digitalWrite(PA8, HIGH); // reset pin high
  delay(100);

  // digitalWrite(PA8, LOW);   // ESP Ena low

  digitalWrite(PA10, HIGH); // reset pin high
  delay(300);
  digitalWrite(PA10, LOW); // reset pin low
  delay(300);
  digitalWrite(PA10, HIGH); // reset pin high

}
void setup()
{
  // char buffer[80];
 // Initialize Serial
  Serial.begin(9600);
  while (!Serial); 

  Esp8266.begin(115200);
  
  // SystemClock_Config();
 /* Initialize all configured peripherals */
  MX_I2C1_Init();
  MX_I2C3_Init();

  Wire3.begin();

  // put your setup code here, to run once:
  
  Es2.begin(&Wire3); // Initialize energyShield

  if (Es2.readVMPP() != -1)
    Es2.setVMPP(-1, 1); // Disable VMPP regulation to allow charging from any source (7V - 23V) and prevent excessive EEPROM writes

  pinMode(13, OUTPUT); // default LED

  if(Bmp180.begin(&Wire3))
    Serial.println("BMP180  Init SUCCESS!");
  else
    Serial.println("BMP180  Init FAILED!");

  baseline = getPressure();

  Serial.print("BMP180 baseline pressure ");
  Serial.print(baseline,2);
  Serial.print(" mBars ");
  float hG = (29.92 * baseline) / 1013.25;
  Serial.print(hG, 2);
  Serial.println(" Hg");

  EspReset();

  // Print header
  Serial.println("Voltage, V\tCurrent, A\tFull Capacity, mAh\tRemaining Capacity, mAh\tState of Charge, %\tInput Voltage, V\tTemperature, C\tTime, HH:MM:SS\tDate, DD/MM/YY\tTemp");
}

void loop()
{
  // Blinks three times
  blink();
  blink();
  blink();

  // Read values from energyShield2
  float batteryVoltage = (float)Es2.batteryVoltage() / 1000.0;
  float batteryCurrent = (float)Es2.batteryCurrent() / 1000.0;
  int fullCapacity = Es2.fullChargeCapacity();
  int remainingCapacity = Es2.remainingCapacity();
  int stateOfCharge = Es2.SOC();
  float inputVoltage = (float)Es2.inputVoltage() / 1000;
  float temperature = (float)Es2.temperature() / 10;

  // Print Results
  Serial.print(batteryVoltage, 3);
  Serial.print("V    \t");
  Serial.print(batteryCurrent, 3);
  Serial.print("A   \t");
  Serial.print(fullCapacity);
  Serial.print("mAh             \t");
  Serial.print(remainingCapacity);
  Serial.print("mAh              \t");
  Serial.print(stateOfCharge);
  Serial.print("%              \t");
  Serial.print(inputVoltage, 2);
  Serial.print("V            \t");
  Serial.print(temperature, 1);
  Serial.print("C       \t");

  // Read time and date from energyShield and store locally
  // Local values will not update until readClock is called again
  Es2.readClock();

  // Print time and date from locally stored values
  Serial.print(Es2.hour());
  Serial.print(":");
  if (Es2.minute() < 10)
    Serial.print("0");
  Serial.print(Es2.minute());
  Serial.print(":");
  if (Es2.second() < 10)
    Serial.print("0");
  Serial.print(Es2.second());
  Serial.print("     \t");
  Serial.print(Es2.month());
  Serial.print("/");
  Serial.print(Es2.dayOfMonth());
  Serial.print("/");
  Serial.print(Es2.year());

  double tempC = getTemperature();
  double tempF = 1.8 * tempC + 32;

  Serial.print("\t");
  Serial.print(tempF, 1);
  Serial.print(" F");

  // Print carriage return to start new line
  Serial.println();

  


  // Wait between reads
  delay(2000);
  char ch = 0;  
  Serial.print("Weather> ");
  while(ch != 'Q') 
  {
    while(Esp8266.available())
    {
      Serial.write(Esp8266.read());
    }
    while(Serial.available())
    {
      ch = Serial.read();
      Serial.write(ch);
      Esp8266.write(ch);
    }
  }
  Serial.println(" ");
  // Sleeps power for 4 seconds
  // Es2.sleepSeconds(4);
}


double getPressure()
{
  char status;
  double T,P;   // ,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = Bmp180.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = Bmp180.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = Bmp180.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = Bmp180.getPressure(P,T);
        if (status != 0)
        {
          return P;
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  return P;
}

double getTemperature()
{
  char status; 
  double retVal;

  status = Bmp180.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = Bmp180.getTemperature(retVal);
    if(status == 0)
    {
      Serial.print("BMP180(272): Error reading temperature");
      Serial.println(Bmp180.getError());
    }
  }
  else
  {
    Serial.print("BMP180(278): Error starting temperature");
    Serial.println(Bmp180.getError());
  }
  return retVal;
}
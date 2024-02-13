///////////////////////
// Includes and Defines
///////////////////////

#include <ArduinoJson.h> // JSON parsing library
#include "bsec.h"         // Air Quality sensor library
#include Wire.h           // Required for interfacing with I²C devices
#include hp_BH1750.h      // Light sensor library

#define WDT_KEY (0xA5)
#define BAUD_RATE 115200
#define NodeMCU_Serial Serial1

// Timers and Intervals
// Define timers and intervals for specific events and tasks

// Store the timestamp of the latest execution for each task
unsigned long previousMillis_LUX = 0;
unsigned long previousMillis_Air = 0;
unsigned long previousMillis_TDS = 0;
unsigned long previousMillis_EC = 0;
unsigned long previousMillis_Wlevel = 0;
unsigned long previousMillis_Pump_Dur = 0;
unsigned long previousSending = 0;

// Declare the interval times for each task
const unsigned long eventTime_Air = 1000; // interval in ms
const unsigned long eventTime_LUX = 60000; // interval in ms
const unsigned long eventTime_TDS = 5000; // interval in ms
const unsigned long eventTime_EC = 10000; // interval in ms
const unsigned long eventTime_Wlevel = 60000; // interval in ms
const unsigned long eventTime_Pump_Dur = 2000; // interval in ms
const unsigned long eventTime_Sending = 30000; // interval in ms

/////////////////////////////
// Pump, Fan and Led strip Setup
/////////////////////////////
const int pumpPin = 9;   // Pin connected to the pump relay on Arduino Due
int pumpStatus;
int pumpStatus2;
unsigned long pumpActivationInterval = 30 * 60 * 1000; // 30 minutes in milliseconds
unsigned long pumpOnDuration = 2;               // Initial duration: 2 minutes in milliseconds
unsigned long lastPumpActivationTime = 0;

const int fan = 14;
int fanStatus_real = 0;

const int lightpin = 10; // Pin connected to the light relay on Arduino Due
int light_status = 0;

////////////////////////
// Roots Moisture Setup
////////////////////////
const int moisturePin1 = A1; // Replace with the actual pin for sensor 1
const int moisturePin2 = A2; // Replace with the actual pin for sensor 2
const int dryThreshold = 680;
const int wetThreshold = 300;
unsigned long moistureReadInterval = 30000;
unsigned long previousMoistureReadMillis = 0;
unsigned int cycleCount = 1;
bool isWet = false;
String fullMoistureOutput = "Zero Cycles";
int mappedMoisture;
int cycle_Duration;

/////////////////////
// Water Sensor Setup
/////////////////////
const int WlevelSensor = 0;
int waterLevelState = 0; // water level reading

/////////////////////
// TDS and EC Sensor Setup
/////////////////////
#define TdsSensorPin A0 
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  10           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;
float ecValue = 0;

///////////////////////
// BME680 and Light Sensors Setup
///////////////////////
float lux;
hp_BH1750 BH1750;

float temp;
float Hum;
float Co2;
float Pr;
float VOC;
float IAQ;

// Create an object of the class Bsec
Bsec iaqSensor;

String output;

// Functions
void bme680Initialize(void);
void checkIaqSensorStatus(void);
void errLeds(void);
void watchdogSetup(void) {/*** watchdogDisable (); ***/}

float round1(float value) {
    return (int)(value * 10 + 0.5) / 10.0;
}


/**
 * @brief Sets up the environment for executing the Greenhouse control system
 *
 * This function sets up various components and peripherals used throughout the project, including serial communication, watchdog timer, pins, sensors, and other modules.
 */
void setup(void) {
  // Initialize serial communication
  Serial.begin(BAUD_RATE); // Start the primary serial communication with the specified baud rate
  delay(1000);            // Add a delay for stability reasons before starting secondary serial communication
  NodeMCU_Serial.begin(9600); // Start the secondary serial communication with a lower baud rate

  // Enable watchdog reset with 1 minute timeout
  WDT->WDT_MR = WDT_MR_WDD(0xFFF) | WDT_MR_WDRPROC | WDT_MR_WDRSTEN | WDT_MR_WDV(256 * 60);

  // Initialize pins
  // Set pumpPin, lightpin, and fan pins as outputs and write their default state as LOW
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW);

  pinMode(lightpin, OUTPUT);
  digitalWrite(lightpin, LOW);

  pinMode(fan, OUTPUT);

  // Initialize BH1750 light meter
  bool avail = BH1750.begin(BH1750_TO_GROUND);
  BH1750.calibrateTiming();
  BH1750.start(BH1750_QUALITY_HIGH2, BH1750_MTREG_DEFAULT);
  if (!avail) {
    Serial.println("No BH1750 sensor found!");
    while (true) {};
  }

  // Initialize water level sensor
  pinMode(WlevelSensor, INPUT_PULLUP);
  waterLevelState = 0;

  // Initialize BME680
  pinMode(LED_BUILTIN, OUTPUT);
  bme680Initialize();
  checkIaqSensorStatus();
  Wire.begin();
}

void loop(void) {

//Reset the watchdog timer - if this didn't happen it means the while loop func. is stopped and the watchdog timer will reset the mcu
WDT->WDT_CR = WDT_CR_KEY(WDT_KEY) | WDT_CR_WDRSTT;

//BME680
// Check if the time difference between the current and previous iteration exceeds the eventTime_Air threshold
if ((unsigned long)(millis() - previousMillis_Air) > eventTime_Air) {
  unsigned long time_trigger = millis();

  // Run the BME680 sensor and collect data if new data is available
  if (iaqSensor.run()) {
    digitalWrite(LED_BUILTIN, LOW);

    // Collect all measured data and format it as a string
    output = String(time_trigger);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.stabStatus);
    output += ", " + String(iaqSensor.runInStatus);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.gasPercentage);

    // Display the collected data
    Serial.println(output);
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    checkIaqSensorStatus();
  }

  // Extract individual measurements from the BME680 sensor
  Serial.print("Pressure: ");
  Serial.print(iaqSensor.pressure/100.0);
  Serial.println(" hPa");
  Pr = iaqSensor.pressure/100.0;

  Serial.print("Temperature: ");
  Serial.print(iaqSensor.temperature);
  Serial.println(" *C");
  temp = iaqSensor.temperature;

  Serial.print("Humidity: ");
  Serial.print(iaqSensor.humidity);
  Serial.println(" %");
  Hum = iaqSensor.humidity;

  Serial.print("IAQ: ");
  Serial.print(iaqSensor.staticIaq);
  Serial.println();
  IAQ = iaqSensor.staticIaq;

  Serial.print("CO2 Equivalent: ");
  Serial.print(iaqSensor.co2Equivalent);
  Serial.println(" PPM");
  Co2 = iaqSensor.co2Equivalent;

  Serial.print("Breath VOC Equivalent: ");
  Serial.print(iaqSensor.breathVocEquivalent);
  Serial.println();
  VOC=iaqSensor.breathVocEquivalent;

  // Prepare for the next iteration
  previousMillis_Air = millis();
  }

// starts a measurement of lux sensor
if ((unsigned long)(millis() - previousMillis_LUX) > eventTime_LUX) {
  if (BH1750.hasValue()) {    // Check if there is a valid result from the previous measurement
    lux = BH1750.getLux();   // read the result
    BH1750.start(BH1_QUALITY_HIGH2, BH1750_MTREG_DEFAULT); // only start the next measurement after getting the result
  }
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  previousMillis_LUX = millis();
  }

// Measure TDS value when enough time has passed and water level is detected
if ((unsigned long)((millis() - previousMillis_TDS) > eventTime_TDS) && waterLevelState == 1) {
  TDSsensorread();
  previousMillis_TDS = millis();
  }

// Measure EC value when enough time has passed and water level is detected
if ((unsigned long)((millis() - previousMillis_EC) > eventTime_EC) && waterLevelState == 1) {
  ECsensorread();
  previousMillis_EC = millis();
  }

// Periodically send JSON document containing sensor data
if ((unsigned long)(millis() - previousSending) > eventTime_Sending) {
  previousSending = millis();

  StaticJsonDocument<350> doc;
  doc["LUX"] = round1(lux);
  doc["tds"] = round1(tdsValue);
  doc["ec"] = round1(ecValue);
  doc["temp"] = round1(temp);
  doc["Hum"] = round1(Hum);
  doc["Pr"] = round1(Pr);
  doc["Co2"] = round1(Co2);
  doc["IAQ"] = round1(IAQ);
  doc["VOC"] = round1(VOC);
  doc["IAQ_AC"]= iaqSensor.iaqAccuracy;
  doc["waterLevelState"] = waterLevelState;
  doc["fullMoistureOutput"] = fullMoistureOutput;
  doc["rootsMoisturePercent"] = mappedMoisture;
  doc["pumpStatus2"] = pumpStatus2;
  doc["cycle_Duration"] = cycle_Duration;

  serializeJson(doc, NodeMCU_Serial);
  previousSending = millis();
  }

// water level sensor and fan and light management
if ((unsigned long)(millis() - previousMillis_Wlevel) > eventTime_Wlevel) {
  // Reset the water level flag and read its state
  waterLevelState = 0;
  waterLevelState = digitalRead(WlevelSensor);
  Serial.print("waterLevelState ");
  Serial.print(waterLevelState);
  Serial.println(" Yes ?");

  // Show the current fan and light state
  Serial.print("Fan State ");
  Serial.println(fanStatus_real);
  Serial.print("light State ");
  Serial.println(light_status);

  // Control the fan according to fanStatus_real value
  if (fanStatus_real == 1) {
    digitalWrite(fan, HIGH);
  } else if (fanStatus_real == 0) {
    digitalWrite(fan, LOW);
  }

  // Control the light according to lux and light_status values
  if (lux <= 5 || light_status == 1) {
    digitalWrite(lightpin, HIGH);
  } else if (lux > 5 || light_status == 0) {
    digitalWrite(lightpin, LOW);
  }

  // Save the current timestamp for future comparisons
  previousMillis_Wlevel = millis();
  }

// Code for the pump
unsigned long currentTimePump = millis();

// Check if the pump activation duration has passed
if (currentTimePump - lastPumpActivationTime >= int(pumpOnDuration) * 60 * 1000) {
  digitalWrite(pumpPin, LOW);
  //Serial.println("Pump deactivated");
  pumpStatus2 = 0;
}

// Manage the pump based on water level and wetness condition
if (waterLevelState == 1 && isWet == false) {
    // Check if at least 30 minutes have passed since the last pump activation
  if (currentTimePump - lastPumpActivationTime >= pumpActivationInterval) {
    // Activate the pump and set the new duration (if provided)
    digitalWrite(pumpPin, HIGH);
    lastPumpActivationTime = currentTimePump;
    Serial.println("Pump activated");
    pumpStatus2 = 1;
  }
} else if (waterLevelState == 0 || isWet == true) {
  digitalWrite(pumpPin, LOW);
  pumpStatus2 = 0;
}

// Update pump duration and other settings received from NodeMCU_Serial
if ((unsigned long)(millis() - previousMillis_Pump_Dur) > eventTime_Pump_Dur) {
  unsigned long currentMillis = millis();

  if (NodeMCU_Serial.available()) {
    StaticJsonDocument<130> pump_on_dur;
    DeserializationError err = deserializeJson(pump_on_dur, NodeMCU_Serial);

    if (err == DeserializationError::Ok) {
      pumpOnDuration = pump_on_dur["pumpOnDuration"].as<int>();
      pumpStatus = pump_on_dur["pumpStatus"].as<int>();
      fanStatus_real = pump_on_dur["fanStatus_real"].as<int>();
      light_status = pump_on_dur["light_status"].as<int>();

      Serial.print("Pump duration ");
      Serial.print(pumpOnDuration);
      Serial.println(" mins");
      Serial.print("pumpStatus ");
      Serial.print(pumpStatus);
      Serial.print("fanStatus ");
      Serial.print(fanStatus_real);
    } else { // Print error to the "debug" serial port
      Serial.print("deserializeJson() returned ");
      Serial.println(err.c_str());
    }

    // Clear the input buffer
    while (NodeMCU_Serial.available() > 0) {
      NodeMCU_Serial.read();
    }
  }

  previousMillis_Pump_Dur = millis();
  }


// Roots moisture sensors
unsigned long currentMillis = millis();

// Check if the moisture read interval has passed
if (currentMillis - previousMoistureReadMillis >= moistureReadInterval) {
  previousMoistureReadMillis = currentMillis;

  // Read sensor values
  int moistureValue1 = analogRead(moisturePin1);
  //int moistureValue2 = analogRead(moisturePin2);

  // Optional: Adjust extreme sensor values
  // if (moistureValue1 < 250) {
  //   moistureValue1 = 310;
  // } else if (moistureValue1 > 760) {
  //   moistureValue1 = 680;
  // }

  // // Only apply adjustment if sensor 2 is malfunctioning
  // if (moistureValue2 < 250) {
  //   moistureValue2 = 310;
  // } else if (moistureValue2 > 760) {
  //   moistureValue2 = 680;
  // }

  // Optionally calculate the average moisture value from the two sensors
  // int avgMoisture = (moistureValue1 + moistureValue2) / 2;
  
  // Convert the moisture value to a mapped percentage
  mappedMoisture = map(moistureValue1, dryThreshold, wetThreshold, 0, 100);

  // Ensure the mapped value stays within a reasonable range
  if (mappedMoisture < 0) {
    mappedMoisture = 0;
  } else if (mappedMoisture > 100) {
    mappedMoisture = 100;
  }

  // Log sensor values and mapped moisture percentage
  Serial.println("Sensor 1 Analog Output: " + String(moistureValue1));
  // Serial.println("Sensor 2 Analog Output: " + String(moistureValue2));
  Serial.println("Average Percentage of Moisture: " + String(mappedMoisture));

  // Track wet vs dry cycles
  if (moistureValue1 < wetThreshold && !isWet) {
    isWet = true;
    startTime = now();
  } else if (moistureValue1 > dryThreshold && isWet) {
    isWet = false;
    endTime = now();

    // Calculate and log the duration of the drying cycle
    cycle_Duration = (endTime - startTime) / 60;
    fullMoistureOutput = "Cycle: " + String(cycleCount) + ", Duration of roots to dry: " + String(cycle_Duration) + " mins";
    Serial.println(fullMoistureOutput);

    // Increase the cycle counter
    cycleCount++;
    }
  }
}


/**
 * @brief Reads TDS value from a conductivity sensor
 *
 * This function samples the TDS sensor periodically and prints the TDS value in parts per million (ppm). 
 * It uses a moving window buffer to smooth out fluctuations caused by noise. Temperature compensation is applied 
 * to increase the precision of the results.
 */
///
void TDSsensorread() {
  static unsigned long analogSampleTimepoint = millis();

  // Every 40 milliseconds, read the analog value from the ADC
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    // read the analog value and store into the buffer
    analogBufferIndex++;

    // Loop back to the beginning of the buffer when full
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();

  // Print the TDS value every 800 milliseconds
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();

    // Copy the contents of the circular buffer
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }

    // Calculate the average voltage using the median value
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

    // Apply temperature compensation
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVolatge = averageVoltage / compensationCoefficient;

    // Convert the voltage to TDS value
    tdsValue = (133.42 * pow(compensationVolatge, 3) - 255.86 * pow(compensationVolatge, 2) + 857.39 * compensationVolatge) * 0.5;

    // Print the calculated TDS value
    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Serial.println(" ppm");
  }
}

/**
 * @brief Computes the median value of an integer array
 *
 * Given an integer array and its length, this function sorts the array and returns the middle element as the median.
 * For even lengths, it takes the mean of the two central elements.
 *
 * @param bArray Pointer to the integer array
 * @param iFilterLen Length of the array
 *
 * @returns The median value of the sorted array
 */
 ///
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];

  // Make a local copy of the original array
  for (byte i = 0; i<iFilterLen; i++)
    bTab[i] = bArray[i];

  int i, j, bTemp;

  // Sort the copied array in ascending order
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }

  // Find the middle element in the sorted array
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;

  // Return the computed median value
  return bTemp;
}

void ECsensorread(){
  ecValue = tdsValue/0.55;
  Serial.print("EC Value: ");
  Serial.print(ecValue,0);
  Serial.println(" μS/cm");
}

/**
 * @brief Configures and initializes the BME680 environmental sensor
 *
 * This function initializes the BME680 sensor, configures virtual sensors, subscription rates, and displays the BSEC library version.
 * After successful initialization, it prints headers indicating columns for exported data.
 */
 ///
void bme680Initialize(void) {
  iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);

  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);

  bsec_virtual_sensor_t sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };

  iaqSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);

  // Print the header
  output = "Timestamp [ms], IAQ, IAQ accuracy, Static IAQ, CO2 equivalent, breath VOC equivalent, raw temp[°C], pressure [hPa], raw relative humidity [%], gas [Ohm], Stab Status, run in status, comp temp[°C], comp humidity [%], gas percentage";
  Serial.println(output);
}

/**
 * @brief Verifies the status of the BSEC and BME68x libraries
 *
 * This function checks for errors or warnings reported by the BSEC and BME68x libraries.
 * Upon detection of critical issues, it initiates a soft reset procedure for both the libraries.
 * Non-critical issues yield informational messages and no action is taken beyond logging the message.
 */
 ///
void checkIaqSensorStatus(void) {
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
      errLeds();

      // Soft reset for BSEC library
      Wire.beginTransmission(BME68X_I2C_ADDR_LOW);
      Wire.write(0x60);
      Wire.write(0xB6);
      Wire.endTransmission();
      delay(10);

      bme680Initialize();
      for (;;)
        errLeds(); // Halt in case of failure
    } else {
      output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
      errLeds();

      // Soft reset for BME68x library
      Wire.beginTransmission(BME68X_I2C_ADDR_LOW);
      Wire.write(0x60);
      Wire.write(0xB6);
      Wire.endTransmission();
      delay(10);

      bme680Initialize();
      for (;;)
        errLeds(); // Halt in case of failure
    } else {
      output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

// float calculateAverage(float sensorData) {
//   int readings[5]; // Array to store the last 5 readings
//   int count = 0; // Variable to keep track of the number of non-zero readings
//   int index = 0; // Index to keep track of the current position in the array

//   // Iterate through the sensor readings and filter out zeros
//   while (count < 5) {
//     if (sensorData != 0) {
//       readings[index] = sensorData;
//       count++;
//       index = (index + 1) % 5; // Update the index, wrapping around if necessary
//     }
//     // The variable sensorData will update itself automatically (every second, as you mentioned)
//     // No need for a separate function to get the next sensor reading
//   }

//   // Calculate the average of non-zero readings
//   int sum = 0;
//   for (int i = 0; i < 5; i++) {
//     sum += readings[i];
//   }
//   return (float)sum / 5;
// }

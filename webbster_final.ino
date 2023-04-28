#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TiCoServo.h>
#include <Encoder.h>
#include <AltSoftSerial.h>
#include <TFMini.h>
#include <NewPing.h>
#include <TimerFreeTone.h>
#include <EEPROM.h>
#include <utility/imumaths.h>
#include <avr/pgmspace.h>
#include <TimerOne.h>


enum State {
  IDLE,
  TOOL_SELECT,
  SENSOR_SELECT,
  SENSOR_CHANGE,
  SENSOR_SAVE,
  TOOL_SAVE,
  TOOL_SAVE_CONFIRMED,
  TOOL_LOAD,
  TOOL_LOAD_CONFIRMED
};

enum Command {
  UNKNOWN,
  TEST_ENCODA,
  TEST_ENCODB,
  TEST_ULTRA,
  TEST_TFMINI,
  TEST_NINEAXIS,
  TEST_MIC,
  TEST_NEOSTICK,
  TEST_SERVO,
  TEST_OUTPUT
};


State currentState = IDLE;


//Progmem neopixels colors
const uint8_t neopixelColors[8][3] PROGMEM = {
  {0, 0, 0},         // Black
  {255, 0, 0},      // Red
  {0, 255, 0},      // Green
  {0, 0, 255},      // Blue
  {255, 255, 0},    // Yellow
  {255, 0, 255},    // Magenta
  {0, 255, 255},    // Cyan
  {255, 255, 255},  // White
};


//primary struct for tuning each sensor as instrument
struct Sensor {
  uint8_t outPin;
  unsigned int outValue;
  unsigned int outDuration;

  uint8_t volume = 50;
  unsigned int min = 0;
  unsigned int max = 1000;
  unsigned char mapIndex = 0;
  unsigned int pulse = 250;
};


//user inputs struct for managing encoder values
struct Inputs {
  bool enocdA_pressed;
  bool enocdA_held;
  unsigned char enocdA_currentButtonState;
  unsigned int enocdA_lastEncoderAButtonPress = 0;
  unsigned int enocdA_debounceDelay = 50;
  unsigned int encodA_pressedDelay = 3000;
  int enocdA__value;
  int encodA_position;

  bool enocdB_pressed;
  bool enocdB_held;
  unsigned char enocdB_currentButtonState;
  unsigned int enocdB_lastEncoderBButtonPress = 0;
  unsigned int enocdB_debounceDelay = 50;
  unsigned int encodB_pressedDelay = 3000;
  int enocdB__value;
  int encodB_position;

  byte sensorIndex;
  byte toolIndex;
  byte slot;
};


//servo, neopixels, toneOut, eeprom saves and loads
struct Actuators {
  unsigned int updateInterval = 100;
  unsigned int lastUpdate = 0;

  //Servo
  unsigned char targetAngle;
  unsigned char currentAngle;

  //Neopixel
  imu::Vector<3> targetValues[8];
  imu::Vector<3> currentValues[8];

  //Tone Output
  imu::Vector<3> tonePulseVelocityValues[8];
  unsigned int previousMillis[4];
  int arefValue;

};


//sanity check for process debugging
struct Processor {
  bool ok;
};


/////////////////////// Pin definitions///////////////////////////

//tfmini lidar
constexpr byte TFmini_TX = 13;
constexpr byte TFmini_RX = 5;

//Microphone
constexpr byte microphonePin = A0;
constexpr byte microphonePotPin = A1;

//Ultrasonic
constexpr byte ultrasonicPin = A5;

//Encoders
constexpr byte encoderA_trig = 14;
constexpr byte encoderA_echo = 15;
constexpr byte encoderB_trig = 6;
constexpr byte encoderB_echo = 19;
constexpr byte encoderBButtonPin = 4;
constexpr byte encoderAButtonPin = 8;

//9axis DOF
constexpr byte nineAxis_SDA = 2;
constexpr byte nineAxis_SCL = 3;

//Servo
constexpr byte servoPin = 7;

//Neopixel
constexpr byte neopixelPin = 9;
constexpr byte numPixels = 8;

//Tone Out
constexpr byte toneOut_pinB = 10;
constexpr byte toneOut_pinA = 11;

//Speaker Out
constexpr byte speakerOutPin = 12;



//global variables 

bool isTesting;


////////////////////Object initializations////////////////////////


//9axis
Adafruit_BNO055 bno = Adafruit_BNO055();


//Neopixel stick
Adafruit_NeoPixel neopixel = Adafruit_NeoPixel(numPixels, neopixelPin, NEO_GRB + NEO_KHZ800);


//Servo
Adafruit_TiCoServo servo;


//Encoders
Encoder encodA(encoderA_trig, encoderA_echo);
Encoder encodB(encoderB_trig, encoderB_echo);


//TFmini
AltSoftSerial altSerial(TFmini_RX, TFmini_TX);
TFMini tfmini;


//Ultarsonic
NewPing sonar(ultrasonicPin, ultrasonicPin, 500);


/////////////////////////Function pointers////////////////////

typedef Sensor (*SensorUpdateFunction)(Sensor &);

typedef Inputs (*InputsUpdateFunction)(Inputs &);

typedef Processor (*ProcessorUpdateFunction)(Sensor* sensors, Inputs&, Actuators&, Processor&);

typedef Actuators (*ActuatorsUpdateFunction)(Actuators &);

typedef int (*DistanceMappingFunction)(int input, int inputMin, int inputMax, int outputMin, int outputMax);

typedef int (*AxisMappingFunction)(uint16_t x, uint16_t y, uint16_t z, imu::Vector<3> euler, imu::Vector<3> acceleration, int outputMin, int outputMax);



//////////////////////Setup///////////////////////////

void setup() {

  //Debug, test "sensor"
  Serial.begin(9600);


  //Configure TFmini
  altSerial.begin(115200);
  tfmini.begin(&altSerial);

  // Configure 9axis DOF, BNO055 sensor
  Wire.begin();
  if (!bno.begin()) {
    Serial.println(F("No BNO055 detected!"));
    while (1);
  }
  bno.setExtCrystalUse(true);

  // Configure NeoPixel
  neopixel.begin();
  neopixel.show();

  // Attach Servo
  servo.attach(servoPin);

  // Set analog reference to external for LittleBits audio pins
  analogReference(EXTERNAL);

  //Speaker Out interupt
  Timer1.initialize(23); // Set the timer interval to 23 microseconds for a 44.1 kHz sample rate
  Timer1.attachInterrupt(writeArefToSpeaker);


  Serial.println(F("Setup complete. Type 'test <sensor>' to test a specific sensor (e.g., 'test ultra' or 'help')"));
}



/////////////////////Interupts//////////////
void writeArefToSpeaker() {
  int arefValue = analogRead(EXTERNAL);
  analogWrite(speakerOutPin, arefValue);
}

//////////////////////Map Functions///////////////////////////

//Vectors

bool vectorsAreEqual(const imu::Vector<3>& a, const imu::Vector<3>& b) {
  return (a.x() == b.x()) && (a.y() == b.y()) && (a.z() == b.z());
}


//Ultrasonic and TFmini Lidar

int linearMapping(int input, int inputMin, int inputMax, int outputMin, int outputMax){
  return map(input, inputMin, inputMax, outputMin, outputMax);
}


int exponentialMapping(int input, int inputMin, int inputMax, int outputMin, int outputMax) {
  float inputRange = inputMax - inputMin;
  float outputRange = outputMax - outputMin;

  float normalizedInput = (input - inputMin) / inputRange;
  float exponentialValue = pow(normalizedInput, 2);

  int output = round(exponentialValue * outputRange) + outputMin;
  return output;
}


int logarithmicMapping(int input, int inputMin, int inputMax, int outputMin, int outputMax) {
  float inputRange = inputMax - inputMin;
  float outputRange = outputMax - outputMin;

  float normalizedInput = (input - inputMin) / inputRange;
  float logarithmicValue = log10(normalizedInput * 9 + 1);

  int output = round(logarithmicValue * outputRange) + outputMin;
  return output;
}


int sigmoidMapping(int input, int inputMin, int inputMax, int outputMin, int outputMax) {
  float inputRange = inputMax - inputMin;
  float outputRange = outputMax - outputMin;

  float normalizedInput = (input - inputMin) / inputRange;
  float sigmoidValue = 1 / (1 + exp(-10 * (normalizedInput - 0.5)));

  int output = round(sigmoidValue * outputRange) + outputMin;
  return output;
}


//9axis DOF BNO055 sensor

int linearMappingXYZ(uint16_t x, uint16_t y, uint16_t z, imu::Vector<3> euler, imu::Vector<3> acceleration, int outputMin, int outputMax) {
  float _x = abs(x);
  float _y = abs(y);
  float _z = abs(z);
  
  float average = (_x + _y + _z) / 3.0;

  // Define the input range based on the expected min and max values of the components
  float inputMin = 0;
  float inputMax = 20; // Adjust this value based on the expected maximum value of the components
  
  return map(average, inputMin, inputMax, outputMin, outputMax);
}


int accelMagnitudeMapping(uint16_t x, uint16_t y, uint16_t z, imu::Vector<3> euler, imu::Vector<3> acceleration, int outputMin, int outputMax) {
  float magnitude = acceleration.magnitude();
  
  // Define the input range based on the expected min and max acceleration values
  float inputMin = 0;
  float inputMax = 20; // Adjust this value based on the expected maximum acceleration
  
  return map(magnitude, inputMin, inputMax, outputMin, outputMax);
}


int pitchMapping(uint16_t x, uint16_t y, uint16_t z, imu::Vector<3> euler, imu::Vector<3> acceleration, int outputMin, int outputMax) {
  float pitch = euler.x(); // Euler angle X (pitch)
  
  // Define the input range based on the expected min and max pitch values
  float inputMin = -180;
  float inputMax = 180;
  
  return map(pitch, inputMin, inputMax, outputMin, outputMax);
}


int rollMapping(uint16_t x, uint16_t y, uint16_t z, imu::Vector<3> euler, imu::Vector<3> acceleration, int outputMin, int outputMax) {
  float roll = euler.y(); // Euler angle Y (roll)
  
  // Define the input range based on the expected min and max roll values
  float inputMin = -90;
  float inputMax = 90;
  
  return map(roll, inputMin, inputMax, outputMin, outputMax);
}


//Neopixel

Actuators setTargetValuesFromProgmem(int position, bool fill, int colorIndex, Actuators &act) {
  imu::Vector<3> color;

  if (position >= 0 && position <= 8) {
    byte r = pgm_read_byte(&(neopixelColors[colorIndex][0]));
    byte g = pgm_read_byte(&(neopixelColors[colorIndex][1]));
    byte b = pgm_read_byte(&(neopixelColors[colorIndex][2]));
    color = imu::Vector<3>(r, g, b);

    if (fill) {
      for (int i = 0; i < position; i++) {
        act.targetValues[i] = color;
      }
    } else if (position < 8) {
      act.targetValues[position] = color;
    }
  }
  return act;
}


//servo

int mapHeadingToServoAngle(float currentHeading, float targetHeading) {
  float angleDifference = targetHeading - currentHeading;
  if (angleDifference < -180) {
    angleDifference += 360;
  } else if (angleDifference > 180) {
    angleDifference -= 360;
  }

  // Map angleDifference to servo angle range (e.g., 0 to 180)
  int servoAngle = map(angleDifference, -180, 180, 0, 180);

  return servoAngle;
}

//////////////////////Algorythms///////////////////////////////

DistanceMappingFunction distanceMappingFunctions[] = {
  linearMapping,
  exponentialMapping,
  logarithmicMapping,
  sigmoidMapping,
};


AxisMappingFunction axisMappingFunctions[] = {
  linearMappingXYZ,
  accelMagnitudeMapping,
  pitchMapping,
  rollMapping,
};


Sensor distanceMap(int distance, int max, Sensor &sensor){
  //mapIndex is set through sensor and function maps the distance value
  //@todo maybe need to change for ultra or tfmini
    sensor.outValue = distanceMappingFunctions[sensor.mapIndex](distance, 0, max, sensor.min, sensor.max) / sensor.volume;

      //@todo add the option for both, double click or whatever
      if (sensor.mapIndex <= 4){
        sensor.outPin = toneOut_pinA;
      }else {
        sensor.outPin = toneOut_pinB;
      }

    return sensor;
}


Sensor axisMap(uint16_t x, uint16_t y, uint16_t z, imu::Vector<3> euler, imu::Vector<3> acceleration, Sensor &sensor){
    //mapIndex is set through sensor and function maps the 9axis DOF values
    sensor.outValue = axisMappingFunctions[sensor.mapIndex](x, y, z, euler, acceleration, sensor.min, sensor.max) / sensor.volume;
      //@todo add the option for both, double click or whatever
      if (sensor.mapIndex <= 4){
        sensor.outPin = toneOut_pinA;
      }else {
        sensor.outPin = toneOut_pinB;
      }

      return sensor;
}


Actuators servoMap(Actuators &act) {
  // Compass reading
  //@todo add more mapping options for servo, right now it's being set it process functions

  // Read the compass heading from the 9-axis sensor
  sensors_event_t event;
  bno.getEvent(&event);

  float currentHeading = event.orientation.x;

  // Call the mapHeadingToServoAngle function with the current heading and target heading (0 degrees for North)
  act.targetAngle = mapHeadingToServoAngle(currentHeading, 0);

  return act;
}



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////Functions///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

Sensor getUltra(Sensor &sensor) {
  int ultra_distance = sonar.ping_cm();

  sensor = distanceMap(ultra_distance, 500, sensor);

  if (isTesting){
      Serial.print(F("Ultrasonic distance: "));
      Serial.print(ultra_distance);
      Serial.println(F(" mm"));
  }
  return sensor;
}


Sensor getTFmini(Sensor &sensor) {
  uint16_t tfmini_distance = tfmini.getDistance();
  uint16_t tfmini_strength = tfmini.getRecentSignalStrength();

  sensor = distanceMap(tfmini_distance, 1200, sensor);

  if (isTesting){
    Serial.print("Distance: ");
    Serial.print(tfmini_distance);
    Serial.print(" cm\t");
    Serial.print("Signal strength: ");
    Serial.println(tfmini_strength);
  }

  return sensor;
}


Sensor getNineAxis(Sensor &sensor) {

  sensors_event_t event;
  bno.getEvent(&event);

  uint16_t nineAxis_x = event.orientation.x;
  uint16_t nineAxis_y = event.orientation.y;
  uint16_t nineAxis_z = event.orientation.z;

  imu::Vector<3> nineAxis_euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> nineAxis_acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  int8_t nineAxis_temperature = bno.getTemp();

  sensor = axisMap(nineAxis_x, nineAxis_y, nineAxis_z, nineAxis_euler, nineAxis_acceleration, sensor);

  if (isTesting)
  {
      Serial.print(F("Orientation: "));
      Serial.print(F("X: "));
      Serial.print(nineAxis_x, 4);
      Serial.print(F(" Y: "));
      Serial.print(nineAxis_y, 4);
      Serial.print(F(" Z: "));
      Serial.println(nineAxis_z, 4);

      Serial.print(F("Heading: "));
      Serial.print(nineAxis_euler.x());
      Serial.print(F(" deg\t"));
      Serial.print(F("Roll: "));
      Serial.print(nineAxis_euler.y());
      Serial.print(F(" deg\t"));
      Serial.print(F("Pitch: "));
      Serial.print(nineAxis_euler.z());
      Serial.println(F(" deg"));

      Serial.print(F("Accel X: "));
      Serial.print(nineAxis_acceleration.x());
      Serial.print(F(" m/s^2\t"));
      Serial.print(F("Accel Y: "));
      Serial.print(nineAxis_acceleration.y());
      Serial.print(F(" m/s^2\t"));
      Serial.print(F("Accel Z: "));
      Serial.print(nineAxis_acceleration.z());
      Serial.println(F(" m/s^2"));

      Serial.print(F("Temperature: "));
      Serial.print(nineAxis_temperature);
      Serial.println(F(" C"));
  }

  return sensor;

}


Sensor getMic(Sensor &sensor) {
  // Read microphone and potentiometer values
  int mic_value = analogRead(microphonePin);
  int mic_potValue = analogRead(microphonePotPin);

  // Map potentiometer value to a sensitivity range (e.g., 1 to 5)
  int mic_sensitivity = map(mic_potValue, 0, 1023, 1, 5);

  // Adjust the microphone value based on the sensitivity
  int mic_adjustedMicValue = mic_value / mic_sensitivity;

  //@todo add more mapping functions for mic
  sensor.outValue = mic_adjustedMicValue;
  sensor.outDuration = -1;

  if (isTesting){
        Serial.print("Mic: ");
        Serial.print(mic_value);
        Serial.print("\tAdjusted: ");
        Serial.println(mic_adjustedMicValue);
  }

  return sensor;
}

///////////////////////////Inputs//////////////////////////////


Inputs getEncodA(Inputs &inputs) {
  inputs.enocdA_pressed = false;
  inputs.enocdA_held = false;
  inputs.enocdA_currentButtonState = digitalRead(encoderAButtonPin);
  
  //get pressed or held
  unsigned long currentTime = millis();

  if (inputs.enocdA_currentButtonState == LOW) {
    if ((currentTime - inputs.enocdA_lastEncoderAButtonPress) > inputs.enocdA_debounceDelay) {
      inputs.enocdA_pressed = true;
    }
    
    if ((currentTime - inputs.enocdA_lastEncoderAButtonPress) > inputs.encodA_pressedDelay) {
      inputs.enocdA_held = true;
      inputs.enocdA_pressed = false;
    }
  } else {
    inputs.enocdA_lastEncoderAButtonPress = currentTime;
  }

  //get position 0-7 clamp
  inputs.enocdA__value = encodA.read();
  inputs.encodA_position += inputs.enocdA__value;

  if (inputs.encodA_position > 7) {
    inputs.encodA_position = 7;
  }
  if (inputs.encodA_position < 0) {
     inputs.encodA_position = 0;
  }

  if (isTesting) {
    Serial.print(F("Encoder A value: "));
    Serial.print(inputs.enocdA__value);
    Serial.print(F(" Pressed: "));
    Serial.print(inputs.enocdA_pressed);
    Serial.print(F(" Held: "));
    Serial.println(inputs.enocdA_held);
  }

  return inputs;
}	


Inputs getEncodB(Inputs &inputs) {
  inputs.enocdB_pressed = false;
  inputs.enocdB_held = false;
  inputs.enocdB_currentButtonState = digitalRead(encoderBButtonPin);

  //get pressed or held
  unsigned long currentTime = millis();
  if (inputs.enocdB_currentButtonState == LOW) {
    if ((currentTime - inputs.enocdB_lastEncoderBButtonPress) > inputs.enocdB_debounceDelay) {
      inputs.enocdB_pressed = true;
    }
    
    if ((currentTime - inputs.enocdB_lastEncoderBButtonPress) > inputs.encodB_pressedDelay) {
      inputs.enocdB_held = true;
      inputs.enocdB_pressed = false;
    }
  } else {
    inputs.enocdB_lastEncoderBButtonPress = currentTime;
  }

  //get position 0-7
  inputs.enocdB__value = encodB.read();
  inputs.encodB_position += inputs.enocdB__value;

  if (inputs.encodB_position > 7) {
    inputs.encodB_position = 7;
  }
  if (inputs.encodB_position < 0) {
     inputs.encodB_position = 0;
  }

  if (isTesting) {
    Serial.print(F("Encoder B value: "));
    Serial.print(inputs.enocdB__value);
    Serial.print(F(" Pressed: "));
    Serial.print(inputs.enocdB_pressed);
    Serial.print(F(" Held: "));
    Serial.println(inputs.enocdB_held);
  }

  return inputs;
}


//load State should go here and it returns inputs and makes all those changes
//that way state can always be loaded from inputs.



////////////////////////Processors//////////////////////////////


Actuators loadState(Sensor* sensors, Inputs &inputs, Actuators &act, Processor &proc) {
  //state machine from user inputs: encoders
  switch (currentState) {
    case IDLE:
      if (inputs.enocdA_pressed || inputs.enocdB_pressed || inputs.enocdA__value != 0 || inputs.enocdB__value != 0) {
        currentState = TOOL_SELECT;
      } else if (inputs.enocdA_held) {
        currentState = TOOL_SAVE;
      } else if (inputs.enocdB_held) {
        currentState = TOOL_LOAD;
      }
      break;

    case TOOL_SELECT:
      if (inputs.enocdA_pressed) {
        inputs.toolIndex = inputs.encodA_position;
        currentState = SENSOR_SELECT;
      }
      break;
    
    case SENSOR_SELECT:
      if (inputs.enocdB_pressed) {
        inputs.sensorIndex = inputs.encodB_position;
        currentState = SENSOR_CHANGE;
      }
      break;

    case SENSOR_CHANGE:
      if (inputs.enocdB_pressed) {
        currentState = IDLE;
      }
      break;

    case SENSOR_SAVE:
      if (inputs.enocdB_pressed) {
        currentState = IDLE;
      }
      break;

    case TOOL_SAVE:
      if (inputs.enocdA_pressed) {
        inputs.slot = inputs.encodA_position;
        currentState = TOOL_SAVE_CONFIRMED;
      } else if (inputs.enocdB_pressed) {
        currentState = IDLE;
      }
      break;

    case TOOL_SAVE_CONFIRMED:
      if (inputs.enocdA_pressed) {
        currentState = IDLE;
      }else if (inputs.enocdB_pressed) {
        currentState = IDLE;
      }
      break;

    case TOOL_LOAD:
      if (inputs.enocdB_pressed) {
        inputs.slot = inputs.encodB_position;
        currentState = TOOL_LOAD_CONFIRMED;
      } else if (inputs.enocdA_pressed) {
        currentState = IDLE;
      }
      break;

    case TOOL_LOAD_CONFIRMED:
      if (inputs.enocdB_pressed) {
        currentState = IDLE;
      }
      break;
    
    default:
      currentState = IDLE;
      break;
  }
  
  return act;
}


Actuators loadSetEeprom(Sensor* sensors, Inputs &inputs, Actuators &act, Processor &proc) {

  if (currentState == TOOL_SAVE_CONFIRMED){
    int startAddress = inputs.slot * sizeof(Sensor) * 4;
    for (int i = 0; i < 4; i++) {
      EEPROM.put(startAddress + i * sizeof(Sensor), sensors[i]);
    }
  }

  if (currentState == TOOL_LOAD_CONFIRMED){
    int startAddress = inputs.slot * sizeof(Sensor) * 4;
    for (int i = 0; i < 4; i++) {
      EEPROM.get(startAddress + i * sizeof(Sensor), sensors[i]);
    }
  }
  return act;
}


Processor loadSensors(Sensor* sensors, Inputs &inputs, Actuators &act, Processor &proc) {
  //change sensor data based on encoders and state, e.g. change volume of ultrasonic distance mapped tone
  switch (currentState) {

    case SENSOR_CHANGE:
      byte toolIndex = inputs.toolIndex;
      byte index = inputs.sensorIndex;

      if (index == 0){
        // sensors[toolIndex].outValue = map(inputs.encodB_position, 0, 7, 0, 100);
      }else if (index == 1){
        // sensors[toolIndex].outDuration = map(inputs.encodB_position, 0, 7, 0, 100);
      }else if (index == 2){
        // sensors[toolIndex].outPin = map(inputs.encodB_position, 0, 7, 0, 100);
      }else if (index == 3){
        sensors[toolIndex].volume = map(inputs.encodB_position, 0, 7, 0, 100);
      }else if (index == 4){
        sensors[toolIndex].min = map(inputs.encodB_position, 0, 7, 0, 1000);
      }else if (index == 5){
        sensors[toolIndex].max = map(inputs.encodB_position, 0, 7, 0, 1000);
      }else if (index == 6){
        sensors[toolIndex].mapIndex = inputs.encodB_position;
      }else if (index == 7){
        sensors[toolIndex].pulse = map(inputs.encodB_position, 0, 7, 0, 2000);
      }
      break;

    default:
      currentState = IDLE;
      break;
  }

  return proc;
}


Processor loadNeopixels(Sensor* sensors, Inputs &inputs, Actuators &act, Processor &proc) {
  ///set neopioxels lights based on state
  switch (currentState) {
    case IDLE:
      int val = map(act.arefValue, 0, 1023, 0, 7);
      act = setTargetValuesFromProgmem(val, true, 1, act);
      break;

    case TOOL_SELECT:
      act = setTargetValuesFromProgmem(inputs.encodA_position, false, 1, act);
      break;
    
    case SENSOR_SELECT:
      act = setTargetValuesFromProgmem(inputs.encodB_position, false, 2, act);
      break;

    case SENSOR_CHANGE:
      act = setTargetValuesFromProgmem(inputs.encodB_position, true, 3, act);
      break;

    case SENSOR_SAVE:
      act = setTargetValuesFromProgmem(7, true, 4, act);
      break;

    case TOOL_SAVE:
      act = setTargetValuesFromProgmem(inputs.encodA_position, false, 5, act);
      break;

    case TOOL_SAVE_CONFIRMED:
      act = setTargetValuesFromProgmem(7, true, 6, act);
      break;

    case TOOL_LOAD:
      act = setTargetValuesFromProgmem(inputs.encodB_position, false, 7, act);
      break;

    case TOOL_LOAD_CONFIRMED:
      act = setTargetValuesFromProgmem(7, true, 8, act);
      break;

    default:
      currentState = IDLE;
      break;
      
  }
  return proc;
}


Processor loadServo(Sensor* sensors, Inputs &inputs, Actuators &act, Processor &proc) {
  //set servo based on state
  switch (currentState) {
    case IDLE:
      act = servoMap(act);
      break;

    case TOOL_SELECT:
      act.targetAngle = map(inputs.encodA_position, 0, 7, 0, 180);
      break;

    case SENSOR_SELECT:
      act.targetAngle = map(inputs.encodB_position, 0, 7, 0, 180);
      break;

    default:
      currentState = IDLE;
      break;
      
  }
  
  return proc;
}


Processor loadOutput(Sensor* sensors, Inputs &inputs, Actuators &act, Processor &proc) {
  //load all sensor outValue, OutDuration, OutPin to vector3
  imu::Vector<3> sensorValues[4];
  for (int i = 0; i < 4; ++i) {
    sensorValues[i] = imu::Vector<3>(sensors[i].outValue, sensors[i].outDuration, sensors[i].outPin);
  }

  // Set act.tonePulseVelocityValues based on sensorValues
  for (int i = 0; i < 4; ++i) {
    act.tonePulseVelocityValues[i] = sensorValues[i];
  }

  return proc;
}



////////////////////////Actuators//////////////////////////////

Actuators setNeoStick(Actuators &act) {

  if (isTesting){

      neopixel.clear();
      neopixel.show();

      uint32_t color = neopixel.Color(0, 50, 0);
      for (int i = 0; i < neopixel.numPixels(); i++) {
        neopixel.setPixelColor(i, color);
      }

      neopixel.show();

  }else{
    
    if (millis() - act.lastUpdate >= act.updateInterval) {
      act.lastUpdate = millis();

      for (int i = 0; i < numPixels; i++) {
        if (!vectorsAreEqual(act.currentValues[i], act.targetValues[i])) {
          for (int j = 0; j < 3; j++) {
            if (act.currentValues[i][j] < act.targetValues[i][j]) {
              act.currentValues[i][j]++;
            } else if (act.currentValues[i][j] > act.targetValues[i][j]) {
              act.currentValues[i][j]--;
            }
          }
          neopixel.setPixelColor(i, neopixel.Color(act.currentValues[i].x(), act.currentValues[i].y(), act.currentValues[i].z()));
          neopixel.show();
        }
      }
    }
  }

  return act;
}


Actuators setServo(Actuators &act) {
  if (isTesting){
      for (int pos = 0; pos <= 180; pos += 1) {
        servo.write(pos);
        delay(15);
      }
      for (int pos = 180; pos >= 0; pos -= 1) {
        servo.write(pos);
        delay(15);
      }
  } else {
    if (millis() - act.lastUpdate >= act.updateInterval) {
      act.lastUpdate = millis();

      if (act.currentAngle != act.targetAngle) {
        if (act.currentAngle < act.targetAngle) {
          act.currentAngle++;
        } else if (act.currentAngle > act.targetAngle) {
          act.currentAngle--;
        }
        servo.write(act.currentAngle);
      }
    }
  }

  return act;
}


Actuators setOutput(Actuators &act, bool isTesting = false){
  if (isTesting){
      TimerFreeTone(toneOut_pinA, 600, 1000);
      delay(1000);
      TimerFreeTone(toneOut_pinB, 600, 1000);
      delay(1000);
  } else {

  unsigned int currentMillis = millis();

  for (int i = 0; i < sizeof(act.tonePulseVelocityValues) / sizeof(act.tonePulseVelocityValues[0]); ++i) {
    imu::Vector<3> currentData = act.tonePulseVelocityValues[i];

    if (currentMillis - act.previousMillis[i] >= currentData.y()) {
      act.previousMillis[i] = currentMillis;

      // Select the output pin
      int toneOutPin = (currentData.z() == 0) ? toneOut_pinA : toneOut_pinB;


      //check if instrument or mic
      if (currentData.y() > 0){
        // Map the value to the frequency range
        int frequency = map(currentData.x(), 0, 1200, 27, 20000);

        // Play the tone
        TimerFreeTone(toneOutPin, frequency, currentData.y());

      } else {

        analogWrite(toneOutPin, currentData.x());
      }

      // Read the AREF value and write it to the speaker output pin
      act.arefValue = analogRead(EXTERNAL);

      //@todo could do this in main loop, done but how does it perform?
      analogWrite(speakerOutPin, act.arefValue);
    }
  }

}
  return act;
}



Command parseCommand(const String& command) {
  if (command == F("test encodA")) return TEST_ENCODA;
  if (command == F("test encodB")) return TEST_ENCODB;
  if (command == F("test ultra")) return TEST_ULTRA;
  if (command == F("test TFmini")) return TEST_TFMINI;
  if (command == F("test nineAxis")) return TEST_NINEAXIS;
  if (command == F("test mic")) return TEST_MIC;
  if (command == F("test NeoStick")) return TEST_NEOSTICK;
  if (command == F("test servo")) return TEST_SERVO;
  if (command == F("test output")) return TEST_OUTPUT;
  return UNKNOWN;
}

void handleSerial(Sensor* sensors, Inputs &inputs, Actuators &act, Processor &proc) {
  String commandString = Serial.readStringUntil('\n');
  commandString.trim();
  Command command = parseCommand(commandString);

  isTesting = true;

  switch (command) {
    case TEST_ENCODA:
      inputs = getEncodA(inputs);
      break;
    case TEST_ENCODB:
      inputs = getEncodB(inputs);
      break;
    case TEST_ULTRA:
      sensors[0] = getUltra(sensors[2]);
      break;
    case TEST_TFMINI:
      sensors[1] = getTFmini(sensors[3]);
      break;
    case TEST_NINEAXIS:
      sensors[2] = getNineAxis(sensors[4]);
      break;
    case TEST_MIC:
      sensors[3] = getMic(sensors[4]);
      break;
    case TEST_NEOSTICK:
      act = setNeoStick(act);
      break;
    case TEST_SERVO:
      act = setServo(act);
      break;
    case TEST_OUTPUT:
      act = setOutput(act);
      break;
    default:
      Serial.println(F("Commands: "));
      Serial.println(F("    test ultra"));
      Serial.println(F("    test TFmini"));
      Serial.println(F("    test nineAxis"));
      Serial.println(F("    test mic"));
      Serial.println(F("    test NeoStick"));
      Serial.println(F("    test servo"));
      Serial.println(F("    test encodA"));
      Serial.println(F("    test encodB"));
      Serial.println(F("    test output"));
  }

  isTesting = false;
}



SensorUpdateFunction sensorUpdateFunctions[] = {
  getUltra,
  getTFmini,
  getNineAxis,
  getMic,
};


InputsUpdateFunction inputsUpdateFunctions[] = {
  getEncodA,
  getEncodB,
};


ProcessorUpdateFunction processorUpdateFunctions[] = {
  loadState,
  loadSensors,
  loadSetEeprom,
  loadNeopixels,
  loadServo,
  loadOutput,
};


ActuatorsUpdateFunction actuatorsUpdateFunctions[] = {
  setNeoStick,
  setServo,
  setOutput,
};



void updateInputs(Inputs &inputs){
  for (InputsUpdateFunction func : inputsUpdateFunctions) {
    inputs = func(inputs);
  }
}


void updateSensors(Sensor* sensors) {
  byte index = 0;
  for (SensorUpdateFunction func : sensorUpdateFunctions) {
    sensors[index] = func(sensors[index]);
    index ++;
  }
}


void updateProcessors(Sensor* sensors, Inputs &inputs, Actuators &act, Processor &proc){
  for (ProcessorUpdateFunction func : processorUpdateFunctions) {
    proc = func(sensors, inputs, act, proc);
  }
}


void updateActuators(Actuators &act){
  for (ActuatorsUpdateFunction func : actuatorsUpdateFunctions) {
    act = func(act);
  }
}


void processSensorData(Sensor* sensors, Inputs &inputs, Actuators &act, Processor &proc) {
  // 1. Update atribbutes structs with new sensor values
  updateInputs(inputs);
  // 2. Read sensor values
  updateSensors(sensors);
  // 3. Process data according to the current state and Sensor
  updateProcessors(sensors, inputs, act, proc);
  // 4. Update actuators based on the processed data
  updateActuators(act);
}



void loop() {

  Sensor sensors[4];
  Inputs inputs;
  Processor processor;
  Actuators actuators;

  while(true) {
      if (Serial.available() > 0) {
          handleSerial(sensors, inputs, actuators, processor);
      } else {
          processSensorData(sensors, inputs, actuators, processor);
      }
  }

}
#include <Arduino.h>
#include <Servo.h>

//FRAME STRUCTURE
//1 - START = 35 
//2 - COMMAND
//3 - DATA1
//4 - DATA2
//5 - DATA3
//6 - DATA4
//7 - CRC
//8 - END = 38

struct TFrame{
  byte data[8];
};

const int STARTING_FRAME = 35;
const int ENDING_FRAME = 38;

//COMMANDS
const byte STEERING_COMMAND = 1;//byte velocity, byte steering, byte reverseState: 1 true 0 false
const byte BATTERYSTATE_COMMAND = 2;
const byte BATTERYINTERVAL_COMMAND = 3;
const byte STARTSTOPTRANSMITION_COMMAND = 4;

Servo Serwo;
struct TFrame Frame;
int beginTime;
int endTime;
int sendingIntervalInMillis;
int pwmMotorA = 5;
int inMotorA1 = 4;
int inMotorA2 = 3;
int batteryPIN = A0; //pin odczytujący napięcie zasilania
int serwoPIN = 9;
int watchdogTimeout = 3000;
unsigned long timeOfLastFrameReceived = 0;
boolean isTransmitted;

void decodeFrame(TFrame frame){
  byte startFrame;
  byte endFrame;
  byte command;
  byte data1;
  byte data2;
  byte data3;
  byte data4;

  startFrame = frame.data[0];
  command = frame.data[1];
  data1 = frame.data[2];
  data2 = frame.data[3];
  data3 = frame.data[4];
  data4 = frame.data[5];
  endFrame = frame.data[7];
  
  if ((startFrame == STARTING_FRAME) && (endFrame == ENDING_FRAME)){
    if (checkCRC(frame) == true) {
      switch (command){
        case STEERING_COMMAND:
          setVelocity(data1,data3,data4);
          setSteering(data2);    
        break;
        case BATTERYINTERVAL_COMMAND:
          setBatteryInterval(data1);
        break;
        case STARTSTOPTRANSMITION_COMMAND:
          startOrStopTransmition(data1);
        break;
        default:
        break;
      }
    }
  }
}

boolean checkCRC(TFrame frame){
  return true;
}

void sendFrame(byte command, byte data1, byte data2, byte data3, byte data4){
  byte data[8];

  data[0] = STARTING_FRAME;
  data[1] = command;
  data[2] = data1;
  data[3] = data2;
  data[4] = data3;
  data[5] = data4;
  data[6] = calculateCRC(command,data1,data2,data3,data4);
  data[7] = ENDING_FRAME; 
  
  Serial.write(data,8);
}

void wordToByteArray(byte *buffor, word value){
  buffor[0] = (value >> 8) & 0xFF;
  buffor[1] = (value) & 0xFF;
}

void sendFrame(byte command, word word1, word word2){
  byte word1InBytes[2];
  byte word2InBytes[2];

  wordToByteArray(word1InBytes,word1);
  wordToByteArray(word2InBytes,word2);

  sendFrame(command,word1InBytes[0],word1InBytes[1],word2InBytes[0],word2InBytes[1]);
}

byte calculateCRC(byte command, byte data1, byte data2, byte data3, byte data4){
  return 0;
}

void setBatteryInterval(byte timeInSec){
  sendingIntervalInMillis = timeInSec*1000;  
}

void setVelocity(byte velocity, byte reverseState, byte stopState){
  boolean isReverse = false;
  boolean isStop = false;
  
  if (reverseState == 1){isReverse = true;}
  else isReverse = false;

  if (stopState == 1){isStop = true;}
  else isStop = false;

  if (isStop)
  {
    setEmergencyStop();
  }
  else
  {
    if (isReverse){
      digitalWrite(inMotorA1, LOW);
      digitalWrite(inMotorA2, HIGH);
      analogWrite(pwmMotorA,velocity);
    }
    else{
      digitalWrite(inMotorA1, HIGH);
      digitalWrite(inMotorA2, LOW);
      analogWrite(pwmMotorA,velocity);
    }
  }
}

void setSteering(byte steering){
    if (steering < 40)
    steering = 40;
    if (steering > 140)
    steering = 140;
    Serwo.write(steering);
}

void startOrStopTransmition(byte data){
  if (data == 0){
    isTransmitted = false;
  }
  else if (data == 1){
    isTransmitted = true;
  }
}

boolean checkIsConnected(unsigned long currentTime){
  if ((currentTime - timeOfLastFrameReceived) > watchdogTimeout){
    return false;
  }
  else {
    return true;
  }
}

void setEmergencyStop(){
  digitalWrite(inMotorA1, HIGH);
  digitalWrite(inMotorA2, HIGH);
  analogWrite(pwmMotorA,255);
}

void printFrame(byte *data){
  Serial.println(data[0]);
  Serial.println(data[1]);
  Serial.println(data[2]);
  Serial.println(data[3]);
  Serial.println(data[4]);
  Serial.println(data[5]);
  Serial.println(data[6]);
  Serial.println(data[7]);  
}

void setup() {
  Serial.begin(9600);
  Serwo.attach(9);
  beginTime = millis();
  endTime = 0;
  isTransmitted = false;
  sendingIntervalInMillis = 2000;
}

void loop() {
  if (Serial.available() == 8){
    timeOfLastFrameReceived = millis();
    Serial.readBytes(Frame.data,8);
    decodeFrame(Frame);
    //printFrame(Frame.data);
  }

  if (!checkIsConnected(millis())){
    setEmergencyStop();
  }

  if (isTransmitted){
    endTime = millis();
    if ((endTime - beginTime) > sendingIntervalInMillis){
      beginTime = endTime;
      sendFrame(BATTERYSTATE_COMMAND,(word)analogRead(batteryPIN),0);
    }
  }
  delay(1);
} 

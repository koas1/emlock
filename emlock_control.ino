#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <MPU6050.h>
SoftwareSerial BTSerial(8, 7);

MPU6050 mpu;

int out[500]; 
int readIndex = 0;
float percent = 0;
char input2;
const int MPU = 0x68;
int16_t AcX, AcY, AcZ;
float AccX, AccY, AccZ;
unsigned long previousMillis = 0;
const long interval = 2000;
int relay = 5; // 릴레이 핀번호
boolean toggleFlag = 0;

const int bufferSize = 50;
int bufferIndex = 0;
float AccX_data[bufferSize];
unsigned long currentMillis=9999;

// Calibration values
int16_t xOffset;

void setup() {
  Wire.begin();
  BTSerial.begin(9600);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);
  calibrateMPU6050();
}

void loop() {
    char input = BTSerial.read();
    if (input == '4') {
      digitalWrite(relay, HIGH);
      BTSerial.println("Deactivate EM Lock");
      }
    if (input == '3') {
      digitalWrite(relay, LOW);
      BTSerial.println("Activate EM Lock");
      }
    if (input == '0') input2 ='1';
    if (input =='1') input2='2';
    if (input =='5') {
      input2='2';
      int sum0 = 0;
      for (int i = 0; i < readIndex; i++) sum0 += out[i];
      percent = (float)sum0 / readIndex * 100;
      BTSerial.print("BCI ");
      BTSerial.print(percent);
      BTSerial.println('%');
    }
    
    if (input2 == '1') {
      int16_t xAccelRaw, yAccelRaw, zAccelRaw;
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      AcX = Wire.read() << 8 | Wire.read();
      mpu.getAcceleration(&xAccelRaw, &yAccelRaw, &zAccelRaw);
      AccX = (float)(xAccelRaw - xOffset)/16384.0;
      BTSerial.println(AccX*9.81); // 실시간 가속도 측정값 출력
      AccX_data[bufferIndex++]= AccX*9.81;
      
      if (bufferIndex == bufferSize) {
        bufferIndex = 0;
        }

      float mean = calculateMean(AccX_data, bufferSize);
      float stdDev = calculateStdDev(AccX_data, bufferSize, mean);
 
      float lastSample = AccX_data[bufferIndex];
      float thresholdValue = mean * 0.9 + lastSample * 0.1;

      float threshold_1 = thresholdValue + 1.8 * stdDev;
      float threshold_2 = thresholdValue - 1.8 * stdDev;

      if (AccX*9.81 > threshold_1 or AccX*9.81 < threshold_2) out[readIndex] = 0; 
      else out[readIndex] = 1;
      readIndex++;

       if (AccX*9.81 > 0.5 || AccX*9.81 < -0.5) {
          currentMillis = millis();
          digitalWrite(relay, HIGH); // 전자석 풀기
          toggleFlag=1;
        }
        if(toggleFlag==1 and (millis()-currentMillis)>=interval){// 전자석 해제후 2초후
          digitalWrite(relay, LOW);
          toggleFlag=0;
          }
          delay(200);
      }
  }

float calculateMean(float data[], int dataSize) {
  float sum = 0;
  for (int i = 0; i < dataSize; i++) sum += data[i];
  return sum / dataSize;
}

float calculateStdDev(float data[], int dataSize, float mean) {
  float sum = 0;
  for (int i = 0; i < dataSize; i++) {
    sum += pow(data[i] - mean, 2);
  }
  return sqrt(sum / dataSize);
}

void calibrateMPU6050() {
  const int numSamples = 1000; // Number of samples for calibration
  
  int32_t xSum = 0, ySum = 0, zSum = 0;
  
  for (int i = 0; i < numSamples; i++) {
    int16_t xAccelRaw, yAccelRaw, zAccelRaw;
    
    mpu.getAcceleration(&xAccelRaw, &yAccelRaw, &zAccelRaw);
    
    xSum += xAccelRaw;
    ySum += yAccelRaw;
    zSum += zAccelRaw;
    
    delay(10); // You can adjust the delay time between samples if needed
  }
  
  xOffset = xSum / numSamples;
}

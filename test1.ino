#include <Wire.h> //i2c library so esp can communicate with mpu

#define AIN1 25 //motor1 pins for drv8833 пины DRV8833 для мотора 1
#define AIN2 26 //motor1 pins for drv8833
#define BIN1 32 //motor2 pins for drv8833 пины DRV8833 для мотора 2
#define BIN2 33 //motor2 pins for drv8833

#define M1_A 35 // encoder pins for motor1 пины энкодера мотора 1
#define M1_B 23 // encoder pins for motor1
#define M2_A 27 // encoder pins for motor2 пины энкодера мотора 2
#define M2_B 19 // encoder pins for motor2

volatile long count1 = 0; // volatile — variable changes in interrupt, счётчик мотора 1
volatile long count2 = 0; // compiler must not cache it счётчик мотора 2

// volatile — переменная меняется в прерывании, 
// компилятор не должна её кешировать

void IRAM_ATTR encoder1() {
  // IRAM_ATTR — function stored in fast memory
  // called automatically when encoder sends a pulse
  // IRAM_ATTR — функция хранится в быстрой памяти
  // вызывается автоматически когда энкодер даёт импульс
  if (digitalRead(M1_B)) count1++; // forward
  else count1--; // backward
}

void IRAM_ATTR encoder2() {
  if (digitalRead(M2_B)) count2++;
  else count2--; // same but for the second motor
}

long accelX; // raw accelerometer data сырые данные акселерометра
float gForceX; // angle in g units угол в единицах g
float offset = 0; // initial angle = our zero начальный угол = наш ноль
float smoothAngle = 0; // smoothed angle сглаженный угол
float lastAngle = 0; // previous angle for D component прошлый угол для D компонента


float Kp = 700;  // основная сила || reaction strength to angle сила реакции на угол
float Kd = 200;  // торможение при приближении к нулю || braking when approaching zero торможение при приближении к нулю

void setup() {
  Serial.begin(115200); // Serial Monitor for debugging Serial Monitor для отладки
  Wire.begin(); // start I2C запускаем I2C
  setupMPU(); // configure MPU6050 настраиваем MPU6050

  pinMode(AIN1, OUTPUT); // set motor pins as outputs
  pinMode(AIN2, OUTPUT); // настраиваем пины моторов как выходы
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(M1_A, INPUT); // set encoder pins as inputs
  pinMode(M1_B, INPUT); // настраиваем пины энкодеров как входы
  pinMode(M2_A, INPUT);
  pinMode(M2_B, INPUT);
  attachInterrupt(M1_A, encoder1, RISING);// attach interrupts — \\ every encoder pulse calls encoder1() or encoder2()
  attachInterrupt(M2_A, encoder2, RISING); // подключаем прерывания — // при каждом импульсе энкодера вызывается encoder1() или encoder2()

  delay(500); // wait for MPU to stabilize ждём стабилизации MPU
  recordAccelRegisters(); // read initial angle читаем начальный угол
  offset = gForceX; // save as zero запоминаем как ноль
  Serial.print("Offset="); // печатает текст БЕЗ новой строки
  Serial.println(offset); // печатает число И переходит на новую строку
}

void loop() {
  recordAccelRegisters(); // read angle читаем угол

  float angle = gForceX - offset; // deviation from zero отклонение от нуля
  smoothAngle = 0.9 * smoothAngle + 0.1 * angle; // smoothing — removes sensor noise сглаживание — убираем шум

  float derivative = smoothAngle - lastAngle; // D component — rate of angle change D компонент — скорость изменения угла
  lastAngle = smoothAngle;

  float output = Kp * smoothAngle + Kd * derivative; // PD controller — calculate output signal PD регулятор — считаем выходной сигнал
  int speed = constrain(abs(output), 0, 255); // motor speed — 0 to 255, minimum 90 скорость мотора — от 0 до 255, минимум 90
  if (speed > 0 && speed < 90) speed = 90;

  // motor direction depends on sign of output направление моторов по знаку output
  if (output > 0.05) {
    // motors forward
    analogWrite(AIN1, 0);
    analogWrite(AIN2, speed);
    analogWrite(BIN1, speed);
    analogWrite(BIN2, 0);
  } else if (output < -0.05) {
    // motors backward
    analogWrite(AIN1, speed);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, speed);
  } else {
    // stop
    analogWrite(AIN1, 0);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, 0);
  }

  Serial.print("angle="); //things to see in serial monitor like whyy
  Serial.print(smoothAngle);
  Serial.print(" output=");
  Serial.print(output);
  Serial.print(" M1=");
  Serial.print(count1);
  Serial.print(" M2=");
  Serial.println(count2);

  delay(10);
}

void setupMPU(){
  // register 0x6B = power, write 0 = disable sleep
  // register 0x1B = gyroscope, 0 = range ±250°/s
  // register 0x1C = accelerometer, 0 = range ±2g
  // регистр 0x6B = питание, пишем 0 = выключаем sleep
  // регистр 0x1B = гироскоп, 0 = диапазон ±250°/с
  // регистр 0x1C = акселерометр, 0 = диапазон ±2g
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);// address of X accelerometer register
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); // request 6 bytes (X, Y, Z)
  while(Wire.available() < 6); // wait for all bytes
  accelX = Wire.read()<<8|Wire.read(); // two bytes → one number
  Wire.read(); Wire.read(); // Y — not used
  Wire.read(); Wire.read(); // Z — not used
  gForceX = (int16_t)accelX / 16384.0; // convert to g \\ 16384 = maximum value at ±2g range
}
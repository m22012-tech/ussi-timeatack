#include <SPI.h>


const int csPin = 13;
const int pwmPin = 9;  // Timer1A (PB5)
const int cds_pin = 5;
const float cds_r = 47000.0;
const int tr_pin = 4;
const float tr_r = 4700.0;

const float RL = 1500.0;
const float VCC = 5.0;
const float SPEC_LUX = 100.0;
const float SPEC_CURRENT = 0.00026;
const float BASE_GAMMA = 1.0;
const float maxDuty = 4.0 / 12.0;

// { 照度計(lx), Serial表示(lx) }
const float calib_data[][2] = {
  { 108, 99 },
  { 186, 168 },
  { 296, 264.5 },
  { 469, 450 },
  { 763, 677 },
  { 828, 722 },
  { 928, 797 },
  { 986, 830 }
};
const int num_calib = sizeof(calib_data) / sizeof(calib_data[0]);

// 目標照度たち
const float lx_ref[19] = {
  10, 20, 30, 40, 50, 60, 70, 80, 90,
  100, 200, 300, 400, 500, 600, 700, 800, 900, 1000
};
static bool mea = false;
static int lx_index = 0;
const int length_lx_ref = sizeof(lx_ref) / sizeof(lx_ref[0]);

const float lx_margin = 0.2;


float calibrated_gamma = BASE_GAMMA;
float calibrated_k = 0.0;  // 電流 = k * (lx)^gamma の k



float K_dc = maxDuty / 1300.0;
float K_P = 0.0;
float K_I = 0.0;
const float Ts = 0.050;
const float fc = 1.0;

// float targetLux = 500.0;
// float currentLux = 0.0;
float u_prev = 0.0;
float e_prev = 0.0;
unsigned long previousMillis = 0;



void applyCalibration() {
  if (num_calib < 2) {
    calibrated_gamma = BASE_GAMMA;
    calibrated_k = SPEC_CURRENT / pow(SPEC_LUX, BASE_GAMMA);
    return;
  }

  float sumX = 0, sumY = 0, sumX2 = 0, sumXY = 0;
  for (int i = 0; i < num_calib; i++) {
    float L_ref = calib_data[i][0];
    float L_disp = calib_data[i][1];

    float I_calc = SPEC_CURRENT * pow((L_disp / SPEC_LUX), BASE_GAMMA);

    float x = log10(L_ref);
    float y = log10(I_calc);

    sumX += x;
    sumY += y;
    sumX2 += x * x;
    sumXY += x * y;
  }

  // 最小二乗法で y = a*x + b を求める (a = gamma, b = log10(k))
  float n = (float)num_calib;
  float denominator = n * sumX2 - sumX * sumX;

  if (denominator != 0) {
    calibrated_gamma = (n * sumXY - sumX * sumY) / denominator;
    float b = (sumY - calibrated_gamma * sumX) / n;
    calibrated_k = pow(10.0, b);
  } else {
    calibrated_gamma = BASE_GAMMA;
    calibrated_k = SPEC_CURRENT / pow(SPEC_LUX, BASE_GAMMA);
  }
}

void setupPWM() {
  pinMode(pwmPin, OUTPUT);
  // Fast PWM Mode 14 (TOP = ICR1)
  // WGM13=1, WGM12=1, WGM11=1, WGM10=0
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  // Prescaler = 8 (16MHz / 8 = 2MHz)
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  ICR1 = 9999;  // 2MHz / 200Hz - 1 = 9999
  OCR1A = 0;
}

void setPWMDuty(float duty) {
  if (duty < 0.0) duty = 0.0;
  if (duty > maxDuty) duty = maxDuty;
  OCR1A = (uint16_t)(duty * 9999.0);
}

float readLux() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  uint16_t msb = SPI.transfer(0x00);
  uint16_t lsb = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  uint32_t rawCode = (msb << 8) | lsb;
  float voltage = (rawCode / 65535.0) * VCC;
  float current_A = voltage / RL;

  float lux = 0.0;
  if (current_A > 0.0) {
    // I = k * L^gamma より、 L = (I / k)^(1 / gamma)
    lux = pow((current_A / calibrated_k), (1.0 / calibrated_gamma));
  }
  return lux;
}

float identifyDCGain() {
  float duty_tests[] = { 0.2 * maxDuty, 0.4 * maxDuty, 0.6 * maxDuty, 0.8 * maxDuty };
  float gain_sum = 0.0;

  setPWMDuty(0.0);
  delay(200);
  readLux();
  delay(50);
  float baseLux = readLux();

  for (int i = 0; i < 4; i++) {
    setPWMDuty(duty_tests[i]);
    delay(200);
    readLux();
    delay(50);
    float lx = readLux();
    gain_sum += (lx - baseLux) / duty_tests[i];
  }

  setPWMDuty(0.0);
  delay(100);
  return gain_sum / 4.0;
}


void setup() {
  Serial.begin(9600);
  Serial.dtr();

  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {}

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  SPI.begin();

  //setupPWM();

  applyCalibration();

  //K_dc = identifyDCGain();
  if (K_dc <= 10.0) K_dc = 1300.0;

  float wc = 2.0 * PI * fc;
  K_I = wc / K_dc;
  K_P = K_I * Ts;

  Serial.println("target_lux [lx] ,current_lux [lx], cds_ohm [Ω], tr_current [A]");
}


void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= (unsigned long)(Ts * 1000)) {
    previousMillis = currentMillis;

    float currentLux = readLux();

    float cds_ohm_v = 5.0 * (analogRead(cds_pin) / 1023.0);
    float cds_ohm = cds_r * cds_ohm_v / (5.0 - cds_ohm_v);

    float tr_v = 5.0 * (analogRead(tr_pin) / 1023.0);
    float tr_current = tr_v / tr_r;

    float targetLux = lx_ref[lx_index];

    Serial.print(targetLux);
    Serial.print(", ");
    Serial.print(currentLux);
    Serial.print(", ");
    Serial.print(cds_ohm, 4);
    Serial.print(", ");
    Serial.print(tr_current, 8);
    Serial.println();

    bool is_lx_captured = targetLux * (1 - lx_margin / 2) < currentLux && targetLux * (1 + lx_margin / 2) > currentLux;
    if (mea) {
      lx_index++;
      if (length_lx_ref < lx_index) lx_index = 0;
      Serial.print(targetLux);
      Serial.print(", ");
      Serial.print(currentLux);
      Serial.print(", ");
      Serial.print(cds_ohm, 4);
      Serial.print(", ");
      Serial.print(tr_current, 8);
      Serial.println();
      mea = 0;
    }
  }
  if (Serial.available() > 0) {
    Serial.println("target_lux [lx] ,current_lux [lx], cds_ohm [Ω], tr_current [A]");
    lx_index = 0;
    mea = 1;
    while (Serial.available() > 0) Serial.read();
  }
}
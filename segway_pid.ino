#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

// モーターピン定義
// モーターA（左輪）
const int pinAPhase = 4;   // 方向制御(HIGH/LOW)
const int pinAEnable = 5;  // 速度制御(PWM: 0〜255)
// モーターB（右輪）
const int pinBPhase = 7;   // 方向制御(HIGH/LOW)
const int pinBEnable = 6;  // 速度制御(PWM: 0〜255)

// --- ここを調整して実験する（ゲイン調整）
float Kp = -25.0;   // Pゲイン：大きくすると戻る力が強くなる
float Ki = -3.0;    // Iゲイン：重心のズレをじわじわ修正する力
float Kd = -2.0;    // Dゲイン：大きくするとブレーキが強くなる（揺れを抑える）

// --- 制御用の変数
float targetAngle = 0;  // 目標位置（今回は起動した場所「0」をキープ）
float lastError = 0;       // 前回のズレ（D制御用）
float integral = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // ピンを出力モードにする
  pinMode(pinAPhase, OUTPUT);
  pinMode(pinAEnable, OUTPUT);
  pinMode(pinBPhase, OUTPUT);
  pinMode(pinBEnable, OUTPUT);

  Serial.println("MPU6050を初期化します。ロボットを動かさずに待ってください...");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("初期化完了！");
}

void loop() {
  // センサーの最新データを読み込み
  mpu6050.update();
  float currentAngle = mpu6050.getAngleX();

  // 安全対策：45度以上傾いたら、暴走防止のためにモーターを緊急停止する
  if (abs(currentAngle) > 45.0) {
    stopMotors();
    integral = 0; // 積分もリセット
  }

  float error = targetAngle - currentAngle;

  // 2. I（積分）の計算
  integral += error;
  integral = constrain(integral, -400, 400);  // ワインドアップ（たまりすぎ）防止

  // 3. D（微分）の計算
  long errorSpeed = error - lastError;
  lastError = error;    // 次回のために保存

  // 4. PID出力を合算
  float totalOutput = (error * Kp) + (integral * Ki) + (errorSpeed * Kd);

  // 5. 左右のモーターに出力を適用
  driveMotors(totalOutput);
  
  // シリアルモニタに出力
  Serial.print("Angle： "); Serial.print(currentAngle);
  Serial.print(" | Output: "); Serial.println(totalOutput);

  delay(10);  // 10ミリ秒ごとに更新(100Hz)
}

// 左右のモーターを駆動する関数
void driveMotors(float output) {
  // 出力が正（前に倒れそう）なら前進、負（後ろに倒れそう）なら後退
  // もしロボットが逆に動く場合はHIGH/LOWの指定を全て逆にする
  boolean dir = (output >= 0) ? HIGH : LOW;

  // モーターA（左）の方向と出力
  digitalWrite(pinAPhase, dir);
  int pwmA = constrain(abs(output), 0, 255);
  analogWrite(pinAEnable, pwmA);

  // モーターB（右）の方向と出力
  digitalWrite(pinBPhase, dir);
  int pwmB = constrain(abs(output), 0, 255);
  analogWrite(pinBEnable, pwmB);
  }

void stopMotors() {
  analogWrite(pinAEnable, 0);
  analogWrite(pinBEnable, 0);
}
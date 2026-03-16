// =======================
// DUAL MOTOR + DUAL ENCODER TEST (Serial Command + REV with DECEL/BRAKE)
// ESP32 + L298N + JGB37-520 Quadrature Encoder
// Based on your single-wheel code :contentReference[oaicite:1]{index=1}
// =======================

#include <Arduino.h>
#include <math.h>

// ===== ROS2 CMD_VEL MODE =====
bool velMode = false;       // เปิดโหมดนี้เมื่อรับคำสั่ง V ...
float WHEEL_R = 0.033f;     // 3.3cm
float WHEEL_BASE = 0.36f;  // ระยะห่างล้อซ้าย-ขวา (เมตร) ใส่ค่าจริงของคุณ            // P speed control (เริ่ม 2~6)

bool velStoppedLatch = false;

//PI Gains (คุมความเร็ว)
float KP_V_L = 2.8f;
float KI_V_L = 0.4f;
float KP_V_R = 3.2f;
float KI_V_R = 0.4f;

float KI_V = 0.5f;  // (unused) เหลือไว้จากเวอร์ชันเก่า / ยังไม่ได้ใช้ในโค้ดนี้

//GAIN ตัวคูณชดเชยความแรงมอเตอร์
//(Forward)
float GAIN_L_F = 0.9575f;  // ลดซ้ายลง 7% (ลอง 0.90–0.99)
float GAIN_R_F = 0.900f;   // ปกติ 1.00

//(Backward)
float GAIN_L_B = 0.958f;
float GAIN_R_B = 0.900f;

inline float gainL(bool forward) {
  return forward ? GAIN_L_F : GAIN_L_B;
}
inline float gainR(bool forward) {
  return forward ? GAIN_R_F : GAIN_R_B;
}

//ตัวแปรสถานะ PI
float tgtRpmL_cmd = 0, tgtRpmR_cmd = 0;  // เป้าจาก teleop/ROS (เปลี่ยนทันทีได้)
float tgtRpmL = 0, tgtRpmR = 0;          // เป้าจริงที่ไต่ขึ้นลง (เอาไปเข้า PI)
float iTermL = 0, iTermR = 0;            //อินทิเกรตสะสมของ PI (กัน steady-state error)
int pwmCmdL = 0, pwmCmdR = 0;            //PWM สุดท้ายที่จะสั่งมอเตอร์หลังคำนวณ + gain + deadband

// ปรับความนุ่มตอนออกตัว: หน่วย "rpm ต่อ วินาที"
// ค่ายิ่งน้อย = นิ่มขึ้น/ออกตัวช้าลง, ยิ่งมาก = ไต่ไว/กระชากขึ้น
float ACC_RPM_PER_S = 180.0f;  // ลอง 120~250
float DEC_RPM_PER_S = 260.0f;  // ชะลอเร็วกว่าเร่งนิดนึง (กันไหล)

// ===== REV loop timers (timeout/stuck detect) =====
unsigned long revStartMs = 0, stuckMs = 0;
long lastMovedL_ = 0, lastMovedR_ = 0;

// ===== CMD_VEL PI loop timing (run every ~50ms) =====
unsigned long lastVelMs = 0;

// ===== ซ้าย (ตอนนี้ไปช่อง A: OUT1/OUT2) =====
#define ENA 25  //PWM pin ซ้าย
#define IN1 26  // DIR L (ตามโค้ด)
#define IN2 27  // DIR L (กลับค่า = กลับทิศ)

// ===== ขวา (ตอนนี้ไปช่อง B: OUT3/OUT4) =====
#define ENB 14  //PWM pin ขวา
#define IN3 12  //ทิศทางขวา
#define IN4 13  //ทิศทางขวา

// ----- Encoder LEFT -----
#define ENC_L_A 34  //ใช้ interrupt ขา A
#define ENC_L_B 35  //ใช้อ่านทิศทาง

// ----- Encoder RIGHT -----
#define ENC_R_A 32  //ใช้ interrupt ขา A
#define ENC_R_B 33  //ใช้อ่านทิศทาง

// -------Serial--------
#define RXD2 16
#define TXD2 17

#define ROS Serial2  // ✅ ให้ ROS ใช้พอร์ตนี้
#define DBG Serial   // ✅ debug ใช้ USB

#define DEBUG_LOG 0  // 1=เปิด log, 0=ปิด log

#if DEBUG_LOG
#define DPRINTLN(x) DBG.println(x)
#define DPRINTF(...) DBG.printf(__VA_ARGS__)
#else
#define DPRINTLN(x) \
  do { \
  } while (0)
#define DPRINTF(...) \
  do { \
  } while (0)
#endif

// ----- PWM (ESP32 LEDC) -----
static const int PWM_FREQ = 20000;  //20kHz ลดเสียงหอน
static const int PWM_RES = 8;       // 0-255
static const int PWM_CH_L = 0;      //channel PWM ของซ้าย
static const int PWM_CH_R = 1;      //channel PWM ของขวา

volatile long countL = 0;  //ตัวนับ tick encoder มันสะสมไปเรื่อยๆ เช่น วิ่งไป 1 เมตรอาจเพิ่มขึ้น ~4860 tick (ISR จะ count++ หรือ count--)
volatile long countR = 0;  //ตัวนับ tick encoder มันสะสมไปเรื่อยๆ เช่น วิ่งไป 1 เมตรอาจเพิ่มขึ้น ~4860 tick (ISR จะ count++ หรือ count--)

// ตั้งจากที่คุณคาลิเบรต (ปรับได้ภายหลัง)
float COUNTS_PER_REV_L = 989.2;
float COUNTS_PER_REV_R = 989.2;
//ถ้าค่านับ/รอบไม่ตรงนี้ผิด → rpm และเป้าหมาย rev จะเพี้ยน

//tick ต่อ เมตร ถ้าอยากให้วิ่ง 1 เมตรแม่น ต้องจูนค่านี้
float TICKS_PER_M_L = 4860.0f;
float TICKS_PER_M_R = 4860.0f;

// Decel (ผ่อนความเร็ว)
long DECEL_L = 450;
long DECEL_R = 450;
// ให้เริ่มผ่อนเร็วกว่าถ้าขวาชอบพุ่งเกิน //จำนวน tick ช่วงท้ายที่จะเริ่มลด PWM (เหมือน ramp ลง) มากขึ้น = ผ่อนนานขึ้น (นิ่มขึ้น แต่ช้า) น้อย = ผ่อนสั้น (อาจ overshoot)

int PWM_MIN = 35;  //PWM ขั้นต่ำที่พอให้ล้อเริ่มหมุน/ไม่ติด

//โหมด “finish mode” ตอนล้ออีกข้างหยุดแล้ว แต่อีกล้อยังต้องไปให้ถึงเป้า
int FINISH_PWM = 55;  // แรงพอให้คืบตอนท้าย (ลอง 50–70)

//เวลาที่ “สั่ง brake” (IN=HIGH,HIGH) แล้ว delay ค้างกี่ ms
int BRAKE_MS_L = 160;
int BRAKE_MS_R = 190;  // เพิ่มเบรกขวาถ้ายังไหลเกิน

//“หน้าต่าง” ก่อนถึงเป้า ถ้าเข้า window นี้จะเบรกทันทีเพื่อหยุดแม่นขึ้น
long BRAKE_WINDOW_L = 0;  // ลอง 20-40
long BRAKE_WINDOW_R = 0;

//ใกล้เป้าถึงจุดนี้แล้ว “ปล่อย PWM=0” เพื่อลดการไหล/overshoot
long PRECUT_L = 0;
long PRECUT_R = 0;

float KP_SYNC = 2.0;  // คุมแรงชดเชยตาม “ความก้าวหน้า (progress)” ของล้อซ้าย-ขวา
int SYNC_MAX = 40;    // จำกัดค่าสูงสุดของการชดเชย PWM กันแกว่ง

// สำหรับคำนวณ RPM
long lastCountL = 0, lastCountR = 0;
unsigned long lastMs = 0;
float lastRpmL = 0, lastRpmR = 0;

// อินเวิร์ตทิศ encoder รายล้อ (ถ้าทิศ count กลับ) /ถ้าทิศ encoder กลับ ให้ toggle ด้วยคำสั่ง xl / xr
bool invEncL = false;
bool invEncR = false;

// -----------------------
// Encoder ISR
// -----------------------
void IRAM_ATTR isrEncL() {
  int b = digitalRead(ENC_L_B);
  long d = (b == HIGH) ? +1 : -1;
  if (invEncL) d = -d;
  countL += d;
}

void IRAM_ATTR isrEncR() {
  int b = digitalRead(ENC_R_B);
  long d = (b == HIGH) ? +1 : -1;
  if (invEncR) d = -d;
  countR += d;
}

// -----------------------
// Motor control helpers
// -----------------------
void coastStopL() {
  ledcWrite(PWM_CH_L, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
void coastStopR() {
  ledcWrite(PWM_CH_R, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void coastStopAll() {
  coastStopL();
  coastStopR();
}

void brakeStopL(int ms) {
  ledcWrite(PWM_CH_L, 0);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  delay(ms);
  brakeHoldL();
}
void brakeStopR(int ms) {
  ledcWrite(PWM_CH_R, 0);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  delay(ms);
  brakeHoldR();
}
void brakeStopAll(int ms) {
  brakeStopL(ms);
  brakeStopR(ms);
}

// --- brake HOLD (no delay) ---
void brakeHoldL() {
  ledcWrite(PWM_CH_L, 0);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}
void brakeHoldR() {
  ledcWrite(PWM_CH_R, 0);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}


void setMotorL(int pwm, bool forward) {
  pwm = constrain(pwm, 0, 255);
  if (pwm == 0) {
    coastStopL();
    return;
  }
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW : HIGH);
  ledcWrite(PWM_CH_L, pwm);
}
void setMotorR(int pwm, bool forward) {
  pwm = constrain(pwm, 0, 255);
  if (pwm == 0) {
    coastStopR();
    return;
  }
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW : HIGH);
  ledcWrite(PWM_CH_R, pwm);
}

// -----------------------
// RPM compute
// -----------------------
//
void computeRpms(unsigned long nowMs, float &rpmL, float &rpmR) {
  unsigned long dt = nowMs - lastMs;  //ดูเวลาห่างจากครั้งก่อน
  if (dt < 50) {                      // เรียกถี่เกิน tick เปลี่ยนน้อย ทำให้ rpm กระโดด -> ใช้ค่าครั้งก่อนให้เรียบ
    rpmL = lastRpmL;
    rpmR = lastRpmR;
    return;
  }

  long cL = countL, cR = countR;
  long dcL = cL - lastCountL;  //ดู encoder เพิ่มขึ้นเท่าไหร่
  long dcR = cR - lastCountR;

  float revL = (COUNTS_PER_REV_L > 0) ? ((float)dcL / COUNTS_PER_REV_L) : 0.0f;  //แปลง tick → รอบ
  float revR = (COUNTS_PER_REV_R > 0) ? ((float)dcR / COUNTS_PER_REV_R) : 0.0f;

  rpmL = revL * (60000.0f / (float)dt);  //แปลง “รอบใน dt ms” → RPM
  rpmR = revR * (60000.0f / (float)dt);

  lastCountL = cL;
  lastCountR = cR;
  lastMs = nowMs;
  lastRpmL = rpmL;
  lastRpmR = rpmR;
}

// ===== ODOM STATE =====
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_yaw = 0.0f;

long odom_lastL = 0;
long odom_lastR = 0;
unsigned long odom_lastMs = 0;

static inline float wrapPi(float a) {
  while (a > 3.1415926f) a -= 2.0f * 3.1415926f;
  while (a < -3.1415926f) a += 2.0f * 3.1415926f;
  return a;
}

void updateOdometry(unsigned long nowMs) {
  if (odom_lastMs == 0) {
    odom_lastMs = nowMs;
    odom_lastL = countL;
    odom_lastR = countR;
    return;
  }

  unsigned long dtMs = nowMs - odom_lastMs;
  if (dtMs < 20) return;  // ไม่ต้องถี่มาก

  long cL = countL, cR = countR;
  long dL = cL - odom_lastL;
  long dR = cR - odom_lastR;

  odom_lastL = cL;
  odom_lastR = cR;
  odom_lastMs = nowMs;

  // tick -> meters
  float distL = (TICKS_PER_M_L > 0) ? ((float)dL / TICKS_PER_M_L) : 0.0f;
  float distR = (TICKS_PER_M_R > 0) ? ((float)dR / TICKS_PER_M_R) : 0.0f;

  float ds = (distL + distR) * 0.5f;
  float dtheta = (distR - distL) / WHEEL_BASE;

  // integrate
  float yawMid = odom_yaw + dtheta * 0.5f;
  odom_x += ds * cosf(yawMid);
  odom_y += ds * sinf(yawMid);
  odom_yaw = wrapPi(odom_yaw + dtheta);
}


// -----------------------
// Serial readLine
// -----------------------
//อ่าน char จนเจอ \n หรือ \r แล้วคืนเป็น 1 บรรทัด
bool readLine(String &out) {
  static String buf;
  while (ROS.available()) {
    char ch = ROS.read();
    if (ch == '\r' || ch == '\n') {
      if (buf.length() == 0) continue;
      out = buf;
      buf = "";
      out.trim();
      return true;
    }
    buf += ch;
    if (buf.length() > 160) buf.remove(0, buf.length() - 160);
  }
  return false;
}


//พิมพ์คำสั่งที่รองรับทั้งหมด
void printHelp() {
  DPRINTLN("\n=== DUAL MOTOR + DUAL ENCODER READY ===");
  DPRINTLN("LEFT only:  el <pwm> , bl <pwm> , vl <rev> <pwm>");
  DPRINTLN("RIGHT only: er <pwm> , br <pwm> , vr <rev> <pwm>");
  DPRINTLN("BOTH:       ea <pwmL> <pwmR> , ba <pwmL> <pwmR> , va <rev> <pwmL> <pwmR>");
  DPRINTLN("Common:     s(stop) r(read) z(zero) d<cnt> p<pwmmin> k<ms> h(help)");
  DPRINTLN("Optional:   xl (invert enc L), xr (invert enc R)");
  DPRINTLN("ROS2:       v <linear_mps> <angular_radps> , stopv");
}

// -----------------------
// REV state machine
// -----------------------
//ตอนนี้กำลังสั่งวิ่งแบบไหน (ไม่วิ่ง/ซ้าย/ขวา/สองล้อ)
enum RevMode { REV_NONE,
               REV_L,
               REV_R,
               REV_BOTH };
RevMode revMode = REV_NONE;

//ถ้า “ล้อนี้เบรกไปแล้ว” ให้ latch ไว้ แล้วห้าม setMotor ซ้ำ
bool doneLatchedL = false;
bool doneLatchedR = false;

long startL = 0, startR = 0;          //tick ณ ตอนเริ่มคำสั่ง (ใช้คำนวณ moved)
long targetAbsL = 0, targetAbsR = 0;  //จำนวน tick เป้าหมายแบบ “ค่าสัมบูรณ์” (ใช้ labs)

bool fwdL = true, fwdR = true;  //ทิศทางของคำสั่ง rev/ticks
int cmdPwmL = 0, cmdPwmR = 0;   //PWM ที่ใช้เป็น “เพดาน” ก่อน ramp / sync / brake

//แปลง rev เป็น tick ด้วย COUNTS_PER_REV_X เก็บ start count ตั้ง mode + สั่งมอเตอร์เริ่มวิ่ง
void startRevL(float rev, int pwm) {
  doneLatchedL = false;
  doneLatchedR = false;
  fwdL = (rev > 0);
  cmdPwmL = constrain(pwm, 0, 255);

  revStartMs = millis();  //เวลาที่เริ่มคำสั่งนี้ (ไว้ timeout 15s)
  //ตรวจว่าถึงสั่งแล้ว encoder ไม่ขยับ (ติด/สายหลุด/แบตตก) ถ้าติดเกิน 800ms → abort เพื่อความปลอดภัย
  stuckMs = millis();
  lastMovedL_ = 0;
  lastMovedR_ = 0;

  startL = countL;
  startR = countR;  // ✅ เพิ่มบรรทัดนี้

  targetAbsL = (long)lroundf(fabs(rev) * COUNTS_PER_REV_L);
  if (targetAbsL < 1) targetAbsL = 1;

  revMode = REV_L;
  setMotorL(cmdPwmL, fwdL);
}

//แปลง rev เป็น tick ด้วย COUNTS_PER_REV_X เก็บ start count ตั้ง mode + สั่งมอเตอร์เริ่มวิ่ง
void startRevR(float rev, int pwm) {
  doneLatchedL = false;
  doneLatchedR = false;
  fwdR = (rev > 0);
  cmdPwmR = constrain(pwm, 0, 255);

  revStartMs = millis();
  stuckMs = millis();
  lastMovedL_ = 0;
  lastMovedR_ = 0;

  startL = countL;  // ✅ จะเก็บไว้ก็ได้เพื่อให้ print ไม่เพี้ยน
  startR = countR;  // ✅ ให้ถูกต้องแน่นอน

  targetAbsR = (long)lroundf(fabs(rev) * COUNTS_PER_REV_R);
  if (targetAbsR < 1) targetAbsR = 1;

  revMode = REV_R;
  setMotorR(cmdPwmR, fwdR);
}

//แปลง rev เป็น tick ด้วย COUNTS_PER_REV_X เก็บ start count ตั้ง mode + สั่งมอเตอร์เริ่มวิ่ง
void startRevBoth(float rev, int pwmL, int pwmR) {
  doneLatchedL = false;
  doneLatchedR = false;
  bool fwd = (rev > 0);
  fwdL = fwdR = fwd;
  cmdPwmL = constrain(pwmL, 0, 255);
  cmdPwmR = constrain(pwmR, 0, 255);

  revStartMs = millis();
  stuckMs = millis();
  lastMovedL_ = 0;
  lastMovedR_ = 0;

  startL = countL;
  startR = countR;
  targetAbsL = (long)lroundf(fabs(rev) * COUNTS_PER_REV_L);
  targetAbsR = (long)lroundf(fabs(rev) * COUNTS_PER_REV_R);
  if (targetAbsL < 1) targetAbsL = 1;
  if (targetAbsR < 1) targetAbsR = 1;
  revMode = REV_BOTH;
  setMotorL(cmdPwmL, fwdL);
  setMotorR(cmdPwmR, fwdR);
  DPRINTF("REV BOTH START | targetL=%ld targetR=%ld | dir=%s | pwmL=%d pwmR=%d\n",
          targetAbsL, targetAbsR, fwdL ? "FWD" : "BWD", cmdPwmL, cmdPwmR);
}

//ใช้สำหรับคำสั่ง m (เมตร→tick) หรือถ้าคุณอยากสั่ง tick ตรงๆ
void startTicksBoth(long ticksL, long ticksR, int pwmL, int pwmR, bool forward = true) {
  doneLatchedL = false;
  doneLatchedR = false;

  fwdL = forward;
  fwdR = forward;

  cmdPwmL = constrain(pwmL, 0, 255);
  cmdPwmR = constrain(pwmR, 0, 255);

  revStartMs = millis();
  stuckMs = millis();
  lastMovedL_ = 0;
  lastMovedR_ = 0;

  startL = countL;
  startR = countR;

  targetAbsL = max(1L, labs(ticksL));
  targetAbsR = max(1L, labs(ticksR));

  revMode = REV_BOTH;

  setMotorL(cmdPwmL, fwdL);
  setMotorR(cmdPwmR, fwdR);

  DPRINTF("TICK BOTH START | targetL=%ld targetR=%ld | dir=%s | pwmL=%d pwmR=%d\n",
          targetAbsL, targetAbsR, forward ? "FWD" : "BWD", cmdPwmL, cmdPwmR);
}


// ramp helper
int rampPwm(long remain, int cmdPwm, long decelCounts) {

  int out = cmdPwm;
  if (remain <= decelCounts) {
    float t = (float)remain / (float)decelCounts;
    out = (int)lroundf(PWM_MIN + t * (cmdPwm - PWM_MIN));
    out = constrain(out, PWM_MIN, cmdPwm);
  }
  return out;
}

void setup() {
  DBG.begin(115200);                          // ดู log ผ่าน USB
  ROS.begin(115200, SERIAL_8N1, RXD2, TXD2);  // คุยกับ Pi

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // NOTE: GPIO34/35 have no internal pullups. If your encoder outputs are strong digital, INPUT is OK.
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncR, RISING);

  //attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncR, CHANGE);

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_L);
  ledcAttachPin(ENB, PWM_CH_R);

  coastStopAll();
  lastMs = millis();
  lastCountL = countL;
  lastCountR = countR;

  printHelp();
}

void loop() {
  unsigned long now = millis();
  updateOdometry(now);

  // ---- ALWAYS publish encoder for ROS2 bridge ----
  static unsigned long lastE = 0;
  if (now - lastE >= 50) {  // 20Hz
    lastE = now;
    ROS.printf("E %lu %ld %ld\n", now, countL, countR);
  }

  // ✅ ODOM PRINT (ใส่ตรงนี้)
  static unsigned long lastOdomPrint = 0;
  if (now - lastOdomPrint >= 100) {
    lastOdomPrint = now;
    ROS.printf("O %.3f %.3f %.3f\n", odom_x, odom_y, odom_yaw);
  }

  String line;

  // =========================
  // 1) CMD_VEL MODE (RUN)
  // =========================
  if (velMode) {

    // อ่านคำสั่งเข้ามา (ถ้ามี)
    if (readLine(line)) {
      line.toLowerCase();

      if (line == "stopv") {
        velMode = false;
        velStoppedLatch = false;
        iTermL = iTermR = 0;
        pwmCmdL = pwmCmdR = 0;
        tgtRpmL_cmd = tgtRpmR_cmd = 0;
        tgtRpmL = tgtRpmR = 0;
        coastStopAll();
        DPRINTLN("CMD_VEL STOPPED");
        return;
      }

      if (line.startsWith("v ")) {
        // parse: v <linear_mps> <angular_radps>
        int sp1 = line.indexOf(' ');
        int sp2 = line.indexOf(' ', sp1 + 1);

        if (sp1 < 0 || sp2 < 0) {
          DPRINTLN("BAD v cmd");
          return;
        }

        float v = line.substring(sp1 + 1, sp2).toFloat();

        String sw = line.substring(sp2 + 1);
        sw.trim();
        if (sw.length() == 0) {
          DPRINTLN("BAD v cmd");
          return;
        }
        float w = sw.toFloat();

        float vL = v - (w * WHEEL_BASE * 0.5f);
        float vR = v + (w * WHEEL_BASE * 0.5f);

        float circ = 2.0f * 3.1415926f * WHEEL_R;
        tgtRpmL_cmd = (vL / circ) * 60.0f;
        tgtRpmR_cmd = (vR / circ) * 60.0f;

        if (fabs(tgtRpmL_cmd) < 0.5f) tgtRpmL_cmd = 0;
        if (fabs(tgtRpmR_cmd) < 0.5f) tgtRpmR_cmd = 0;

        DPRINTF("CMD_VEL UPDATE | v=%.3f w=%.3f | cmdRpmL=%.2f cmdRpmR=%.2f\n",
                v, w, tgtRpmL_cmd, tgtRpmR_cmd);
      }
    }

    // ---- (B) PI speed control (magnitude-based, supports backward) ----
    if (now - lastVelMs >= 50) {
      lastVelMs = now;

      float rpmL, rpmR;
      computeRpms(now, rpmL, rpmR);

      // ===== RAMP TARGET RPM (soft start/stop) =====
      float dt = 0.05f;  // 50ms
      float stepUp = ACC_RPM_PER_S * dt;
      float stepDn = DEC_RPM_PER_S * dt;

      auto stepToward = [&](float cur, float target) -> float {
        float step = (fabs(target) > fabs(cur)) ? stepUp : stepDn;  // เร่งใช้ stepUp, ผ่อนใช้ stepDn
        if (target > cur + step) return cur + step;
        if (target < cur - step) return cur - step;
        return target;
      };

      tgtRpmL = stepToward(tgtRpmL, tgtRpmL_cmd);
      tgtRpmR = stepToward(tgtRpmR, tgtRpmR_cmd);

      bool forwardL = (tgtRpmL >= 0);
      bool forwardR = (tgtRpmR >= 0);

      float tgtAbsL = fabs(tgtRpmL);
      float tgtAbsR = fabs(tgtRpmR);
      float rpmAbsL = fabs(rpmL);
      float rpmAbsR = fabs(rpmR);

      float eL = tgtAbsL - rpmAbsL;
      float eR = tgtAbsR - rpmAbsR;

      iTermL = constrain(iTermL + KI_V_L * eL, -80, 80);
      iTermR = constrain(iTermR + KI_V_R * eR, -80, 80);

      int uL = (int)(KP_V_L * eL + iTermL);
      int uR = (int)(KP_V_R * eR + iTermR);

      pwmCmdL = constrain(uL, 0, 255);
      pwmCmdR = constrain(uR, 0, 255);

      // --- GAIN compensation (ซ้าย/ขวาไม่เท่ากัน) ---
      pwmCmdL = (int)(pwmCmdL * gainL(forwardL));
      pwmCmdR = (int)(pwmCmdR * gainR(forwardR));
      pwmCmdL = constrain(pwmCmdL, 0, 255);
      pwmCmdR = constrain(pwmCmdR, 0, 255);

      // deadband + PWM_MIN
      if (tgtAbsL < 0.5f) pwmCmdL = 0, iTermL = 0;
      if (tgtAbsR < 0.5f) pwmCmdR = 0, iTermR = 0;

      if (tgtAbsL >= 0.5f && pwmCmdL > 0 && pwmCmdL < PWM_MIN) pwmCmdL = PWM_MIN;
      if (tgtAbsR >= 0.5f && pwmCmdR > 0 && pwmCmdR < PWM_MIN) pwmCmdR = PWM_MIN;

      // ✅ ถ้าทั้งสองข้างเป้าหมายเป็น 0 -> เคลียร์ state และหยุดทันที
      if (fabs(tgtRpmL_cmd) < 0.5f && fabs(tgtRpmR_cmd) < 0.5f) {
        if (!velStoppedLatch) {
          velStoppedLatch = true;
          tgtRpmL_cmd = tgtRpmR_cmd = 0;
          tgtRpmL = tgtRpmR = 0;
          iTermL = iTermR = 0;
          pwmCmdL = pwmCmdR = 0;
          brakeHoldL();
          brakeHoldR();
          delay(80);
          coastStopAll();
        }
        return;
      } else {
        velStoppedLatch = false;
      }


      setMotorL(pwmCmdL, forwardL);
      setMotorR(pwmCmdR, forwardR);
    }
    return;
  }


  // ---------- REV LOOP ----------
  if (revMode != REV_NONE) {
    long cL = countL, cR = countR;

    long movedL = labs(cL - startL);
    long movedR = labs(cR - startR);

    // ✅ คำนวณ done ก่อน เพื่อใช้กับ stuck detect
    bool doneL = (revMode == REV_R) ? true : (movedL >= targetAbsL);
    bool doneR = (revMode == REV_L) ? true : (movedR >= targetAbsR);

    // --- timeout กันค้างยาว ---
    if (now - revStartMs > 15000) {  // 15s ปรับได้
      DPRINTLN("REV TIMEOUT -> ABORT");
      brakeHoldL();
      brakeHoldR();
      delay(120);
      coastStopAll();
      revMode = REV_NONE;
      doneLatchedL = doneLatchedR = false;
      printHelp();
      return;
    }

    // --- stuck detect (เฉพาะล้อที่ "ยังต้องวิ่ง") ---
    bool needMoveL = (revMode == REV_L || revMode == REV_BOTH) && !doneL;
    bool needMoveR = (revMode == REV_R || revMode == REV_BOTH) && !doneR;

    // true ถ้าล้อทุกล้อที่ "ต้องวิ่ง" ติดหมด
    bool allNeededStuck = true;
    if (needMoveL) allNeededStuck &= (movedL == lastMovedL_);
    if (needMoveR) allNeededStuck &= (movedR == lastMovedR_);
    if (!needMoveL && !needMoveR) allNeededStuck = false;

    if (allNeededStuck) {
      if (now - stuckMs > 800) {
        DPRINTLN("REV STUCK -> ABORT");
        brakeHoldL();
        brakeHoldR();
        delay(120);
        coastStopAll();
        revMode = REV_NONE;
        doneLatchedL = doneLatchedR = false;
        DPRINTLN("REV ABORTED");
        printHelp();
        return;
      }
    } else {
      stuckMs = now;
    }

    // ✅ อัปเดต lastMoved ทุกลูป
    lastMovedL_ = movedL;
    lastMovedR_ = movedR;

    // ... แล้วค่อยไปคำนวณ remainL/remainR ต่อ ...

    long remainL = (revMode == REV_L || revMode == REV_BOTH) ? (targetAbsL - movedL) : 999999999;
    long remainR = (revMode == REV_R || revMode == REV_BOTH) ? (targetAbsR - movedR) : 999999999;

    // ✅ clamp ไม่ให้ติดลบ (สำคัญ)
    if (remainL < 0) remainL = 0;
    if (remainR < 0) remainR = 0;

    int outL = (revMode == REV_L || revMode == REV_BOTH) ? rampPwm(remainL, cmdPwmL, DECEL_L) : 0;
    int outR = (revMode == REV_R || revMode == REV_BOTH) ? rampPwm(remainR, cmdPwmR, DECEL_R) : 0;

    // ===== SYNC (เฉพาะช่วง cruise และยังไม่ถึงเป้าทั้งคู่) =====
    if (revMode == REV_BOTH) {
      bool inCruise = (remainL > (long)(DECEL_L * 0.6f)) && (remainR > (long)(DECEL_R * 0.6f));
      if (inCruise && !doneL && !doneR) {
        float progL = (float)movedL / (float)targetAbsL;
        float progR = (float)movedR / (float)targetAbsR;
        float errP = progL - progR;

        int corr = (int)(KP_SYNC * errP * 255.0f);
        corr = constrain(corr, -SYNC_MAX, SYNC_MAX);

        outL -= corr;
        outR += corr;

        outL = constrain(outL, 0, min(255, cmdPwmL + SYNC_MAX));
        outR = constrain(outR, 0, min(255, cmdPwmR + SYNC_MAX));
      }
    }

    // ✅ PRECUT: ใกล้ถึงแล้วปล่อย PWM = 0 (กันไหล/กัน overshoot)
    if (revMode == REV_BOTH) {
      if (remainL <= PRECUT_L) outL = 0;
      if (remainR <= PRECUT_R) outR = 0;
    }

    // ✅ BRAKE WINDOW (โหมด BOTH)
    if (revMode == REV_BOTH) {
      if (!doneLatchedL && !doneL && remainL <= BRAKE_WINDOW_L) {
        DPRINTLN("L WINDOW(BOTH) -> BRAKE");
        brakeStopL(BRAKE_MS_L);
        doneLatchedL = true;
        outL = 0;
      }
      if (!doneLatchedR && !doneR && remainR <= BRAKE_WINDOW_R) {
        DPRINTLN("R WINDOW(BOTH) -> BRAKE");
        brakeStopR(BRAKE_MS_R);
        doneLatchedR = true;
        outR = 0;
      }
    }

    // ✅ BRAKE WINDOW (โหมดเดี่ยว) ให้หยุดแม่นขึ้น
    if (revMode == REV_L) {
      if (!doneLatchedL && remainL <= BRAKE_WINDOW_L) {
        DPRINTLN("L WINDOW(SINGLE) -> BRAKE");
        brakeStopL(BRAKE_MS_L);
        doneLatchedL = true;
        outL = 0;
        // optional
      }
    }

    if (revMode == REV_R) {
      if (!doneLatchedR && remainR <= BRAKE_WINDOW_R) {
        DPRINTLN("R WINDOW(SINGLE) -> BRAKE");
        brakeStopR(BRAKE_MS_R);
        doneLatchedR = true;
        outR = 0;
        // optional
      }
    }

    // ✅ (A) BOTH: ล้อไหนถึงก่อน เบรกทันที 1 ครั้ง แล้ว latch
    if (revMode == REV_BOTH) {
      if (doneL && !doneLatchedL) {
        DPRINTLN("L HIT -> BRAKE");
        brakeStopL(BRAKE_MS_L);
        doneLatchedL = true;
        outL = 0;
        // optional
      }

      if (doneR && !doneLatchedR) {
        DPRINTLN("R HIT -> BRAKE");
        brakeStopR(BRAKE_MS_R);
        doneLatchedR = true;
        outR = 0;
        // optional
      }
    }

    // ✅ (B) ถ้าเบรกไปแล้ว ห้ามสั่งมอเตอร์ล้อนั้นอีก
    if (revMode == REV_BOTH) {
      if (doneLatchedL) outL = 0;
      if (doneLatchedR) outR = 0;
    }

    static unsigned long lastDbg = 0;
    if (now - lastDbg > 300) {
      lastDbg = now;
      DPRINTF("DBG movedL=%ld movedR=%ld remainL=%ld remainR=%ld outL=%d outR=%d latchL=%d latchR=%d doneL=%d doneR=%d\n",
              movedL, movedR, remainL, remainR, outL, outR, doneLatchedL, doneLatchedR, doneL, doneR);
    }

    // ----- print -----
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 200) {
      lastPrint = now;
      float rpmL, rpmR;
      computeRpms(now, rpmL, rpmR);
      DPRINTF("REV | L:%ld/%ld rem:%ld pwm:%d rpm:%.2f | R:%ld/%ld rem:%ld pwm:%d rpm:%.2f\n",
              movedL, targetAbsL, remainL, (revMode == REV_R ? 0 : outL), rpmL,
              movedR, targetAbsR, remainR, (revMode == REV_L ? 0 : outR), rpmR);
    }

    // ===== APPLY =====
    if (revMode == REV_L || revMode == REV_BOTH) {
      if (!doneLatchedL) {
        // ถ้าอีกล้อถูก latch แล้ว แต่เรายังไม่ done -> finish mode ให้แรงพอ
        int pwmL = outL;
        if (revMode == REV_BOTH && doneLatchedR && !doneL) {
          pwmL = max(pwmL, FINISH_PWM);
        }
        setMotorL(pwmL, fwdL);
      } else {
        brakeHoldL();
      }
    } else {
      coastStopL();
    }

    if (revMode == REV_R || revMode == REV_BOTH) {
      if (!doneLatchedR) {
        int pwmR = outR;
        if (revMode == REV_BOTH && doneLatchedL && !doneR) {
          pwmR = max(pwmR, FINISH_PWM);
        }
        setMotorR(pwmR, fwdR);
      } else {
        brakeHoldR();
      }
    } else {
      coastStopR();
    }

    // ===== DONE =====
    if (doneL && doneR) {
      // ✅ กันไหล: hold ไว้ก่อน
      brakeHoldL();
      brakeHoldR();
      delay(200);  // 150–300ms ปรับได้

      // ✅ แล้วค่อยปล่อย (ลดร้อน/กินกระแส)
      coastStopAll();

      revMode = REV_NONE;
      doneLatchedL = false;
      doneLatchedR = false;

      DPRINTF("REV DONE. Final L=%ld | R=%ld\n", (long)countL, (long)countR);
      printHelp();
    }
    return;
  }

  // ---------- NORMAL MODE ----------
  if (!readLine(line)) return;
  if (!line.length()) return;

  line.toLowerCase();
  if (line == "h") {
    printHelp();
    return;
  }
  if (line == "s") {
    coastStopAll();
    DPRINTLN("Motor: STOP (coast)");
    return;
  }
  if (line == "z") {
    // ✅ หยุดโหมดก่อน
    revMode = REV_NONE;
    velMode = false;
    doneLatchedL = doneLatchedR = false;
    coastStopAll();

    // ✅ แล้วค่อยรีเซ็ต encoder
    noInterrupts();
    countL = 0;
    countR = 0;
    interrupts();

    lastCountL = 0;
    lastCountR = 0;

    // ✅ reset odom state
    odom_x = 0;
    odom_y = 0;
    odom_yaw = 0;
    odom_lastL = 0;
    odom_lastR = 0;
    odom_lastMs = 0;

    DPRINTLN("Encoder count ZEROED (L+R) + ODOM RESET.");
    return;
  }
  if (line == "r") {
    float rpmL, rpmR;
    computeRpms(now, rpmL, rpmR);
    DPRINTF("Read | L:%ld rpm:%.2f | R:%ld rpm:%.2f\n", (long)countL, rpmL, (long)countR, rpmR);
    return;
  }
  if (line == "xl") {
    invEncL = !invEncL;
    DPRINTF("invEncL=%s\n", invEncL ? "TRUE" : "FALSE");
    return;
  }
  if (line == "xr") {
    invEncR = !invEncR;
    DPRINTF("invEncR=%s\n", invEncR ? "TRUE" : "FALSE");
    return;
  }

  // helpers: parse tokens
  auto tok = [&](int idx) -> String {
    int n = 0, start = 0;
    for (int i = 0; i <= line.length(); i++) {
      if (i == line.length() || line[i] == ' ') {
        if (i > start) {
          if (n == idx) return line.substring(start, i);
          n++;
        }
        start = i + 1;
      }
    }
    return "";
  };

  String c0 = tok(0);
  if (c0.length() == 0) return;

  // tuning
  // d <counts>  => ตั้ง DECEL ทั้ง L และ R (รักษาสัดส่วนเดิม)

  if (c0 == "d") {
    long v = tok(1).toInt();
    if (v >= 10) {
      long delta = DECEL_R - DECEL_L;  // เก็บความต่างเดิมไว้
      DECEL_L = v;
      DECEL_R = v + delta;
      DPRINTF("DECEL_L=%ld | DECEL_R=%ld\n", DECEL_L, DECEL_R);
    } else {
      DPRINTLN("Usage: d <counts>");
    }
    return;
  }
  if (c0 == "p") {
    int v = tok(1).toInt();
    if (v >= 0 && v <= 255) {
      PWM_MIN = v;
      DPRINTF("PWM_MIN=%d\n", PWM_MIN);
    } else DPRINTLN("Usage: p <0-255>");
    return;
  }
  // k <ms> => ตั้ง BRAKE ทั้ง L และ R (รักษาสัดส่วนเดิม)
  if (c0 == "k") {
    int v = tok(1).toInt();
    if (v >= 0 && v <= 500) {
      int delta = BRAKE_MS_R - BRAKE_MS_L;
      BRAKE_MS_L = v;
      BRAKE_MS_R = v + delta;
      DPRINTF("BRAKE_MS_L=%d | BRAKE_MS_R=%d\n", BRAKE_MS_L, BRAKE_MS_R);
    } else {
      DPRINTLN("Usage: k <0-500>");
    }
    return;
  }

  if (c0 == "v") {
    float v = tok(1).toFloat();  // m/s
    float w = tok(2).toFloat();  // rad/s
    velMode = true;
    lastVelMs = millis();
    revMode = REV_NONE;

    float vL = v - (w * WHEEL_BASE * 0.5f);
    float vR = v + (w * WHEEL_BASE * 0.5f);

    float circ = 2.0f * 3.1415926f * WHEEL_R;
    tgtRpmL_cmd = (vL / circ) * 60.0f;
    tgtRpmR_cmd = (vR / circ) * 60.0f;

    if (fabs(tgtRpmL_cmd) < 0.5f) tgtRpmL_cmd = 0;
    if (fabs(tgtRpmR_cmd) < 0.5f) tgtRpmR_cmd = 0;

    // (เลือกได้) รีเซ็ต ramped+PI ตอนเพิ่งเข้าโหมด เพื่อไม่ให้ carry ค่าเดิม
    // tgtRpmL = 0; tgtRpmR = 0;
    // iTermL = iTermR = 0;

    DPRINTF("CMD_VEL | v=%.3f w=%.3f | cmdRpmL=%.2f cmdRpmR=%.2f\n",
            v, w, tgtRpmL_cmd, tgtRpmR_cmd);
    return;
  }

  if (c0 == "stopv") {  // ปิดโหมด cmd_vel
    velMode = false;
    velStoppedLatch = false;
    iTermL = iTermR = 0;
    pwmCmdL = pwmCmdR = 0;
    tgtRpmL_cmd = tgtRpmR_cmd = 0;
    tgtRpmL = tgtRpmR = 0;
    coastStopAll();
    DPRINTLN("CMD_VEL STOPPED");
    return;
  }

  // dl <counts>, dr <counts>
  if (c0 == "dl") {
    long v = tok(1).toInt();
    if (v >= 10) {
      DECEL_L = v;
      DPRINTF("DECEL_L=%ld\n", DECEL_L);
    } else DPRINTLN("Usage: dl <counts>");
    return;
  }
  if (c0 == "dr") {
    long v = tok(1).toInt();
    if (v >= 10) {
      DECEL_R = v;
      DPRINTF("DECEL_R=%ld\n", DECEL_R);
    } else DPRINTLN("Usage: dr <counts>");
    return;
  }

  // kl <ms>, kr <ms>
  if (c0 == "kl") {
    int v = tok(1).toInt();
    if (v >= 0 && v <= 500) {
      BRAKE_MS_L = v;
      DPRINTF("BRAKE_MS_L=%d\n", BRAKE_MS_L);
    } else DPRINTLN("Usage: kl <0-500>");
    return;
  }
  if (c0 == "kr") {
    int v = tok(1).toInt();
    if (v >= 0 && v <= 500) {
      BRAKE_MS_R = v;
      DPRINTF("BRAKE_MS_R=%d\n", BRAKE_MS_R);
    } else DPRINTLN("Usage: kr <0-500>");
    return;
  }

  // single wheel manual run
  if (c0 == "el") {
    int pwm = tok(1).toInt();
    setMotorL(pwm, true);
    DPRINTF("LEFT FORWARD pwm=%d\n", pwm);
    return;
  }
  if (c0 == "bl") {
    int pwm = tok(1).toInt();
    setMotorL(pwm, false);
    DPRINTF("LEFT BACKWARD pwm=%d\n", pwm);
    return;
  }
  if (c0 == "er") {
    int pwm = tok(1).toInt();
    setMotorR(pwm, true);
    DPRINTF("RIGHT FORWARD pwm=%d\n", pwm);
    return;
  }
  if (c0 == "br") {
    int pwm = tok(1).toInt();
    setMotorR(pwm, false);
    DPRINTF("RIGHT BACKWARD pwm=%d\n", pwm);
    return;
  }

  // both manual run
  if (c0 == "ea") {
    int pL = constrain(tok(1).toInt(), 0, 255);
    int pR = constrain(tok(2).toInt(), 0, 255);


    int pL2 = (int)lroundf(pL * GAIN_L_F);
    int pR2 = (int)lroundf(pR * GAIN_R_F);

    pL2 = constrain(pL2, 0, 255);
    pR2 = constrain(pR2, 0, 255);

    setMotorL(pL2, true);
    setMotorR(pR2, true);

    DPRINTF("BOTH FORWARD pwmL=%d pwmR=%d (applied L=%d R=%d) | gainL=%.3f gainR=%.3f\n",
            pL, pR, pL2, pR2, GAIN_L_F, GAIN_R_F);
    return;
  }

  if (c0 == "ba") {
    int pL = constrain(tok(1).toInt(), 0, 255);
    int pR = constrain(tok(2).toInt(), 0, 255);

    int pL2 = (int)lroundf(pL * GAIN_L_B);
    int pR2 = (int)lroundf(pR * GAIN_R_B);

    pL2 = constrain(pL2, 0, 255);
    pR2 = constrain(pR2, 0, 255);

    setMotorL(pL2, false);
    setMotorR(pR2, false);

    DPRINTF("BOTH BACKWARD pwmL=%d pwmR=%d (applied L=%d R=%d) | gainL=%.3f gainR=%.3f\n",
            pL, pR, pL2, pR2, GAIN_L_B, GAIN_R_B);
    return;
  }


  // REV commands
  if (c0 == "vl") {
    float rev = tok(1).toFloat();
    int pwm = constrain(tok(2).toInt(), 0, 255);

    bool fwd = (rev > 0);
    int pwm2 = (int)lroundf(pwm * gainL(fwd));  // สำหรับล้อซ้าย
    pwm2 = constrain(pwm2, 0, 255);

    if (rev != 0 && pwm > 0 && pwm2 > 0) startRevL(rev, pwm2);
    else DPRINTLN("Usage: vl <rev> <pwm>");
    return;
  }

  if (c0 == "vr") {
    float rev = tok(1).toFloat();
    int pwm = constrain(tok(2).toInt(), 0, 255);

    bool fwd = (rev > 0);
    int pwm2 = (int)lroundf(pwm * gainR(fwd));  // สำหรับล้อขวา
    pwm2 = constrain(pwm2, 0, 255);

    if (rev != 0 && pwm > 0 && pwm2 > 0) startRevR(rev, pwm2);
    else DPRINTLN("Usage: vr <rev> <pwm>");
    return;
  }

  if (c0 == "va") {
    float rev = tok(1).toFloat();
    int pwmL = constrain(tok(2).toInt(), 0, 255);
    int pwmR = constrain(tok(3).toInt(), 0, 255);

    bool fwd = (rev > 0);

    int pwmL2 = (int)lroundf(pwmL * gainL(fwd));
    int pwmR2 = (int)lroundf(pwmR * gainR(fwd));
    pwmL2 = constrain(pwmL2, 0, 255);
    pwmR2 = constrain(pwmR2, 0, 255);

    if (rev != 0 && pwmL > 0 && pwmR > 0 && pwmL2 > 0 && pwmR2 > 0)
      startRevBoth(rev, pwmL2, pwmR2);
    else
      DPRINTLN("Usage: va <rev> <pwmL> <pwmR>");
    return;
  }

  if (c0 == "m") {
    float meters = tok(1).toFloat();
    int pwm = constrain(tok(2).toInt(), 0, 255);
    if (meters == 0 || pwm <= 0) {
      DPRINTLN("Usage: m <meters> <pwm>");
      return;
    }

    bool forward = (meters > 0);
    float mabs = fabs(meters);

    long targetL = lroundf(mabs * TICKS_PER_M_L);
    long targetR = lroundf(mabs * TICKS_PER_M_R);

    int pL2 = (int)lroundf(pwm * gainL(forward));
    int pR2 = (int)lroundf(pwm * gainR(forward));
    pL2 = constrain(pL2, 0, 255);
    pR2 = constrain(pR2, 0, 255);
    DPRINTF("M CMD | meters=%.3f pwm=%d | targetL=%ld targetR=%ld | appliedL=%d appliedR=%d\n",
            meters, pwm, targetL, targetR, pL2, pR2);
    startTicksBoth(targetL, targetR, pL2, pR2, forward);
    return;
  }

  DPRINTLN("Unknown command. Type h");
}

#include <Servo.h>

// تعريف الدبابيس
const int trigPin = 12;    // TRIG للمستشعر
const int echoPin = 13;    // ECHO للمستشعر
Servo motor;               // محرك سيرفو
const int motorPin = 5;    // دبوس المحرك

// متغيرات التحكم
float setPoint = 17.0;     // النقطة المرجعية (منتصف العارضة)
float Kp =1;            // ثابت التناسب (P) - زيادة طفيفة
float Ki = 0.0;           // ثابت التكامل (I) - قيمة صغيرة لمنع تراكم الأخطاء الزائد
float Kd = 0;            // ثابت التفاضل (D) - زيادة بسيطة لتقليل التجاوز

// متغيرات PID
float error = 0, lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// دوال
float readDistance();
void pidControl(float distance);

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  motor.attach(motorPin);  // توصيل السيرفو
  motor.write(90);         // البداية عند الوضع الأفقي
  
  Serial.begin(9600);      // لمراقبة النتائج
}

void loop() {
  // قراءة المسافة
  float distance = readDistance();
  
  if (distance >= 0 && distance <= 30) { // إذا كانت القراءة صحيحة
    Serial.print("Distance: ");
    Serial.println(distance);
    
    // التحكم بالموتور باستخدام PID
    pidControl(distance);
  } else {
     motor.write(90); // الوضع الأفقي الافتراضي
  }
  
  delay(10); // تقليل التأخير لضمان استجابة أسرع
}

// قراءة المسافة باستخدام Ultrasonic
float readDistance() {
  // إرسال نبضة TRIG قصيرة
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // عرض نبضة أطول قليلاً
  digitalWrite(trigPin, LOW);
  
  // قراءة مدة النبضة المرتدة مع مهلة زمنية قصيرة
  unsigned long duration = pulseIn(echoPin, HIGH, 20000); // مهلة 20ms
  if (duration == 0) {
    return -1; // لم تُرصد إشارة
  }
  
  // حساب المسافة وإرجاعها
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

// خوارزمية التحكم PID
void pidControl(float distance) {
  // حساب الخطأ
  error = setPoint - distance;
  
  // حساب التكامل والتفاضل
  integral += error * 0.01; // تحسين التكامل بقيمة صغيرة (لتجنب التراكم الزائد)
  derivative = (error - lastError) / 0.01; // حساب الفرق على فاصل زمني ثابت
  
  // حساب الخرج
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // ضبط زوايا المحرك (محدودة بين 0 و180)
  int angle = 90 + output;  // 90 هي الوضع الأفقي
  angle = constrain(angle, 70, 110);
   Serial.println(angle);
  
  motor.write(angle);       // تحريك الموتور
  lastError = error;        // تحديث الخطأ السابق
}

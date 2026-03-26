#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float desired_angle = 0;
float complementary_angle;

float acel_weight = 0.02;
float gyro_weight = 0.98;

int IN1 = 8;
int IN2 = 9;
int ENA = 10;

float Kp = 20; // empieza bajo (luego subes)
float Ki = 0.1; // empieza bajo (luego subes)
float error_accumulation = 0; // Para el control integral
float Imax = 100; // Límite para la acumulación del error integral (anti-windup)
float angle_increment = 0.001;
float prev_error = 0; // Para el control derivativo
float Kd = 0.5; // empieza bajo (luego subes)

unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    if (!mpu.begin()) {
        while (1) delay(10);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    complementary_angle = atan2(a.acceleration.x, a.acceleration.y) * 180 / PI;

    lastTime = millis();
    prev_error = desired_angle - complementary_angle;
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Δt real (MUY importante)
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Ángulo acelerómetro
    float angle_acel = atan2(a.acceleration.x, a.acceleration.y) * 180 / PI;

    // Giroscopio
    float gyroZ = g.gyro.z * 180 / PI;
    float gyro_angle = complementary_angle + gyroZ * dt;

    // Filtro complementario
    complementary_angle = acel_weight * angle_acel +
                          gyro_weight * gyro_angle;

    // Control proporcional
    float error = desired_angle - complementary_angle;

    // Integrate the error
    error_accumulation += error * dt;

    //Dithering
    if (error < 0) {
        desired_angle -= angle_increment;
    } else {
        desired_angle += angle_increment;
    }

    if(error_accumulation > Imax) {
        error_accumulation = Imax;
    } else if(error_accumulation < -Imax) {
        error_accumulation = -Imax;
    }   
    float error_deriv = error - prev_error;
    

    float duty = (Kp * error) + (Ki * error_accumulation) + (Kd * error_deriv);
    prev_error = error;
    // Limitar PWM
    duty = constrain(duty, -255, 255);

    // Control del motor (L293D)
    if (duty > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, duty);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, -duty);
    }

    Serial.print("Angle: ");
    Serial.print(complementary_angle);
    Serial.print(" | PWM: ");
    Serial.println(duty);

    delay(10);
}
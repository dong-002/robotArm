//发送方
#include <ESP8266WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <espnow.h>

//接收方的mac地址
uint8_t broadcastAddress[] = { 0x24, 0xD7, 0xEB, 0xC8, 0xE5, 0xA5 };
//发送数据的结构体
typedef struct struct_message {
  int pitch;  //y轴
  int roll;   //x轴
  int yaw;    //z轴
} struct_message;
struct_message myData;

Adafruit_MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
const float alpha = 0.9;  // 互补滤波器的alpha值，用于平滑角速度值
float angleX = 0;         // 绕X轴的角度
float angleY = 0;         // 绕Y轴的角度
float angleZ = 0;         // 绕Z轴的角度

//这是一个回调函数，将在发送消息时执行。
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
  } else {
    Serial.println("Delivery fail");
  }
}

void setup() {

  Wire.begin();
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  // if (!mpu.begin()) {
  //   Serial.println("Failed to find MPU6050 chip");
  //   while (1) {
  //     delay(10);
  //   }
  // }
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  //与另一个ESP-NOW设备配对以发送数据
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
  if (mpu.begin()) {
    /* 使用读数获取新的传感器事件 */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ////acceleration就是加速度的意思///
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;

    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;
    // 计算角速度值（单位：度/秒）
    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    // 使用互补滤波器平滑角速度值
    angleX = alpha * (angleX + gyroX * 0.01) + (1 - alpha) * (ax / 16384.0);
    angleY = alpha * (angleY + gyroY * 0.01) + (1 - alpha) * (ay / 16384.0);
    angleZ = alpha * (angleZ + gyroZ * 0.01) + (1 - alpha) * (az / 16384.0);

    // 将角度转换为旋转角度（单位：度）
    float pitch = atan2(-angleX, sqrt(angleY * angleY + angleZ * angleZ)) * 180.0 / PI;
    float roll = atan2(angleY, angleZ) * 180.0 / PI;

    // 打印旋转角度
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(", Roll: ");
    Serial.println(roll);
    // 将传感器数据转换为机械臂控制信号
    int pos1 = map(pitch, -90, 90, 0, 180);
    int pos2 = map(roll, -90, 90, 0, 180);
    int pos3 = map(az, -90, 90, 0, 180);

    myData.pitch = pos1;
    myData.roll = pos2;
    myData.yaw = pos3;
    //发送消息
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    // delay(5);
  }
}
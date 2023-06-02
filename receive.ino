// 接收方
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Servo.h>
#include <espnow.h>

// 双方的结构体要一致
typedef struct struct_message {
  int pitch; //y轴
  int roll; //x轴
  int yaw; //z轴
} struct_message;
struct_message myData;

// 机械臂舵机配置
Servo servo1;
Servo servo2;
Servo servo3;

//创建一个回调函数作为接收数据后的串口显示
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("pitch: ");
  Serial.print(myData.pitch);
  Serial.print("roll: ");
  Serial.print(myData.roll);
  Serial.print("yaw: ");
  Serial.println(myData.yaw);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  //初始化 ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //设置ESP8266角色：
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  //先前创建的功能 测试ESP-NOW通信
  esp_now_register_recv_cb(OnDataRecv);

  // 设置机械臂舵机
  servo1.attach(D9);
  servo2.attach(D10);
  servo3.attach(D11);
}

void loop() {
  // 控制机械臂运动
    servo1.write(myData.pitch);
    servo2.write(myData.roll);
    servo3.write(myData.yaw);
    delay(15);
}

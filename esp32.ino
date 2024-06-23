#include <string>
#include <Arduino.h>
#include "s8_uart.h"
#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Adafruit_GrayOLED.h>
#include "Adafruit_SHT4x.h"
#include <GyverBME280.h>


#define DEBUG_BAUDRATE 115200 // указывается скорость передачи данных

#define S8_UART_PORT  2 // указывается через какие пины будет работать sensair

#define WIFI_SSID "Xiaomi_77AD" // указывается к какой сети wifi нужно подключатся
#define WIFI_PASSWORD "12138739" //  указывается пароль для выбранной сети wifi

#define MQTT_HOST IPAddress(192, 168, 31, 50) // указывается место mqtt host к которому нужно подключатся
#define MQTT_PORT 1883 // указывается порт через который нужно подключатся к выбранному mqtt host

#define MQTT_USERNAME "orangepi" // указывается user от имени которого нужно подключатся к mqtt
#define MQTT_PASSWORD "orangepi" // указывается пароль юзера для подключения к mqtt

#define MQTT_SUB_TEST "test" // указывается топик mqtt к которому нужно подключатся

GyverBME280 bme;

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

HardwareSerial S8_serial(S8_UART_PORT); // создается переменная в которой указаны пины для подключения sensair

S8_UART *sensor_S8; // 
S8_sensor sensor; // 

AsyncWebServer server(80);

Adafruit_AHTX0 aht; // переменная для обращения к датчику aht 20

AsyncMqttClient mqttClient; // переменная для взаимодействия с mqtt
TimerHandle_t mqttReconnectTimer; // таймер подключения к mqtt
TimerHandle_t wifiReconnectTimer; // таймер подключения к wifi

unsigned long previousMillis = 0; //
const long interval = 300000; //

String messageTemp;

float Temp() // функция которая получает данные от aht 20 и возвращает полученный показатель температуры
{
  sensors_event_t humidity, temp; // запрашивает у датчика данные температуры и влажности, датчик делает замер
  while (!aht.getEvent(&humidity, &temp)) // проверяет замеряется ли сейчас температура и влажность
  {
    continue; // если датчик не замеряет, то продолжить выполнение функции
  }

  return temp.temperature; // вернуть полученные данные по температуре
}

float Hum() // функция которая получает данные от aht 20 и возвращает полученный показатель влажности
{
  sensors_event_t humidity, temp; // запрашивает у датчика данные температуры и влажности, датчик делает замер
    while (!aht.getEvent(&humidity, &temp)) // проверяет замеряется ли сейчас температура и влажность
  {
    continue; // если датчик не замеряет, то продолжить выполнение функции
  }

  return humidity.relative_humidity; // вернуть полученные данные по влажности
}

void connectToWifi() // функция подключающая к wifi
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // подключится к указанной сети wifi с указанным паролем
}

void connectToMqtt() // функция подключающая mqtt
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect(); // подключить mqtt
}

void WiFiEvent(WiFiEvent_t event) // функция проверяющая подключение к wifi
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) // проверяет какой отклик получен от wifi
  {
  case SYSTEM_EVENT_STA_GOT_IP: // ивент получаемый при успешном подключении wifi
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt(); // подключить mqtt
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED: // ивент получаемый при провальном подключении wifi
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // остановить таймер mqtt
    xTimerStart(wifiReconnectTimer, 0); // запустить таймер wifi
    break;
  }
}

void onMqttConnect(bool sessionPresent) // функция подключения к топику mqtt
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_SUB_TEST, 0); // подключится к указанному топику
  Serial.print("Subscribing at QoS 0, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  String messageTemp;

  for (int i = 0; i < len; i++)
    {
      Serial.print((char)payload[i]);
      messageTemp += (char)payload[i];
    }

  if (messageTemp == "Temperature")
    {
      float temp;
      temp = Temp();

      while(temp == (-50.00))
      {
        temp = Temp();  
      }

      uint16_t packetIdPub1 = mqttClient.publish(MQTT_SUB_TEST, 1, true, String(temp).c_str());   

    }

  if (messageTemp == "Humidity")
    {
      float hum;
      hum = Hum();

      while(hum == (-50.00))
      {
        hum = Hum();  
      }      

      uint16_t packetIdPub1 = mqttClient.publish(MQTT_SUB_TEST, 1, true, String(hum).c_str());   

    }

    if (messageTemp == "co2")
    {
        sensor.co2 = sensor_S8->get_co2();
        printf("CO2 value = %d ppm\n", sensor.co2);

        uint16_t packetIdPub1 = mqttClient.publish(MQTT_SUB_TEST, 1, true, String(sensor.co2).c_str());
    }

  if (messageTemp == "Condition")
    {

      sensors_event_t humidity, temp;
  
      uint32_t timestamp = millis();
      sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
      timestamp = millis() - timestamp;

      sensor.co2 = sensor_S8->get_co2();
      printf("CO2 value = %d ppm\n", sensor.co2);

      uint16_t packetIdPub1 = mqttClient.publish(MQTT_SUB_TEST, 1, true, String(bme.readPressure()).c_str());
      uint16_t packetIdPub2 = mqttClient.publish(MQTT_SUB_TEST, 1, true, String(humidity.relative_humidity).c_str());
      uint16_t packetIdPub3 = mqttClient.publish(MQTT_SUB_TEST, 1, true, String(temp.temperature).c_str());
      uint16_t packetIdPub4 = mqttClient.publish(MQTT_SUB_TEST, 1, true, String(sensor.co2).c_str());
    }

   if (strcmp(topic, MQTT_SUB_TEST) == 0)
    {
      Serial.println("TEST");
    }
}

void onMqttPublish(uint16_t packetId)
{
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void sendMqttMessage(){
  
}

void setup()
{
  Serial.begin(115200);
  Serial.println();

  if (!bme.begin(0x76)) Serial.println("bmp 280 not found");

  Serial.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }

  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
     case SHT4X_HIGH_PRECISION: 
       Serial.println("High precision");
       break;
     case SHT4X_MED_PRECISION: 
       Serial.println("Med precision");
       break;
     case SHT4X_LOW_PRECISION: 
       Serial.println("Low precision");
       break;
  }

  sht4.setHeater(SHT4X_NO_HEATER);
  switch (sht4.getHeater()) {
     case SHT4X_NO_HEATER: 
       Serial.println("No heater");
       break;
     case SHT4X_HIGH_HEATER_1S: 
       Serial.println("High heat for 1 second");
       break;
     case SHT4X_HIGH_HEATER_100MS: 
       Serial.println("High heat for 0.1 second");
       break;
     case SHT4X_MED_HEATER_1S: 
       Serial.println("Medium heat for 1 second");
       break;
     case SHT4X_MED_HEATER_100MS: 
       Serial.println("Medium heat for 0.1 second");
       break;
     case SHT4X_LOW_HEATER_1S: 
       Serial.println("Low heat for 1 second");
       break;
     case SHT4X_LOW_HEATER_100MS: 
       Serial.println("Low heat for 0.1 second");
       break;
  }
  
  S8_serial.begin(S8_BAUDRATE);
  sensor_S8 = new S8_UART(S8_serial);

  sensor_S8->get_firmware_version(sensor.firm_version);
  int len = strlen(sensor.firm_version);
  if (len == 0) {
      Serial.println("SenseAir S8 CO2 sensor not found!");
      while (1) { delay(1); };
  }

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();

  // server.on ( "/", HTTP_GET, [] ( AsyncWebServerRequest * request) {
  // request->send(200, "text/plain", "Hi! I am ESP32.");
  // });

  // AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  // server.begin();
  // Serial.println("HTTP server started");
}

void loop()
{
  // AsyncElegantOTA.loop();
}
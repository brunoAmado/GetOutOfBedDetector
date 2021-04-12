#include <Arduino.h>
/* Bibliotecas FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
/* Biblioteca Wifi */
#include <WiFi.h>
/* Biblioteca MQTT exemplos em : https://platformio.org/lib/show/89/PubSubClient/examples */
#include <PubSubClient.h>
/* Biblioteca Sensor */
#include <Adafruit_Sensor.h>
//Temp sensor
#include <DHT.h>
//Sonar
#include <HCSR04.h>
//Adding Alexa
#include "fauxmoESP.h"

#include "credentials.h"

/* Definição de Handlers de Fila para publicar leituras */
QueueHandle_t xQueue;

//serial
#define SERIAL_BAUDRATE 9600

//mqttServer
#define MQTTSERVER "192.168.1.2"
#define MQTTSERVERPORT 1883

#define DEVICE_1 "Sonar"
#define DEVICE_2 "Touch"

//sonar Param
#define MAXDISTANCE 120
#define MINDISTANCE 2

//Led
#define LED_PIN 2

//temp sensor
#define DHT_PIN 5

//Touch
#define TOUCH_PIN 13
#define MAXSENCIBILITY 30 //heavy touch value
#define MINSENCIBILITY 73 //light touch value

//relay
#define RELAY_PIN_1 32
#define RELAY_PIN_2 33

#define TRIGGER_PIN 21
//byte triggerPin = 21;
#define ECHO_PIN 12
//byte echoPin = 12;

//virtual devices for Alexa
fauxmoESP fauxmo;

//Struture to save value of Sensor
struct MySensor
{
  float temperature;
  float humidity;
  float distance;
  float touch;
};

void vTaskPublish(void *pvParameters)
{
  Serial.begin(SERIAL_BAUDRATE);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(RELAY_PIN_1, OUTPUT);
  digitalWrite(RELAY_PIN_1, LOW);

  pinMode(RELAY_PIN_2, OUTPUT);
  digitalWrite(RELAY_PIN_2, LOW);

  // Ligar ao Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Connecting to WiFi..");
  }

  // By default, fauxmoESP creates it's own webserver on the defined port
  // The TCP port must be 80 for gen3 devices (default is 1901)
  // This has to be done before the call to enable()
  fauxmo.createServer(true); // not needed, this is the default value
  fauxmo.setPort(80);        // This is required for gen3 devices

  // You have to call enable(true) once you have a WiFi connection
  // You can enable or disable the library at any moment
  // Disabling it will prevent the devices from being discovered and switched
  fauxmo.enable(true);
  // You can use different ways to invoke alexa to modify the devices state:
  // "Alexa, turn lamp two on"

  // Add virtual devices
  fauxmo.addDevice(DEVICE_1);
  fauxmo.addDevice(DEVICE_2);

  fauxmo.onSetState([](unsigned char device_id, const char *device_name, bool state, unsigned char value) {
    // Callback when a command from Alexa is received.
    // You can use device_id or device_name to choose the element to perform an action onto (relay, LED,...)
    // State is a boolean (ON/OFF) and value a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
    // Just remember not to delay too much here, this is a callback, exit as soon as possible.
    // If you have to do something more involved here set a flag and process it in your main loop.

    Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);

    if ((strcmp(device_name, DEVICE_1) == 0))
    {
      // this just sets a variable that the main loop() does something about
      //Serial.println("RELAY 1 switched by Alexa");
      //digitalWrite(RELAY_PIN_1, !digitalRead(RELAY_PIN_1));
      if (state && !digitalRead(RELAY_PIN_1))
      {
        Serial.println("RELAY 1 switched ON by Alexa");
        digitalWrite(RELAY_PIN_1, HIGH);
      }
      else if (!state && digitalRead(RELAY_PIN_1))
      {
        Serial.println("RELAY 1 switched OFF by Alexa");
        digitalWrite(RELAY_PIN_1, LOW);
      }
      else
      {
        Serial.println("RELAY 1 switch is already as Asked by Alexa");
      }
    }

    if ((strcmp(device_name, DEVICE_2) == 0))
    {
      // this just sets a variable that the main loop() does something about
      //Serial.println("RELAY 2 switched by Alexa");
      //digitalWrite(RELAY_PIN_2, !digitalRead(RELAY_PIN_2));
      if (state && !digitalRead(RELAY_PIN_2))
      {
        Serial.println("RELAY 2 switched ON by Alexa");
        digitalWrite(RELAY_PIN_2, HIGH);
      }
      else if (!state && digitalRead(RELAY_PIN_2))
      {
        Serial.println("RELAY 2 switched OFF by Alexa");
        digitalWrite(RELAY_PIN_2, LOW);
      }
      else
      {
        Serial.println("RELAY 2 switch is already as Asked by Alexa");
      }
    }
  });

  WiFiClient espClient;                             //Acesso a Interface WiFi
  PubSubClient mqttClient(espClient);               //Criação do cliente MQTT
  mqttClient.setServer(MQTTSERVER, MQTTSERVERPORT); // COnfiguração do Servidor

  MySensor sensor;

  while (1)
  {

    // fauxmoESP uses an async TCP server but a sync UDP server
    // Therefore, we have to manually poll for UDP packets
    fauxmo.handle();

    if (xQueueReceive(xQueue, &sensor, portMAX_DELAY) == pdTRUE) //API REF: https://www.freertos.org/a00118.html
    {
      //touch LEd and no object in range
      if (sensor.touch < MINSENCIBILITY && sensor.touch > MAXSENCIBILITY)
      {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("Light Touch Detected");
      }
      else if (sensor.touch < MAXSENCIBILITY)
      {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("Heavy Touch Detected");
      }
      else
      {
        digitalWrite(LED_PIN, LOW);
        Serial.println("No Touch Detected");
      }

      //Touch do aReset and Off Relay_1 for some time
      if (sensor.touch < MINSENCIBILITY)
      {
        fauxmo.setState("SensorAvo", true, 1);
      }

      if (sensor.distance < MAXDISTANCE && sensor.distance > MINDISTANCE)
      {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("Object Detected ");
      }
      else
      {
        digitalWrite(LED_PIN, LOW);
        Serial.println("No Object Detected");
      }

      if (!mqttClient.connected())
      {
        mqttClient.connect(String(WiFi.macAddress()).c_str(), MQTTUSERNAME, MQTTPASSWORD);
        Serial.printf("OFFLINE: Temperatura: %f Humidade: %f Distance: %f Touch: %f \n", sensor.temperature, sensor.humidity, sensor.distance, sensor.touch);
      }
      else
      {
        Serial.printf("ONLINE: Temperatura: %f Humidade: %f Distance: %f Touch: %f \n", sensor.temperature, sensor.humidity, sensor.distance, sensor.touch);
        mqttClient.publish("sensor/bruno_amado/temperature", String(sensor.temperature).c_str());
        mqttClient.publish("sensor/bruno_amado/humidity", String(sensor.humidity).c_str());
        mqttClient.publish("sensor/bruno_amado/distance", String(sensor.distance).c_str());
        mqttClient.publish("sensor/bruno_amado/touch", String(sensor.touch).c_str());
      }
    }

    // If your device state is changed by any other means (MQTT, physical button,...)
    // you can instruct the library to report the new state to Alexa on next request:

    //swith to value 0 to 254 from max
    int progressDistance = (int)round((254 * sensor.distance / (int)MAXDISTANCE));
    //swith to value 0 to 254 from max
    int progressTouch = (int)round((254 * (int)MAXSENCIBILITY) / sensor.touch);

    Serial.printf("progressDistance: %d  progressTouch: %d \n", (progressDistance > 254) ? 254 : progressDistance, (progressTouch > 254) ? 254 : progressTouch);

    fauxmo.setState(DEVICE_1, digitalRead(RELAY_PIN_1), (progressDistance > 254) ? 254 : progressDistance);
    fauxmo.setState(DEVICE_2, digitalRead(RELAY_PIN_2), (progressTouch > 254) ? 254 : progressTouch);

    vTaskDelay(pdMS_TO_TICKS(1)); //Prevent Starvation
  }
}
void vTaskSensor(void *pvParameters)
{ // Sensor DHT
  DHT dht(5, DHT11);
  dht.begin();
  //sonar sensor
  HCSR04.begin(TRIGGER_PIN, ECHO_PIN);

  MySensor sensor;
  while (1)
  {
    sensor.temperature = dht.readTemperature();
    sensor.humidity = dht.readHumidity();
    double *distances = HCSR04.measureDistanceCm(sensor.temperature);
    sensor.distance = distances[0];
    sensor.touch = touchRead(TOUCH_PIN);

    vTaskDelay(pdMS_TO_TICKS(2000));
    if (!isnan(sensor.temperature) && !isnan(sensor.humidity) && !isnan(sensor.distance) && !isnan(sensor.touch))
    {
      xQueueSend(xQueue, &sensor, portMAX_DELAY); //API REF: https://www.freertos.org/a00117.html
    }
  }
}

void setup()
{
  // Inicialização da Fila
  xQueue = xQueueCreate(5, sizeof(MySensor)); //API REF: https://www.freertos.org/a00116.html
  /**
   * ESP32 tem 2 Nucleos
   * 0 -> PRO_CPU_NUM
   * 1 -> APP_CPU_NUM
   **/

  xTaskCreatePinnedToCore(vTaskPublish, "TASK_PUBLISH", configMINIMAL_STACK_SIZE + 4028, NULL, 2, NULL, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(vTaskSensor, "TASK_SENSOR", configMINIMAL_STACK_SIZE + 1024, NULL, 3, NULL, APP_CPU_NUM);
  //xTaskCreatePinnedToCore(vTaskTimer, "TASK_SENSOR", configMINIMAL_STACK_SIZE + 1024, NULL, 3, NULL, APP_CPU_NUM);
}

void loop()
{
  //remover a task loop da aplicação para libertar tempo de CPU
  vTaskDelete(NULL);
}

#include <Arduino.h>
#include "WiFi.h"
#include <ETH.h>
#include "PubSubClient.h"
#include "ArduinoJson.h"
#include <ModbusRTU.h>

//#define RXD2 16
//#define TXD2 17
//#define DIR 13

#define THINGS_BOARD_SERVER "mqtt.viis.tech"
#define TOKEN "rCIAwdEKuoZzXrITQeue"

#define ETH_ADDR 1
#define ETH_POWER_PIN -1 // Do not use it, it can cause conflict during the software reset.
#define ETH_POWER_PIN_ALTERNATIVE 14
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN

#define TIME_CHECK_WIFI 2000
#define PINDD1 35 // 4 // chân 
#define PINDD2 32 // 2
#define PINDD3 33 // 15
#define PINMN1 5  // 5, 34 Chan cam biến mực nước 
#define PINMN2 34

int ValueDD1;
int ValueDD2;
int ValueDD3;
bool ValueMucNuoc1;
bool ValueMucNuoc2;

IPAddress local_ip(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2 = (uint32_t)0x00000000;
static bool eth_connected = false;
void WiFiEvent(WiFiEvent_t event);

// int16_t ValueClo[1]={true};
// int16_t ValuePH[1]={true};
// int16_t ValueTDS[1]={true};
// int16_t ValueDoDuc[1]={true};

// typedef enum key{
//  CLO,
//  PH,
//  DoDuc,
//  TDS,
//}KEY;
// KEY _key=CLO;

// ModbusRTU mb;

// bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
//  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());

// return true;

//}

WiFiClient espClient;
PubSubClient client(espClient);

const char *ssid = "MLTECH_SHOP";
const char *pass = "mltech@2019";
void wifiConnect();

void mqtt_sendTelemertry();
void mqttInit();
void on_message(const char *topic, byte *payload, unsigned int length);
void mqtt_loop();
void mqtt_reconnect();
void readCamBien();
int chuyenDoi(long i);
void setup()
{
  Serial.begin(9600);
  pinMode(ETH_POWER_PIN_ALTERNATIVE, OUTPUT);
	digitalWrite(ETH_POWER_PIN_ALTERNATIVE, HIGH);

	WiFi.onEvent(WiFiEvent);

	ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); // Enable ETH

	ETH.config(local_ip, gateway, subnet, dns1, dns2); // Static IP, leave without this line to get IP via DHCP
	

  //WiFi.begin(ssid, pass);
  mqttInit();
  pinMode(PINDD1, INPUT);
  pinMode(PINDD2, INPUT);
  pinMode(PINDD3, INPUT);
  pinMode(PINMN1, INPUT);
  pinMode(PINMN2, INPUT);
}
void loop()
{
 
  //wifiConnect();
 // WiFiEvent();
  mqtt_loop();
  //readCamBien();
}

void wifiConnect()
{
  static unsigned long preTime = millis();
  if ((millis() - preTime > TIME_CHECK_WIFI) && (WiFi.status() != WL_CONNECTED))
  {
    Serial.println("WiFi connecting...");
    preTime = millis();
  }
}
void mqttInit()
{
  client.setServer(THINGS_BOARD_SERVER, 1883);
  client.setCallback(on_message);
}
void on_message(const char *topic, byte *payload, unsigned int length)
{
  StaticJsonDocument<200> doc;
  Serial.println("On message");
  char json[length + 1];
  strncpy(json, (char *)payload, length);
  json[length] = '\0';
  Serial.println("TOPIC: " + (String)topic);
  Serial.println("Message: " + (String)json);
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    Serial.println("deserializeJson failed");
    Serial.println(error.f_str());
    return;
  }

  mqtt_sendTelemertry();
  String responseTopic = String(topic);
  responseTopic.replace("request", "response");
  Serial.println(responseTopic.c_str());
  // client.publish(responseTopic.c_str(),);
}
void mqtt_loop()
{
  if (!client.connected())
  {
    mqtt_reconnect();
  }
  static unsigned long timeUpdate;

  if (millis() - timeUpdate >= 2000)
  {
    readCamBien();
    mqtt_sendTelemertry();
    timeUpdate = millis();
  }

  client.loop();
}
void mqtt_reconnect()
{
  if ((!client.connected()))
  {
    Serial.println("Connecting to thingsboard...");
    if (client.connect("nguyen_id_d", TOKEN, NULL))
    {
      Serial.println("Connected");
      client.subscribe("v1/devices/me/rpc/request/+");
      client.subscribe("v1/devices/me/attributes");
      mqtt_sendTelemertry();
    }
    else
    {
      Serial.println("Connect fail");
      Serial.println(client.state());

      //  delay(2000);
    }
  }
}
void mqtt_sendTelemertry()
{
  DynamicJsonDocument data(1024);
  data["ph"] = random(0, 10);
  data["clo"] = random(10, 20);
  data["tds"] = random(20, 30);
  data["turbi_1"] = ValueDD1;
  data["turbi_2"] = ValueDD2;
  data["turbi_3"] = ValueDD3;
  data["low_level"] = ValueMucNuoc1;
  data["high_level"] = ValueMucNuoc2;

  data["set_turbi"] = 10;
  data["set_ph_high"] = 20;
  data["set_ph_low"] = 10;
  data["set_clo"] = 30;

  String objectString;
  serializeJson(data, objectString);
  //  Serial.println(objectString);
  client.publish("v1/devices/me/telemetry", objectString.c_str());
}

void readCamBien()
{

  ValueDD1 = analogRead(PINDD1);
  ValueDD1 = chuyenDoi(ValueDD1);
  Serial.print("DD1: ");
  Serial.println(ValueDD1);

  ValueDD2 = analogRead(PINDD2);
  ValueDD2 = chuyenDoi(ValueDD2);
  Serial.print("DD2: ");
  Serial.println(ValueDD2);

  ValueDD3 = analogRead(PINDD3);
  ValueDD3 = chuyenDoi(ValueDD3);
  Serial.print("DD3: ");
  Serial.println(ValueDD3);

  ValueMucNuoc1 = digitalRead(PINMN1);
  Serial.print("Muc Nuoc 1: ");
  Serial.println(ValueMucNuoc1);

  ValueMucNuoc2 = digitalRead(PINMN2);
  Serial.print("Muc Nuoc 2 ");
  Serial.println(ValueMucNuoc2);
}
int chuyenDoi(long i)
{
  i = map(i, 0, 922, 0, 1000);
  return i;
}

void WiFiEvent(WiFiEvent_t event)
{
	switch (event)
	{
	case SYSTEM_EVENT_ETH_START:
		Serial.println("ETH Started");
		// set eth hostname here
		ETH.setHostname("esp32-ethernet");
		break;
	case SYSTEM_EVENT_ETH_CONNECTED:
		Serial.println("ETH Connected");
		break;
	case SYSTEM_EVENT_ETH_GOT_IP:
		Serial.print("ETH MAC: ");
		Serial.print(ETH.macAddress());
		Serial.print(", IPv4: ");
		Serial.print(ETH.localIP());
		if (ETH.fullDuplex())
		{
			Serial.print(", FULL_DUPLEX");
		}
		Serial.print(", ");
		Serial.print(ETH.linkSpeed());
		Serial.println("Mbps");
		eth_connected = true;
		break;
	case SYSTEM_EVENT_ETH_DISCONNECTED:
		Serial.println("ETH Disconnected");
		eth_connected = false;
		break;
	case SYSTEM_EVENT_ETH_STOP:
		Serial.println("ETH Stopped");
		eth_connected = false;
		break;
	default:
		break;
	}
}
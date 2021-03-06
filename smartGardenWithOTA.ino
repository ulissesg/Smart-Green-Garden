/************************* Inclusão das Bibliotecas *********************************/
#include <stdlib.h>
#include "ESP8266WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h" 
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <NTPClient.h>

/************************* Conexão WiFi*********************************/

#define WIFI_SSID       "Genguini's house" // nome de sua rede wifi
#define WIFI_PASS       ""     // senha de sua rede wifi

/********************* Credenciais Adafruit io *************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "" // Seu usuario cadastrado na plataforma da Adafruit
#define AIO_KEY         ""       // Sua key da dashboard

/********************** Variaveis globais *******************************/

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "a.st1.ntp.br", -3 * 3600, 60000);

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

int statusSystem = 1;

int rele01 = 16; // pino do rele
int rele02 = 5;

int umidadeLiga;
int umidadeDesliga;

int horaOn = 6;
int horaOff = 17;

int modo;

int delayTimeMode = 1 * 60000;

int lastOnPump = NULL;

long previousMillis = 0;

/****************************** Declaração dos Feeds ***************************************/

/* feed responsavel por receber os dados da nossa dashboard */
Adafruit_MQTT_Subscribe _rele = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Pump", MQTT_QOS_1);

Adafruit_MQTT_Subscribe Mode = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/mode", MQTT_QOS_1);

Adafruit_MQTT_Publish ModePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/mode", MQTT_QOS_1);

Adafruit_MQTT_Publish _relePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pump", MQTT_QOS_1);
 
Adafruit_MQTT_Publish umidade_graph = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity_graph", MQTT_QOS_1);

Adafruit_MQTT_Publish IOSub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/IO", MQTT_QOS_1);

Adafruit_MQTT_Subscribe IO = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/IO", MQTT_QOS_1);

Adafruit_MQTT_Subscribe OnValue = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/OnValue", MQTT_QOS_1);

Adafruit_MQTT_Subscribe OffValue = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/OffValue", MQTT_QOS_1);

Adafruit_MQTT_Publish OnValueSub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/OnValue", MQTT_QOS_1);

Adafruit_MQTT_Publish OffValueSub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/OffValue", MQTT_QOS_1);

/* Observe em ambas declarações acima a composição do tópico mqtt
  --> AIO_USERNAME "/feeds/mcp9808"
  O mpc9808 será justamente o nome que foi dado la na nossa dashboard, portanto o mesmo nome atribuido la, terá de ser colocado aqui tambem
*/

/*************************** Declaração dos Prototypes ************************************/

void initSerial();
void initPins();
void initEEPROM();
void initWiFi();
void OTAInit();
void initMQTT();
void conectar_broker();
float LeituraUmidade();
void releControlHumidity(int UmidadePercentual);
void releControlTime();

/*************************** Sketch ************************************/

void setup() {
  initSerial();
  initEEPROM();
  initPins();
  initWiFi();
  initMQTT();
  timeClient.begin();
}

void loop() {
  

  ArduinoOTA.handle();
  conectar_broker();    
  mqtt.processPackets(5000);


  EEPROM.begin(4);
  modo = EEPROM.read(0);
  umidadeLiga = EEPROM.read(1);
  umidadeDesliga = EEPROM.read(2);
  EEPROM.end();//Fecha a EEPROM.

  timeClient.update();

  int umidade = LeituraUmidade();

  if (statusSystem == 1){
     if (modo == 0){
      releControlHumidity(umidade);
    }else if(modo == 1){
      releControlTime();
    }
  }

  publishStateSystem();
  OnValueSub.publish(umidadeLiga);
  OffValueSub.publish(umidadeDesliga);
  umidade_graph.publish(umidade);
  delay(10000);
}

/*************************** Implementação dos Prototypes ************************************/

/* Conexao Serial */
void initSerial() {
  Serial.begin(115200);
  delay(10);
  Serial.println("Booting");
}

/* inicializacao da EEPROM */

void initEEPROM(){
 EEPROM.begin(4);
// EEPROM.write(0,0);
 modo = EEPROM.read(0);// valor no endereço 0 novamente.
 umidadeLiga = EEPROM.read(1);
 umidadeDesliga = EEPROM.read(2);
 EEPROM.end();//Fecha a EEPROM.
}

/* Configuração dos pinos */
void initPins() {

  pinMode(rele01, OUTPUT);
  digitalWrite(rele01, HIGH);

  pinMode(rele02, OUTPUT);
  digitalWrite(rele02, HIGH);

}

/* Configuração da conexão WiFi */
void initWiFi() {
  int tentativas;
  WiFi.mode(WIFI_STA);
  Serial.print("Conectando-se na rede "); Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  if (WiFi.status() != WL_NO_SSID_AVAIL && tentativas < 50){
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      tentativas ++;
    }
    
    Serial.println();
    OTAInit();
    Serial.println("Conectado à rede com sucesso"); Serial.println("Endereço IP: "); Serial.println(WiFi.localIP());
  }else{
      Serial.println("Não foi possivel conectar ao roteador");
      ESP.restart();
  }

  
}

/*init OTA connection*/

void OTAInit(){
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
   ArduinoOTA.setHostname("ESP SMART GARDEN");

  // No authentication by default
   ArduinoOTA.setPassword("");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
//   ArduinoOTA.setPasswordHash("");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}

/* Configuração da conexão MQTT */
void initMQTT() {
  _rele.setCallback(rele_callback);
  mqtt.subscribe(&_rele);
  Mode.setCallback(mode_callback);
  mqtt.subscribe(&Mode);
  IO.setCallback(io_callback);
  mqtt.subscribe(&IO);
  OnValue.setCallback(OnValue_callback);
  mqtt.subscribe(&OnValue);
  OffValue.setCallback(OffValue_callback);
  mqtt.subscribe(&OffValue);
  
}

/*************************** Implementação dos Callbacks ************************************/

/* callback responsavel por tratar o feed do rele */
void rele_callback(char *data, uint16_t len) {
  String state = data;

  if (state == "ON") {
    digitalWrite(rele01, LOW);
    digitalWrite(rele02, LOW);
    
  } else if(state == "OFF") {
    digitalWrite(rele01, HIGH);
    digitalWrite(rele02, HIGH);
  }

}

void mode_callback(char *data, uint16_t len){
  String state = data;

  if (state == "Time"){
    EEPROM.begin(4);
    EEPROM.write(0, 1);
    EEPROM.commit();
    EEPROM.end();
    lastOnPump = NULL;

  }else if (state == "Humi"){
    EEPROM.begin(4);
    EEPROM.write(0, 0);
    EEPROM.commit();
    EEPROM.end();
  }
  
}

void io_callback(char *data, uint16_t len){
  String state = data;

  if (state == "ON") {
    statusSystem = 1;
    
  } else if(state == "OFF") {
    statusSystem = 0;
  }
}

void OnValue_callback(char *data, uint16_t len){
  String state = data;

    EEPROM.begin(4);
    EEPROM.write(1, atoi(state.c_str()));
    EEPROM.commit();
    EEPROM.end();  
}

void OffValue_callback(char *data, uint16_t len){
  String state = data;

    EEPROM.begin(4);
    EEPROM.write(2, atoi(state.c_str()));
    EEPROM.commit();
    EEPROM.end();  
}

/*************************** Demais implementações ************************************/

/* Conexão com o broker e também servirá para reestabelecer a conexão caso caia */
void conectar_broker() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.println("Conectando-se ao broker mqtt...");

  uint8_t num_tentativas = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Falha ao se conectar. Tentando se reconectar em 5 segundos.");
    mqtt.disconnect();
    delay(5000);
    num_tentativas--;
    if (num_tentativas == 0) {
      Serial.println("Seu ESP será resetado.");
      while (1);
    }
  }

  Serial.println("Conectado ao broker com sucesso.");
}

/* leitura do sensor de humidade */

float LeituraUmidade() {
  
  int pinoSensorUmidade = 0;

  float UmidadePercentual;
  int analogSoloSeco = 1024; 
  int analogSoloMolhado = 0; 
  int percSoloSeco = 0; 
  int percSoloMolhado = 100; 

  UmidadePercentual = map(analogRead(pinoSensorUmidade),analogSoloSeco,analogSoloMolhado,percSoloSeco,percSoloMolhado); //EXECUTA A FUNÇÃO "map" DE ACORDO COM OS PARÂMETROS PASSADOS

  return UmidadePercentual;
}

/* liga e desliga o rele de acordo com a umidade */

void releControlHumidity(int UmidadePercentual){
    if (UmidadePercentual <= umidadeLiga && timeClient.getHours() > horaOn && timeClient.getHours() < horaOff){
    pumpOn();
    
  }else if (UmidadePercentual >= umidadeDesliga){
    pumpOff();
  }
}

/* liga e desliga a bomba a cada 30 minutos */
void releControlTime(){
  if (timeClient.getHours() >= horaOn && timeClient.getHours() <= horaOff){
    if (lastOnPump == NULL){
      onOffPump();
    } else if (timeClient.getMinutes() == 0 || timeClient.getMinutes() == 30){
      onOffPump();
    }else if (lastOnPump == horaOff && timeClient.getHours() == horaOn){
      onOffPump();
    }
  }
  
}

void onOffPump(){
  pumpOn();

  delay(delayTimeMode);
  
  pumpOff();     
  
  lastOnPump = timeClient.getHours();
}

void pumpOn(){
  digitalWrite(rele01, LOW);
  digitalWrite(rele02, LOW);
  _relePub.publish("ON");
}

void pumpOff(){
  digitalWrite(rele01, HIGH);
  digitalWrite(rele02, HIGH);
  _relePub.publish("OFF");  
}

void publishStateSystem(){
  if (statusSystem == 1){
      IOSub.publish("ON");
  }
  if (statusSystem == 0){
      IOSub.publish("OFF");
  }
}

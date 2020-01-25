/************************* Inclusão das Bibliotecas *********************************/
#include "ESP8266WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <virtuabotixRTC.h>  
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

/************************* Conexão WiFi*********************************/

#define WIFI_SSID       "Genguini's house" // nome de sua rede wifi
#define WIFI_PASS       "01042017"     // senha de sua rede wifi

/********************* Credenciais Adafruit io *************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "ulissesg" // Seu usuario cadastrado na plataforma da Adafruit
#define AIO_KEY         "daf0fe66e7be4af19b34523241d1a66c"       // Sua key da dashboard

/********************** Variaveis globais *******************************/

virtuabotixRTC myRTC(4, 0, 2);

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

int rele01 = 16; // pino do rele
int rele02 = 5;

int umidadeLiga = 60;
int umidadeDesliga = 70;

int horaOn = 6;
int horaOff = 17;

int modo;

int delayTimeMode = 3 * 60000;

int lastOnPump = NULL;

long previousMillis = 0;

/****************************** Declaração dos Feeds ***************************************/

/* feed responsavel por receber os dados da nossa dashboard */
Adafruit_MQTT_Subscribe _rele = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Pump", MQTT_QOS_1);

Adafruit_MQTT_Subscribe Mode = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/mode", MQTT_QOS_1);

Adafruit_MQTT_Publish ModePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/mode", MQTT_QOS_1);

Adafruit_MQTT_Publish _relePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pump", MQTT_QOS_1);

Adafruit_MQTT_Publish Hora = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Hora", MQTT_QOS_1);
 
Adafruit_MQTT_Publish umidade_graph = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity_graph", MQTT_QOS_1);
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
//  myRTC.setDS1302Time(00, 48, 10, 6, 24, 01, 2020);
  initSerial();
  initEEPROM();
  initPins();
  initWiFi();
  if (WiFi.status() == WL_CONNECTED){
      initMQTT();
  }
}

void loop() {
  
  if (WiFi.status() == WL_CONNECTED){
    ArduinoOTA.handle();
    conectar_broker();    
    mqtt.processPackets(5000);
  }

  EEPROM.begin(4);
  modo = EEPROM.read(0);// valor no endereço 0 novamente.
  EEPROM.end();//Fecha a EEPROM.

  int umidade = LeituraUmidade();

  if (modo == 0){
    releControlHumidity(umidade);
  }else if(modo == 1){
    releControlTime();
  }

  if (WiFi.status() == WL_CONNECTED){
    myRTC.updateTime(); 
    Hora.publish(myRTC.hours);
    umidade_graph.publish(umidade);
    
//    if (modo == 0){
//      ModePub.publish("Humi");
//    }else if (modo == 1){
//      ModePub.publish("Time");
//    }
  }
  
  delay(5000);
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
 EEPROM.end();//Fecha a EEPROM.
}

/* Configuração dos pinos */
void initPins() {

  pinMode(rele01, OUTPUT);
  digitalWrite(rele01, HIGH);

  pinMode(rele02, OUTPUT);
  digitalWrite(rele02, HIGH);

  pinMode(4, INPUT);
  pinMode(0, INPUT);
  pinMode(2, INPUT);
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
//   ArduinoOTA.setPassword("01042017");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
//   ArduinoOTA.setPasswordHash("01042017");

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
}

/*************************** Implementação dos Callbacks ************************************/

/* callback responsavel por tratar o feed do rele */
void rele_callback(char *data, uint16_t len) {
  String state = data;

  if (state == "ON") {
    digitalWrite(rele01, LOW);
    digitalWrite(rele02, LOW);
    Serial.print("Bomba: "); Serial.println(state);
    
  } else if(state == "OFF") {
    digitalWrite(rele01, HIGH);
    digitalWrite(rele02, HIGH);
    Serial.print("Bomba: "); Serial.println(state);
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
  
  int pinoSensorUmidade1 = 0;
  int pinoSensorUmidade2 = 1;

  float UmidadePercentual;
  int analogSoloSeco = 1024; 
  int analogSoloMolhado = 0; 
  int percSoloSeco = 0; 
  int percSoloMolhado = 100; 

  UmidadePercentual = constrain(analogRead(pinoSensorUmidade1),analogSoloMolhado,analogSoloSeco); //MANTÉM valorLido DENTRO DO INTERVALO (ENTRE analogSoloMolhado E analogSoloSeco)
  UmidadePercentual = map(UmidadePercentual,analogSoloMolhado,analogSoloSeco,percSoloMolhado,percSoloSeco); //EXECUTA A FUNÇÃO "map" DE ACORDO COM OS PARÂMETROS PASSADOS
  Serial.print("Umidade do solo: "); //IMPRIME O TEXTO NO MONITOR SERIAL
  Serial.print(UmidadePercentual); //IMPRIME NO MONITOR SERIAL O PERCENTUAL DE UMIDADE DO SOLO
  Serial.println("%"); //IMPRIME O CARACTERE NO MONITOR SERIAL
  Serial.print("Leitura Sensor:");
  Serial.println(analogRead(pinoSensorUmidade1));

  return UmidadePercentual;
}

/* liga e desliga o rele de acordo com a umidade */

void releControlHumidity(int UmidadePercentual){
    if (UmidadePercentual <= umidadeLiga && myRTC.hours > horaOn && myRTC.hours < horaOff){
    digitalWrite(rele01, LOW);
    digitalWrite(rele02, LOW);
    if (WiFi.status() == WL_CONNECTED){
      _relePub.publish("ON");
    }
  }else if (UmidadePercentual >= umidadeDesliga){
    digitalWrite(rele01, HIGH);
    digitalWrite(rele02, HIGH);
    if (WiFi.status() == WL_CONNECTED){
      _relePub.publish("OFF");      
    }
  }
}

/* liga e desliga a bomba a cada 1 hora */
void releControlTime(){
  if (myRTC.hours > horaOn && myRTC.hours <= horaOff){
    if (lastOnPump == NULL){
      onOffPump();
    } else if (lastOnPump < myRTC.hours){
      onOffPump();
    }else if (lastOnPump == horaOff && myRTC.hours == horaOn){
      onOffPump();
    }
  }
  
}

void onOffPump(){
  digitalWrite(rele01, LOW);
  digitalWrite(rele02, LOW);
  if (WiFi.status() == WL_CONNECTED){
    _relePub.publish("ON");
  }

  delay(delayTimeMode);
  
  digitalWrite(rele01, HIGH);
  digitalWrite(rele02, HIGH);
  if (WiFi.status() == WL_CONNECTED){
    _relePub.publish("OFF");      
  }
  
  lastOnPump = myRTC.hours;
}

/************************* Inclusão das Bibliotecas *********************************/
#include "ESP8266WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <virtuabotixRTC.h>  
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

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

long previousMillis = 0;

/****************************** Declaração dos Feeds ***************************************/

/* feed responsavel por receber os dados da nossa dashboard */
Adafruit_MQTT_Subscribe _rele = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Pump", MQTT_QOS_1);

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
void initWiFi();
void OTAInit();
void initMQTT();
void conectar_broker();
float LeituraUmidade();
void imprime_dia_da_semana(int dia);
void imprimeDataHora ();
String hora ();

/*************************** Sketch ************************************/

void setup() {
//  myRTC.setDS1302Time(00, 32, 16, 2, 20, 01, 2020);
  initSerial();
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

  int umidade = LeituraUmidade();
  imprimeDataHora ();

  if (WiFi.status() == WL_CONNECTED){
    Hora.publish(myRTC.hours);
    umidade_graph.publish(umidade);
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
      Serial.println("Não foi possivel conectar a internet");
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
   ArduinoOTA.setPassword("01042017");

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
}

/*************************** Implementação dos Callbacks ************************************/

/* callback responsavel por tratar o feed do rele */
void rele_callback(char *data, uint16_t len) {
  String state = data;

  if (state == "ON") {
    digitalWrite(rele01, LOW);
    digitalWrite(rele02, LOW);
    Serial.print("Bomba: "); Serial.println(state);
    delay(15000);
    
  } else if(state == "OFF") {
    digitalWrite(rele01, HIGH);
    digitalWrite(rele02, HIGH);
    Serial.print("Bomba: "); Serial.println(state);
//    delay(30000);
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

  int umidadeLiga = 55;
  int umidadeDesliga = 60;
  
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
  
  if (UmidadePercentual <= umidadeLiga && myRTC.hours > 6 && myRTC.hours < 17){
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

  return UmidadePercentual;
}

/* imprime hora e data */

void imprimeDataHora (){
  myRTC.updateTime(); 
  //Imprime as informacoes no serial monitor
  Serial.print("\nData : ");
  //Chama a rotina que imprime o dia da semana
  imprime_dia_da_semana(myRTC.dayofweek);
  Serial.print(", ");
  Serial.print(myRTC.dayofmonth);
  Serial.print("/");
  Serial.print(myRTC.month);
  Serial.print("/");
  Serial.print(myRTC.year);
  Serial.print("  ");
  Serial.print("Hora : ");
  Serial.print(hora());
}

String hora (){
  String hora;
  myRTC.updateTime(); 
  if (myRTC.hours < 10){
    hora = "0";
  }
  hora = hora + myRTC.hours + " : ";
  //Adiciona um 0 caso o valor dos minutos seja <10
  if (myRTC.minutes < 10){
    hora = hora  + "0"; 
  }
  hora = hora + myRTC.minutes + " : ";
  //Adiciona um 0 caso o valor dos segundos seja <10
  if (myRTC.seconds < 10){
    hora  = hora + "0";
  }
  hora =  hora + myRTC.seconds;
  return hora;
}


/* imprime dia da semana */

void imprime_dia_da_semana(int dia){
  switch (dia){
    case 1:
      Serial.print("Domingo");
      break;
    case 2:
      Serial.print("Segunda");
      break;
    case 3:
      Serial.print("Terca");
      break;
    case 4:
      Serial.print("Quarta");
      break;
    case 5:
      Serial.print("Quinta");
      break;
    case 6:
      Serial.print("Sexta");
      break;
    case 7:
      Serial.print("Sabado");
      break;
  }
}

/************************* Inclusão das Bibliotecas *********************************/
#include "ESP8266WiFi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <virtuabotixRTC.h>  

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

//Adafruit_MQTT_Publish sensorHumidade = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity", MQTT_QOS_1);
 
Adafruit_MQTT_Publish umidade_graph = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity_graph", MQTT_QOS_1);
/* Observe em ambas declarações acima a composição do tópico mqtt
  --> AIO_USERNAME "/feeds/mcp9808"
  O mpc9808 será justamente o nome que foi dado la na nossa dashboard, portanto o mesmo nome atribuido la, terá de ser colocado aqui tambem
*/

/*************************** Declaração dos Prototypes ************************************/

void initSerial();
void initPins();
void initWiFi();
void initMQTT();
void conectar_broker();
float LeituraUmidade();
void imprime_dia_da_semana(int dia);
void imprimeDataHora ();

/*************************** Sketch ************************************/

void setup() {
//  myRTC.setDS1302Time(00, 37, 16, 2, 30, 12, 2019);
  initSerial();
  initPins();
  initWiFi();
  if (WiFi.status() == WL_CONNECTED){
      initMQTT();
  }
}

void loop() {

  if (WiFi.status() == WL_CONNECTED){
    conectar_broker();
    mqtt.processPackets(5000);
  }

  int umidade = LeituraUmidade();

  if (WiFi.status() == WL_CONNECTED){
//    sensorHumidade.publish(umidade);
    umidade_graph.publish(umidade);
  }

  imprimeDataHora ();
  
  delay(5000);
}

/*************************** Implementação dos Prototypes ************************************/

/* Conexao Serial */
void initSerial() {
  Serial.begin(115200);
  delay(10);
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
  Serial.print("Conectando-se na rede "); Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  if (WiFi.status() != WL_NO_SSID_AVAIL && tentativas < 50){
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      tentativas ++;
    }
    
    Serial.println();
    Serial.println("Conectado à rede com sucesso"); Serial.println("Endereço IP: "); Serial.println(WiFi.localIP());
  }else{
      Serial.println("Não foi possivel conectar a internet");
      ESP.reset();
  }

  
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
//    delay(10 * 60000);
    
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

  int umidadeLiga = 60;
  int umidadeDesliga = 75;
  
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
  //Adiciona um 0 caso o valor da hora seja <10
  if (myRTC.hours < 10){
    Serial.print("0");
  }
  Serial.print(myRTC.hours);
  Serial.print(":");
  //Adiciona um 0 caso o valor dos minutos seja <10
  if (myRTC.minutes < 10){
    Serial.print("0");
  }
  Serial.print(myRTC.minutes);
  Serial.print(":");
  //Adiciona um 0 caso o valor dos segundos seja <10
  if (myRTC.seconds < 10){
    Serial.print("0");
  }
  Serial.println(myRTC.seconds);
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

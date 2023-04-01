#include <Arduino.h>
#include <SPIFFS.h>             //Manejador de archivos
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>


// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
//#define BOARD_ID 1

// Digital pin connected to the DHT sensor
#define DHTPIN 2  
#define SENDOK 12                                      //GPIO12 envio ok
#define SENDFAIL 15                                     //GPIO15 fallo el envio
#define Interrupcion 35 
#define tiempoDeRebote 250      
// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);
float min2 = 100;  //temperatura min si no se igual a 100 se va a 0
float max2;  //temperatura maxima
//MAC Address of the receiver osea de este dispositivo aun hay que compara para ver si se esta utilizando *******************************************
uint8_t broadcastAddress[] = {0x70, 0xB8, 0xF6, 0x5B, 0x93, 0x2C};
//*************************************************************************
void IRAM_ATTR funcionDeInterrupcion(); //IRAM_ATTR hace que la funcion se guarde en la memoria RAM y no en la flash para que sea mas rapido
volatile unsigned long tiempoDeInterrupcionAnterior = 0;  //sirve para omitir rebotes
//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int id;
    float temp;
    float hum;
    float readingId;
    float min;
    float max;
} struct_message;

//Create a struct_message called myData
struct_message myData;
int board=1;
int numboards=3; //maximo numero de esp32 comunicandose al espwebserver
unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 2000;        // Interval at which to publish sensor readings
float readingId = 0;
//unsigned int readingId = 0; //cuando era solo entero

// Insert your SSID
constexpr char WIFI_SSID[] = "ESPWROOM325BF6B8702C93"; // "INFINITUM37032"//INFINITUM59W1_2.4//INFINITUMD378
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

float readDHTTemperature() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  else {
    Serial.println(t);
    return t;
  }
}
float readDHTTemperature2() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t2 = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t2)) {    
    Serial.println("Failed to read from DHT sensor!");
    return 500;
  }
  else {
    Serial.println(t2);
    return t2;
  }
}

float readDHTHumidity() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  else {
    Serial.println(h);
    return h;
  }
}

float tempMin(){
  
  float min = readDHTTemperature2();
  if(min < min2){
    min2 = min;
  }else if(min == 0){
    min2 = readDHTTemperature2();
  }
  Serial.println(min2);
  return min2;
}
  
float tempMax(){
  
  float max = dht.readTemperature();
  if(max > max2){
    max2 = max;
  }
  Serial.println(max2);
  return max2;
}
float getWiFiRsi(const char *ssid2) {
if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i=0; i<n; i++) {
        if (!strcmp(ssid2, WiFi.SSID(i).c_str())) {
          Serial.println(WiFi.RSSI(i));
          return WiFi.RSSI(i);
        }
    }
}
return 0;
}
//interrupcion
void IRAM_ATTR funcionDeInterrupcion(){ //esta funcion de interrupcion no se manda a llamar se ejecuta sola cuando sucede la interrupcion
    if(millis() - tiempoDeInterrupcionAnterior > tiempoDeRebote){
    Serial.println("Interrupción "+board);
    Serial.flush();
    if(board > numboards){
      board=1;
    }else{
      board++;
    }
  }
}
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  delay(1000);
  //Init Serial Monitor
  Serial.begin(115200);
  pinMode(SENDOK, OUTPUT);
  pinMode(SENDFAIL, OUTPUT);
  digitalWrite(SENDOK, LOW);
  digitalWrite(SENDFAIL, LOW);
  dht.begin();
  //boton de interrupcion
  pinMode(Interrupcion, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Interrupcion), funcionDeInterrupcion, RISING); 
  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  //Init ESP-NOW
  if (!esp_now_init()) {
    //si el resultado es un 0 la conexion ESP-NOW esta ok
    Serial.println("[INFO: espnow.hpp ] Conexion ESP-NOW inicializada.");
  }else{
    Serial.println("[INFO: espnow.hpp ] Conexion ESP-NOW no inicializada.");
    ESP.restart();
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  //Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); //si no quitar y encontrar la version 1.0.6
  memcpy(peerInfo.peer_addr, broadcastAddress, sizeof(peerInfo.peer_addr));
  peerInfo.encrypt = false;
  
  //Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    //Set values to send
    myData.id = board;//BOARD_ID;
    myData.temp = readDHTTemperature();
    myData.hum = readDHTHumidity();
    myData.readingId = getWiFiRsi(WIFI_SSID); //readingId++; //-------------------------------------------------------------
    myData.min = tempMin();
    myData.max = tempMax();
    //Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      digitalWrite(SENDOK, HIGH);
      
    }
    else {
      Serial.println("Error sending the data");
      digitalWrite(SENDOK, LOW);
      
    }
  }
}
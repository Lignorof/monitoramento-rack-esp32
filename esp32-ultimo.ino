#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <DHT.h>
#include <MQUnifiedsensor.h>

// Definições de pinos
#define FAN_LIGADO_PIN 2  // Liga manualmente fan.

#define G_TEMP_PIN 18     // Temp normal
#define R_TEMP_PIN 19     // Temp alta
#define G_UMID_PIN 21     // Umidade normal
#define R_UMID_PIN 3      // Umidade alta
#define FUMACA 5          // Presença de fumaça
#define SEN_AGUA 22       // Presença de água
#define FAN1_EN_PIN 32   
#define FAN1_IN1_PIN 25   
#define FAN1_IN2_PIN 26   
#define FAN2_EN_PIN 33   
#define FAN2_IN3_PIN 27   
#define FAN2_IN4_PIN 14   

/*
#define FAN_LIGADO_PIN 2
#define G_TEMP_PIN 32   // antigo 18
#define R_TEMP_PIN 33   // antigo 19
#define G_UMID_PIN 25   // antigo 21
#define R_UMID_PIN 26   // antigo 3
#define FUMACA 18       // antigo 5
#define SEN_AGUA 22     // algum dia
#define FAN1_EN_PIN 21  // antigo 32 
#define FAN2_EN_PIN 3  // antigo 33
*/
// Definições do sensor DHT22
#define DHTPIN 17
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Definições do sensor de fumaça MQ-2
#define Board ("ESP32")
#define Pin (36) // ADC2 pino 12 ADC1 pino 34
#define Type ("MQ-2") //MQ2
#define Voltage_Resolution (3.3) // entrada 3.3V
#define ADC_Bit_Resolution (12) // 12-bit de resolução
#define RatioMQ2CleanAir (9.83) //RS / R0 = 9.83 ppm
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

// Variáveis globais para temperatura e umidade
float tempc = 0.0;
float humid = 0.0;

// Configuração de controle PWM para ventoinhas
const int freq = 5000;
const int resolution = 8;
const int FAN1_PWM_CHANNEL = 0;
const int FAN2_PWM_CHANNEL = 1;

// Conexão WiFi e Firebase
#define WIFI_SSID "Matheus p8"    // Matheus p8
#define WIFI_PASSWORD "12345678"  // 12345678
#define API_KEY "AIzaSyC4lVfdKsLQJnEPEx6P_OZyQ7MBZRo79Rc"
#define DATABASE_URL "https://monitoramento-rack-datacenter-default-rtdb.firebaseio.com/"

// Configurações do Firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

// Variáveis de controle
bool manualOverride = false;
unsigned long manualOverrideStartTime = 0;
const unsigned long manualOverrideDuration = 30 * 1000; // 30 segundos

// Funções do controle do ar condicionado (substitua pela biblioteca específica)
void turnOnAC() {
  // Implementar função para ligar o ar condicionado
  Serial.println("Ar condicionado ligado.");
}

void turnOffAC() {
  // Implementar função para desligar o ar condicionado
  Serial.println("Ar condicionado desligado.");
}

// Função de setup
void setup() {
  Serial.begin(115200);

  // Configuração dos pinos
  pinMode(G_TEMP_PIN, OUTPUT);
  pinMode(R_TEMP_PIN, OUTPUT);
  pinMode(G_UMID_PIN, OUTPUT);
  pinMode(R_UMID_PIN, OUTPUT);
  pinMode(FUMACA, OUTPUT);
  pinMode(SEN_AGUA, OUTPUT);
  pinMode(FAN_LIGADO_PIN, OUTPUT);

  // Configuração dos canais PWM para ventoinhas
  ledcSetup(FAN1_PWM_CHANNEL, freq, resolution);
  ledcAttachPin(FAN1_EN_PIN, FAN1_PWM_CHANNEL);
  ledcSetup(FAN2_PWM_CHANNEL, freq, resolution);
  ledcAttachPin(FAN2_EN_PIN, FAN2_PWM_CHANNEL);

  // Inicialização do DHT22
  dht.begin();

  // Inicialização do sensor de fumaça MQ-2
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(36974); MQ2.setB(-3.109); // Valores dados para o gas CO
  MQ2.init();

  // Conexão WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Conectado com IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Configuração do Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Conexão com Firebase OK");
    signupOK = true;
  } else {
    Serial.printf("Erro de conexão com Firebase: %s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// Função para ler dados do DHT22
void readDHT() {
  tempc = dht.readTemperature();
  humid = dht.readHumidity();

  if (isnan(tempc) || isnan(humid)) {
    Serial.println("Falha na leitura do sensor DHT22!");
  } else {
    Serial.print("Temperatura: ");
    Serial.print(tempc);
    Serial.print(" °C, Umidade: ");
    Serial.print(humid);
    Serial.println(" %");
  }
}

// Função para calcular o ponto de orvalho
float calculateDewPoint(float temp, float humidity) {
  const float a = 17.27;
  const float b = 237.7;
  float alpha = ((a * temp) / (b + temp)) + log(humidity / 100.0);
  float dewPoint = (b * alpha) / (a - alpha);
  return dewPoint;
}

// Função para verificar presença de orvalho
bool checkDewPresence(float temp, float humidity) {
  float dewPoint = calculateDewPoint(temp, humidity);
  Serial.print("Ponto de Orvalho: ");
  Serial.print(dewPoint);
  Serial.println(" °C");
  return temp <= dewPoint;
}

// Função para ler dados do sensor de fumaça MQ-2
bool readMQ2() {
  MQ2.update(); // Atualizar leituras do sensor
  float smoke = MQ2.readSensor();
  Serial.print("MQ-2 leitura de fumaça: ");
  Serial.println(smoke);

  // Defina um limiar para detectar fumaça
  return smoke > 10.0; // Ajuste conforme necessário
}

// Função para atualizar o Firebase
void updateFirebase() {
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();
    
    // Enviar temperatura
    if (Firebase.RTDB.setFloat(&fbdo, "Sensor/TempC", tempc)) {
      Serial.print("Temperatura enviada: ");
      Serial.println(tempc);
    } else {
      Serial.print("Erro ao enviar temperatura: ");
      Serial.println(fbdo.errorReason());
    }

    // Enviar umidade
    if (Firebase.RTDB.setFloat(&fbdo, "Sensor/Umid", humid)) {
      Serial.print("Umidade enviada: ");
      Serial.println(humid);
    } else {
      Serial.print("Erro ao enviar umidade: ");
      Serial.println(fbdo.errorReason());
    }

    // Enviar presença de fumaça
    bool smokeDetected = readMQ2();
    if (Firebase.RTDB.setBool(&fbdo, "Sensor/fumaca", smokeDetected)) {
      Serial.print("Presença de fumaça enviada: ");
      Serial.println(smokeDetected);
    } else {
      Serial.print("Erro ao enviar presença de fumaça: ");
      Serial.println(fbdo.errorReason());
    }

    // Receber comando de override manual
    if (Firebase.RTDB.getBool(&fbdo, "/Control/ManualOverride")) {
      bool manualOverrideCommand = fbdo.boolData();
      if (manualOverrideCommand) {
        manualOverride = true;
        manualOverrideStartTime = millis();
        // Ligar ventoinhas a 100%
        //digitalWrite(FAN1_IN1_PIN, HIGH);
        //digitalWrite(FAN1_IN2_PIN, LOW);
        ledcWrite(FAN1_PWM_CHANNEL, 255); // 100% de duty cycle
        //digitalWrite(FAN2_IN3_PIN, HIGH);
        //digitalWrite(FAN2_IN4_PIN, LOW);
        ledcWrite(FAN2_PWM_CHANNEL, 255); // 100% de duty cycle
        Serial.println();
        Serial.println("Ventoinhas em 100% até segunda ordem");
        Serial.println();
        // Ligar ar condicionado
        turnOnAC();
        Serial.println("Override manual ativado: ventoinhas e ar condicionado ligados.");
      }
    } else {
      Serial.print("Erro ao receber comando de override manual: ");
      Serial.println(fbdo.errorReason());
    }
  }
}

// Função para controlar ventoinhas automaticamente
void controlFans() {
  if (manualOverride) {
    // Verificar se o tempo de override manual expirou
    if (millis() - manualOverrideStartTime >= manualOverrideDuration) {
      manualOverride = false;
      // Desativar override manual: retomar controle automático
      digitalWrite(FAN_LIGADO_PIN, LOW);
      Serial.println("Override manual desativado: controle automático retomado.");
    }
  } else {
    // Controle automático baseado na temperatura e umidade
    if (tempc > 30.0) {
      // Temperatura crítica: ligar ventoinhas a 100%
      //digitalWrite(FAN1_IN1_PIN, HIGH);
      //digitalWrite(FAN1_IN2_PIN, LOW);
      ledcWrite(FAN1_PWM_CHANNEL, 255); // 100% de duty cycle
      //digitalWrite(FAN2_IN3_PIN, HIGH);
      //digitalWrite(FAN2_IN4_PIN, LOW);
      ledcWrite(FAN2_PWM_CHANNEL, 255); // 100% de duty cycle
      digitalWrite(R_TEMP_PIN, HIGH);
      digitalWrite(G_TEMP_PIN, LOW);
      Serial.println();
      Serial.println("Led vermelho ligado!");
              Serial.println();
        Serial.println("Ventoinhas ligadas em 100%");
        Serial.println();
    } else if (tempc > 25.0) {
      // Temperatura alta: ligar ventoinhas a 50%
      //digitalWrite(FAN1_IN1_PIN, HIGH);
      //digitalWrite(FAN1_IN2_PIN, LOW);
      ledcWrite(FAN1_PWM_CHANNEL, 128); // 50% de duty cycle
      //digitalWrite(FAN2_IN3_PIN, HIGH);
      //digitalWrite(FAN2_IN4_PIN, LOW);
      ledcWrite(FAN2_PWM_CHANNEL, 128); // 50% de duty cycle
      digitalWrite(R_TEMP_PIN, LOW);
      digitalWrite(G_TEMP_PIN, HIGH);
        Serial.println();
        Serial.println("Ventoinhas ligadas em 50%");
        Serial.println("Led verde ligado!");
        Serial.println();
    } else {
      // Temperatura normal: desligar ventoinhas
      ledcWrite(FAN1_PWM_CHANNEL, 0); // 0% de duty cycle
      ledcWrite(FAN2_PWM_CHANNEL, 0); // 0% de duty cycle
      digitalWrite(R_TEMP_PIN, LOW);
      digitalWrite(G_TEMP_PIN, HIGH);
              Serial.println();
        Serial.println("Ventoinhas DESLIGADAS!");
        Serial.println();
    }

    // Controle de umidade
    if (humid > 70.0) {
      digitalWrite(R_UMID_PIN, HIGH);
      digitalWrite(G_UMID_PIN, LOW);
    } else {
      digitalWrite(R_UMID_PIN, LOW);
      digitalWrite(G_UMID_PIN, HIGH);
    }

    // Verificação de fumaça
    if (readMQ2()) {
      digitalWrite(FUMACA, HIGH);
    } else {
      digitalWrite(FUMACA, LOW);
    }
  }
}


// Loop principal
void loop() {
  readDHT();
  updateFirebase();
  controlFans();
  delay(2000);
}

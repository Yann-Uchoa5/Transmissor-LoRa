#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LoRa.h>
#include <HX711.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define BAND 433E6

Adafruit_BME280 bme; // Objeto para o sensor BME280 (temperatura, umidade, pressão)
HX711 balanca; // Objeto para o sensor de peso HX711

struct DadosSensor {
  float temperatura;
  float umidade;
  float pressao;
  float peso;
};

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial com taxa de 9600 bps
  while (!Serial); // Aguarda até que a porta serial esteja pronta

  unsigned status;
  status = bme.begin(0x76); // Inicializa o sensor BME280 com endereço I2C 0x76

  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  Serial.println("-- Default Test --");

  if (!LoRa.begin(BAND)) { // Inicializa a comunicação LoRa na banda especificada
    Serial.println("LoRa initialization failed.");
    while (1);
  }

  LoRa.setTxPower(20); // Define a potência de transmissão em dBm
  LoRa.setSpreadingFactor(7); // Define o fator de espalhamento
  LoRa.setSignalBandwidth(125E3); // Define a largura de banda

  balanca.begin(3, 4); /* Inicializa o objeto HX711 com os pinos SCK=3 e DT=4 */
  balanca.set_scale(); /* Configura a escala da balança */
  balanca.tare(); /* Configura a tara da balança */

  long zero_factor = balanca.read_average(); /* Realiza a leitura para ajuste da tara */
}

void loop() {
  balanca.set_scale(); /* Ajusta a escala da balança com o fator de calibração */
  float peso = balanca.get_units(); /* Obtém o peso em unidades da balança */
  if (peso < 0) {
    peso = 0.00; /* Se o peso for negativo, define como zero */
  }

  DadosSensor data;
  data.temperatura = bme.readTemperature(); // Lê a temperatura do sensor BME280
  data.umidade = bme.readHumidity(); // Lê a umidade do sensor BME280
  data.pressao = bme.readPressure(); // Lê a pressão atmosférica do sensor BME280
  data.peso = peso;

  // Envia os dados para o receptor usando LoRa
  sendToReceiver(data);

  delay(2000); // Aguarda 2 segundos antes de repetir o processo
}

void sendToReceiver(DadosSensor data) {
  // Inicializa a transmissão LoRa
  LoRa.beginPacket();
  // Envia os dados como bytes para o receptor
  LoRa.write((uint8_t*)&data, sizeof(DadosSensor));
  // Finaliza a transmissão LoRa
  LoRa.endPacket();
}

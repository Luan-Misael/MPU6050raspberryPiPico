#include "mpu6050.h"

/* O que é: Variável para armazenar o último momento em que uma leitura foi feita (em milissegundos).
   Para que serve: Controla o intervalo de tempo entre leituras do sensor para evitar sobrecarga do loop.
   Caso de uso: Garante que o MPU6050 seja lido a cada 50ms, permitindo estabilidade no controle do submarino. */
unsigned long previousMillis = 0;

/* O que é: Constante que define o intervalo entre leituras do sensor (em milissegundos).
   Para que serve: Estabelece a frequência de atualização dos dados (20 Hz com 50ms).
   Caso de uso: Usado para atualizar os dados de aceleração e giroscópio no controle de yaw ou inclinação do submarino. */
const unsigned long interval = 50; // Intervalo entre leituras (ms)

/* O que é: Função de inicialização do Arduino.
   Para que serve: Configura a comunicação serial e inicializa o MPU6050, incluindo calibração.
   Caso de uso: Executada uma vez ao ligar o Raspberry Pi Pico para preparar o sensor para leituras no submarino. */
void setup() {
  Serial.begin(57600); // Inicia comunicação serial a 57600 baud
  while (!Serial) delay(10); // Aguarda conexão serial (necessário para USB no Pico)

  // Inicializa o MPU6050
  if (!mpu_begin()) {
    Serial.println("Erro: Falha ao inicializar MPU6050!");
    while (1); // Para o programa se o sensor não for detectado
  }

  Serial.println("Calibrando, mantenha o sensor parado!");
  delay(1000);
  mpu_calibrate(3000); // Calibra com 3000 leituras
  Serial.println("===== Calibrado! =====\n");
}

/* O que é: Função principal que executa repetidamente.
   Para que serve: Lê e processa dados do MPU6050 em intervalos regulares, exibindo aceleração e giroscópio.
   Caso de uso: Atualiza os dados do sensor a cada 50ms para controlar a orientação ou profundidade do submarino. */
void loop() {
  unsigned long currentMillis = millis();

  // Executa a leitura a cada 'interval' milissegundos
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Serial.println("Executando loop...");
    mpu_loop(); // Processa dados do sensor

    // Exibe leituras processadas
    Serial.print("AccX (m/s²): "); Serial.println(getAccX(), 3);
    Serial.print("AccY (m/s²): "); Serial.println(getAccY(), 3);
    Serial.print("AccZ (m/s²): "); Serial.println(getAccZ(), 3);
    Serial.print("GyroX (rad/s): "); Serial.println(getGyroX(), 3);
    Serial.print("GyroY (rad/s): "); Serial.println(getGyroY(), 3);
    Serial.print("GyroZ (rad/s): "); Serial.println(getGyroZ(), 3);
    Serial.print("Norma (m/s²): "); Serial.println(getNorm(), 2);
    // Serial.println("Fim de loop");
    Serial.println();
  }
}
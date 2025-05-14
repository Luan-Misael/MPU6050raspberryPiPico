#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <math.h>

/* O que é: Constante que define o endereço I2C do MPU6050.
   Para que serve: Identifica o sensor MPU6050 no barramento I2C para comunicação.
   Caso de uso: Usado para iniciar a comunicação com o MPU6050 conectado ao Raspberry Pi Pico. */
const int MPU = 0x68;

/* O que é: Constante para conversão de unidades de g para m/s².
   Para que serve: Converte leituras do acelerômetro de g para m/s² (1g = 9.80665 m/s²).
   Caso de uso: Transforma os dados brutos do MPU6050 em m/s² para cálculos de inclinação no submarino. */
const float G_TO_MS2 = 9.80665f; // Conversão de g para m/s²

/* O que é: Constante para conversão de °/s para rad/s.
   Para que serve: Converte velocidades angulares do giroscópio de °/s para rad/s (π/180).
   Caso de uso: Ajusta os dados do giroscópio para rad/s, usado no controle de yaw do submarino. */
const float DEG_PER_SEC_TO_RAD_PER_SEC = 0.0174532925f; // π / 180

/* O que é: Variáveis globais para armazenar a aceleração nos eixos X, Y, Z.
   Para que serve: Guardam os valores processados do acelerômetro em m/s² após calibração e filtros.
   Caso de uso: Usadas para detectar inclinação do submarino, como ajustar profundidade com base em AccX. */
float AccX, AccY, AccZ; // Aceleração (m/s²)

/* O que é: Variáveis globais para armazenar a velocidade angular nos eixos X, Y, Z.
   Para que serve: Guardam os valores processados do giroscópio em rad/s após calibração e filtros.
   Caso de uso: Usadas para controlar a rotação (yaw) do submarino com base em GyroZ. */
float GyroX, GyroY, GyroZ; // Velocidade angular (rad/s)

/* O que é: Variáveis globais para armazenar os offsets de calibração do acelerômetro.
   Para que serve: Compensam erros sistemáticos nas leituras do acelerômetro (em g).
   Caso de uso: Aplicadas para zerar AccX/Y e ajustar AccZ (~1g) quando o sensor está parado. */
float accOffsetX, accOffsetY, accOffsetZ; // Offsets de calibração do acelerômetro (g)

/* O que é: Variáveis globais para armazenar os offsets de calibração do giroscópio.
   Para que serve: Compensam deriva nas leituras do giroscópio (em °/s).
   Caso de uso: Aplicadas para zerar GyroX/Y/Z quando o sensor está parado, garantindo precisão no controle de yaw. */
float gyroOffsetX, gyroOffsetY, gyroOffsetZ; // Offsets de calibração do giroscópio (°/s)

/* O que é: Variável global para armazenar a norma da aceleração.
   Para que serve: Representa a magnitude total da aceleração (m/s²) para verificar consistência.
   Caso de uso: Usada para confirmar que a aceleração total é ~9.81 m/s² quando o submarino está parado. */
float norm; // Norma da aceleração (m/s²)

// Funções declaradas
bool mpu_begin();
void readAcel();
void readGiro();
void mpu_calibrate(int leituras);
void mpu_loop();
void mpu_reset();
float getAccX();
float getAccY();
float getAccZ();
float getGyroX();
float getGyroY();
float getGyroZ();
float getNorm();

/* O que é: Função para inicializar o MPU6050.
   Para que serve: Configura o barramento I2C, verifica a conexão e define as faixas do sensor (±2g, ±250°/s).
   Caso de uso: Chamada no setup() para preparar o MPU6050 antes de leituras no submarino. */
bool mpu_begin() {
  Wire.begin(); // Inicia I2C (usa GP4 para SDA e GP5 para SCL no Pico)
  Wire.setClock(100000); // Define I2C para 100 kHz (alinhado com Arduino Uno)

  // Verifica se o MPU6050 está respondendo
  Wire.beginTransmission(MPU);
  int error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("Erro I2C: "); Serial.println(error);
    return false;
  }

  // Verifica registrador WHO_AM_I (0x75)
  Wire.beginTransmission(MPU);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 1, true);
  int whoAmI = Wire.read();
  Serial.print("WHO_AM_I (0x75): 0x"); Serial.println(whoAmI, HEX);
  if (whoAmI != 0x68) {
    Serial.println("Erro: MPU6050 não detectado!");
    return false;
  }

  // Reseta o MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x80); // Reset
  Wire.endTransmission(true);
  delay(200);

  // Desativa modo sleep
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(50);

  // Verifica registrador 0x6B
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 1, true);
  int reg6B = Wire.read();
  Serial.print("Registrador 0x6B (power): 0x"); Serial.println(reg6B, HEX);
  if (reg6B != 0x00) {
    Serial.println("Erro: Falha ao desativar modo sleep!");
    return false;
  }

  // Configura faixa do acelerômetro para ±2g
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Configura faixa do giroscópio para ±250°/s
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Verifica configurações
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 1, true);
  int reg1B = Wire.read();
  Serial.print("Registrador 0x1B (giroscópio): 0x"); Serial.println(reg1B, HEX);
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 1, true);
  int reg1C = Wire.read();
  Serial.print("Registrador 0x1C (acelerômetro): 0x"); Serial.println(reg1C, HEX);

  if (reg1B != 0x00 || reg1C != 0x00) {
    Serial.println("Erro: Configuração dos registradores 0x1B ou 0x1C falhou!");
    return false;
  }

  delay(50);
  return true;
}

/* O que é: Função para ler os dados do acelerômetro.
   Para que serve: Obtém os valores brutos do acelerômetro, aplica offsets, converte para m/s² e filtra ruídos.
   Caso de uso: Chamada em mpu_loop() para atualizar AccX/Y/Z, usado para detectar inclinação do submarino. */
void readAcel() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Registrador inicial dos dados do acelerômetro
  Wire.endTransmission(false);
  int bytesRead = Wire.requestFrom(MPU, 6, true);
  // Serial.print("Bytes lidos do acelerômetro: "); Serial.println(bytesRead);
  if (bytesRead != 6) {
    Serial.println("Erro: Falha na leitura do acelerômetro!");
    AccX = AccY = AccZ = 0;
    norm = 0;
    return;
  }

  // Converte dados brutos para m/s² (16384 LSB/g, 1g = 9.80665 m/s²)
  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();
  // Serial.print("Raw AccX: "); Serial.println(rawAccX);
  // Serial.print("Raw AccY: "); Serial.println(rawAccY);
  // Serial.print("Raw AccZ: "); Serial.println(rawAccZ);
  if (rawAccX == 0 && rawAccY == 0 && rawAccZ == 0) {
    Serial.println("Erro: Leituras nulas do acelerômetro!");
    AccX = AccY = AccZ = 0;
    norm = 0;
    return;
  }

  // Debugging: exibe offsets
  // Serial.print("accOffsetX: "); Serial.println(accOffsetX, 3);
  // Serial.print("accOffsetY: "); Serial.println(accOffsetY, 3);
  // Serial.print("accOffsetZ: "); Serial.println(accOffsetZ, 3);

  AccX = ((float)rawAccX / 16384.0f - accOffsetX) * G_TO_MS2;
  AccY = ((float)rawAccY / 16384.0f - accOffsetY) * G_TO_MS2;
  AccZ = ((float)rawAccZ / 16384.0f - accOffsetZ) * G_TO_MS2;

  // Debugging: exibe valores antes do filtro
  // Serial.print("AccX antes do filtro (m/s²): "); Serial.println(AccX, 3);
  // Serial.print("AccY antes do filtro (m/s²): "); Serial.println(AccY, 3);
  // Serial.print("AccZ antes do filtro (m/s²): "); Serial.println(AccZ, 3);

  // Aplica filtro (limiar de 0.49 m/s², equivalente a 0.05g)
  if (fabs(AccX) < 0.49) AccX = 0;
  if (fabs(AccY) < 0.49) AccY = 0;
  // Normaliza AccZ para ~9.80665 m/s² se próximo
  if (fabs(AccZ - G_TO_MS2) < 0.49) AccZ = G_TO_MS2;

  // Debugging: exibe valores após aplicar offsets e filtro
  // Serial.print("AccX após offset e filtro (m/s²): "); Serial.println(AccX, 3);
  // Serial.print("AccY após offset e filtro (m/s²): "); Serial.println(AccY, 3);
  // Serial.print("AccZ após offset e filtro (m/s²): "); Serial.println(AccZ, 3);

  // Calcula a magnitude da aceleração (m/s²)
  norm = sqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);
  // Serial.print("Norma da aceleração (m/s²): "); Serial.println(norm, 2);
}

/* O que é: Função para ler os dados do giroscópio.
   Para que serve: Obtém os valores brutos do giroscópio, aplica offsets, converte para rad/s e filtra ruídos.
   Caso de uso: Chamada em mpu_loop() para atualizar GyroX/Y/Z, usado para controlar rotação do submarino. */
void readGiro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Registrador inicial dos dados do giroscópio
  Wire.endTransmission(false);
  int bytesRead = Wire.requestFrom(MPU, 6, true);
  // Serial.print("Bytes lidos do giroscópio: "); Serial.println(bytesRead);
  if (bytesRead != 6) {
    Serial.println("Erro: Falha na leitura do giroscópio!");
    GyroX = GyroY = GyroZ = 0;
    return;
  }

  // Converte dados brutos para rad/s (131 LSB/°/s)
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();
  // Serial.print("Raw GyroX: "); Serial.println(rawGyroX);
  // Serial.print("Raw GyroY: "); Serial.println(rawGyroY);
  // Serial.print("Raw GyroZ: "); Serial.println(rawGyroZ);
  if (rawGyroX == 0 && rawGyroY == 0 && rawGyroZ == 0) {
    Serial.println("Erro: Leituras nulas do giroscópio!");
    GyroX = GyroY = GyroZ = 0;
    return;
  }

  // Debugging: exibe offsets antes da subtração
  // Serial.print("gyroOffsetX: "); Serial.println(gyroOffsetX, 3);
  // Serial.print("gyroOffsetY: "); Serial.println(gyroOffsetY, 3);
  // Serial.print("gyroOffsetZ: "); Serial.println(gyroOffsetZ, 3);

  // Converte para rad/s e aplica offsets
  GyroX = ((float)rawGyroX / 131.0f - gyroOffsetX) * DEG_PER_SEC_TO_RAD_PER_SEC;
  GyroY = ((float)rawGyroY / 131.0f - gyroOffsetY) * DEG_PER_SEC_TO_RAD_PER_SEC;
  GyroZ = ((float)rawGyroZ / 131.0f - gyroOffsetZ) * DEG_PER_SEC_TO_RAD_PER_SEC;

  // Debugging: exibe valores antes do filtro
  // Serial.print("GyroX antes do filtro (rad/s): "); Serial.println(GyroX, 3);
  // Serial.print("GyroY antes do filtro (rad/s): "); Serial.println(GyroY, 3);
  // Serial.print("GyroZ antes do filtro (rad/s): "); Serial.println(GyroZ, 3);

  // Aplica filtro (limiar de 0.0087 rad/s, equivalente a 0.5°/s)
  if (fabs(GyroX) < 0.0087) GyroX = 0;
  if (fabs(GyroY) < 0.0087) GyroY = 0;
  if (fabs(GyroZ) < 0.0087) GyroZ = 0;

  // Debugging: exibe valores após aplicar offsets e filtro
  // Serial.print("GyroX após offset e filtro (rad/s): "); Serial.println(GyroX, 3);
  // Serial.print("GyroY após offset e filtro (rad/s): "); Serial.println(GyroY, 3);
  // Serial.print("GyroZ após offset e filtro (rad/s): "); Serial.println(GyroZ, 3);
}

/* O que é: Função para calibrar o acelerômetro e giroscópio.
   Para que serve: Calcula os offsets médios para compensar erros sistemáticos nas leituras.
   Caso de uso: Chamada no setup() para garantir que AccX/Y/Z e GyroX/Y/Z sejam precisos antes do uso no submarino. */
void mpu_calibrate(int leituras) {
  accOffsetX = 0; accOffsetY = 0; accOffsetZ = 0;
  gyroOffsetX = 0; gyroOffsetY = 0; gyroOffsetZ = 0;
  int validReadings = 0;
  float rawAccZSum = 0;

  Serial.println("Calibrando acelerômetro e giroscópio...");
  for (int i = 0; i < leituras; i++) {
    // Lê acelerômetro
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU, 6, true) == 6) {
      int16_t rawAccX = Wire.read() << 8 | Wire.read();
      int16_t rawAccY = Wire.read() << 8 | Wire.read();
      int16_t rawAccZ = Wire.read() << 8 | Wire.read();
      // Valida leituras (relaxada)
      if (abs(rawAccX) < 32768 && abs(rawAccY) < 32768 && abs(rawAccZ) < 32768) {
        accOffsetX += rawAccX / 16384.0f;
        accOffsetY += rawAccY / 16384.0f;
        accOffsetZ += rawAccZ / 16384.0f;
        rawAccZSum += rawAccZ;
        validReadings++;
      }
    }

    // Lê giroscópio
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    if (Wire.requestFrom(MPU, 6, true) == 6) {
      int16_t rawGyroX = Wire.read() << 8 | Wire.read();
      int16_t rawGyroY = Wire.read() << 8 | Wire.read();
      int16_t rawGyroZ = Wire.read() << 8 | Wire.read();
      // Valida leituras (relaxada)
      if (abs(rawGyroX) < 32768 && abs(rawGyroY) < 32768 && abs(rawGyroZ) < 32768) {
        gyroOffsetX += rawGyroX / 131.0f;
        gyroOffsetY += rawGyroY / 131.0f;
        gyroOffsetZ += rawGyroZ / 131.0f;
      }
    }

    delay(20); // Atraso aumentado para estabilidade
  }

  // Calcula a média dos offsets
  if (validReadings > 0) {
    accOffsetX /= validReadings;
    accOffsetY /= validReadings;
    accOffsetZ /= validReadings;
    accOffsetZ -= 1.0f; // Remove a gravidade (1g)
    gyroOffsetX /= validReadings;
    gyroOffsetY /= validReadings;
    gyroOffsetZ /= validReadings;
    Serial.print("Raw AccZ médio: "); Serial.println(rawAccZSum / validReadings);
  } else {
    Serial.println("Erro: Nenhuma leitura válida durante a calibração!");
  }

  Serial.print("Leituras válidas: "); Serial.println(validReadings);
  Serial.print("Offsets Acelerômetro: X="); Serial.print(accOffsetX, 3);
  Serial.print(", Y="); Serial.print(accOffsetY, 3);
  Serial.print(", Z="); Serial.println(accOffsetZ, 3);
  Serial.print("Offsets Giroscópio: X="); Serial.print(gyroOffsetX, 3);
  Serial.print(", Y="); Serial.print(gyroOffsetY, 3);
  Serial.print(", Z="); Serial.println(gyroOffsetZ, 3);
}

/* O que é: Função para processar os dados do sensor.
   Para que serve: Chama readAcel() e readGiro() para atualizar os valores de aceleração e giroscópio.
   Caso de uso: Chamada no loop() para atualizar continuamente os dados do MPU6050 no controle do submarino. */
void mpu_loop() {
  // Serial.println("Executando mpu_loop...");
  readAcel();
  readGiro();
  // Serial.println("Fim de mpu_loop");
}

/* O que é: Função para resetar os offsets do sensor.
   Para que serve: Zera os offsets de calibração do acelerômetro e giroscópio.
   Caso de uso: Usada para reiniciar a calibração se necessário, antes de recalibrar o sensor no submarino. */
void mpu_reset() {
  accOffsetX = 0;
  accOffsetY = 0;
  accOffsetZ = 0;
  gyroOffsetX = 0;
  gyroOffsetY = 0;
  gyroOffsetZ = 0;
}

/* O que é: Função para obter o valor de AccX.
   Para que serve: Retorna a aceleração no eixo X em m/s².
   Caso de uso: Usada no loop() para exibir AccX ou no controle de inclinação do submarino. */
float getAccX() { return AccX; }

/* O que é: Função para obter o valor de AccY.
   Para que serve: Retorna a aceleração no eixo Y em m/s².
   Caso de uso: Usada para monitoramento ou controle secundário de inclinação no submarino. */
float getAccY() { return AccY; }

/* O que é: Função para obter o valor de AccZ.
   Para que serve: Retorna a aceleração no eixo Z em m/s², normalmente ~9.80665 m/s² quando parado.
   Caso de uso: Usada para verificar a orientação vertical do submarino ou detectar acelerações verticais. */
float getAccZ() { return AccZ; }

/* O que é: Função para obter o valor de GyroX.
   Para que serve: Retorna a velocidade angular no eixo X em rad/s.
   Caso de uso: Usada para monitoramento ou controle secundário de rotação no submarino. */
float getGyroX() { return GyroX; }

/* O que é: Função para obter o valor de GyroY.
   Para que serve: Retorna a velocidade angular no eixo Y em rad/s.
   Caso de uso: Usada para monitoramento ou controle secundário de rotação no submarino. */
float getGyroY() { return GyroY; }

/* O que é: Função para obter o valor de GyroZ.
   Para que serve: Retorna a velocidade angular no eixo Z em rad/s, essencial para controle de yaw.
   Caso de uso: Usada para ajustar motores diferenciais e corrigir a rotação do submarino. */
float getGyroZ() { return GyroZ; }

/* O que é: Função para obter a norma da aceleração.
   Para que serve: Retorna a magnitude total da aceleração em m/s².
   Caso de uso: Usada para verificar se a aceleração total é ~9.81 m/s², confirmando que o submarino está parado. */
float getNorm() { return norm; }

#endif // MPU6050_H
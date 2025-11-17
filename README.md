#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h> 

// --- ⚠️ CONFIGURACIÓN DEL PROYECTO ⚠️ ---
// 1. SUSTITUYE con el nombre de tu librería real de Edge Impulse
#include "uwuwuwuwu_inferencing.h" 

// 2. Nombre del dispositivo Bluetooth
#define BT_DEVICE_NAME "Muñeco_BT" 
// 3. PIN de emparejamiento (4 es la longitud del PIN "1234")
#define BT_PIN "1234" 

// --- VARIABLES DE REPORTE DINÁMICO ---
unsigned long reporte_intervalo_min = 1; // INICIAMOS EN 1 MINUTO PARA PRUEBAS
unsigned long reporte_intervalo_ms = 1 * 60 * 1000; 
unsigned long proximoReporteMS = 0; 
unsigned long tiempoInicioTerapia = 0;

// --- CONFIGURACIÓN DE HARDWARE Y IA ---
#define TCAADDR 0x70 
#define I2C_SDA_PIN 21 
#define I2C_SCL_PIN 22
Adafruit_MPU6050 mpu; 
BluetoothSerial SerialBT; 

const int CANALES_IMU[] = {0, 1, 2, 3, 4};
const int NUM_SENSORES = 5;
#define NUM_EJES 30 // 5 sensores * 6 ejes (AccX, Y, Z, GyroX, Y, Z) = 30
const unsigned long UMBRAL_RAFAGA_MS = 1500; // 1.5 segundos para ráfagas

// --- VARIABLES GLOBALES Y DE INFERENCIA ---
ei_impulse_result_t result = {0}; // Declarada Globalmente para todo el archivo
signal_t ei_signal_input; 

// --- LÓGICA DE CONTEO Y SCORING ---
unsigned long tiempoUltimoEvento[EI_CLASSIFIER_LABEL_COUNT] = {0};
int contadorFrecuencia[EI_CLASSIFIER_LABEL_COUNT] = {0}; 
unsigned long ultimoGolpeMS = 0;

int totalMovimientosAgresivos = 0;
int totalMovimientosInteraccion = 0;
int totalGolpesAislados = 0; 
int totalRafagas = 0;        

// Variables de Inferencia
float buffer_inferencia[NUM_EJES] = {0};
unsigned long ultima_inferencia = 0;


//-----------------------------------------------------------------------
// FUNCIÓN: tcaSelect - Controla el Multiplexor I2C
//-----------------------------------------------------------------------
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

//-----------------------------------------------------------------------
// FUNCIÓN: ei_fusion_sample_callback - Interfaz de Edge Impulse
//-----------------------------------------------------------------------
int ei_fusion_sample_callback(size_t offset, size_t length, float *out_ptr) {
  if (offset + length > EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) return -1;
  memcpy(out_ptr, buffer_inferencia + offset, length * sizeof(float));
  return 0;
}

//-----------------------------------------------------------------------
// FUNCIÓN: leerSensores - Captura 30 ejes de datos
//-----------------------------------------------------------------------
bool leerSensores() {
  sensors_event_t a, g, temp;
  int indice = 0;
  
  for (int i = 0; i < NUM_SENSORES; i++) {
    tcaSelect(CANALES_IMU[i]);
    if (!mpu.getEvent(&a, &g, &temp)) {
        // Por seguridad, llenaremos con cero si hay error de lectura
        buffer_inferencia[indice++] = 0.0;
        buffer_inferencia[indice++] = 0.0;
        buffer_inferencia[indice++] = 0.0;
        buffer_inferencia[indice++] = 0.0;
        buffer_inferencia[indice++] = 0.0;
        buffer_inferencia[indice++] = 0.0;
    } else {
        // Lectura exitosa
        buffer_inferencia[indice++] = a.acceleration.x;
        buffer_inferencia[indice++] = a.acceleration.y;
        buffer_inferencia[indice++] = a.acceleration.z;
        buffer_inferencia[indice++] = g.gyro.x;
        buffer_inferencia[indice++] = g.gyro.y;
        buffer_inferencia[indice++] = g.gyro.z;
    }
  }
  tcaSelect(0); // Volver al canal 0
  return true;
}


//-----------------------------------------------------------------------
// FUNCIÓN: Generar Reporte Periódico (CORREGIDO DE AMBITO DE VARIABLE)
//-----------------------------------------------------------------------
void generarReporte() {
    // 1. Calcular el Puntaje de Agresividad Ponderado (PAP)
    const float W_R = 3.0; // Peso Ráfaga
    const float W_G = 2.0; // Peso Golpe Aislado
    const float W_I = 1.0; // Peso Interacción
    
    float puntajePonderado = (totalRafagas * W_R) + (totalGolpesAislados * W_G) + (totalMovimientosInteraccion * W_I);
    
    // 2. Determinar la duración del periodo de reporte
    float duracionMinutos = (millis() - tiempoInicioTerapia) / 60000.0;
    if (duracionMinutos < 1.0) duracionMinutos = 1.0; 
    
    float papPorMinuto = puntajePonderado / duracionMinutos;

    // 3. Asignar la Etiqueta Final (Interpretación Clínica)
    String etiquetaFinal = "REPOSO PREDOMINANTE";

    if (papPorMinuto > 3.5) {
        etiquetaFinal = "AGRESIVO ALTO";
    } else if (papPorMinuto >= 2.0) { 
        etiquetaFinal = "DESCARGA MODERADA";
    } else if (papPorMinuto >= 1.0) { 
        etiquetaFinal = "JUEGO EXPLORATORIO";
    }
    
    // 4. Construir el Reporte (Resumen)
    String reporte = "REPORTE_FRECUENCIA|AGR_TOTAL:" + String(totalMovimientosAgresivos) + 
                     "|INT_TOTAL:" + String(totalMovimientosInteraccion) +
                     "|MINUTO_ACTUAL:" + String(duracionMinutos, 1) +
                     "|PAP_MIN:" + String(papPorMinuto, 2) + 
                     "|ESTADO_FINAL:" + etiquetaFinal; 
                     
    // 5. Desglose de los movimientos por clase (DETALLE) - Usa SOLO variables de conteo globales
    reporte += "\n[DETALLE]";
    reporte += "|RÁFAGAS:" + String(totalRafagas);
    reporte += "|GOLPES_AISLADOS:" + String(totalGolpesAislados);
    reporte += "|TOTAL_INTERACCIÓN:" + String(totalMovimientosInteraccion);
    
    if (SerialBT.connected()) {
        SerialBT.println(reporte);
    }
    Serial.println("--- REPORTE PERIÓDICO ENVIADO --- " + reporte);
}

//-----------------------------------------------------------------------
// FUNCIÓN: PROCESAR COMANDOS BLUETOOTH (Control Remoto)
//-----------------------------------------------------------------------
void checkBluetoothCommands() {
    if (SerialBT.available()) {
        String comando = SerialBT.readStringUntil('\n');
        comando.trim();

        if (comando.startsWith("MINUTOS:")) {
            int minutos = comando.substring(8).toInt(); 
            
            if (minutos > 0 && minutos <= 60) {
                reporte_intervalo_min = minutos;
                reporte_intervalo_ms = minutos * 60 * 1000;
                proximoReporteMS = millis() + reporte_intervalo_ms;

                String respuesta = "INTERVALO CAMBIADO A " + String(minutos) + " MINUTOS.";
                SerialBT.println(respuesta);
                Serial.println(respuesta);
            } else {
                SerialBT.println("ERROR: Rango MINUTOS: entre 1 y 60.");
            }
        }
    }
}


//-----------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  // Inicialización de Bluetooth con corrección de sintaxis de setPin
  SerialBT.begin(BT_DEVICE_NAME); 
  SerialBT.setPin(BT_PIN, 4); // Corrección de sintaxis para setPin

  tiempoInicioTerapia = millis(); 
  proximoReporteMS = tiempoInicioTerapia + reporte_intervalo_ms;
  Serial.printf("Bluetooth iniciado. Dispositivo: %s.\n", BT_DEVICE_NAME);

  // Inicialización de I2C y Sensores
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  for (int i = 0; i < NUM_SENSORES; i++) {
    tcaSelect(CANALES_IMU[i]); 
    if (!mpu.begin()) {
      Serial.printf("Error MPU-6050 en Canal %d. Verifique el cableado.\n", CANALES_IMU[i]);
    } else {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    }
  }
  tcaSelect(0); // Volver al canal 0
  
  ei_signal_input.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  ei_signal_input.get_data = &ei_fusion_sample_callback;

  Serial.println("Sistema de Detección e Inferencia Inicializado.");
}

//-----------------------------------------------------------------------
// LOOP CORREGIDO (MÉTODO 1: Reporte y Comando Siempre Verificados)
//-----------------------------------------------------------------------
void loop() {
  // 1. Chequea comandos Bluetooth (Control Remoto)
  checkBluetoothCommands();

  // 2. Reporte Periódico (PRIORIDAD ALTA: Se verifica SIEMPRE)
  if (millis() >= proximoReporteMS) {
      generarReporte();
      proximoReporteMS += reporte_intervalo_ms;
  }
  
  // 3. Control de frecuencia de inferencia
  if (millis() - ultima_inferencia < EI_CLASSIFIER_INTERVAL_MS) {
      return; 
  }
  ultima_inferencia = millis();

  // 4. Leer los 5 sensores e inferir
  if (!leerSensores()) return;
  
  int err = run_classifier(&ei_signal_input, &result, false); 
  if (err != 0) return;

  // 5. Procesar el resultado de la IA
  int indiceMejorClase = -1;
  float probabilidadMax = 0.0;
  
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > probabilidadMax) {
      probabilidadMax = result.classification[i].value;
      indiceMejorClase = i;
    }
  }

  // 6. Lógica de Detección (Umbral REDUCIDO para pruebas)
  if (probabilidadMax > 0.40) { // Umbral al 40% para mayor sensibilidad
    String evento = result.classification[indiceMejorClase].label;
    
    String categoria = "REPOSO";
    
    // --- LÓGICA DE CLASIFICACIÓN CON TUS ETIQUETAS REALES ---
    // Etiqueta: "golpe_cuerpo", "apreton_fuerte", "caida" son AGRESION
    // Otras etiquetas como "colgado_inmovil" o las que inicien con "golpe_" se asumen AGRESIÓN
    if (evento.equals("golpe_cuerpo") || evento.equals("apreton_fuerte") || evento.equals("caida") || evento.startsWith("golpe_")) { 
        categoria = "AGRESION";
    } 
    // Si grabaste movimientos de juego/interacción (ej. "juego_lento", "abrazo_suave"), agrégales aquí:
    // else if (evento.equals("juego_lento") || evento.equals("abrazo_suave")) {
    //     categoria = "INTERACCION";
    // }
    // --------------------------------------------------------
    
    if (categoria != "REPOSO") { 
      
      unsigned long tiempoActual = millis();
      unsigned long lapsoDesdeUltimo = tiempoActual - tiempoUltimoEvento[indiceMejorClase];
      tiempoUltimoEvento[indiceMejorClase] = tiempoActual;
      contadorFrecuencia[indiceMejorClase]++; 
      
      // LÓGICA CIENTÍFICA: Detección de Ráfaga y Contadores Ponderados
      String flagRafaga = "NO";
      if (categoria == "AGRESION") {
          totalMovimientosAgresivos++;
          
          if (tiempoActual - ultimoGolpeMS < UMBRAL_RAFAGA_MS) {
              flagRafaga = "RAFAGA"; 
              totalRafagas++; 
          } else {
              totalGolpesAislados++; 
          }
          ultimoGolpeMS = tiempoActual;
      } else if (categoria == "INTERACCION") {
          totalMovimientosInteraccion++;
      }
      
      // 7. Enviar Datos por BLUETOOTH (En tiempo real, cada evento)
      if (SerialBT.connected()) {
          String mensaje = categoria + 
                           "|" + evento + 
                           "|" + String(lapsoDesdeUltimo) + 
                           "|" + String(totalGolpesAislados) + 
                           "|" + String(totalRafagas) + 
                           "|" + flagRafaga +
                           "|" + String(probabilidadMax, 2);
                           
          SerialBT.println(mensaje);
      }
    }
  }
}

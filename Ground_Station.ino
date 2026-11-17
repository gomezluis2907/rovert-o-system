#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_mac.h>
#include <time.h>
#include <sys/time.h>
#include <stddef.h>

// ========================================
// 1. CONFIGURACION
// ========================================

uint8_t macTerrena[] = {
  0x8C, 0x94, 0xDF, 0x70, 0x64, 0x34
};

#define CANAL_WIFI 1
#define PIN_BOTON 0

const bool ENVIO_AUTOMATICO = true;

const uint32_t intervaloPing_ms = 1000;
const uint32_t tiempoDebounce_ms = 300;

// Identificacion de la mision
const uint16_t ID_MISION = 1;

// Protocolo de aplicacion
const uint16_t SINCRONIZACION = 0xA55A;
const uint8_t VERSION_TRAMA = 1;

const uint8_t MSG_PING = 1;
const uint8_t MSG_PONG = 2;

// Categorias de datos
const uint8_t TELEMETRIA_CONTROL = 1;
const uint8_t TELEMETRIA_CARGA_UTIL = 2;

// Por ahora transmitimos estado del rover
const uint8_t CLASE_ACTUAL = TELEMETRIA_CONTROL;

// ========================================
// 2. ESTRUCTURA DE DATOS
// ========================================

// Datos disponibles sin sensores externos
typedef struct __attribute__((packed)) {
  uint32_t uptime_ms;
  uint16_t estado;
  uint16_t eventos;
} DatosTrama_t;

// Trama completa
typedef struct __attribute__((packed)) {

  uint16_t sincronizacion;
  uint8_t version;
  uint16_t longitud;

  uint32_t fecha;       // AAAAMMDD
  uint32_t hora;        // HHMMSS UTC
  uint8_t fechaValida;  // 1 = sincronizada

  uint16_t mision;
  uint8_t tipoTelemetria;
  uint8_t tipoMensaje;

  uint32_t contador;
  uint32_t t_envio_ms;

  DatosTrama_t datos;

  uint16_t crc16;

} Trama_t;

static_assert(sizeof(DatosTrama_t) == 8,
              "Error en tamano de datos");

static_assert(sizeof(Trama_t) == 36,
              "La trama debe tener 36 bytes");

// ========================================
// 3. VARIABLES DEL SISTEMA
// ========================================

esp_now_peer_info_t peerInfo;

QueueHandle_t colaPong = nullptr;
QueueHandle_t colaAck = nullptr;

uint32_t contadorPing = 0;
uint32_t ultimoPing_ms = 0;
uint32_t ultimoRebote_ms = 0;

bool botonAnterior = false;
bool enlaceListo = false;
bool relojSincronizado = false;

// ========================================
// 4. CRC-16/CCITT-FALSE
// Polinomio 0x1021, inicio 0xFFFF
// ========================================

uint16_t calcularCRC(const Trama_t &trama) {

  const uint8_t *datos =
    reinterpret_cast<const uint8_t *>(&trama);

  size_t longitud =
    offsetof(Trama_t, crc16);

  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < longitud; i++) {

    crc ^= ((uint16_t)datos[i] << 8);

    for (int j = 0; j < 8; j++) {

      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

// ========================================
// 5. VALIDACION DE TRAMA
// ========================================

bool verificarTrama(const Trama_t &trama) {

  bool estructuraOK =
    trama.sincronizacion == SINCRONIZACION &&
    trama.version == VERSION_TRAMA &&
    trama.longitud == sizeof(Trama_t) &&
    trama.mision == ID_MISION &&
    (trama.tipoMensaje == MSG_PING ||
     trama.tipoMensaje == MSG_PONG) &&
    (trama.tipoTelemetria == TELEMETRIA_CONTROL ||
     trama.tipoTelemetria == TELEMETRIA_CARGA_UTIL) &&
    trama.fechaValida <= 1;

  if (trama.fechaValida == 0) {
    estructuraOK = estructuraOK &&
                   trama.fecha == 0 &&
                   trama.hora == 0;
  } else {
    estructuraOK = estructuraOK &&
                   trama.fecha >= 20200101UL &&
                   trama.hora <= 235959UL &&
                   (trama.hora / 10000UL) <= 23 &&
                   ((trama.hora / 100UL) % 100) <= 59 &&
                   (trama.hora % 100) <= 59;
  }

  uint16_t crcCalculado = calcularCRC(trama);

  bool crcOK = (crcCalculado == trama.crc16);

  Serial.printf(
    "[CHECK] Estructura: %s\n",
    estructuraOK ? "OK" : "ERROR"
  );

  Serial.printf(
    "[CHECK] CRC-16: %s\n",
    crcOK ? "OK" : "ERROR"
  );

  Serial.printf(
    "[CHECK] CRC recibido: 0x%04X\n",
    trama.crc16
  );

  Serial.printf(
    "[CHECK] CRC calculado: 0x%04X\n",
    crcCalculado
  );

  return estructuraOK && crcOK;
}

// ========================================
// 6. SINCRONIZACION DE FECHA Y HORA
// ========================================

// Entrada por Monitor Serie:
// HORA=2026-11-11 20:30:00
//
// La hora se interpreta como UTC.

void procesarComandosSerial() {

  if (!Serial.available()) return;

  String comando = Serial.readStringUntil('\n');
  comando.trim();

  int anio, mes, dia, hora, minuto, segundo;

  int campos = sscanf(
    comando.c_str(),
    "HORA=%d-%d-%d %d:%d:%d",
    &anio, &mes, &dia,
    &hora, &minuto, &segundo
  );

  if (campos != 6) {
    Serial.println(
      "[RELOJ] Formato: HORA=AAAA-MM-DD HH:MM:SS"
    );
    return;
  }

  if (anio < 2020 || anio > 2099 ||
      mes < 1 || mes > 12 ||
      dia < 1 || dia > 31 ||
      hora < 0 || hora > 23 ||
      minuto < 0 || minuto > 59 ||
      segundo < 0 || segundo > 59) {

    Serial.println("[RELOJ] Fecha u hora invalida");
    return;
  }

  struct tm fecha = {};

  fecha.tm_year = anio - 1900;
  fecha.tm_mon = mes - 1;
  fecha.tm_mday = dia;
  fecha.tm_hour = hora;
  fecha.tm_min = minuto;
  fecha.tm_sec = segundo;
  fecha.tm_isdst = 0;

  // Configurar la zona horaria UTC
  setenv("TZ", "UTC0", 1);
  tzset();

  time_t epoch = mktime(&fecha);

  // Verificar que mktime no normalizo
  // una fecha inexistente (ej. 31 de febrero)
  if (epoch == (time_t)-1 ||
      fecha.tm_year != anio - 1900 ||
      fecha.tm_mon != mes - 1 ||
      fecha.tm_mday != dia ||
      fecha.tm_hour != hora ||
      fecha.tm_min != minuto ||
      fecha.tm_sec != segundo) {

    Serial.println("[RELOJ] Fecha invalida");
    return;
  }

  struct timeval tv;
  tv.tv_sec = epoch;
  tv.tv_usec = 0;

  if (settimeofday(&tv, nullptr) == 0) {

    relojSincronizado = true;

    Serial.println(
      "[RELOJ] Fecha y hora UTC sincronizadas"
    );

  } else {
    Serial.println(
      "[RELOJ] Error al configurar el reloj"
    );
  }
}

// ========================================
// 7. LLENAR FECHA Y HORA
// ========================================

void llenarFechaHora(Trama_t &trama) {

  trama.fecha = 0;
  trama.hora = 0;
  trama.fechaValida = 0;

  if (!relojSincronizado) return;

  time_t tiempoActual = time(nullptr);
  struct tm fechaUTC;

  if (gmtime_r(&tiempoActual, &fechaUTC) == nullptr) {
    return;
  }

  trama.fecha =
    (fechaUTC.tm_year + 1900) * 10000UL +
    (fechaUTC.tm_mon + 1) * 100UL +
    fechaUTC.tm_mday;

  trama.hora =
    fechaUTC.tm_hour * 10000UL +
    fechaUTC.tm_min * 100UL +
    fechaUTC.tm_sec;

  trama.fechaValida = 1;
}

// ========================================
// 8. MOSTRAR MAC
// ========================================

bool mostrarMAC() {

  uint8_t mac[6];

  esp_err_t err = esp_read_mac(
    mac, ESP_MAC_WIFI_STA
  );

  if (err == ESP_OK) {
    Serial.printf(
      "MAC STA calculada: %02X:%02X:%02X:%02X:%02X:%02X\n",
      mac[0], mac[1], mac[2],
      mac[3], mac[4], mac[5]
    );
  }

  err = esp_wifi_get_mac(WIFI_IF_STA, mac);

  if (err != ESP_OK) {
    Serial.printf(
      "ERROR MAC: %s\n",
      esp_err_to_name(err)
    );
    return false;
  }

  Serial.printf(
    "MAC STA activa: %02X:%02X:%02X:%02X:%02X:%02X\n",
    mac[0], mac[1], mac[2],
    mac[3], mac[4], mac[5]
  );

  uint8_t suma = 0;

  for (int i = 0; i < 6; i++) {
    suma |= mac[i];
  }

  return suma != 0;
}

// ========================================
// 9. IMPRIMIR CONTENIDO DE LA TRAMA
// ========================================

void imprimirTrama(const Trama_t &trama) {

  Serial.println();
  Serial.println("================================");
  Serial.println("       TRAMA DE TELEMETRIA");
  Serial.println("================================");

  Serial.printf(
    "Longitud        : %u bytes\n",
    (unsigned)trama.longitud
  );

  Serial.printf(
    "Version         : %u\n",
    trama.version
  );

  if (trama.fechaValida) {

    Serial.printf(
      "Fecha UTC       : %08lu\n",
      (unsigned long)trama.fecha
    );

    Serial.printf(
      "Hora UTC        : %06lu\n",
      (unsigned long)trama.hora
    );

  } else {

    Serial.println(
      "Fecha y hora    : SIN SINCRONIZAR"
    );
  }

  Serial.printf(
    "Mision          : %u\n",
    trama.mision
  );

  Serial.printf(
    "Telemetria      : %s\n",
    trama.tipoTelemetria == TELEMETRIA_CONTROL
      ? "CONTROL / ESTADO"
      : "CARGA UTIL"
  );

  Serial.printf(
    "Mensaje         : %s\n",
    trama.tipoMensaje == MSG_PING
      ? "PING"
      : "PONG"
  );

  Serial.printf(
    "Contador        : %lu\n",
    (unsigned long)trama.contador
  );

  Serial.printf(
    "Tiempo envio    : %lu ms\n",
    (unsigned long)trama.t_envio_ms
  );

  Serial.println("--- DATOS ---");

  Serial.printf(
    "Uptime          : %lu ms\n",
    (unsigned long)trama.datos.uptime_ms
  );

  Serial.printf(
    "Estado sistema  : %u\n",
    trama.datos.estado
  );

  Serial.printf(
    "Eventos         : %u\n",
    trama.datos.eventos
  );

  Serial.println("--- CHECK DE TRAMA ---");

  bool valida = verificarTrama(trama);

  Serial.printf(
    "Resultado final : %s\n",
    valida ? "TRAMA VALIDA" : "TRAMA INVALIDA"
  );

  Serial.println("--- BYTES DE LA TRAMA ---");

  const uint8_t *bytes =
    reinterpret_cast<const uint8_t *>(&trama);

  for (size_t i = 0; i < sizeof(Trama_t); i++) {
    Serial.printf("%02X ", bytes[i]);
  }

  Serial.println();
  Serial.println("================================");
}

// ========================================
// 10. CALLBACK DE ENVIO
// ========================================

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)

void OnDataSent(
  const esp_now_send_info_t *info,
  esp_now_send_status_t status
) {

#else

void OnDataSent(
  const uint8_t *mac_addr,
  esp_now_send_status_t status
) {

#endif

  uint8_t ack =
    (status == ESP_NOW_SEND_SUCCESS) ? 1 : 0;

  if (colaAck != nullptr) {
    xQueueSend(colaAck, &ack, 0);
  }
}

// ========================================
// 11. CALLBACK DE RECEPCION
// ========================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {

  if (len != sizeof(Trama_t)) return;

  if (info == nullptr ||
      info->src_addr == nullptr ||
      data == nullptr) {
    return;
  }

  // Solo aceptar respuesta de la base
  if (memcmp(info->src_addr, macTerrena, 6) != 0) {
    return;
  }

  Trama_t trama;
  memcpy(&trama, data, sizeof(trama));

  if (colaPong != nullptr) {
    xQueueSend(colaPong, &trama, 0);
  }
}

// ========================================
// 12. ENVIAR TRAMA PING
// ========================================

void enviarPing() {

  contadorPing++;

  Trama_t trama = {};

  trama.sincronizacion = SINCRONIZACION;
  trama.version = VERSION_TRAMA;
  trama.longitud = sizeof(Trama_t);

  llenarFechaHora(trama);

  trama.mision = ID_MISION;
  trama.tipoTelemetria = CLASE_ACTUAL;
  trama.tipoMensaje = MSG_PING;

  trama.contador = contadorPing;
  trama.t_envio_ms = millis();

  // Datos reales disponibles del ESP32
  trama.datos.uptime_ms = millis();
  trama.datos.estado = 1; // Programa activo
  trama.datos.eventos = (uint16_t)contadorPing;

  // El CRC debe calcularse al final
  trama.crc16 = calcularCRC(trama);

  // Mostrar contenido y comprobacion
  imprimirTrama(trama);

  esp_err_t resultado = esp_now_send(
    macTerrena,
    (uint8_t *)&trama,
    sizeof(trama)
  );

  Serial.printf(
    "[ROVER] Envio PING #%lu: %s\n",
    (unsigned long)contadorPing,
    esp_err_to_name(resultado)
  );
}

// ========================================
// 13. SETUP
// ========================================

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(1000);

  Serial.println();
  Serial.println("============================");
  Serial.println("       ROVER ONBOARD");
  Serial.println("============================");

  // Reloj en UTC. Sin sincronizacion inicial.
  setenv("TZ", "UTC0", 1);
  tzset();

  pinMode(PIN_BOTON, INPUT_PULLUP);

  if (!WiFi.mode(WIFI_STA)) {
    Serial.println("ERROR: WiFi STA");
    return;
  }

  delay(100);

  if (!mostrarMAC()) {
    Serial.println("ERROR: MAC invalida");
    return;
  }

  esp_err_t resultado = esp_wifi_set_channel(
    CANAL_WIFI,
    WIFI_SECOND_CHAN_NONE
  );

  if (resultado != ESP_OK) {
    Serial.printf(
      "ERROR CANAL: %s\n",
      esp_err_to_name(resultado)
    );
    return;
  }

  uint8_t canalActivo;
  wifi_second_chan_t canalSecundario;

  resultado = esp_wifi_get_channel(
    &canalActivo,
    &canalSecundario
  );

  if (resultado != ESP_OK ||
      canalActivo != CANAL_WIFI) {
    Serial.println("ERROR: Canal WiFi incorrecto");
    return;
  }

  Serial.printf(
    "Canal WiFi activo: %u\n",
    canalActivo
  );

  colaPong = xQueueCreate(
    10, sizeof(Trama_t)
  );

  colaAck = xQueueCreate(
    10, sizeof(uint8_t)
  );

  if (colaPong == nullptr ||
      colaAck == nullptr) {
    Serial.println("ERROR: Crear colas");
    return;
  }

  resultado = esp_now_init();

  if (resultado != ESP_OK) {
    Serial.printf(
      "ERROR ESP-NOW: %s\n",
      esp_err_to_name(resultado)
    );
    return;
  }

  resultado = esp_now_register_send_cb(OnDataSent);

  if (resultado != ESP_OK) {
    Serial.println("ERROR: Callback envio");
    return;
  }

  resultado = esp_now_register_recv_cb(OnDataRecv);

  if (resultado != ESP_OK) {
    Serial.println("ERROR: Callback recepcion");
    return;
  }

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, macTerrena, 6);

  peerInfo.channel = CANAL_WIFI;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  resultado = esp_now_add_peer(&peerInfo);

  if (resultado != ESP_OK) {
    Serial.printf(
      "ERROR PEER: %s\n",
      esp_err_to_name(resultado)
    );
    return;
  }

  enlaceListo = true;
  ultimoPing_ms = millis();

  Serial.println("ESP-NOW inicializado.");
  Serial.printf(
    "Longitud de trama: %u bytes\n",
    (unsigned)sizeof(Trama_t)
  );

  Serial.println(
    "Para sincronizar fecha y hora UTC:"
  );

  Serial.println(
    "HORA=AAAA-MM-DD HH:MM:SS"
  );

  Serial.println(
    ENVIO_AUTOMATICO
      ? "PING automatico cada segundo."
      : "Presiona BOOT para transmitir."
  );
}

// ========================================
// 14. LOOP
// ========================================

void loop() {

  // Permitir configurar el reloj por Serial
  procesarComandosSerial();

  if (!enlaceListo) {
    delay(500);
    return;
  }

  uint32_t ahora = millis();

  // Envio automatico
  if (ENVIO_AUTOMATICO) {

    if (ahora - ultimoPing_ms >= intervaloPing_ms) {

      ultimoPing_ms = ahora;
      enviarPing();
    }

  } else {

    // Envio manual mediante BOOT
    bool presionado =
      (digitalRead(PIN_BOTON) == LOW);

    if (presionado &&
        !botonAnterior &&
        ahora - ultimoRebote_ms >= tiempoDebounce_ms) {

      ultimoRebote_ms = ahora;
      enviarPing();
    }

    botonAnterior = presionado;
  }

  // Confirmaciones de envio
  uint8_t ack;

  while (xQueueReceive(
           colaAck, &ack, 0
         ) == pdTRUE) {

    Serial.println(
      ack == 1
        ? "[ROVER] ACK recibido"
        : "[ROVER] ACK fallido"
    );
  }

  // Procesar tramas recibidas
  Trama_t rxTrama;

  while (xQueueReceive(
           colaPong, &rxTrama, 0
         ) == pdTRUE) {

    Serial.println();
    Serial.println("[ROVER] Trama recibida:");

    if (!verificarTrama(rxTrama)) {
      Serial.println(
        "[ERROR] Trama descartada"
      );
      continue;
    }

    if (rxTrama.tipoMensaje != MSG_PONG) {
      Serial.println(
        "[ERROR] No es un PONG"
      );
      continue;
    }

    // Solo aceptar respuesta del ultimo PING
    if (rxTrama.contador != contadorPing) {
      Serial.println(
        "[ERROR] Contador no coincide"
      );
      continue;
    }

    uint32_t rtt =
      millis() - rxTrama.t_envio_ms;

    Serial.printf(
      "[ROVER] PONG #%lu recibido\n",
      (unsigned long)rxTrama.contador
    );

    Serial.printf(
      "[ROVER] RTT: %lu ms\n",
      (unsigned long)rtt
    );

    Serial.println(
      "[CHECK] Trama recibida correctamente"
    );
  }

  delay(10);
}
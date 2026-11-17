#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_mac.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stddef.h>

// ========================================
// 1. CONFIGURACION
// ========================================

// MAC del ROVER
// IMPORTANTE: confirmar su MAC STA activa
uint8_t macRover[] = {
  0x58, 0x2A, 0xBD, 0x71, 0xE5, 0xE4
};

#define CANAL_WIFI 1

const uint16_t ID_MISION = 1;
const uint16_t SINCRONIZACION = 0xA55A;
const uint8_t VERSION_TRAMA = 1;

const uint8_t MSG_PING = 1;
const uint8_t MSG_PONG = 2;

const uint8_t TELEMETRIA_CONTROL = 1;
const uint8_t TELEMETRIA_CARGA_UTIL = 2;

// ========================================
// 2. ESTRUCTURA DE DATOS
// IDENTICA A LA DEL ROVER
// ========================================

typedef struct __attribute__((packed)) {
  uint32_t uptime_ms;
  uint16_t estado;
  uint16_t eventos;
} DatosTrama_t;

typedef struct __attribute__((packed)) {

  uint16_t sincronizacion;
  uint8_t version;
  uint16_t longitud;

  uint32_t fecha;
  uint32_t hora;
  uint8_t fechaValida;

  uint16_t mision;
  uint8_t tipoTelemetria;
  uint8_t tipoMensaje;

  uint32_t contador;
  uint32_t t_envio_ms;

  DatosTrama_t datos;

  uint16_t crc16;

} Trama_t;

static_assert(sizeof(DatosTrama_t) == 8,
              "Error en longitud de datos");

static_assert(sizeof(Trama_t) == 36,
              "La trama debe medir 36 bytes");

// Evento recibido con la MAC de origen
typedef struct {
  uint8_t origen[6];
  Trama_t trama;
} EventoRecepcion_t;

// Evento de confirmacion de envio
typedef struct {
  uint8_t resultado;
} EventoAck_t;

// ========================================
// 3. VARIABLES GENERALES
// ========================================

esp_now_peer_info_t peerInfo;

QueueHandle_t colaRecepcion = nullptr;
QueueHandle_t colaAck = nullptr;

bool baseLista = false;

uint32_t tramasRecibidas = 0;
uint32_t tramasValidas = 0;
uint32_t tramasInvalidas = 0;
uint32_t pongsSolicitados = 0;

// ========================================
// 4. CRC-16/CCITT-FALSE
// ========================================

uint16_t calcularCRC(const Trama_t &trama) {

  const uint8_t *datos =
    reinterpret_cast<const uint8_t *>(&trama);

  size_t longitud = offsetof(Trama_t, crc16);

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
// 5. VALIDAR ESTRUCTURA Y CRC
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

  Serial.println("--- CHECK DE TRAMA ---");

  Serial.printf(
    "Estructura      : %s\n",
    estructuraOK ? "OK" : "ERROR"
  );

  Serial.printf(
    "CRC-16          : %s\n",
    crcOK ? "OK" : "ERROR"
  );

  Serial.printf(
    "CRC recibido    : 0x%04X\n",
    trama.crc16
  );

  Serial.printf(
    "CRC calculado   : 0x%04X\n",
    crcCalculado
  );

  bool resultado = estructuraOK && crcOK;

  Serial.printf(
    "Resultado final : %s\n",
    resultado ? "TRAMA VALIDA" : "TRAMA INVALIDA"
  );

  return resultado;
}

// ========================================
// 6. MOSTRAR MAC REAL
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
  } else {
    Serial.printf(
      "ERROR lectura MAC: %s\n",
      esp_err_to_name(err)
    );
  }

  err = esp_wifi_get_mac(WIFI_IF_STA, mac);

  if (err != ESP_OK) {
    Serial.printf(
      "ERROR MAC activa: %s\n",
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
// 7. MOSTRAR TRAMA RECIBIDA
// ========================================

void imprimirTrama(const Trama_t &trama) {

  Serial.println();
  Serial.println("================================");
  Serial.println("    TELEMETRIA BASE TERRENA");
  Serial.println("================================");

  Serial.printf(
    "Sincronizacion  : 0x%04X\n",
    trama.sincronizacion
  );

  Serial.printf(
    "Version         : %u\n",
    trama.version
  );

  Serial.printf(
    "Longitud        : %u bytes\n",
    (unsigned)trama.longitud
  );

  if (trama.fechaValida == 1) {

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

  const char *categoria = "DESCONOCIDA";

  if (trama.tipoTelemetria == TELEMETRIA_CONTROL) {
    categoria = "CONTROL / ESTADO";
  } else if (
    trama.tipoTelemetria == TELEMETRIA_CARGA_UTIL
  ) {
    categoria = "CARGA UTIL";
  }

  Serial.printf(
    "Telemetria      : %s\n",
    categoria
  );

  const char *mensaje = "DESCONOCIDO";

  if (trama.tipoMensaje == MSG_PING) {
    mensaje = "PING";
  } else if (trama.tipoMensaje == MSG_PONG) {
    mensaje = "PONG";
  }

  Serial.printf(
    "Mensaje         : %s\n",
    mensaje
  );

  Serial.printf(
    "Contador        : %lu\n",
    (unsigned long)trama.contador
  );

  Serial.printf(
    "Tiempo envio    : %lu ms\n",
    (unsigned long)trama.t_envio_ms
  );

  Serial.println("--- DATOS RECIBIDOS ---");

  Serial.printf(
    "Uptime rover    : %lu ms\n",
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

  Serial.println("--- BYTES RECIBIDOS ---");

  const uint8_t *bytes =
    reinterpret_cast<const uint8_t *>(&trama);

  for (size_t i = 0; i < sizeof(Trama_t); i++) {
    Serial.printf("%02X ", bytes[i]);
  }

  Serial.println();
}

// ========================================
// 8. CALLBACK DE ENVIO
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

  EventoAck_t evento;

  evento.resultado =
    (status == ESP_NOW_SEND_SUCCESS) ? 1 : 0;

  if (colaAck != nullptr) {
    xQueueSend(colaAck, &evento, 0);
  }
}

// ========================================
// 9. CALLBACK DE RECEPCION
// ========================================

void OnDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {

  // Solo aceptar paquetes de 36 bytes
  if (len != sizeof(Trama_t)) {
    return;
  }

  if (info == nullptr ||
      info->src_addr == nullptr ||
      incomingData == nullptr) {
    return;
  }

  EventoRecepcion_t evento = {};

  // Guardar MAC del emisor
  memcpy(
    evento.origen,
    info->src_addr,
    6
  );

  // Guardar el paquete completo
  memcpy(
    &evento.trama,
    incomingData,
    sizeof(Trama_t)
  );

  // Enviar al bucle principal
  if (colaRecepcion != nullptr) {
    xQueueSend(colaRecepcion, &evento, 0);
  }
}

// ========================================
// 10. RESPONDER PONG
// ========================================

void responderPong(const Trama_t &ping) {

  // Copiar los datos originales
  Trama_t pong = ping;

  // Cambiar tipo de mensaje
  pong.tipoMensaje = MSG_PONG;

  // Conservar:
  // - Fecha y hora originales
  // - Identificador de mision
  // - Tipo de telemetria
  // - Contador
  // - Marca de tiempo del rover
  // - Datos de la trama

  // Recalcular CRC por el cambio PING -> PONG
  pong.crc16 = calcularCRC(pong);

  Serial.printf(
    "[TERRENA] Enviando PONG #%lu\n",
    (unsigned long)pong.contador
  );

  esp_err_t err = esp_now_send(
    macRover,
    (uint8_t *)&pong,
    sizeof(pong)
  );

  if (err == ESP_OK) {

    pongsSolicitados++;

    Serial.println(
      "[TERRENA] Solicitud PONG aceptada"
    );

  } else {

    Serial.printf(
      "[ERROR] Envio PONG: %s\n",
      esp_err_to_name(err)
    );
  }
}

// ========================================
// 11. ESTADISTICAS
// ========================================

void mostrarEstadisticas() {

  Serial.println();
  Serial.println("--- ESTADISTICAS ---");

  Serial.printf(
    "Tramas recibidas : %lu\n",
    (unsigned long)tramasRecibidas
  );

  Serial.printf(
    "Tramas validas   : %lu\n",
    (unsigned long)tramasValidas
  );

  Serial.printf(
    "Tramas invalidas : %lu\n",
    (unsigned long)tramasInvalidas
  );

  Serial.printf(
    "PONG solicitados : %lu\n",
    (unsigned long)pongsSolicitados
  );
}

// ========================================
// 12. SETUP
// ========================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================");
  Serial.println("        BASE TERRENA ESP32");
  Serial.println("================================");

  if (!WiFi.mode(WIFI_STA)) {
    Serial.println("ERROR: WiFi STA");
    return;
  }

  delay(100);

  if (!mostrarMAC()) {
    Serial.println("ERROR: MAC invalida");
    return;
  }

  // Configurar canal de comunicacion
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
    Serial.println("ERROR: Canal incorrecto");
    return;
  }

  Serial.printf(
    "Canal WiFi activo: %u\n",
    canalActivo
  );

  colaRecepcion = xQueueCreate(
    10,
    sizeof(EventoRecepcion_t)
  );

  colaAck = xQueueCreate(
    10,
    sizeof(EventoAck_t)
  );

  if (colaRecepcion == nullptr ||
      colaAck == nullptr) {
    Serial.println("ERROR: Crear colas");
    return;
  }

  // Inicializar ESP-NOW
  resultado = esp_now_init();

  if (resultado != ESP_OK) {
    Serial.printf(
      "ERROR ESP-NOW: %s\n",
      esp_err_to_name(resultado)
    );
    return;
  }

  // Registrar callbacks
  resultado = esp_now_register_send_cb(OnDataSent);

  if (resultado != ESP_OK) {
    Serial.println("ERROR callback envio");
    return;
  }

  resultado = esp_now_register_recv_cb(OnDataRecv);

  if (resultado != ESP_OK) {
    Serial.println("ERROR callback recepcion");
    return;
  }

  // Registrar rover
  memset(&peerInfo, 0, sizeof(peerInfo));

  memcpy(
    peerInfo.peer_addr,
    macRover,
    6
  );

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

  baseLista = true;

  Serial.println();
  Serial.println("ESP-NOW inicializado.");
  Serial.printf(
    "Longitud esperada: %u bytes\n",
    (unsigned)sizeof(Trama_t)
  );

  Serial.printf(
    "Mision configurada: %u\n",
    ID_MISION
  );

  Serial.println(
    "Esperando tramas del rover..."
  );
}

// ========================================
// 13. LOOP PRINCIPAL
// ========================================

void loop() {

  if (!baseLista) {
    delay(500);
    return;
  }

  EventoRecepcion_t evento;

  while (xQueueReceive(
           colaRecepcion,
           &evento,
           0
         ) == pdTRUE) {

    tramasRecibidas++;

    Serial.println();
    Serial.println("============================");

    Serial.printf(
      "[TERRENA] Trama recibida #%lu\n",
      (unsigned long)tramasRecibidas
    );

    Serial.printf(
      "MAC origen: %02X:%02X:%02X:%02X:%02X:%02X\n",
      evento.origen[0],
      evento.origen[1],
      evento.origen[2],
      evento.origen[3],
      evento.origen[4],
      evento.origen[5]
    );

    // Comprobar que proviene del rover
    if (memcmp(
          evento.origen,
          macRover,
          6
        ) != 0) {

      Serial.println(
        "[ERROR] MAC no autorizada"
      );

      tramasInvalidas++;
      continue;
    }

    // Mostrar contenido completo
    imprimirTrama(evento.trama);

    // Comprobar estructura y CRC
    if (!verificarTrama(evento.trama)) {

      tramasInvalidas++;

      Serial.println(
        "[ERROR] Trama descartada"
      );

      continue;
    }

    // La trama supero el check
    tramasValidas++;

    // Solo responder mensajes PING
    if (evento.trama.tipoMensaje == MSG_PING) {

      Serial.println(
        "[TERRENA] PING valido recibido"
      );

      responderPong(evento.trama);

    } else {

      Serial.println(
        "[TERRENA] Trama valida, no es PING"
      );
    }

    mostrarEstadisticas();
  }

  // Procesar confirmaciones
  EventoAck_t ack;

  while (xQueueReceive(
           colaAck,
           &ack,
           0
         ) == pdTRUE) {

    Serial.println(
      ack.resultado == 1
        ? "[TERRENA] ACK PONG recibido"
        : "[TERRENA] ACK PONG fallido"
    );
  }

  delay(10);
}
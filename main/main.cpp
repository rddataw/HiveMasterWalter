#include <cstdio>
#include <stdint.h>
#include <time.h>
// #include <secret.h>

#include "esp_sleep.h"

/**
 *
 *
 *
 * sudo fuser -k 6666/tcp
 *
 * ss -ltnp 'sport = :6666'
 *
 *
 *
kill -TERM 15782 && ss -ltnp 'sport = :6666'

ss -ltnp 'sport = :6666'; ps -p 15782 -o pid=,ppid=,comm=,args=

State  Recv-Q Send-Q Local Address:Port Peer Address:PortProcess
LISTEN 0      1          127.0.0.1:6666      0.0.0.0:*    users:(("openocd",pid=76572,fd=3))

kill -TERM 76572 && sleep 1; ss -ltnp 'sport = :6666'

 */

#include <WalterModem.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_mac.h>
// #include "secret.h"

#define MQTT_PORT 1883
#define MQTT_HOST "mqtt.flespi.io"
#define MQTT_TOPIC "bz/test" //!!!! Upravit
#define MQTT_CLIENT_ID "hivemaster"

// const char *FLESPI_TOKEN = "1cSeh0jEzfki6ji4glpKzUqz0NaRzxbCeWJCkZyU1gS1vf8JqGvaKeBtC3xJV6cJ";
#define FLESPI_TOKEN "1cSeh0jEzfki6ji4glpKzUqz0NaRzxbCeWJCkZyU1gS1vf8JqGvaKeBtC3xJV6cJ"

#define UART_PORT_NUM UART_NUM_2 // Samostatný UART pro komunikaci se STM32
#define UART_BUFFER_SIZE 2048    // Vaše velikost bufferu
#define UART_RX_TIMEOUT_SYMBOLS 20
#define HIVE_EPOCH_UNIX_SECONDS 1767225600LL // 1.1.2026 00:00:00 UTC v unixtime
#define WAKEUP_GPIO GPIO_NUM_17
#define LTE_PSM_REQ_TAU "01000001"    // Request periodic TAU: 10 hours
#define LTE_PSM_REQ_ACTIVE "00000001" // Request active time: 2 seconds
#define RX_ESP 9
#define TX_ESP 10
#define RTS_ESP 8
#define CTS_ESP 18

// size_t uartPos = 0;
uint8_t uartBuffer[UART_BUFFER_SIZE];
size_t bufferDataLength;
QueueHandle_t uart2EventQueue;
uint32_t hiveTime = 0;

/**
 * @brief The approximate size of a published message in bytes.
 */
#define MQTT_MESSAGE_SIZE 2000

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Flag indicating whether to publish a message.
 */
volatile bool mqtt_connected = false;

/**
 * @brief The buffer to receive from the MQTT server.
 * @note Make sure this is sufficiently large enough for incoming data. (Up to 4096 bytes supported
 * by Sequans)
 */
uint8_t in_buf[4096] = {0};

/**
 * @brief ESP-IDF log prefix.
 */
static constexpr const char *TAG = "[HIVE]";

// -----------------------------------------------------------------------------
// Function: lteConnected
// -----------------------------------------------------------------------------
/**
 * @brief This function checks if we are connected to the LTE network
 *
 * @return true when connected, false otherwise
 */
bool lteConnected()
{
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

// -----------------------------------------------------------------------------
// Function: waitForNetwork
// -----------------------------------------------------------------------------
/**
 * @brief This function waits for the modem to be connected to the LTE network.
 *
 * @param timeout_sec The amount of seconds to wait before returning a time-out.
 *
 * @return true if connected, false on time-out.
 */
bool waitForNetwork(int timeout_sec = 300)
{
  ESP_LOGI(TAG, "Connecting to the network...");
  int time = 0;
  while (!lteConnected())
  {
    printf(".");
    vTaskDelay(pdMS_TO_TICKS(1000));
    time++;
    if (time > timeout_sec)
      return false;
  }
  printf("\r\n");
  ESP_LOGI(TAG, "Connected to the network");
  return true;
}

// -----------------------------------------------------------------------------
// Function: lteDisconnect
// -----------------------------------------------------------------------------
/**
 * @brief Disconnect from the LTE network.
 *
 * This function will disconnect the modem from the LTE network and block until
 * the network is actually disconnected. After the network is disconnected the
 * GNSS subsystem can be used.
 *
 * @return true on success, false on error.
 */
bool lteDisconnect()
{
  /* Set the operational state to minimum */
  if (modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM))
  {
    ESP_LOGI(TAG, "Successfully set operational state to MINIMUM");
  }
  else
  {
    ESP_LOGE(TAG, "Could not set operational state to MINIMUM");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while (regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    regState = modem.getNetworkRegState();
  }

  ESP_LOGI(TAG, "Disconnected from the network");
  return true;
}

// -----------------------------------------------------------------------------
// Function: lteConnect
// -----------------------------------------------------------------------------
/**
 * @brief This function tries to connect the modem to the cellular network.
 *
 * @return true on success, false on error.
 */
bool lteConnect()
{
  /* Set the operational state to NO RF */
  if (modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF))
  {
    ESP_LOGI(TAG, "Successfully set operational state to NO RF");
  }
  else
  {
    ESP_LOGE(TAG, "Could not set operational state to NO RF");
    return false;
  }

  /* Create PDP context */
  if (modem.definePDPContext())
  {
    ESP_LOGI(TAG, "Created PDP context");
  }
  else
  {
    ESP_LOGE(TAG, "Could not create PDP context");
    return false;
  }

  /* Set the operational state to full */
  if (modem.setOpState(WALTER_MODEM_OPSTATE_FULL))
  {
    ESP_LOGI(TAG, "Successfully set operational state to FULL");
  }
  else
  {
    ESP_LOGE(TAG, "Could not set operational state to FULL");
    return false;
  }

  /* Set the network operator selection to automatic */
  if (modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC))
  {
    ESP_LOGI(TAG, "Network selection mode was set to automatic");
  }
  else
  {
    ESP_LOGE(TAG, "Could not set the network selection mode to automatic");
    return false;
  }

  return waitForNetwork();
}

// -----------------------------------------------------------------------------
// Function: myNetworkEventHandler
// -----------------------------------------------------------------------------
/**
 * @brief The network registration event handler.
 *
 * This function will be called when network registration state changes or when
 * eDRX parameters are received from the network.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking
 * the event processing task.
 *
 * @param[out] event The network registration state event.
 * @param[out] data The registration event data including state and PSM info.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myNetworkEventHandler(WMNetworkEventType event, const WMNetworkEventData *data,
                                  void *args)
{
  if (event == WALTER_MODEM_NETWORK_EVENT_REG_STATE_CHANGE)
  {
    switch (data->cereg.state)
    {
    case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
      ESP_LOGI(TAG, "Network registration: Registered (home)");
      break;

    case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
      ESP_LOGI(TAG, "Network registration: Registered (roaming)");
      break;

    case WALTER_MODEM_NETWORK_REG_NOT_SEARCHING:
      ESP_LOGI(TAG, "Network registration: Not searching");
      break;

    case WALTER_MODEM_NETWORK_REG_SEARCHING:
      ESP_LOGI(TAG, "Network registration: Searching");
      break;

    case WALTER_MODEM_NETWORK_REG_DENIED:
      ESP_LOGI(TAG, "Network registration: Denied");
      break;

    case WALTER_MODEM_NETWORK_REG_UNKNOWN:
      ESP_LOGI(TAG, "Network registration: Unknown");
      break;

    default:
      break;
    }
  }
}

// -----------------------------------------------------------------------------
// Function: myMQTTEventHandler
// -----------------------------------------------------------------------------
/**
 * @brief The MQTT event handler.
 *
 * This function will be called on various MQTT events such as connection, disconnection,
 * subscription, publication and incoming messages. You can modify this handler to implement your
 * own logic based on the events received.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] event The type of MQTT event.
 * @param[out] data The data associated with the event.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myMQTTEventHandler(WMMQTTEventType event, const WMMQTTEventData *data, void *args)
{
  switch (event)
  {
  case WALTER_MODEM_MQTT_EVENT_CONNECTED:
    if (data->rc != 0)
    {
      mqtt_connected = false;
      ESP_LOGI(TAG, "MQTT: Connection could not be established. (code: %d)", data->rc);
    }
    else
    {
      mqtt_connected = true;
      ESP_LOGI(TAG, "MQTT: Connected successfully");

      /* Subscribe to the test topic
      if(modem.mqttSubscribe(MQTT_TOPIC)) {
        ESP_LOGI(TAG, "Subscribing to '%s'...", MQTT_TOPIC);
      } else {
        ESP_LOGI(TAG, "Subscribing failed");
      }
        */
    }
    break;

  case WALTER_MODEM_MQTT_EVENT_DISCONNECTED:
    if (data->rc != 0)
    {
      ESP_LOGI(TAG, "MQTT: Connection was interrupted (code: %d)", data->rc);
    }
    else
    {
      ESP_LOGI(TAG, "MQTT: Disconnected");
    }
    mqtt_connected = false;
    break;

  case WALTER_MODEM_MQTT_EVENT_SUBSCRIBED:
    if (data->rc != 0)
    {
      ESP_LOGI(TAG, "MQTT: Could not subscribe to topic. (code: %d)", data->rc);
    }
    else
    {
      ESP_LOGI(TAG, "MQTT: Successfully subscribed to topic '%s'", data->topic);
      mqtt_connected = true;
    }
    break;

  case WALTER_MODEM_MQTT_EVENT_PUBLISHED:
    if (data->rc != 0)
    {
      ESP_LOGI(TAG, "MQTT: Could not publish message (id: %d) to topic. (code: %d)", data->mid,
               data->rc);
    }
    else
    {
      ESP_LOGI(TAG, "MQTT: Successfully published message (id: %d)", data->mid);
    }
    break;

  case WALTER_MODEM_MQTT_EVENT_MESSAGE:
    ESP_LOGI(TAG, "MQTT: Message (id: %d) received on topic '%s' (size: %u bytes)", data->mid,
             data->topic, data->msg_length);

    /* Keep room for the terminator, the message is printed as a string */
    if (data->msg_length >= sizeof(in_buf))
    {
      ESP_LOGI(TAG, "Could not receive MQTT message (%u bytes do not fit)", data->msg_length);
      break;
    }

    /* Receive the MQTT message from the modem buffer, exactly the size reported by the ring */
    memset(in_buf, 0, sizeof(in_buf));
    if (modem.mqttReceive(data->topic, data->mid, in_buf, data->msg_length))
    {
      ESP_LOGI(TAG, "Received message: %s", in_buf);
    }
    else
    {
      ESP_LOGI(TAG, "Could not receive MQTT message");
    }
    break;

  case WALTER_MODEM_MQTT_EVENT_MEMORY_FULL:
    ESP_LOGI(TAG, "MQTT: Memory full");
    break;
  }
}

// -----------------------------------------------------------------------------
// Function: uartStmConfig - uart pro komunikaci se STM32
// -----------------------------------------------------------------------------
void uartStmConfig()
{
  // 1. Konfigurace parametrů UART s hardwarovým řízením toku
  uart_config_t uart_config{};
  uart_config.baud_rate = 115200;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
  uart_config.rx_flow_ctrl_thresh = 120;
  uart_config.source_clk = UART_SCLK_DEFAULT;

  uart_param_config(UART_PORT_NUM, &uart_config);

  // 2. Nastavení TX, RX, RTS a CTS pinů
  uart_set_pin(UART_PORT_NUM, TX_ESP, RX_ESP, RTS_ESP, CTS_ESP);

  // 3. Instalace ovladače UART s frontou událostí
  ESP_ERROR_CHECK(uart_driver_install(
      UART_PORT_NUM,
      UART_BUFFER_SIZE,
      UART_BUFFER_SIZE,
      20,
      &uart2EventQueue,
      0));
  ESP_ERROR_CHECK(uart_set_rx_timeout(UART_PORT_NUM, UART_RX_TIMEOUT_SYMBOLS));
  uart_flush_input(UART_PORT_NUM);
  xQueueReset(uart2EventQueue);
  ESP_LOGI(TAG, "UART pro STM32 inicializovan a vycisten");
}

// -----------------------------------------------------------------------------
// Function: mqttPublishBinarni
// -----------------------------------------------------------------------------
bool mqttPublishBinarni(const char *topic, const uint8_t *payload, size_t payloadLen)
{
  ESP_LOGI(TAG, "Delka zpravy: %u", (unsigned)payloadLen);
  ESP_LOGI(TAG, "Topic: %s", topic);

  if (!modem.mqttPublish(topic, (uint8_t *)payload, payloadLen, 1))
  {
    ESP_LOGE(TAG, "MQTT publish failed");
    mqtt_connected = false;
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
// Function: crc16_ccitt (CRC-16/CCITT)
// -----------------------------------------------------------------------------
uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;

    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }

  return crc;
}

// -----------------------------------------------------------------------------
// Function: Posli cas z modemu do STM32
// -----------------------------------------------------------------------------
bool sendTime()
{
  WalterModemRsp response{};
  if (!modem.getClock(&response) ||
      response.type != WALTER_MODEM_RSP_DATA_TYPE_CLOCK)
  {
    ESP_LOGE(TAG, "Could not read LTE modem clock");
    return false;
  }

  const int64_t unixTime = response.data.clock.epochTime;
  if (unixTime < HIVE_EPOCH_UNIX_SECONDS)
  {
    ESP_LOGE(TAG, "Walter modem returned an invalid or pre-2026 time");
    return false;
  }

  const time_t lteTime = static_cast<time_t>(unixTime);
  struct tm utcTime{};
  if (gmtime_r(&lteTime, &utcTime) == nullptr)
  {
    ESP_LOGE(TAG, "Could not convert LTE time to UTC date and time");
    return false;
  }
  ESP_LOGI(TAG, "LTE time (UTC): %04d-%02d-%02d %02d:%02d:%02d",
           utcTime.tm_year + 1900,
           utcTime.tm_mon + 1,
           utcTime.tm_mday,
           utcTime.tm_hour,
           utcTime.tm_min,
           utcTime.tm_sec);

  const uint64_t secondsSinceHiveEpoch =
      static_cast<uint64_t>(unixTime - HIVE_EPOCH_UNIX_SECONDS);
  if (secondsSinceHiveEpoch > UINT32_MAX)
  {
    ESP_LOGE(TAG, "Time since 2026 does not fit in uint32_t");
    return false;
  }

  hiveTime = static_cast<uint32_t>(secondsSinceHiveEpoch);

  uint8_t message[7];
  message[0] = 10;
  message[1] = static_cast<uint8_t>(hiveTime);
  message[2] = static_cast<uint8_t>(hiveTime >> 8);
  message[3] = static_cast<uint8_t>(hiveTime >> 16);
  message[4] = static_cast<uint8_t>(hiveTime >> 24);

  const uint16_t crc = crc16_ccitt(message, 5);
  message[5] = static_cast<uint8_t>(crc >> 8);
  message[6] = static_cast<uint8_t>(crc);

  const int bytesWritten = uart_write_bytes(UART_PORT_NUM, message, sizeof(message));
  if (bytesWritten != sizeof(message) ||
      uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(10000)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Could not send time frame over UART2");
    return false;
  }

  ESP_LOGI(TAG, "Sent hive time: %u", (unsigned)hiveTime);
  return true;
}

// -----------------------------------------------------------------------------
// Function: gotoSleep
// -----------------------------------------------------------------------------
bool gotoSleep()
{
  gpio_config_t wakePinConfig{};
  wakePinConfig.pin_bit_mask = 1ULL << WAKEUP_GPIO;
  wakePinConfig.mode = GPIO_MODE_INPUT;
  wakePinConfig.pull_up_en = GPIO_PULLUP_DISABLE;
  wakePinConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
  wakePinConfig.intr_type = GPIO_INTR_DISABLE;

  if (gpio_config(&wakePinConfig) != ESP_OK)
  {
    ESP_LOGE(TAG, "Could not configure GPIO17 as wake input");
    return false;
  }

  if (gpio_get_level(WAKEUP_GPIO) == 1)
  {
    ESP_LOGE(TAG, "GPIO17 is already high; refusing to enter deep sleep");
    return false;
  }

  if (esp_sleep_enable_ext1_wakeup_io(
          1ULL << WAKEUP_GPIO,
          ESP_EXT1_WAKEUP_ANY_HIGH) != ESP_OK)
  {
    ESP_LOGE(TAG, "Could not configure GPIO17 as deep-sleep wake source");
    return false;
  }

  if (!modem.configPSM(
          WALTER_MODEM_PSM_ENABLE,
          LTE_PSM_REQ_TAU,
          LTE_PSM_REQ_ACTIVE))
  {
    ESP_LOGE(TAG, "Could not enable LTE PSM");
    return false;
  }

  ESP_LOGI(TAG, "Entering deep sleep; GPIO17 high will wake the ESP32");
  esp_deep_sleep_start();
  return true;
}

// -----------------------------------------------------------------------------
// Function: zpracujBuffer
// Protokol: [type:1B][sn:1B][len:1B][data:len B] ...
//   - hlavicka = 3 B (type, sn, len)
//   - len       = pocet datovych bajtu ZA hlavickou (NE celkova delka structu!)
//   - celkova delka zaznamu = 3 + len
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Function: nactiBuffer (Upraveno pro čtení dávky více paketů naráz)
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// 1. Funkce: nactiBuffer
//    Načte z UARTu hlavičku (3B) + data podle délky z 3. bajtu.
//    Podporuje načtení více paketů najednou, pokud jsou v bufferu připraveny.
// -----------------------------------------------------------------------------
bool nactiBuffer()
{
  size_t totalReceived = 0;

  // A) Čekáme na první paket (alespoň 3B hlavička)
  int bytesRead = uart_read_bytes(UART_PORT_NUM, &uartBuffer[totalReceived], 3, pdMS_TO_TICKS(2000));
  if (bytesRead != 3)
  {
    ESP_LOGW(TAG, "UART timeout: přijato pouze %d bajtů ze 3 (STM32 neodpovídá)", bytesRead);
    return false;
  }
  totalReceived += 3;

  // 3. bajt hlavičky určuje délku dat (len)
  uint8_t len = uartBuffer[totalReceived - 1];

  // Dočteme data prvního paketu
  if (len > 0)
  {
    bytesRead = uart_read_bytes(UART_PORT_NUM, &uartBuffer[totalReceived], len, pdMS_TO_TICKS(500));
    if (bytesRead != len)
    {
      ESP_LOGE(TAG, "Neúplná data prvního paketu: očekáváno %u B, přijato %d B", len, bytesRead);
      uart_flush_input(UART_PORT_NUM);
      return false;
    }
    totalReceived += len;
  }

  // B) Cyklus: Zkusíme načíst i případné DALŠÍ pakety, které už čekají v UART bufferu
  while (true)
  {
    size_t available = 0;
    uart_get_buffered_data_len(UART_PORT_NUM, &available);

    if (available < 3)
    {
      break;
    }

    uint8_t nextHeader[3];
    if (uart_read_bytes(UART_PORT_NUM, nextHeader, 3, 0) != 3)
    {
      break;
    }

    uint8_t nextLen = nextHeader[2];
    size_t nextPacketTotal = 3 + nextLen;

    if (totalReceived + nextPacketTotal > sizeof(uartBuffer))
    {
      ESP_LOGW(TAG, "Buffer je plný, další paket se již nevejde");
      break;
    }

    uart_get_buffered_data_len(UART_PORT_NUM, &available);
    if (available < nextLen)
    {
      break;
    }

    uartBuffer[totalReceived++] = nextHeader[0];
    uartBuffer[totalReceived++] = nextHeader[1];
    uartBuffer[totalReceived++] = nextHeader[2];

    if (nextLen > 0)
    {
      uart_read_bytes(UART_PORT_NUM, &uartBuffer[totalReceived], nextLen, 0);
      totalReceived += nextLen;
    }
  }

  bufferDataLength = totalReceived;
  ESP_LOGI(TAG, "Úspěšně načteno celkem %u bajtů dat", (unsigned)bufferDataLength);
  return true;
}

// -----------------------------------------------------------------------------
// 2. Funkce: zpracujBuffer
// -----------------------------------------------------------------------------
bool zpracujBuffer()
{
  size_t pos = 0;

  while (pos < bufferDataLength)
  {
    if (bufferDataLength - pos < 3)
    {
      ESP_LOGW(TAG, "Zbytkové bajty na konci bufferu (%u B), ukončuji zpracování",
               (unsigned)(bufferDataLength - pos));
      break;
    }

    uint8_t type = uartBuffer[pos++];
    uint8_t sn = uartBuffer[pos++];
    size_t payloadPos = pos;
    uint8_t len = uartBuffer[pos++];

    if (bufferDataLength - pos < (size_t)len)
    {
      ESP_LOGE(TAG, "Neplatná délka záznamu: hlavička+data = 3+%u = %u B, "
                    "ale v bufferu zbývají jen %u B",
               (unsigned)len, 3u + (unsigned)len,
               (unsigned)(bufferDataLength - pos));
      return false;
    }

    ESP_LOGI(TAG, "Record: type=%u sn=%u len=%u (celkem %u B)",
             type, sn, len, 3u + len);

    char topic[32];
    snprintf(topic, sizeof(topic), "bz/%u/%u", type, sn);

    if (!mqttPublishBinarni(topic, &uartBuffer[payloadPos], (size_t)len + 1))
    {
      ESP_LOGE(TAG, "MQTT publish failed pro topic %s", topic);
    }

    pos += len;
  }

  return true;
}

// -----------------------------------------------------------------------------
// Zapnuti modemu
// -----------------------------------------------------------------------------
bool modemStart()
{
  /* Start modem */
  if (modem.begin(UART_NUM_1))
  {
    ESP_LOGI(TAG, "Successfully initialized the modem");
  }
  else
  {
    ESP_LOGE(TAG, "Could not initialize the modem");
    return false;
  }

  if (!lteConnected())
  {
    if (!lteConnect())
    {
      ESP_LOGE(TAG, "Failed to connect to network");
      vTaskDelay(pdMS_TO_TICKS(1000));
      return false;
    }
    mqtt_connected = false;
  }

  /* Set the network registration event handler (optional) */
  modem.setNetworkEventHandler(myNetworkEventHandler, NULL);

  /* Set the MQTT event handler */
  modem.setMQTTEventHandler(myMQTTEventHandler, NULL);

  if (!modem.configPSM(WALTER_MODEM_PSM_DISABLE))
  {
    ESP_LOGE(TAG, "Nemohu prepnout modem do full rezimu!");
    return false;
  }

  // delay(1000); // Wait for PSM to be disabled

  if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL))
  {
    ESP_LOGE(TAG, "Nemohu prepnout modem do full rezimu!");
    return false;
  }

  modem.setRAT(WALTER_MODEM_RAT_LTEM);

  WalterModemRsp rsp;
  if (modem.getRAT(&rsp))
  {
    if (rsp.type == WALTER_MODEM_RSP_DATA_TYPE_RAT)
    {
      switch (rsp.data.rat)
      {
      case WALTER_MODEM_RAT_LTEM:
        ESP_LOGI(TAG, "Pripojeno k Cat-M (LTE-M)");
        break;
      case WALTER_MODEM_RAT_NBIOT:
        ESP_LOGI(TAG, "Pripojeno k NB-IoT");
        break;
      case WALTER_MODEM_RAT_AUTO:
        ESP_LOGI(TAG, "Automaticky vyber RAT");
        break;
      default:
        ESP_LOGI(TAG, "Neznamy RAT: %d", rsp.data.rat);
        break;
      }
    }
  }
  else
  {
    ESP_LOGE(TAG, "Nepodarilo získat RAT");
  }

  return true;
}

bool modemMqttStart()
{
  /* Configure the MQTT client */

  if (modem.mqttConfig(MQTT_CLIENT_ID, FLESPI_TOKEN, "", 0))
  {
    ESP_LOGI(TAG, "Successfully configured the MQTT client");
  }
  else
  {
    ESP_LOGE(TAG, "Error: Failed to configure MQTT client");
    return false;
  }

  //*********** */

  /* Connect to a public MQTT broker */
  if (!mqtt_connected)
  {
    if (modem.mqttConnect(MQTT_HOST, MQTT_PORT, 60))
    {
      ESP_LOGI(TAG, "Connecting to MQTT broker...");
      // Wait for connection
      int attempts = 0;
      while (!mqtt_connected && attempts < 20)
      {
        vTaskDelay(pdMS_TO_TICKS(1000));
        attempts++;
      }
      if (!mqtt_connected)
      {
        ESP_LOGE(TAG, "MQTT connection failed or timeout");
      }
    }
    else
    {
      ESP_LOGE(TAG, "Error: Failed to initiate MQTT connection");
      return false;
    }
  }
  return true;
}

/*
 */
void check_wakeup_and_proceed(void)
{
  uint32_t wakeup_causes = esp_sleep_get_wakeup_causes();

  // Zkontrolujeme, zda probuzení nastalo a zda mezi důvody figuruje EXT1
  // (případně můžete omezit na konkrétní pin maskou, pokud sledujete víc pinů)
  bool woke_by_ext1 = (wakeup_causes != 0) && (wakeup_causes & (1 << ESP_SLEEP_WAKEUP_EXT1));

  if (woke_by_ext1)
  {
    ESP_LOGI(TAG, " EXT1, pokracujeme.");
    return; // nebo return, pokud chcete ukončit funkci
  }
  else
  {
    ESP_LOGI(TAG, "Ostatni probuzeni. Jdeme spat.");
    return; // nebo return, pokud chcete ukončit funkci
    // gotoSleep();
  }
}

// #############################################################################
// Function: app_main
// #############################################################################
/**
 * @brief The main application start method.
 */
extern "C" void app_main()
{

  ESP_LOGI(TAG, "\r\n\r\n=== HiveMqttWalter ===\r\n\r\n");

  // 1. Konfigurace pinu jako vstup a zapnuti pull-up rezistoru pro CTS
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << GPIO_NUM_17),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  if (!modemStart())
  {
    ESP_LOGE(TAG, "Modem start failed");
  }
  else
  {
    ESP_LOGI(TAG, "Modem start OK");
  }

  if (!modemMqttStart())
  {
    ESP_LOGE(TAG, "Modem MQTT start failed");
  }
  else
  {
    ESP_LOGI(TAG, "Modem MQTT start OK");
  }

  // Aktivace UART až po úplném náběhu modemu a MQTT
  uartStmConfig();

  while (true)
  {

    // vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for the modem to be ready

    // check_wakeup_and_proceed();

    ESP_LOGI("GPIO", "Cekam na GPIO17 = 1...");

    while (gpio_get_level(GPIO_NUM_17) == 0)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI("GPIO", "GPIO17 je nyní v logické 1!");

    // Vyčistit UART od smetí / předchozích dat před odesláním času a čtením
    uart_flush_input(UART_PORT_NUM);
    xQueueReset(uart2EventQueue);

    if (!sendTime())
    {
      ESP_LOGE(TAG, "Time frame was not sent");
    }

    ESP_LOGI(TAG, "Waiting for UART response...");
    if (nactiBuffer() == true)
    {
      ESP_LOGI(TAG, "Buffer nacten, delka: %u", (unsigned)bufferDataLength - 3);
      if (zpracujBuffer() == true)
      {
        ESP_LOGI(TAG, "Buffer zpracovan");
      }
      else
      {
        ESP_LOGE(TAG, "Buffer se nepodarilo zpracovat");
      }
    }
    else
    {
      ESP_LOGE(TAG, "Buffer ma malo dat, delka: %u", (unsigned)bufferDataLength);
    }
  }

  gotoSleep();
}

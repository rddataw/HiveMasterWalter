#include <cstdio>
#include <stdint.h>
#include <time.h>
// #include <secret.h>

#include "esp_sleep.h"

/**
 *# Odeslani git na lokalni server
 * git push local main
 * git push github main
 * git remote -v
 *
 * github  https://github.com/rddataw/HiveMasterWalter.git (fetch)
github  https://github.com/rddataw/HiveMasterWalter.git (push)
local   http://192.168.1.10:3003/rad/HiveMasterWalter.git (fetch)
local   http://192.168.1.10:3003/rad/HiveMasterWalter.git (push)
origin  https://github.com/rddataw/HiveMasterWalter.git (fetch)
origin  https://github.com/rddataw/HiveMasterWalter.git (push)
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

#define TAG "UART_PARSER"

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

size_t bufferDataLength;

uint32_t hiveTime = 0;

static QueueHandle_t uart2EventQueue = NULL;

TimerHandle_t TimeoutTimer = NULL;

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

  const uint64_t secondsSinceHiveEpoch =
      static_cast<uint64_t>(unixTime - HIVE_EPOCH_UNIX_SECONDS);
  if (secondsSinceHiveEpoch > UINT32_MAX)
  {
    ESP_LOGE(TAG, "Time since 2026 does not fit in uint32_t");
    return false;
  }

  hiveTime = static_cast<uint32_t>(secondsSinceHiveEpoch);

  // Sestavení zprávy: [type(1B)][time(4B)][CRC(1B)] = celkem 6 bajtů
  uint8_t message[6];
  message[0] = 10;
  message[1] = static_cast<uint8_t>(hiveTime);
  message[2] = static_cast<uint8_t>(hiveTime >> 8);
  message[3] = static_cast<uint8_t>(hiveTime >> 16);
  message[4] = static_cast<uint8_t>(hiveTime >> 24);

  // Výpočet 1bajtového XOR checksumu přes prvních 5 bajtů
  uint8_t checksum = 0;
  for (int i = 0; i < 5; ++i)
  {
    checksum ^= message[i];
  }
  message[5] = checksum;

  const int bytesWritten = uart_write_bytes(UART_PORT_NUM, message, sizeof(message));
  if (bytesWritten != sizeof(message) ||
      uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(10000)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Could not send time frame over UART2");
    return false;
  }

  ESP_LOGI(TAG, "Sent hive time: %u with CRC: 0x%02X", (unsigned)hiveTime, checksum);
  return true;
}

// -----------------------------------------------------------------------------
// Function: gotoSleep
// -----------------------------------------------------------------------------
bool gotoSleep()
{
  if (TimeoutTimer != NULL)
  {
    xTimerStop(TimeoutTimer, 0);
    xTimerDelete(TimeoutTimer, 0);
    TimeoutTimer = NULL; // Vynulujeme ukazatel
  }

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

// Callback hlidani casu behu programu
void vTimeoutTimerCallback(TimerHandle_t pxTimer)
{
  ESP_LOGE("TAG", "Timeout behu. Jdeme spat.");
  gotoSleep();
}

//---------------------------------------------------------
// 1. Inicializace UARTu s hardwarovým řízením toku (CTS/RTS)
//---------------------------------------------------------
void uartStmConfig(void)
{

  // 1. Konfigurace CTS UART 2 jako vstup a zapnuti pull-up
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << GPIO_NUM_18),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  // Konfigurace parametrů UART s hardwarovým řízením toku
  uart_config_t uart_config{};
  uart_config.baud_rate = 115200;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
  uart_config.rx_flow_ctrl_thresh = 120;
  uart_config.source_clk = UART_SCLK_DEFAULT;

  uart_param_config(UART_PORT_NUM, &uart_config);

  // Nastavení TX, RX, RTS a CTS pinů
  uart_set_pin(UART_PORT_NUM, TX_ESP, RX_ESP, RTS_ESP, CTS_ESP);

  // Instalace ovladače UART s frontou událostí
  ESP_ERROR_CHECK(uart_driver_install(
      UART_PORT_NUM,
      UART_BUFFER_SIZE,
      UART_BUFFER_SIZE,
      20,
      &uart2EventQueue,
      0));

  ESP_ERROR_CHECK(uart_set_rx_timeout(UART_PORT_NUM, UART_RX_TIMEOUT_SYMBOLS));
  uart_flush_input(UART_PORT_NUM);

  if (uart2EventQueue != NULL)
  {
    xQueueReset(uart2EventQueue);
  }

  ESP_LOGI(TAG, "UART pro STM32 inicializovan a vycisten");
}

// -----------------------------------------------------------------------------
// Poslani dat z uart na mqtt
// -----------------------------------------------------------------------------
bool uartProcessAndPublishMessages(void)
{
  uint8_t uartBuffer[UART_BUFFER_SIZE];

  // Čekáme na data z UARTu (blokující čtení s timeoutem např. 10000 ms)
  int bytesRead = uart_read_bytes(UART_PORT_NUM, uartBuffer, sizeof(uartBuffer), pdMS_TO_TICKS(10000));

  if (bytesRead <= 0)
  {
    // Timeout – žádná data nepřišla
    return false;
  }

  size_t bufferLength = (size_t)bytesRead;
  ESP_LOGI(TAG, "UART: Přijat celkový blok o velikosti %u B", bufferLength);

  size_t pos = 0;
  bool successCount = false;

  // Parsování celého bloku zpráva po zprávě
  while (pos < bufferLength)
  {
    // A. Kontrola startovacího bajtu 0xAA
    if (uartBuffer[pos] != 0xAA)
    {
      ESP_LOGW(TAG, "Očekáván start 0xAA na pozici %u, nalezeno 0x%02X. Resynchronizace (+1B)",
               (unsigned)pos, uartBuffer[pos]);
      pos++; // Posun o 1 bajt pro nalezení správného začátku v případě rušení
      continue;
    }

    // B. Ověření, zda máme v bufferu dost místa alespoň na hlavičku (1B start + 3B hlavička = 4B)
    if (bufferLength - pos < 4)
    {
      ESP_LOGW(TAG, "Neúplná hlavička na konci bloku. Ukončuji parsování.");
      break;
    }

    uint8_t type = uartBuffer[pos + 1];
    uint8_t sn = uartBuffer[pos + 2];
    uint8_t len = uartBuffer[pos + 3];

    // C. Celková délka této dílčí zprávy: 1 (0xAA) + 3 (hlavička) + len (payload) + 1 (XOR CRC)
    size_t expectedMsgLen = 1 + 3 + (size_t)len + 1;

    // D. Kontrola, zda se celá zpráva vejde do zbývajícího bufferu
    if (bufferLength - pos < expectedMsgLen)
    {
      ESP_LOGE(TAG, "Zpráva (type=%u, sn=%u) vyžaduje %u B, ale v bufferu zbývá jen %u B.",
               type, sn, (unsigned)expectedMsgLen, (unsigned)(bufferLength - pos));
      break;
    }

    // E. Výpočet a kontrola XOR CRC pro tuto konkrétní zprávu (od 0xAA až před CRC)
    size_t crcIndex = pos + expectedMsgLen - 1;
    uint8_t receivedChecksum = uartBuffer[crcIndex];
    uint8_t calculatedChecksum = 0;

    for (size_t i = pos; i < crcIndex; ++i)
    {
      calculatedChecksum ^= uartBuffer[i];
    }

    if (calculatedChecksum != receivedChecksum)
    {
      ESP_LOGE(TAG, "Chyba CRC (type=%u, sn=%u)! Vypočteno: 0x%02X, Přijato: 0x%02X -> Zahazuji tuto zprávu",
               type, sn, calculatedChecksum, receivedChecksum);

      // Při chybě posuneme pozici o 1 bajt, abychom zkusili najít další platný paket
      pos++;
      continue;
    }

    // F. Zpráva je v pořádku -> připravíme payload a odešleme do MQTT
    size_t payloadPos = pos + 4; // Data začínají hned za type, sn, len

    ESP_LOGI(TAG, "Sub-zpráva OK: type=%u sn=%u len=%u (payload %u B)",
             type, sn, len, (unsigned)len);

    char topic[32];
    snprintf(topic, sizeof(topic), "bz/%u/%u", type, sn);

    if (mqttPublishBinarni(topic, &uartBuffer[payloadPos], (size_t)len))
    {
      successCount = true;
    }
    else
    {
      ESP_LOGE(TAG, "MQTT publish failed pro topic %s", topic);
    }

    // G. Posun na začátek další zprávy v bloku
    pos += expectedMsgLen;
  }

  return successCount;
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

  // Konfigurace pinu 17 - to je pin wakeup
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << GPIO_NUM_17),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  // Casovac .. timeout behu programu - max 60 sekund
  if (TimeoutTimer == NULL)
  {
    TimeoutTimer = xTimerCreate(
        "TimeoutTimer",
        pdMS_TO_TICKS(60000),
        pdFALSE, // jednorazovy
        (void *)0,
        vTimeoutTimerCallback);
  };

  if (TimeoutTimer != NULL)
  {
    xTimerStart(TimeoutTimer, 0);
  }

  TickType_t start_tick = xTaskGetTickCount();
  TickType_t timeout_ticks = pdMS_TO_TICKS(10000); // 10000 ms

  // Konfigurace UART pro STM32
  uartStmConfig();

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

    if (!sendTime())
    {
      ESP_LOGE(TAG, "Time frame was not sent");
    }

    ESP_LOGI(TAG, "Waiting for UART response...");
    if (uartProcessAndPublishMessages() == true)
    {
      ESP_LOGI(TAG, "Buffer nacten, delka: %u", (unsigned)bufferDataLength - 3);
    }
    else
    {
      ESP_LOGE(TAG, "Buffer ma malo dat, delka: %u", (unsigned)bufferDataLength);
    }
  }

  gotoSleep();
}

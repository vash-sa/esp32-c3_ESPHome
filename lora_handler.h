#include "esphome.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <vector>
#include <algorithm>

static const char *TAG = "lora_custom";

class LoRaCustomComponent : public PollingComponent {
 public:
  spi_device_handle_t lora_spi;
  
  // Пины для ESP32-C3 (Проверьте, не занят ли GPIO 10 вашей платой!)
  const gpio_num_t LORA_NSS  = GPIO_NUM_7;
  const gpio_num_t LORA_RST  = GPIO_NUM_10; 
  const gpio_num_t LORA_DIO0 = GPIO_NUM_3;
  const int SPI_MISO  = 5;
  const int SPI_MOSI  = 6;
  const int SPI_SCLK  = 4;

  const uint8_t ENCRYPT_KEY[16] = {'A','R','M','A','G','E','D','O','N','M','I','L','L','E','N','N'};
  std::vector<int> registered_nodes;

  LoRaCustomComponent() : PollingComponent(350) {} // Опрос каждые 350мс

  void setup() override {
    ESP_LOGI(TAG, "Initializing LoRa...");

    // 1. Настройка GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_NSS) | (1ULL << LORA_RST);
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_DIO0);
    gpio_config(&io_conf);

    // 2. Инициализация SPI шины
    spi_bus_config_t buscfg = {0};
    buscfg.miso_io_num = SPI_MISO;
    buscfg.mosi_io_num = SPI_MOSI;
    buscfg.sclk_io_num = SPI_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to init SPI: %d", ret);

    spi_device_interface_config_t devcfg = {0};
    devcfg.clock_speed_hz = 1000000; // 1MHz
    devcfg.mode = 0;
    devcfg.spics_io_num = -1; // Ручное управление NSS
    devcfg.queue_size = 7;

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to add SPI device: %d", ret);

    // 3. Аппаратный сброс модуля
    gpio_set_level(LORA_RST, 0);
    delay(20);
    gpio_set_level(LORA_RST, 1);
    delay(20);

    // 4. Проверка связи с чипом (Регистр версии 0x42 должен вернуть 0x12)
    uint8_t version = read_reg(0x42);
    if (version == 0x12) {
      ESP_LOGI(TAG, "LoRa chip detected! Version: 0x%02X", version);
    } else {
      ESP_LOGE(TAG, "LoRa NOT FOUND! Reg 0x42 returned: 0x%02X. Check wiring!", version);
    }

    // 5. Базовая настройка в режим RX (LoRa mode + RXContinuous)
    write_reg(0x01, 0x80 | 0x05); 
    ESP_LOGI(TAG, "LoRa RX Started (ESP-IDF mode)");
  }

  void update() override {
    // Проверка прерывания по получению пакета
    if (gpio_get_level(LORA_DIO0)) {
      uint8_t irq = read_reg(0x12); // RegIrqFlags
      write_reg(0x12, irq);         // Сброс флагов

      if (irq & 0x40) { // PayloadReady
        uint8_t len = read_reg(0x13); // RegRxNbBytes
        uint8_t buffer[128];
        
        write_reg(0x0D, read_reg(0x10)); // Установка FIFO на начало пакета
        
        for(int i = 0; i < len && i < 127; i++) {
            buffer[i] = read_reg(0x00);
        }

        int node_id = buffer[0];
        int pLen = buffer[1];
        if(pLen > 64) pLen = 64;

        // Расшифровка
        for (int i = 0; i < pLen; i++) {
            buffer[i+2] ^= ENCRYPT_KEY[i % 16];
        }
        buffer[2 + pLen] = '\0';

        ESP_LOGD(TAG, "Received from node %d: %s", node_id, (char*)&buffer[2]);

        if (std::find(registered_nodes.begin(), registered_nodes.end(), node_id) == registered_nodes.end()) {
          send_discovery(node_id);
          registered_nodes.push_back(node_id);
        }

        if (mqtt::global_mqtt_client->is_connected()) {
          char payload[256];
          float vcc = 0, temp = 0; int ppm = 0; char smoke[16] = {0};
          // Парсим строку вида: "3.70,25.5,clear,120"
          if (sscanf((char*)&buffer[2], "%f,%f,%[^,],%d", &vcc, &temp, smoke, &ppm) >= 3) {
            snprintf(payload, sizeof(payload), "{\"t\":%.1f,\"v\":%.2f,\"s\":\"%s\",\"g\":%d}", temp, vcc, smoke, ppm);
            char topic[64];
            snprintf(topic, sizeof(topic), "lora/%d", node_id);
            mqtt::global_mqtt_client->publish(topic, payload, strlen(payload), 0, false);
          }
        }
      }
    }
  }

  void write_reg(uint8_t addr, uint8_t val) {
    uint8_t tx_data[2] = { (uint8_t)(addr | 0x80), val };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx_data;
    
    gpio_set_level(LORA_NSS, 0);
    spi_device_polling_transmit(lora_spi, &t);
    gpio_set_level(LORA_NSS, 1);
  }

  uint8_t read_reg(uint8_t addr) {
    uint8_t tx_data[2] = { (uint8_t)(addr & 0x7F), 0x00 };
    uint8_t rx_data[2] = {0};
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    gpio_set_level(LORA_NSS, 0);
    spi_device_polling_transmit(lora_spi, &t);
    gpio_set_level(LORA_NSS, 1);

    return rx_data[1]; // Байт ответа всегда второй
  }

  void send_discovery(int id) {
    if (!mqtt::global_mqtt_client->is_connected()) return;
    ESP_LOGI(TAG, "Sending MQTT Discovery for node %d", id);
    const char* keys[] = {"t", "s", "g", "v"};
    const char* names[] = {"Temperature", "Smoke", "Gas", "Battery"};
    const char* types[] = {"sensor", "binary_sensor", "sensor", "sensor"};

    for (int i = 0; i < 4; i++) {
        char topic[128], payload[512];
        snprintf(topic, sizeof(topic), "homeassistant/%s/lora_%d_%s/config", types[i], id, keys[i]);
        snprintf(payload, sizeof(payload), 
            "{\"name\":\"Node %d %s\",\"stat_t\":\"lora/%d\",\"val_tpl\":\"{{value_json.%s}}\",\"uniq_id\":\"l_%d_%s\",\"dev\":{\"ids\":[\"l_node_%d\"],\"name\":\"LoRa Node %d\"}}", 
            id, names[i], id, keys[i], id, keys[i], id, id);
        mqtt::global_mqtt_client->publish(topic, payload, strlen(payload), 0, true);
    }
  }
};

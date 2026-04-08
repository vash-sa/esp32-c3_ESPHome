#include "esphome.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <vector>
#include <algorithm>

static const char *TAG = "lora_custom";

class LoRaCustomComponent : public PollingComponent {
 public:
  spi_device_handle_t lora_spi;
  
  // Пины для ESP32-C3
  const gpio_num_t LORA_NSS  = GPIO_NUM_7;
  const gpio_num_t LORA_RST  = GPIO_NUM_10;
  const gpio_num_t LORA_DIO0 = GPIO_NUM_3;
  const int SPI_MISO  = 5;
  const int SPI_MOSI  = 6;
  const int SPI_SCLK  = 4;

  const uint8_t ENCRYPT_KEY[16] = {'A','R','M','A','G','E','D','O','N','M','I','L','L','E','N','N'};
  std::vector<int> registered_nodes;

  LoRaCustomComponent() : PollingComponent(50) {}

  void setup() override {
    // 1. Инициализация GPIO через ESP-IDF
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_NSS) | (1ULL << LORA_RST);
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_DIO0);
    gpio_config(&io_conf);

    // 2. SPI Bus
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = SPI_MISO;
    buscfg.mosi_io_num = SPI_MOSI;
    buscfg.sclk_io_num = SPI_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 1000000;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1; 
    devcfg.queue_size = 7;

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi);

    // Сброс
    gpio_set_level(LORA_RST, 0);
    delay(10);
    gpio_set_level(LORA_RST, 1);
    delay(10);

    write_reg(0x01, 0x80 | 0x05); // RX Mode
    ESP_LOGI(TAG, "LoRa RX Started (ESP-IDF mode)");
  }

  void update() override {
    if (gpio_get_level(LORA_DIO0)) {
      uint8_t irq = read_reg(0x12);
      write_reg(0x12, irq); 

      if (irq & 0x40) {
        uint8_t len = read_reg(0x13);
        uint8_t buffer[128];
        write_reg(0x0D, read_reg(0x10));
        for(int i = 0; i < len && i < 127; i++) buffer[i] = read_reg(0x00);

        int node_id = buffer[0];
        int pLen = buffer[1];
        if(pLen > 64) pLen = 64;

        for (int i = 0; i < pLen; i++) buffer[i+2] ^= ENCRYPT_KEY[i % 16];
        buffer[2 + pLen] = '\0';

        if (std::find(registered_nodes.begin(), registered_nodes.end(), node_id) == registered_nodes.end()) {
          send_discovery(node_id);
          registered_nodes.push_back(node_id);
        }

        if (mqtt::global_mqtt_client->is_connected()) {
          char payload[256];
          // Безопасный парсинг данных
          float vcc = 0, temp = 0; int ppm = 0; char smoke[16] = {0};
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
    uint8_t data[2] = { (uint8_t)(addr | 0x80), val };
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = data;
    gpio_set_level(LORA_NSS, 0);
    spi_device_polling_transmit(lora_spi, &t);
    gpio_set_level(LORA_NSS, 1);
  }

  uint8_t read_reg(uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0 };
    uint8_t rx[2] = {0};
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    gpio_set_level(LORA_NSS, 0);
    spi_device_polling_transmit(lora_spi, &t);
    gpio_set_level(LORA_NSS, 1);
    return rx[1];
  }

  void send_discovery(int id) {
    if (!mqtt::global_mqtt_client->is_connected()) return;
    const char* keys[] = {"t", "s", "g", "v"};
    const char* names[] = {"Температура", "Дым", "Газ", "Заряд"};
    const char* types[] = {"sensor", "binary_sensor", "sensor", "sensor"};

    for (int i = 0; i < 4; i++) {
        char topic[128], payload[512];
        snprintf(topic, sizeof(topic), "homeassistant/%s/lora_%d_%s/config", types[i], id, keys[i]);
        snprintf(payload, sizeof(payload), 
            "{\"name\":\"%s %d\",\"stat_t\":\"lora/%d\",\"uniq_id\":\"l_%d_%s\",\"dev\":{\"ids\":[\"l_node_%d\"],\"name\":\"Пожарный датчик %d\"}}", 
            names[i], id, id, id, keys[i], id, id);
        mqtt::global_mqtt_client->publish(topic, payload, strlen(payload), 0, true);
    }
  }
};

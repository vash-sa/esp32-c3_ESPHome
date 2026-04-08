#include "esphome.h"
#include "driver/spi_master.h"
#include <vector>
#include <algorithm>

static const char *TAG = "lora_custom";

class LoRaCustomComponent : public Component, public PollingComponent {
 public:
  spi_device_handle_t lora_spi;
  
  // Пины для ESP32-C3 (согласно вашему коду)
  const int LORA_NSS  = 7;
  const int LORA_RST  = 10;
  const int LORA_DIO0 = 3;
  const int SPI_MISO  = 5;
  const int SPI_MOSI  = 6;
  const int SPI_SCLK  = 4;

  // Ключ шифрования
  const uint8_t ENCRYPT_KEY[16] = {'A','R','M','A','G','E','D','O','N','M','I','L','L','E','N','N'};
  
  // Список зарегистрированных ID нод
  std::vector<int> registered_nodes;

  // Опрос каждые 50мс для быстрой реакции на прерывание DIO0
  LoRaCustomComponent() : PollingComponent(50) {}

  void setup() override {
    ESP_LOGI(TAG, "Starting LoRa Setup...");

    // 1. Инициализация шины SPI
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
    devcfg.spics_io_num = -1; // Управляем вручную через NSS пин
    devcfg.queue_size = 7;

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to init SPI");
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to add SPI device");

    // 2. Настройка GPIO
    pinMode(LORA_NSS, OUTPUT);
    pinMode(LORA_RST, OUTPUT);
    pinMode(LORA_DIO0, INPUT);

    // Сброс чипа
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(10);

    // 3. Конфигурация SX127x
    uint8_t version = read_reg(0x42); // REG_VERSION
    ESP_LOGI(TAG, "SX127x Version: %02X", version);

    write_reg(0x01, 0x80 | 0x00); // LongRangeMode + Sleep
    set_frequency(433000000);
    
    write_reg(0x0E, 0);    // TX Base Addr
    write_reg(0x0F, 0);    // RX Base Addr
    write_reg(0x0C, 0x23); // LNA
    write_reg(0x1D, 0x72); // ModemConfig1
    write_reg(0x1E, 0xC4); // ModemConfig2
    write_reg(0x26, 0x0C); // ModemConfig3
    write_reg(0x40, 0x00); // DIO Mapping
    
    write_reg(0x01, 0x80 | 0x05); // RX Continuous
    ESP_LOGI(TAG, "LoRa RX Started");
  }

  void update() override {
    // Проверка прерывания по получению пакета
    if (digitalRead(LORA_DIO0)) {
      uint8_t irq = read_reg(0x12); // REG_IRQ_FLAGS
      write_reg(0x12, irq);        // Сброс флагов

      if (irq & 0x40) { // RX Done
        uint8_t len = read_reg(0x13); // REG_RX_NB_BYTES
        uint8_t buffer[128];
        
        write_reg(0x0D, read_reg(0x10)); // Установка FIFO на начало пакета
        for(int i = 0; i < len && i < 127; i++) {
          buffer[i] = read_reg(0x00);
        }

        int node_id = buffer[0];
        int pLen = buffer[1];
        if(pLen > 64) pLen = 64;

        // Дешифровка XOR
        for (int i = 0; i < pLen; i++) {
          buffer[i+2] ^= ENCRYPT_KEY[i % 16];
        }
        buffer[2 + pLen] = '\0';
        char* data_str = (char*)&buffer[2];

        ESP_LOGI(TAG, "Node %d raw data: %s", node_id, data_str);

        // Парсинг данных
        float vcc, temp;
        int ppm;
        char smoke[16];
        if (sscanf(data_str, "%f,%f,%[^,],%d", &vcc, &temp, smoke, &ppm) == 4) {
          
          // Проверяем/отправляем Discovery
          if (std::find(registered_nodes.begin(), registered_nodes.end(), node_id) == registered_nodes.end()) {
            send_discovery(node_id);
            registered_nodes.push_back(node_id);
          }

          // Отправка данных в топик ноды
          if (mqtt::global_mqtt_client->is_connected()) {
            char payload[256];
            snprintf(payload, sizeof(payload), "{\"t\":%.1f,\"v\":%.2f,\"s\":\"%s\",\"g\":%d}", temp, vcc, smoke, ppm);
            char topic[64];
            snprintf(topic, sizeof(topic), "lora/%d", node_id);
            mqtt::global_mqtt_client->publish(topic, payload, strlen(payload), 0, false);
          }
        }
      }
    }
  }

  void send_discovery(int id) {
    if (!mqtt::global_mqtt_client->is_connected()) return;

    // Массивы для генерации 4 сущностей HA
    const char* types[] = {"sensor", "binary_sensor", "sensor", "sensor"};
    const char* keys[] = {"t", "s", "g", "v"};
    const char* names[] = {"Температура", "Дым", "Угарный газ", "Заряд"};
    const char* classes[] = {"temperature", "smoke", "carbon_monoxide", "voltage"};
    const char* units[] = {"\"unit_of_meas\":\"°C\",", "", "\"unit_of_meas\":\"ppm\",", "\"unit_of_meas\":\"V\","};

    for (int i = 0; i < 4; i++) {
        char topic[128];
        char payload[1024];
        
        snprintf(topic, sizeof(topic), "homeassistant/%s/lora_%d_%s/config", types[i], id, keys[i]);
        
        // Формируем JSON Discovery
        int pos = snprintf(payload, sizeof(payload), 
            "{\"name\":\"%s\",\"stat_t\":\"lora/%d\",%s\"dev_cla\":\"%s\",\"uniq_id\":\"l_%d_%s\"", 
            names[i], id, units[i], classes[i], id, keys[i]);
            
        // Привязка значения в зависимости от типа
        if (i == 1) // Smoke (binary)
            pos += snprintf(payload + pos, sizeof(payload) - pos, ",\"val_tpl\":\"{{'ON' if value_json.s=='YES' else 'OFF'}}\"");
        else
            pos += snprintf(payload + pos, sizeof(payload) - pos, ",\"val_tpl\":\"{{value_json.%s}}\"", keys[i]);

        // Группировка в одно устройство по ID ноды
        snprintf(payload + pos, sizeof(payload) - pos, 
            ",\"dev\":{\"ids\":[\"l_node_%d\"],\"name\":\"Пожарный датчик %d\"}}", id, id);
        
        mqtt::global_mqtt_client->publish(topic, payload, strlen(payload), 0, true);
    }
    ESP_LOGI(TAG, "Discovery device created in HA for Node %d", id);
  }

  // Низкоуровневые функции работы с регистрами
  void write_reg(uint8_t addr, uint8_t val) {
    uint8_t data[2] = { (uint8_t)(addr | 0x80), val };
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = data;
    digitalWrite(LORA_NSS, LOW);
    spi_device_polling_transmit(lora_spi, &t);
    digitalWrite(LORA_NSS, HIGH);
  }

  uint8_t read_reg(uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0 };
    uint8_t rx[2] = {0};
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    digitalWrite(LORA_NSS, LOW);
    spi_device_polling_transmit(lora_spi, &t);
    digitalWrite(LORA_NSS, HIGH);
    return rx[1];
  }

  void set_frequency(long freq) {
    long frf = ((long long)freq << 19) / 32000000;
    write_reg(0x06, (uint8_t)(frf >> 16));
    write_reg(0x07, (uint8_t)(frf >> 8));
    write_reg(0x08, (uint8_t)(frf));
  }
};

#include "esphome.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "lora_custom";

class LoRaCustomComponent : public Component, public PollingComponent {
 public:
  spi_device_handle_t lora_spi;
  
  // Пины для ESP32-C3
  const int LORA_NSS = 7;
  const int LORA_RST = 10;
  const int LORA_DIO0 = 3;

  LoRaCustomComponent() : PollingComponent(100) {} // Проверка каждые 100мс

  void setup() override {
    spi_bus_config_t buscfg = {
        .miso_io_num = 5, .mosi_io_num = 6, .sclk_io_num = 4,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = -1, .queue_size = 7
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &lora_spi);

    pinMode(LORA_NSS, OUTPUT);
    pinMode(LORA_RST, OUTPUT);
    pinMode(LORA_DIO0, INPUT);

    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    
    // Инициализация регистров (сокращено для краткости, используйте ваши LoRa_WriteReg здесь)
    write_reg(0x01, 0x80 | 0x00); // Sleep
    set_frequency(433000000);
    write_reg(0x01, 0x80 | 0x05); // RX Continuous
    ESP_LOGI(TAG, "LoRa Initialized");
  }

  void update() override {
    if (digitalRead(LORA_DIO0)) {
      uint8_t irq = read_reg(0x12);
      write_reg(0x12, irq); // Clear IRQ

      if (irq & 0x40) { // RX Done
        uint8_t len = read_reg(0x13);
        uint8_t buffer[128];
        write_reg(0x0D, read_reg(0x10));
        for(int i = 0; i < len && i < 127; i++) buffer[i] = read_reg(0x00);
        
        // Здесь ваш парсинг и отправка в ESPHome sensor
        ESP_LOGD(TAG, "Received LoRa packet, length: %d", len);
        // Пример отправки в MQTT через ESPHome API:
        // this->fire_homeassistant_event("lora_data_received", {{"id", to_string(buffer[0])}});
      }
    }
  }

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

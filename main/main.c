#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"

#include "mqtt.h"

#include <sys/time.h>
#include "math.h"

#include "user_config.h"

#define TV_TO_MS(tv) ((tv.tv_sec * 1000) + (tv.tv_usec / 1000))

#define ADC_BITS    12
#define ADC_COUNTS  (1<<ADC_BITS)
#define PUMP_ON_RMS_THRESHOLD 5.0
#define PUMP_MIN_RUN_TIME_MS 1000
#define IRMS_SAMPLE_COUNT 1500
#define MQTT_ONLINE_STATUS_TOPIC "home/basement/sump/online_status"

const float ICAL = 17.75;            // calibration factor

int sample = 0;
double offsetI, sqI, sumI, Irms;
double filteredI = 0;
mqtt_client *mqttClient = 0;


void connected_cb(void *self, void *params)
{
	printf("mqtt connected\n");
}

mqtt_settings settings = {
    .host = MQTT_HOST,
    .port = 1883,
    .client_id = "sump_pump_monitor",
    .lwt_topic = MQTT_ONLINE_STATUS_TOPIC,
    .lwt_msg = "offline",
    .lwt_qos = 1,
    .lwt_retain = 1,
    .clean_session = 1,
    .keepalive = 60,
};

// Funtion originally derived from Emonlib
// https://github.com/openenergymonitor/EmonLib
double calcIrms(unsigned int Number_of_Samples)
{

  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sample = adc1_get_voltage(ADC1_CHANNEL_0);

    offsetI = (offsetI + (sample-offsetI)/1024);
    filteredI = sample - offsetI;

    sqI = filteredI * filteredI;
    sumI += sqI;
  }

  double I_RATIO = ICAL *((3300/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

  sumI = 0;

  return Irms;
}

esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        mqttClient = mqtt_start(&settings);
        if (!mqttClient)
        {
            printf("Failed to create mqtt client!\n");
        } else {
            printf("Created mqtt client\n");
        }
        // Notice that, all callback will called in mqtt_task
        // All function publish, subscribe
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        
        printf("disconnecting from mqtt\n");
        mqtt_stop();
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    default:
        break;
    }
    return ESP_OK;
}

void wifi_conn_init(void)
{
    printf("[APP] Start, connect to Wifi network: %s ..\n", WIFI_SSID);

    tcpip_adapter_init();

    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );

    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&icfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK( esp_wifi_start());
}

void gpio_init(void)
{
    // ADC init
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db);

    // LED for doing some blinking
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
}

struct timeval tv;
int pumpStartTimeMs;

int sampleCount = 0;
double iRmsSum = 0.0;

char jsonBuffer[48];

void take_reading(void)
{
    double iRms = calcIrms(IRMS_SAMPLE_COUNT);

    if (iRms < PUMP_ON_RMS_THRESHOLD) {
        if (!sampleCount) {
            // Pump isn't running, return
            return;
        }

        // Pump was running and is now off, send a message
        double avgRunIRms = iRmsSum / sampleCount;

        if (gettimeofday(&tv, NULL)) {
            printf("Failed to gettimeofday %d\n", errno);

            sampleCount = 0;
            iRmsSum = 0.0;
            return;
        }

        int runTimeMs = TV_TO_MS(tv) - pumpStartTimeMs;

        if (runTimeMs < PUMP_MIN_RUN_TIME_MS) {
            // Crap reading, throw it out
            sampleCount = 0;
            iRmsSum = 0.0;
            mqtt_publish(mqttClient, "home/basement/sump/state", "OFF", 3, 0, 1);
            return;
        }

        int count = sprintf(jsonBuffer, "{\"runTime\":%d,\"averageIRms\":%0.1f}", runTimeMs, avgRunIRms);
        if (count < 0) {
            sampleCount = 0;
            iRmsSum = 0.0;
            return;
        }

        mqtt_publish(mqttClient, "home/basement/sump/state", "OFF", 3, 0, 1);
        mqtt_publish(mqttClient, "home/basement/sump/run_info", jsonBuffer, count, 0, 0);

        sampleCount = 0;
        iRmsSum = 0.0;

        return;
    }
    
    // Pump is on
    if (!sampleCount) {
        mqtt_publish(mqttClient, "home/basement/sump/state", "ON", 2, 0, 1);
        // Note the time w***REMOVED*** we first detect it's on
        if (gettimeofday(&tv, NULL)) {
            printf("Failed to gettimeofday %d\n", errno);

            sampleCount = 0;
            iRmsSum = 0.0;
            return;
        }

        pumpStartTimeMs = TV_TO_MS(tv);
    }

    sampleCount++;
    iRmsSum += iRms;
}

void app_main(void)
{
    nvs_flash_init();
	wifi_conn_init();
    gpio_init();

    while(!mqttClient) {
        printf("waiting for mqttClient...\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    mqtt_publish(mqttClient, MQTT_ONLINE_STATUS_TOPIC, "online", 6, 1, 1);

    // Get some garbage readings w***REMOVED*** we first boot, clear them out
    for (int i = 0; i < 5; i++) {
        calcIrms(IRMS_SAMPLE_COUNT);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    mqtt_publish(mqttClient, "home/basement/sump/state", "OFF", 3, 0, 1);

    while (true) {
        gpio_set_level(GPIO_NUM_4, 1);
        take_reading();
        gpio_set_level(GPIO_NUM_4, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

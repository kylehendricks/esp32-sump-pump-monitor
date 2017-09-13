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
#define SUMP_TOPIC(suffix) SUMP_TOPIC_PREFIX suffix
#define WELL_TOPIC(suffix) WELL_TOPIC_PREFIX suffix
#define MQTT_TOPIC(pump_type, suffix) pump_type == PUMP_TYPE_WELL ? WELL_TOPIC(suffix) : SUMP_TOPIC(suffix)

#define ADC_BITS    12
#define ADC_COUNTS  (1<<ADC_BITS)
#define THRESHOLD_RMS_SUMP 5.0
#define THRESHOLD_RMS_WELL 2.0
#define PUMP_MIN_RUN_TIME_MS 1000
#define IRMS_SAMPLE_COUNT 1500
#define ADC_PIN_SUMP ADC1_CHANNEL_0
#define ADC_PIN_WELL ADC1_CHANNEL_3

typedef enum {
    PUMP_TYPE_SUMP = 0,
    PUMP_TYPE_WELL,
    PUMP_TYPE_MAX
} pump_type_t;

const double THRESHOLD[PUMP_TYPE_MAX] = { THRESHOLD_RMS_SUMP, THRESHOLD_RMS_WELL };
const float ICAL = 17.75;            // calibration factor

int sample = 0;
double offsetI, sqI, sumI, Irms;
double filteredI = 0;
mqtt_client *mqttClient = 0;
struct timeval tv;
int pumpStartTimeMs[PUMP_TYPE_MAX];
int sampleCount[PUMP_TYPE_MAX] = {0};
double iRmsSum[PUMP_TYPE_MAX] = {0.0};
char jsonBuffer[48];

void connected_cb(void *self, void *params)
{
	printf("mqtt connected\n");
}

mqtt_settings settings = {
    .host = MQTT_HOST,
    .username = MQTT_USER,
    .password = MQTT_PASSWORD,
    .port = 1883,
    .client_id = "pump_monitor",
    .lwt_topic = LWT_TOPIC,
    .lwt_msg = "offline",
    .lwt_qos = 1,
    .lwt_retain = 1,
    .clean_session = 1,
    .keepalive = 60,
};

// Funtion originally derived from Emonlib
// https://github.com/openenergymonitor/EmonLib
double calcIrms(adc1_channel_t adcChannel, unsigned int Number_of_Samples)
{

  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sample = adc1_get_voltage(adcChannel);

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
    adc1_config_channel_atten(ADC_PIN_SUMP, ADC_ATTEN_11db);
    adc1_config_channel_atten(ADC_PIN_WELL, ADC_ATTEN_11db);

    // LED for doing some blinking
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
}

void take_reading(pump_type_t pumpType)
{
    double iRms = calcIrms(pumpType == PUMP_TYPE_SUMP ? ADC_PIN_SUMP : ADC_PIN_WELL, IRMS_SAMPLE_COUNT);
    if (pumpType == PUMP_TYPE_SUMP) {
        printf("SUMP: %f\n", iRms);
    } else {
        printf("WELL: %f\n", iRms);
    }

    if (iRms < THRESHOLD[pumpType]) {
        if (!sampleCount[pumpType]) {
            // Pump isn't running, return
            return;
        }

        // Pump was running and is now off, send a message
        double avgRunIRms = iRmsSum[pumpType] / sampleCount[pumpType];

        if (gettimeofday(&tv, NULL)) {
            printf("Failed to gettimeofday %d\n", errno);

            sampleCount[pumpType] = 0;
            iRmsSum[pumpType] = 0.0;
            return;
        }

        int runTimeMs = TV_TO_MS(tv) - pumpStartTimeMs[pumpType];

        if (runTimeMs < PUMP_MIN_RUN_TIME_MS) {
            // Crap reading, throw it out
            sampleCount[pumpType] = 0;
            iRmsSum[pumpType] = 0.0;
            mqtt_publish(mqttClient, MQTT_TOPIC(pumpType, "state"), "OFF", 3, 0, 1);
            return;
        }

        int count = sprintf(jsonBuffer, "{\"runTime\":%d,\"averageIRms\":%0.1f}", runTimeMs, avgRunIRms);
        if (count < 0) {
            sampleCount[pumpType] = 0;
            iRmsSum[pumpType] = 0.0;
            return;
        }

        mqtt_publish(mqttClient, MQTT_TOPIC(pumpType, "state"), "OFF", 3, 0, 1);
        mqtt_publish(mqttClient, MQTT_TOPIC(pumpType, "run_info"), jsonBuffer, count, 0, 0);

        sampleCount[pumpType] = 0;
        iRmsSum[pumpType] = 0.0;

        return;
    }
    
    // Pump is on
    if (!sampleCount[pumpType]) {
        mqtt_publish(mqttClient, MQTT_TOPIC(pumpType, "state"), "ON", 2, 0, 1);
        // Note the time when we first detect it's on
        if (gettimeofday(&tv, NULL)) {
            printf("Failed to gettimeofday %d\n", errno);

            sampleCount[pumpType] = 0;
            iRmsSum[pumpType] = 0.0;
            return;
        }

        pumpStartTimeMs[pumpType] = TV_TO_MS(tv);
    }

    sampleCount[pumpType]++;
    iRmsSum[pumpType] += iRms;
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

    mqtt_publish(mqttClient, LWT_TOPIC, "online", 6, 1, 1);

    // Get some garbage readings when we first boot, clear them out
    for (int i = 0; i < 5; i++) {
        calcIrms(ADC_PIN_SUMP, IRMS_SAMPLE_COUNT);
        calcIrms(ADC_PIN_WELL, IRMS_SAMPLE_COUNT);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    mqtt_publish(mqttClient, MQTT_TOPIC(PUMP_TYPE_SUMP, "state"), "OFF", 3, 0, 1);
    mqtt_publish(mqttClient, MQTT_TOPIC(PUMP_TYPE_WELL, "state"), "OFF", 3, 0, 1);

    while (true) {
        gpio_set_level(GPIO_NUM_4, 1);
        take_reading(PUMP_TYPE_WELL);
        take_reading(PUMP_TYPE_SUMP);
        gpio_set_level(GPIO_NUM_4, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

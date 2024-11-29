#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include <stdlib.h>

#include "cJSON.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_sntp.h"
#include "driver/gpio.h"
#include "esp_wifi_types.h"
#include "freertos/event_groups.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "homeapp.h"
#include "temperature/temperatures.h"
#include "flashmem.h"
#include "ota/ota.h"
#include "device/device.h"
#include "mqtt_client.h"
#include "apwebserver/server.h"
#include "factoryreset.h"
#include "statistics/statistics.h"
#include "rgb7seg.h"

#define TEMP_BUS 17
#define STATISTICS_INTERVAL 1800
#define ESP_INTR_FLAG_DEFAULT 0


#if CONFIG_EXAMPLE_WIFI_SCAN_METHOD_FAST
#define EXAMPLE_WIFI_SCAN_METHOD WIFI_FAST_SCAN
#elif CONFIG_EXAMPLE_WIFI_SCAN_METHOD_ALL_CHANNEL
#define EXAMPLE_WIFI_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#endif

#if CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SIGNAL
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SECURITY
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#endif

#define WIFI_RECONNECT_RETRYCNT 50

#define HEALTHYFLAGS_WIFI 1
#define HEALTHYFLAGS_MQTT 2
#define HEALTHYFLAGS_TEMP 4
#define HEALTHYFLAGS_NTP  8

#define SETUP_ALL     0xff
#define SETUP_COLORS  0x01
#define SETUP_NAMES   0x02
#define SETUP_MISC    0x04
#define SETUP_SENSORS 0x08


struct netinfo {
    char *ssid;
    char *password;
    char *mqtt_server;
    char *mqtt_port;
    char *mqtt_prefix;
};


struct config {
    char specialsensor[20];
    int showinternaltemp;
    int zonelow;
    int zonehigh;
};

struct config setup = { .showinternaltemp = 1,
                         .zonelow = 2300,
                         .zonehigh = 2600};

struct colorname {
    char *name;
    struct color c;
} colornames[] =
{
    {"red",   {50,  0,  0}},
    {"green", { 0, 50,  0}},
    {"blue",  { 0,  0, 50}},
    {"cyan",  { 0, 50, 50}},
    {"purple",{50,  0, 50}},
    {"yellow",{50, 50,  0}},
    {"white", {50, 50, 50}},
    {"pink",  {50,  5, 20}},
    {"gold",  {50, 42,  0}},
    {"orange",{50, 32,  0}},
    {"tomato",{50, 19, 14}},
    {"sky",   { 0, 37, 50}},
    {"aqua",  {25, 50, 41}},
    {"\0",    {50,  0,  0}}
};
// globals

struct netinfo *comminfo;
QueueHandle_t evt_queue = NULL;
char jsondata[512];
uint16_t sendcnt = 0;

static const char *TAG = "RGB7SEGDISP";
static bool isConnected = false;
static uint8_t healthyflags = 0;
uint16_t sensorerrors = 0;

static char statisticsTopic[64];
static char readTopic[64];
static char dataTopic[64];
static char otaUpdateTopic[64];
static int retry_num = 0;
static char *program_version = "";
static char appname[20];
static struct colorname *default_color = &colornames[1];
static struct colorname *low_color = &colornames[2];
static struct colorname *high_color = &colornames[0];

static void sendSetup(esp_mqtt_client_handle_t client, uint8_t *chipid, uint8_t flags);
static void sendInfo(esp_mqtt_client_handle_t client, uint8_t *chipid);


static char *getJsonStr(cJSON *js, char *name)
{
    cJSON *item = cJSON_GetObjectItem(js, name);
    if (item != NULL)
    {
        if (cJSON_IsString(item))
        {
            return item->valuestring;
        }
        else ESP_LOGI(TAG, "%s is not a string", name);
    }
    else ESP_LOGI(TAG,"%s not found from json", name);
    return "\0";
}

static bool getJsonFloat(cJSON *js, char *name, float *val)
{
    bool ret = false;

    cJSON *item = cJSON_GetObjectItem(js, name);
    if (item != NULL)
    {
        if (cJSON_IsNumber(item))
        {
            if (item->valuedouble != *val)
            {
                ret = true;
                *val = item->valuedouble;
                ESP_LOGI(TAG,"received variable %s:%2.2f", name, item->valuedouble);
            }
            ESP_LOGI(TAG,"%s is not changed", name);
        }
        else ESP_LOGI(TAG,"%s is not a number", name);
    }
    else ESP_LOGI(TAG,"%s not found from json", name);
    return ret;
}

static bool getJsonInt(cJSON *js, char *name, int *val)
{
    bool ret = false;

    cJSON *item = cJSON_GetObjectItem(js, name);
    if (item != NULL)
    {
        if (cJSON_IsNumber(item))
        {
            if (item->valueint != *val)
            {
                ret = true;
                *val = item->valueint;
            }
            else ESP_LOGI(TAG,"%s is not changed", name);
        }
        else ESP_LOGI(TAG,"%s is not a number", name);
    }
    else ESP_LOGI(TAG,"%s not found from json", name);
    return ret;
}


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

// if value = 8888 show last known temperature
static void show_internaltemp(float value)
{
    struct colorname *dispcolor;
    static int dispvar = 0;

    if (value != 8888)
    {
        dispvar = value * 100; // we dont have decimal point in display.
    }

    if (dispvar < setup.zonelow) dispcolor = low_color;
    else if (dispvar > setup.zonehigh) dispcolor = high_color;
    else dispcolor = default_color;

    char buff[6];
    sprintf(buff,"%4d", dispvar);
    rgb7seg_display(buff,dispcolor->c);
}


static void sensorFriendlyName(cJSON *root)
{
    char *sensorname;
    char *friendlyname;

    sensorname   = getJsonStr(root, "sensor");
    friendlyname = getJsonStr(root, "name");
    if (temperature_set_friendlyname(sensorname, friendlyname))
    {
        ESP_LOGD(TAG, "writing sensor %s, friendlyname %s to flash",sensorname, friendlyname);
        flash_write_str(sensorname,friendlyname);
        flash_commitchanges();
    }
}

/* ../setsetup -m '{"temperature": 8, "hysteresis": 2, "mintimeon": 120, "lopriceboost": 2, "hipricereduce", 1}'
*/

static struct colorname *get_color(char *name)
{
    int i=0;
    for (; colornames[i].name[0] != 0;i++)
    {
        if (!strcmp(colornames[i].name, name))
            return &colornames[i];
    }
    return NULL;
}

static void readSetupJson(cJSON *root)
{
    bool redisp_needed = false;
    char *cname;
    struct colorname *c;
    // add here the setup parameter reads from json.
    // store them to flash
    if (getJsonInt(root,"showinternaltemp", &setup.showinternaltemp))
    {
        flash_write("inttemp",setup.showinternaltemp);
    }

    cname = getJsonStr(root,"defaultcolor");
    c = get_color(cname);
    if (c != NULL)
    {
        default_color = c;
        flash_write_str("defaultcolor", cname);
        redisp_needed = true;
    }

    cname = getJsonStr(root,"highcolor");
    c = get_color(cname);
    if (c != NULL)
    {
        high_color = c;
        flash_write_str("highcolor",cname);
        redisp_needed = true;
    }

    cname = getJsonStr(root,"lowcolor");
    c = get_color(cname);
    if (c != NULL)
    {
        low_color = c;
        flash_write_str("lowcolor",cname);
        redisp_needed = true;
    }

    if (getJsonInt(root, "zonelow", &setup.zonelow))
    {
        flash_write("zonelow", setup.zonelow);
        redisp_needed = true;
    }

    if (getJsonInt(root, "zonehigh", &setup.zonehigh))
    {
        flash_write("zonehigh", setup.zonehigh);
        redisp_needed = true;
    }

    if (redisp_needed)
    {
        ESP_LOGI(TAG,"doing some reinit stuff.");
        show_internaltemp(8888);
    }
    flash_commitchanges();
}


static uint8_t handleJson(esp_mqtt_event_handle_t event, uint8_t *chipid)
{
    cJSON *root = cJSON_Parse(event->data);
    uint8_t ret = 0;
    char id[20];
    time_t now;

    time(&now);

    if (root != NULL)
    {
        strcpy(id,getJsonStr(root,"id"));
    }
    if (!strcmp(id,"otaupdate"))
    {
        char *fname = getJsonStr(root,"file");
        if (strlen(fname) > 5)
        {
            ota_start(fname);
        }
    }
    else if (!strcmp(id,"setup"))
    {
        readSetupJson(root);
        ret |= SETUP_MISC;
    }
    else if (!strcmp(id,"show"))
    {
        char *data = getJsonStr(root,"data");

        char *name = getJsonStr(root,"color");
        if (name[0] != 0)
        {
            struct colorname *c = get_color(getJsonStr(root,"color"));
            rgb7seg_display(data,c->c);
        }
        else
        {
            rgb7seg_display(data,default_color->c);
        }
    }
    else if (!strcmp(id,"sensorsetup"))
    {
        strncpy(setup.specialsensor,getJsonStr(root,"specialsensor"),20);
        flash_write_str("specsensor", setup.specialsensor);
        ret |= SETUP_SENSORS;
    }
    else if (!strcmp(id,"sensorfriendlyname"))
    {
        sensorFriendlyName(root);
        ret |= SETUP_NAMES;
    }
    cJSON_Delete(root);
    return ret;
}


/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    uint8_t flags=0;
    

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        ESP_LOGI(TAG,"subscribing topic %s", readTopic);
        msg_id = esp_mqtt_client_subscribe(client, readTopic, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, otaUpdateTopic , 0);
        ESP_LOGI(TAG, "sent subscribe %s successful, msg_id=%d", otaUpdateTopic, msg_id);

        msg_id = esp_mqtt_client_subscribe(client, dataTopic , 0);
        ESP_LOGI(TAG, "sent subscribe %s successful, msg_id=%d", dataTopic, msg_id);

        gpio_set_level(MQTTSTATUS_GPIO, true);
        rgb7seg_display("mqtt",default_color->c);
        device_sendstatus(client, comminfo->mqtt_prefix, appname, (uint8_t *) handler_args);
        isConnected = true;
        statistics_getptr()->connectcnt++;
        sendInfo(client, (uint8_t *) handler_args);
        sendSetup(client, (uint8_t *) handler_args, SETUP_ALL);
        temperature_sendall();
        healthyflags |= HEALTHYFLAGS_MQTT;
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        statistics_getptr()->disconnectcnt++;
        isConnected = false;
        gpio_set_level(MQTTSTATUS_GPIO, false);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        flags = handleJson(event,(uint8_t *) handler_args);
        if (flags)
        {
            sendSetup(client, (uint8_t *) handler_args, flags);
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/*  sntp_callback()
**  My influx saver does not want to save records which have bad time.
**  So, it's better to send them again, when correct time has been set.
*/
static void sntp_callback(struct timeval *tv)
{
    (void) tv;
    static bool firstSyncDone = false;

    if (!firstSyncDone)
    {
        temperature_sendall();
        firstSyncDone = true;
        healthyflags |= HEALTHYFLAGS_NTP;
    }
}


static void sntp_start()
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    sntp_set_time_sync_notification_cb(sntp_callback);
}


int getWifiStrength(void)
{
    wifi_ap_record_t ap;

    if (!esp_wifi_sta_get_ap_info(&ap))
        return ap.rssi;
    return 0;
}


static void sendInfo(esp_mqtt_client_handle_t client, uint8_t *chipid)
{
    gpio_set_level(BLINK_GPIO, true);

    char infoTopic[42];

    sprintf(infoTopic,"%s/%s/%x%x%x/info",
         comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);
    sprintf(jsondata, "{\"dev\":\"%x%x%x\",\"id\":\"info\",\"memfree\":%d,\"idfversion\":\"%s\",\"progversion\":%s}",
                chipid[3],chipid[4],chipid[5],
                esp_get_free_heap_size(),
                esp_get_idf_version(),
                program_version);
    esp_mqtt_client_publish(client, infoTopic, jsondata , 0, 0, 1);
    statistics_getptr()->sendcnt++;
    gpio_set_level(BLINK_GPIO, false);
}


/* ../setsetup -m '{"temperature": 8, "hysteresis": 2, "mintimeon": 120, "lopriceboost": 2, "hipricereduce", 1}'
*/


static void sendSetup(esp_mqtt_client_handle_t client, uint8_t *chipid, uint8_t flags)
{
    gpio_set_level(BLINK_GPIO, true);

    char setupTopic[80];

    if (flags & SETUP_COLORS)
    {
        sprintf(setupTopic,"%s/%s/%02x%02x%02x/colors",
           comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);
        sprintf(jsondata, "{\"dev\":\"%x%x%x\",\"id\":\"colors\",\"colors\":[",
            chipid[3],chipid[4],chipid[5]);
        char colorvalue[30];

        for (int i=0; colornames[i].name[0]!=0; i++)
        {
            // multiply colorvalues, otherwise they are not visible enough in web browser.
            sprintf(colorvalue,"{\"name\":\"%s\",\"value\":\"#%02x%02x%02x\"},",
                colornames[i].name, 3 * colornames[i].c.r, 3* colornames[i].c.g, 3 * colornames[i].c.b);
            strcat(jsondata,colorvalue);
        }
        jsondata[strlen(jsondata)-1] = 0; // cut last comma
        strcat(jsondata,"]}");
        esp_mqtt_client_publish(client, setupTopic, jsondata , 0, 0, 1);
        statistics_getptr()->sendcnt++;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (flags & SETUP_MISC)
    {
        sprintf(setupTopic,"%s/%s/%x%x%x/setup",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);
        sprintf(jsondata, "{\"dev\":\"%x%x%x\",\"id\":\"setup\",\"defaultcolor\":\"%s\",\"lowcolor\":\"%s\",\"highcolor\":\"%s\",\"zonelow\":\"%d\",\"zonehigh\":\"%d\",\"showinternaltemp\":%d}",
                    chipid[3],chipid[4],chipid[5],
                    default_color->name,
                    low_color->name,
                    high_color->name,
                    setup.zonelow,
                    setup.zonehigh,
                    setup.showinternaltemp);
        esp_mqtt_client_publish(client, setupTopic, jsondata , 0, 0, 1);
        statistics_getptr()->sendcnt++;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (flags & SETUP_SENSORS)
    {
        sprintf(setupTopic,"%s/%s/%x%x%x/sensorsetup",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);
        sprintf(jsondata, "{\"dev\":\"%x%x%x\",\"id\":\"sensorsetup\",\"specialsensor\":\"%s\"}",
                    chipid[3],chipid[4],chipid[5],
                    setup.specialsensor);
        esp_mqtt_client_publish(client, setupTopic, jsondata , 0, 0, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        statistics_getptr()->sendcnt++;
    }

    if (flags & SETUP_NAMES)
    {
        sprintf(setupTopic,"%s/%s/%x%x%x/tempsensors",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);
        sprintf(jsondata, "{\"dev\":\"%x%x%x\",\"id\":\"tempsensors\",\"names\":[",
            chipid[3],chipid[4],chipid[5]);

        char sensorname[40];

        for (int i = 0; ; i++)
        {
            char *sensoraddr = temperature_getsensor(i);

            if (sensoraddr == NULL) break;
            sprintf(sensorname,"{\"addr\":\"%s\",\"name\":\"%s\"},",
                sensoraddr, temperature_get_friendlyname(i));
            strcat(jsondata,sensorname);
        }
        jsondata[strlen(jsondata)-1] = 0; // cut last comma
        strcat(jsondata,"]}");
        esp_mqtt_client_publish(client, setupTopic, jsondata , 0, 0, 1);
        statistics_getptr()->sendcnt++;
    }
    gpio_set_level(BLINK_GPIO, false);
}

static esp_mqtt_client_handle_t mqtt_app_start(uint8_t *chipid)
{
    char client_id[128];
    char uri[64];
    char deviceTopic[42];
    
    sprintf(client_id,"client_id=%s%x%x%x",
        comminfo->mqtt_prefix ,chipid[3],chipid[4],chipid[5]);
    sprintf(uri,"mqtt://%s:%s",comminfo->mqtt_server, comminfo->mqtt_port);

    ESP_LOGI(TAG,"built client id=[%s]",client_id);
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
        .credentials.client_id = client_id,
        .session.last_will.topic = device_topic(comminfo->mqtt_prefix, deviceTopic, chipid),
        .session.last_will.msg = device_data(jsondata, chipid, appname, 0),
        .session.last_will.msg_len = strlen(jsondata),
        .session.last_will.qos = 0,
        .session.last_will.retain = 1
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, chipid);
    esp_mqtt_client_start(client);
    return client;
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data)
{
    if(event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG,"WIFI CONNECTING");
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG,"WiFi CONNECTED");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG,"WiFi lost connection");
        gpio_set_level(WLANSTATUS_GPIO, false);
        if(retry_num < WIFI_RECONNECT_RETRYCNT){
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG,"Retrying to Connect");
        }
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG,"Wifi got IP\n");
        gpio_set_level(WLANSTATUS_GPIO, true);
        retry_num = 0;
        healthyflags |= HEALTHYFLAGS_WIFI;
        rgb7seg_display("lan",default_color->c);
    }
}


void wifi_connect(char *ssid, char *password)
{
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "",
            .password = "",
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_OPEN,
        }
    };
    strcpy((char*)wifi_configuration.sta.ssid, ssid);
    strcpy((char*)wifi_configuration.sta.password, password);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_connect();
}

static void get_sensor_friendlynames(void)
{
    char *sensorname;
    char *friendlyname;

    for (int i = 0; i < 10; i++)
    {
        sensorname   = temperature_getsensor(i);
        if (sensorname == NULL)
            break;
        friendlyname = flash_read_str(sensorname, sensorname, 20);
        if (strcmp(friendlyname, sensorname))
        {
            if (!temperature_set_friendlyname(sensorname, friendlyname))
            {
                ESP_LOGD(TAG, "Set friedlyname for %s failed", sensorname);
            }
            free(friendlyname);
        }
    }
}

struct netinfo *get_networkinfo()
{
    static struct netinfo ni;
    char *default_ssid = "XXXXXXXX";

    ni.ssid = flash_read_str("ssid",default_ssid, 20);
    if (!strcmp(ni.ssid,"XXXXXXXX"))
        return NULL;

    ni.password    = flash_read_str("password","pass", 20);
    ni.mqtt_server = flash_read_str("mqtt_server","test.mosquitto.org", 20);
    ni.mqtt_port   = flash_read_str("mqtt_port","1883", 6);
    ni.mqtt_prefix = flash_read_str("mqtt_prefix","home/esp", 20);
    return &ni;
}

void readSetup(void)
{
    char *colorname;

    strcpy(setup.specialsensor, flash_read_str("specsensor", setup.specialsensor,12));
    colorname = flash_read_str("defaultcolor", default_color->name, 12);
    default_color = get_color(colorname);

    colorname = flash_read_str("highcolor", high_color->name, 12);
    high_color = get_color(colorname);

    colorname = flash_read_str("lowcolor", low_color->name, 12);
    low_color = get_color(colorname);

    setup.zonelow  = flash_read("zonelow", setup.zonelow);
    setup.zonehigh = flash_read("zonehigh", setup.zonehigh);

    setup.showinternaltemp = flash_read("inttemp", setup.showinternaltemp);
}


static void get_appname(void)
{
    const esp_app_desc_t *app_desc = esp_app_get_description();
    strncpy(appname,app_desc->project_name,20);
}


void app_main(void)
{
    uint8_t chipid[8];
    time_t now, prevStatsTs;
    int packetcount=0;

    esp_efuse_mac_get_default(chipid);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    rgb7seg_init();
    gpio_reset_pin(BLINK_GPIO);
    gpio_reset_pin(WLANSTATUS_GPIO);
    gpio_reset_pin(SETUP_GPIO);
    gpio_reset_pin(MQTTSTATUS_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(WLANSTATUS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SETUP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MQTTSTATUS_GPIO, GPIO_MODE_OUTPUT);

    get_appname();
    flash_open("storage");
    comminfo = get_networkinfo();
    
    rgb7seg_display("init",default_color->c);

    if (comminfo == NULL)
    {
        gpio_set_level(SETUP_GPIO, true);
        server_init();
    }
    else
    {
        evt_queue = xQueueCreate(10, sizeof(struct measurement));
        
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        factoryreset_init();
        if (temperature_init(TEMP_BUS, appname, chipid, 2) > 0)
        {
            get_sensor_friendlynames();
        }
        wifi_connect(comminfo->ssid, comminfo->password);
        readSetup();


        esp_mqtt_client_handle_t client = mqtt_app_start(chipid);
        sntp_start();

        ESP_LOGI(TAG, "[APP] All init done, app_main, last line.");

        sprintf(statisticsTopic,"%s/%s/%x%x%x/statistics",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);
        ESP_LOGI(TAG,"statisticsTopic=[%s]", statisticsTopic);

        sprintf(otaUpdateTopic,"%s/%s/%x%x%x/otaupdate",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);

        sprintf(readTopic,"%s/%s/%x%x%x/setsetup",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);

        sprintf(dataTopic,"%s/%s/%x%x%x/data",
            comminfo->mqtt_prefix, appname, chipid[3],chipid[4],chipid[5]);

        program_version = ota_init(comminfo->mqtt_prefix, appname, chipid);
        prevStatsTs = 0;

        if (!statistics_init(comminfo->mqtt_prefix, appname, chipid))
        {
            ESP_LOGE(TAG,"failed in statistics init");
        }
        ESP_LOGI(TAG, "gpios: mqtt=%d wlan=%d",MQTTSTATUS_GPIO,WLANSTATUS_GPIO);

        while (1)
        {
            struct measurement meas;
            // several things should be running before we acknowledge the ota image is well behaving.
            time(&now);
            if ((now - statistics_getptr()->started > 20) &&
                (healthyflags == (HEALTHYFLAGS_WIFI | HEALTHYFLAGS_MQTT | HEALTHYFLAGS_NTP | HEALTHYFLAGS_TEMP)))
            {
                ota_cancel_rollback();
            }

            if (now > MIN_EPOCH)
            {
                if (statistics_getptr()->started < MIN_EPOCH)
                {
                    statistics_getptr()->started = now;
                }
                if (now - prevStatsTs >= STATISTICS_INTERVAL)
                {
                    if (isConnected)
                    {
                        statistics_send(client);
                        prevStatsTs = now;
                    }
                }
            }
            if(xQueueReceive(evt_queue, &meas, STATISTICS_INTERVAL * 1000 / portTICK_PERIOD_MS)) 
            {
                time(&now);
                uint16_t qcnt = uxQueueMessagesWaiting(evt_queue);
                if (qcnt > statistics_getptr()->maxQElements)
                {
                    statistics_getptr()->maxQElements = qcnt;
                }

                switch (meas.id) {
                    case TEMPERATURE:
                    {
                        char *sensorid = temperature_getsensor(meas.gpio);
                        if (sensorid != NULL)
                        {
                            if (setup.showinternaltemp)
                            {
                                ESP_LOGI(TAG,"comparing %s == %s", setup.specialsensor, sensorid );
                                if (!strcmp(setup.specialsensor, sensorid))
                                {
                                    show_internaltemp(meas.data.temperature);
                                }
                            }    
                            if (isConnected) 
                            {
                                temperature_send(comminfo->mqtt_prefix, &meas, client);
                            }    
                        }
                        healthyflags |= HEALTHYFLAGS_TEMP;
                    }
                    break;

                    case STATE:
                        if (isConnected) 
                        {
                            statistics_getptr()->sendcnt++;
                        }    
                    break;

                    case OTA:
                        ota_status_publish(&meas, client);
                        if (meas.data.count == 0)
                        {
                            rgb7seg_display("boot",default_color->c);
                            packetcount = 0;
                        }
                        else
                        {
                            char buff[6];
                            sprintf(buff,"%d",(int) (meas.data.count / 100));
                            if (packetcount == 2)
                            {
                                rgb7seg_display(buff,default_color->c);
                                packetcount = 0;
                            } 
                            else
                                packetcount++;
                        }
                    break;

                    default:
                        ESP_LOGD(TAG, "unknown data type" );
                }
            }
            else
            {   // timeout
                ESP_LOGI(TAG,"timeout");
            }
        }
    }
}


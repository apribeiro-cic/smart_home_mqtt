#include "pico/stdlib.h"            // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico

#include "hardware/gpio.h"          // Biblioteca de hardware de GPIO
#include "hardware/irq.h"           // Biblioteca de hardware de interrupções
#include "hardware/adc.h"           // Biblioteca de hardware para conversão ADC
#include "hardware/i2c.h"           // Biblioteca da Raspberry Pi Pico para manipulação do barramento I2C
#include "hardware/clocks.h"        // Biblioteca da Raspberry Pi Pico para manipulação de clocks
#include "hardware/timer.h"         // Biblioteca da Raspberry Pi Pico para manipulação de timers
#include "hardware/pwm.h"           // Biblioteca da Raspberry Pi Pico para manipulação de PWM
#include "pico/bootrom.h"           // Biblioteca para manipulação do bootloader USB

#include "lib/ssd1306.h"            // Biblioteca para controle do display OLED SSD1306
#include "lib/font.h"               // Biblioteca para manipulação de fontes no display OLED    
#include "lib/icons.h"              // Biblioteca para manipulação de ícones na matriz
#include "smart_home_mqtt.pio.h"         // Biblioteca para manipulação do PIO (Programmable Input/Output) da Raspberry Pi Pico

#include "lwip/apps/mqtt.h"         // Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h"    // Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"               // Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"         // Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

#define WIFI_SSID "SEU_SSID"                  // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "SEU_PASSORD_WIFI"      // Substitua pela senha da sua rede Wi-Fi
#define MQTT_SERVER "SEU_HOST"                // Substitua pelo endereço do host - broket MQTT: Ex: 192.168.1.107
#define MQTT_USERNAME "SEU_USERNAME_MQTT"     // Substitua pelo nome da host MQTT - Username
#define MQTT_PASSWORD "SEU_PASSWORD_MQTT"     // Substitua pelo Password da host MQTT - credencial de acesso - caso exista

// Definição da escala de temperatura
#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C' // Set to 'F' for Fahrenheit
#endif

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

//Dados do cliente MQTT
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

// Temporização da coleta de temperatura - how often to measure our temperature
#define TEMP_WORKER_TIME_S 10

// Manter o programa ativo - keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// QoS - mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Tópico usado para: last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

// Definição dos pinos dos LEDs
#define LED_PIN CYW43_WL_GPIO_LED_PIN   // GPIO do CI CYW43
#define LED_BLUE_PIN 12                 // GPIO12 - LED azul
#define LED_GREEN_PIN 11                // GPIO11 - LED verde
#define LED_RED_PIN 13                  // GPIO13 - LED vermelho
#define BTN_A 5                         // GPIO5 - Botão A
#define BTN_B 6                         // GPIO6 - Botão B
#define BTN_J 22                        // GPIO22 - Botão Joystick
#define JOYSTICK_X 26                   // Pino do Joystick X
#define BUZZER_A_PIN 10                 // GPIO10 - Buzzer A
#define BUZZER_B_PIN 21                 // GPIO21 - Buzzer B

uint32_t last_time = 0; // Variável para armazenar o tempo do último evento para o debouncing

#define I2C_PORT i2c1  // Define o barramento I2C
#define I2C_SDA 14     // Define o pino SDA
#define I2C_SCL 15     // Define o pino SCL
#define endereco 0x3C  // Endereço do display OLED

#define NUM_PIXELS 25   // Número de pixels da matriz de LEDs
#define MATRIX_PIN 7    // Pino da matriz de LEDs

ssd1306_t ssd; // Estrutura para o display OLED

bool light_state = false; // Variável para armazenar o estado da luz
bool alarm_state = false; // Variável para armazenar o estado do alarme
bool alarm_active = false; // Variável para armazenar o estado do alarme
bool lock_state = false; // Variável para armazenar o estado da fechadura
bool cooling_state = false; // Variável para armazenar o estado do ar-condicionado

// Variáveis para controle de cor e ícone exibido na matriz de LEDs
double red = 0.0, green = 255.0 , blue = 0.0; // Variáveis para controle de cor
int icon = 0; //Armazena o número atualmente exibido
double* icons[6] = {icon_zero, icon_one, icon_two, icon_three, icon_four, icon_five}; //Ponteiros para os desenhos dos números

float simulated_temp = 24.0; // Temperatura simulada para o ar-condicionado

//Leitura de temperatura do microcotrolador
static float read_onboard_temperature(const char unit);

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err);

// Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);

// Publicar temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state);

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err);

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err);

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state);

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

// Inicializar os Pinos GPIO da BitDogLab
static void gpio_bitdog(void);

// Função de callback para tratamento de interrupção dos botões
static void gpio_irq_handler(uint gpio, uint32_t events);

// Função para desenhar uma tela no display OLED
static void draw_screen(void);

// Função para configurar o PWM 
void pwm_setup_gpio(uint gpio, uint freq);

// Rotina para definição da intensidade de cores do led
uint32_t matrix_rgb(double r, double g, double b);

// Rotina para acionar a matrix de leds - ws2812b
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b);

int main(void) {

    // Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
    stdio_init_all();
    INFO_printf("mqtt client starting\n");

    // Inicializa os pinos GPIO da BitDogLab
    gpio_bitdog();

    // Configura os pinos de interrupção para os botões
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BTN_J, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Inicializa o conversor ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
    adc_gpio_init(JOYSTICK_X); // Inicializa o pino do Joystick X para leitura de temperatura

    //Configurações da PIO
    PIO pio = pio0; 
    uint offset = pio_add_program(pio, &pio_matrix_program);
    uint sm = pio_claim_unused_sm(pio, true);
    pio_matrix_program_init(pio, sm, offset, MATRIX_PIN);

    // Inicializa o I2C e configura os pinos SDA e SCL para o display OLED 
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa e configura o display OLED
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); 
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false); // Limpa o display
    ssd1306_send_data(&ssd); // Envia os dados para o display

    // Cria registro com os dados do cliente
    static MQTT_CLIENT_DATA_T state;

    // Inicializa a arquitetura do cyw43
    if (cyw43_arch_init()) {
        panic("Failed to inizialize CYW43");
    }

    // Usa identificador único da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Gera nome único, Ex: pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS enabled
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    // This confirms the indentity of the server and the client
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
            client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif
#endif

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect");
    }
    INFO_printf("\nConnected to Wifi\n");

    //Faz um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente
    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        panic("dns request failed");
    }

    // Loop condicionado a conexão mqtt
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        cyw43_arch_poll();
        adc_select_input(0); 
        uint16_t adc_value = adc_read();
        uint32_t current_time = to_us_since_boot(get_absolute_time()); // Pega o tempo atual em ms
        if (alarm_state == true && current_time - last_time > 1000000 && !alarm_active) { gpio_put(LED_RED_PIN, !gpio_get(LED_RED_PIN)); last_time = current_time; } // Debouncing de 1000ms

        if (alarm_state && (adc_value > 2500 || adc_value < 1500) && !alarm_active) { // Se o alarme estiver ativo e o valor do ADC for maior que 2500
            alarm_active = true; // Ativa o alarme
        }

        if (alarm_active && alarm_state) {
            gpio_put(LED_RED_PIN, 1); // Liga o LED vermelho
            pwm_setup_gpio(BUZZER_A_PIN, 1000); // Ativa o buzzer A
            sleep_ms(800); // Aguarda 1 segundo
            pwm_setup_gpio(BUZZER_A_PIN, 0); // Desativa o buzzer A
        }

        desenho_pio(icons[icon], 0, pio, sm, red, green, blue); // Desenha o ícone na matriz de LEDs
        draw_screen(); // Atualiza a tela do display OLED

        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));
    }

    INFO_printf("mqtt client exiting\n");
    return 0;
}

// Função para ler a temperatura do microcontrolador
static float read_onboard_temperature(const char unit) {
    adc_select_input(4); // Select the temperature sensor input (ADC4)
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C' || unit != 'F') {
        if (cooling_state) return simulated_temp; // Retorna a temperatura simulada se o ar-condicionado estiver ligado
        return tempC;
    } else if (unit == 'F') {
        if (cooling_state) return simulated_temp; // Retorna a temperatura simulada se o ar-condicionado estiver ligado
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        ERROR_printf("pub_request_cb failed %d", err);
    }
}

//Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    // Publish state on /state topic and on/off led board
    const char* message = on ? "On" : "Off";
    if (on)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

// Publicar temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temperature");
    float temperature = read_onboard_temperature(TEMPERATURE_UNITS);
    if (temperature != old_temperature) {
        old_temperature = temperature;
        // Publish temperature on /temperature topic
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        INFO_printf("Publishing %s to %s\n", temp_str, temperature_key);
        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;
}

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/light/set"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/alarm/set"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/lock/set"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/cooling/set"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/cooling/temp"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);
    if (strcmp(basic_topic, "/led") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);

    } else if (strcmp(basic_topic, "/print") == 0) {
        INFO_printf("%.*s\n", len, data);

    } else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);

    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true; // stop the client when ALL subscriptions are stopped
        sub_unsub_topics(state, false); // unsubscribe

    } else if (strcmp(basic_topic, "/light/set") == 0) {
        // payload aceitável: "1"/"0" ou "On"/"Off"
        if ( (lwip_stricmp(state->data, "on") == 0) || (strcmp(state->data, "1") == 0) ) {
            light_state = true;
            gpio_put(LED_BLUE_PIN, 1);
            gpio_put(LED_GREEN_PIN, 1);
            gpio_put(LED_RED_PIN, 1);
        }
        else if ( (lwip_stricmp(state->data, "off") == 0) || (strcmp(state->data, "0") == 0) ) {
            light_state = false;
            gpio_put(LED_BLUE_PIN, 0);
            gpio_put(LED_GREEN_PIN, 0);
            gpio_put(LED_RED_PIN, 0);
        }
        // publica em "/light/state" o novo valor:
        const char *light_state_topic = full_topic(state, "/light/state");
        const char *payload = light_state ? "1" : "0";
        mqtt_publish(state->mqtt_client_inst, light_state_topic, payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);

    } else if (strcmp(basic_topic, "/alarm/set") == 0) {
        if ( (lwip_stricmp(state->data, "on") == 0) || (strcmp(state->data, "1") == 0) ) {
            pwm_setup_gpio(BUZZER_A_PIN, 659);
            sleep_ms(500);
            pwm_setup_gpio(BUZZER_A_PIN, 987);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_A_PIN, 0);
            alarm_state = true;
        }
        else if ( (lwip_stricmp(state->data, "off") == 0) || (strcmp(state->data, "0") == 0) ) {
            pwm_setup_gpio(BUZZER_A_PIN, 987);
            sleep_ms(500);
            pwm_setup_gpio(BUZZER_A_PIN, 659);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_A_PIN, 0);
            alarm_state = false;
            alarm_active = false;
            gpio_put(LED_RED_PIN, 0);
        }
        // publica em "/alarm/state" o novo valor:
        const char *alarm_state_topic = full_topic(state, "/alarm/state");
        const char *payload = alarm_state ? "1" : "0";
        mqtt_publish(state->mqtt_client_inst, alarm_state_topic, payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);

    } else if (strcmp(basic_topic, "/lock/set") == 0) {
        if ( (lwip_stricmp(state->data, "on") == 0) || (strcmp(state->data, "1") == 0) ) {
            red = 255.0; green = 0.0; blue = 0.0;
            pwm_setup_gpio(BUZZER_B_PIN, 1046);
            sleep_ms(150);
            pwm_setup_gpio(BUZZER_B_PIN, 783);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 523);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 0);
            icon = 1;
            lock_state = true;
        }
        else if ( (lwip_stricmp(state->data, "off") == 0) || (strcmp(state->data, "0") == 0) ) {
            red = 0.0; green = 255.0; blue = 0.0;
            pwm_setup_gpio(BUZZER_B_PIN, 523);
            sleep_ms(150);
            pwm_setup_gpio(BUZZER_B_PIN, 783);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 1046);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 0);
            icon = 0;
            lock_state = false;
        }
        // publica em "/lock/state" o novo valor:
        const char *lock_state_topic = full_topic(state, "/lock/state");
        const char *payload = lock_state ? "1" : "0";
        mqtt_publish(state->mqtt_client_inst, lock_state_topic, payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);

    } else if (strcmp(basic_topic, "/cooling/set") == 0) {
        if ( (lwip_stricmp(state->data, "on") == 0) || (strcmp(state->data, "1") == 0) ) {
            cooling_state = true;
            pwm_setup_gpio(BUZZER_B_PIN, 880);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 587);
            sleep_ms(200);
            pwm_setup_gpio(BUZZER_B_PIN, 0);
            sleep_ms(125);
            pwm_setup_gpio(BUZZER_B_PIN, 587);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 880);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 0);
        }
        else if ( (lwip_stricmp(state->data, "off") == 0) || (strcmp(state->data, "0") == 0) ) {
            cooling_state = false;
            pwm_setup_gpio(BUZZER_B_PIN, 587);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 880);
            sleep_ms(200);
            pwm_setup_gpio(BUZZER_B_PIN, 0);
            sleep_ms(125);
            pwm_setup_gpio(BUZZER_B_PIN, 880);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 587);
            sleep_ms(250);
            pwm_setup_gpio(BUZZER_B_PIN, 0);
        }
        // publica em "/cooling/state" o novo valor:
        const char *cooling_state_topic = full_topic(state, "/cooling/state");
        const char *payload = cooling_state ? "1" : "0";
        mqtt_publish(state->mqtt_client_inst, cooling_state_topic, payload, strlen(payload), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "/cooling/temp") == 0) {
        // Converte payload em float
        float nova_temp = atof(state->data);
        simulated_temp = nova_temp;  // atualiza variável global
    }

}

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    publish_temperature(state);
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 100);
}

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publish temperature every 10 sec if it's changed
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) {
            panic("Failed to connect to mqtt server");
        }
    }
    else {
        panic("Unexpected status");
    }
}

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state) {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        panic("MQTT client instance creation error");
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        panic("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        panic("dns request failed");
    }
}

// Inicializar os Pinos GPIO da BitDogLab
void gpio_bitdog(void){
    // Configuração dos LEDs como saída
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, false);
    
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, false);
    
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, false);

    gpio_init(BTN_A);
    gpio_set_dir(BTN_A, GPIO_IN);
    gpio_pull_up(BTN_A);

    gpio_init(BTN_B);
    gpio_set_dir(BTN_B, GPIO_IN);
    gpio_pull_up(BTN_B);

    gpio_init(BTN_J);
    gpio_set_dir(BTN_J, GPIO_IN);
    gpio_pull_up(BTN_J);

    gpio_init(BUZZER_A_PIN);  
    gpio_set_dir(BUZZER_A_PIN, GPIO_OUT);
    gpio_init(BUZZER_B_PIN);  
    gpio_set_dir(BUZZER_B_PIN, GPIO_OUT);
}

// Função de callback para tratamento de interrupção dos botões
void gpio_irq_handler(uint gpio, uint32_t events) { 
    uint32_t current_time = to_us_since_boot(get_absolute_time()); // Pega o tempo atual em ms
    if (current_time - last_time > 250000) { // Debouncing de 250ms
        last_time = current_time;
        if (gpio == BTN_A) { // Verifica se o botão A foi pressionado 
            printf("Botão A pressionado!\n");
        } else if (gpio == BTN_B) { // Verifica se o botão B foi pressionado
            printf("Botão B pressionado!\n");
        } else if (gpio == BTN_J) { // Verifica se o botão do joystick foi pressionado e entra no modo bootsel
            printf("Botão do joystick pressionado!\n");
            reset_usb_boot(0, 0);
        }
    }
}

// Função para desenhar uma tela no display OLED
void draw_screen() {
    ssd1306_fill(&ssd, false); // Limpa o display
    char buffer[32]; // Buffer para armazenar valores de temperatura como string

    ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
    ssd1306_rect(&ssd, 0, 0, 128, 12, true, false);  
    ssd1306_draw_string(&ssd, "SMART HOME", 25, 2); // Desenha o texto "EMBARCATECH" na posição (0, 0)

    if (alarm_state) { ssd1306_draw_string(&ssd, "Alarme: ON", 2, 14); } else { ssd1306_draw_string(&ssd, "Alarme: OFF", 2, 14); }
    if (lock_state) { ssd1306_draw_string(&ssd, "Fechadura: ON", 2, 24); } else { ssd1306_draw_string(&ssd, "Fechadura: OFF", 2, 24); }
    if (cooling_state) { ssd1306_draw_string(&ssd, "A/C: ON", 2, 34); } else { ssd1306_draw_string(&ssd, "A/C: OFF", 2, 34); }

    ssd1306_draw_string(&ssd, "Temp: ", 2, 44); 
    sprintf(buffer, "%.1f", cooling_state ? simulated_temp : read_onboard_temperature(TEMPERATURE_UNITS)); // Formata a temperatura como string
    ssd1306_draw_string(&ssd, buffer, 48, 44);
    
    ssd1306_send_data(&ssd); // Envia os dados para o display
}

// Função para configurar o PWM
void pwm_setup_gpio(uint gpio, uint freq) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);  // Define o pino como saída PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);  // Obtém o slice do PWM

    if (freq == 0) {
        pwm_set_enabled(slice_num, false);  // Desabilita o PWM
        gpio_put(gpio, 0);  // Desliga o pino
    } else {
        uint32_t clock_div = 4; // Define o divisor do clock
        uint32_t wrap = (clock_get_hz(clk_sys) / (clock_div * freq)) - 1; // Calcula o valor de wrap

        // Configurações do PWM (clock_div, wrap e duty cycle) e habilita o PWM
        pwm_set_clkdiv(slice_num, clock_div); 
        pwm_set_wrap(slice_num, wrap);  
        pwm_set_gpio_level(gpio, wrap / 5);
        pwm_set_enabled(slice_num, true);  
    }
}

// Rotina para definição da intensidade de cores do led
uint32_t matrix_rgb(double r, double g, double b) {
    unsigned char R, G, B;
    R = r * red;
    G = g * green;
    B = b * blue;
    return (G << 24) | (R << 16) | (B << 8);
}

// Rotina para acionar a matrix de leds - ws2812b
void desenho_pio(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b) {
    for (int16_t i = 0; i < NUM_PIXELS; i++) {
        valor_led = matrix_rgb(desenho[24-i], desenho[24-i], desenho[24-i]);
        pio_sm_put_blocking(pio, sm, valor_led);
    }
}
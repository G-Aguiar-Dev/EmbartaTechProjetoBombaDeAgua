//-------------------------------------------Bibliotecas-------------------------------------------
#include <stdio.h>                  // Biblioteca padrão de entrada e saída
#include <string.h>                 // Biblioteca padrão de manipulação de strings
#include <ctype.h>                  // Biblioteca padrão de manipulação de caracteres

#include "pico/stdlib.h"            // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico

#include "FreeRTOS.h"               // Biblioteca de FreeRTOS
#include "task.h"                   // Biblioteca de tasks

#include "lwip/tcp.h"               // Biblioteca de LWIP para manipulação de TCP/IP

#include "hardware/gpio.h"          // Biblioteca de hardware de GPIO
#include "hardware/irq.h"           // Biblioteca de hardware de interrupções
#include "hardware/adc.h"           // Biblioteca de hardware para conversão ADC
#include "hardware/pio.h"           // Biblioteca de PIO
#include "hardware/clocks.h"        // Biblioteca de clocks
#include "hardware/pwm.h"           // Biblioteca de hardware para manipulação do PWM

#include "matriz_LED.pio.h"         // Biblioteca gerada pelo PIO para manipulação de uma matriz de LEDs
#include "ssd1306.h"                // Biblioteca para manipulação de displays OLED SSD1306
#include "font.h"                   // Biblioteca de fontes para o display OLED

//-------------------------------------------Definições-------------------------------------------
#define WIFI_SSID "Seu SSID"
#define WIFI_PASS "Sua Senha"

#define LED_PIN_BLUE 12
#define BOTAO_A 5
#define BOTAO_JOY 22
#define JOYSTICK_X 26
#define JOYSTICK_Y 27
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C

//-------------------------------------------Variáveis Globais-------------------------------------------

ssd1306_t ssd;                          // Variável para o display LCD 
static float r = 0.0, b = 0.0, g = 0.0; // Variáveis para controlar a cor dos LEDs

double led_buffer[25][3] = {0};         // Buffer para armazenar o estado dos LEDs

double apagar_leds[25][3] =             // Apagar LEDs da matriz
 {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

struct http_state
{
    char response[4096];
    size_t len;
    size_t sent;
};

//-------------------------------------------HTML-------------------------------------------
const char HTML_BODY[] =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Controle do LED</title>"
    "<style>"
    "body { font-family: sans-serif; text-align: center; padding: 10px; margin: 0; background: #f9f9f9; }"
    ".botao { font-size: 20px; padding: 10px 30px; margin: 10px; border: none; border-radius: 8px; }"
    ".on { background: #4CAF50; color: white; }"
    ".off { background: #f44336; color: white; }"
    ".barra { width: 30%; background: #ddd; border-radius: 6px; overflow: hidden; margin: 0 auto 15px auto; height: 20px; }"

    ".preenchimento { height: 100%; transition: width 0.3s ease; }"
    "#barra_x { background: #2196F3; }"
    "#barra_y { background:rgb(177, 96, 153); }"
    ".label { font-weight: bold; margin-bottom: 5px; display: block; }"
    ".bolinha { width: 20px; height: 20px; border-radius: 50%; display: inline-block; margin-left: 10px; background: #ccc; transition: background 0.3s ease; }"
    "@media (max-width: 600px) { .botao { width: 80%; font-size: 18px; } }"
    "</style>"
    "<script>"
    "function sendCommand(cmd) { fetch('/led/' + cmd); }"
    "function atualizar() {"
    "  fetch('/estado').then(res => res.json()).then(data => {"
    "    document.getElementById('estado').innerText = data.led ? 'Ligado' : 'Desligado';"
    "    document.getElementById('x_valor').innerText = data.x;"
    "    document.getElementById('y_valor').innerText = data.y;"
    "    document.getElementById('botao').innerText = data.botao ? 'Pressionado' : 'Solto';"
    "    document.getElementById('joy').innerText = data.joy ? 'Pressionado' : 'Solto';"
    "    document.getElementById('bolinha_a').style.background = data.botao ? '#2126F3' : '#ccc';"
    "    document.getElementById('bolinha_joy').style.background = data.joy ? '#4C7F50' : '#ccc';"
    "    document.getElementById('barra_x').style.width = Math.round(data.x / 4095 * 100) + '%';"
    "    document.getElementById('barra_y').style.width = Math.round(data.y / 4095 * 100) + '%';"
    "  });"
    "}"
    "setInterval(atualizar, 1000);"
    "</script></head><body>"

    "<h1>Controle do LED</h1>"

    "<p>Estado do LED: <span id='estado'>--</span></p>"

    "<p class='label'>Joystick X: <span id='x_valor'>--</span></p>"
    "<div class='barra'><div id='barra_x' class='preenchimento'></div></div>"

    "<p class='label'>Joystick Y: <span id='y_valor'>--</span></p>"
    "<div class='barra'><div id='barra_y' class='preenchimento'></div></div>"

    "<p class='label'>Botão A: <span id='botao'>--</span> <span id='bolinha_a' class='bolinha'></span></p>"
    "<p class='label'>Botão do Joystick: <span id='joy'>--</span> <span id='bolinha_joy' class='bolinha'></span></p>"

    "<button class='botao on' onclick=\"sendCommand('on')\">Ligar</button>"
    "<button class='botao off' onclick=\"sendCommand('off')\">Desligar</button>"

    "<hr style='margin-top: 20px;'>"
    "<p style='font-size: 15px; color: #336699; font-style: italic; max-width: 90%; margin: 10px auto;'>"
    "Utilização da BitDogLab para exemplificar a comunicação via rede Wi-Fi utilizando o protocolo HTML com JavaScript"
    "</p>"

    "</body></html>";
//---------------------------------------------Protótipos---------------------------------------------

void setup(void);

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);

static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);

static void start_http_server(void);

void pwm_setup(uint8_t GPIO);

uint matrix_rgb(float r, float g, float b);

void desenho_pio(double desenho[25][3], uint32_t valor_led, PIO pio, uint sm);

//-----------------------------------------------Tasks------------------------------------------------

// Task de polling para manter a conexão Wi-Fi ativa
void vPollingTask(void *pvParameters)
{
    while (true)
    {
        cyw43_arch_poll();      // Polling do Wi-Fi para manter a conexão ativa
        vTaskDelay(1000);       // Aguarda 1 segundo antes de repetir
    }
}


//------------------------------------------------MAIN------------------------------------------------
void main(void)
{
    stdio_init_all();               // Inicializa a saída padrão (UART)
    sleep_ms(2000);                 // Aguarda 2 segundos para estabilização

    setup();                        // Configurações iniciais

    if (cyw43_arch_init())          // Inicializa o Wi-fi
    {
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi => FALHA", 0, 0);
        ssd1306_send_data(&ssd);
        return 1;
    }

    cyw43_arch_enable_sta_mode();   // Habilita o modo Station do Wi-Fi
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi => ERRO", 0, 0);
        ssd1306_send_data(&ssd);
        return 1;
    }

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[24];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "WiFi => OK", 0, 0);
    ssd1306_draw_string(&ssd, ip_str, 0, 10);
    ssd1306_send_data(&ssd);

    start_http_server();            // Inicia o servidor HTTP

    //Tasks
    xTaskCreate(vPollingTask, "Polling Task", 256, NULL, 1, NULL); 

    vTaskStartScheduler();          // Inicia o escalonador do FreeRTOS
    panic_unsupported();            // Se o escalonador falhar, entra em pânico
}

//----------------------------------------------Funções------------------------------------------------
// Função de configuração inicial
void setup(void){
    
    gpio_init(LED_PIN_BLUE);
    gpio_set_dir(LED_PIN_BLUE, GPIO_OUT);

    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);

    gpio_init(BOTAO_JOY);
    gpio_set_dir(BOTAO_JOY, GPIO_IN);
    gpio_pull_up(BOTAO_JOY);

    adc_init();
    adc_gpio_init(JOYSTICK_X);
    adc_gpio_init(JOYSTICK_Y);

    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Iniciando Wi-Fi", 0, 0);
    ssd1306_draw_string(&ssd, "Aguarde...", 0, 30);    
    ssd1306_send_data(&ssd);
}

// Função de callback para enviar dados HTTP
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len)
    {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

// Função de callback para receber dados HTTP
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p)
    {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs)
    {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;

    if (strstr(req, "GET /led/on"))
    {
        gpio_put(LED_PIN_BLUE, 1);
        const char *txt = "Ligado";
        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/plain\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(txt), txt);
    }
    else if (strstr(req, "GET /led/off"))
    {
        gpio_put(LED_PIN_BLUE, 0);
        const char *txt = "Desligado";
        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/plain\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(txt), txt);
    }
    else if (strstr(req, "GET /estado"))
    {
        adc_select_input(0);
        uint16_t x = adc_read();
        adc_select_input(1);
        uint16_t y = adc_read();
        int botao = !gpio_get(BOTAO_A);
        int joy = !gpio_get(BOTAO_JOY);

        char json_payload[96];
        int json_len = snprintf(json_payload, sizeof(json_payload),
                                "{\"led\":%d,\"x\":%d,\"y\":%d,\"botao\":%d,\"joy\":%d}\r\n",
                                gpio_get(LED_PIN_BLUE), x, y, botao, joy);

        printf("[DEBUG] JSON: %s\n", json_payload);

        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           json_len, json_payload);
    }
    else
    {
        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(HTML_BODY), HTML_BODY);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);

    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    pbuf_free(p);
    return ERR_OK;
}

// Função de callback para aceitar novas conexões TCP
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

// Função para iniciar o servidor HTTP
static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        printf("Erro ao criar PCB TCP\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}

// Configura PWM para um GPIO específico
void pwm_setup(uint8_t GPIO) {
  gpio_set_function(GPIO, GPIO_FUNC_PWM); // Define função PWM para o pino
  uint slice_num = pwm_gpio_to_slice_num(GPIO); // Obtém o número do slice
  pwm_config config = pwm_get_default_config(); // Configuração padrão
  pwm_config_set_wrap(&config, 4095); // Wrap em 4095 para 12 bits
  pwm_init(slice_num, &config, true); // Inicializa PWM
}

// Função para converter RGB em um valor de 32 bits
uint matrix_rgb(float r, float g, float b) 
{
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// Função para desenhar na matriz
#define NUM_PIXELS 25
void desenho_pio(double desenho[25][3], uint32_t valor_led, PIO pio, uint sm)
{

  for (int16_t i = 0; i < NUM_PIXELS; i++)
  {
    valor_led = matrix_rgb(desenho[i][0], desenho[i][1], desenho[i][2]);
    pio_sm_put_blocking(pio, sm, valor_led);
  };
}
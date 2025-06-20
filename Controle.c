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
#include "queue.h"                  // Biblioteca de FreeRTOS para manipulação de filas

#include "matriz_LED.pio.h"         // Biblioteca gerada pelo PIO para manipulação de uma matriz de LEDs
#include "ssd1306.h"                // Biblioteca para manipulação de displays OLED SSD1306
#include "font.h"                   // Biblioteca de fontes para o display OLED

//-------------------------------------------Definições-------------------------------------------
#define WIFI_SSID "Malu"
#define WIFI_PASS "11042006!"

#define BOMBA 99 // Temporário
#define LED_PIN_GREEN 11
#define LED_PIN_BLUE 12
#define LED_PIN_RED 13
#define BOTAO_A 5
#define BOTAO_B 6
#define BOTAO_JOY 22
#define JOYSTICK_X 26
#define JOYSTICK_Y 27
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C
#define BUZZER_PIN 21
#define PIXELS 25
#define SENSOR_NIVEL 28   // Pino ADC conectado ao potenciômetro da boia
#define LED_MATRIX 7 // Pino GPIO conectado à matriz de LEDs

//-------------------------------------------Variáveis Globais-------------------------------------------

ssd1306_t ssd;                                      // Variável para o display LCD 
static volatile float r = 0.0, b = 0.0, g = 0.0;    // Variáveis para controlar a cor dos LEDs
static volatile uint8_t volume_agua = 0;            // Variável para armazenar o volume de água (0-100%)
volatile bool estado_display = false;               // Estado do display OLED
volatile bool estado_bomba = false;                 // Estado do LED
PIO pio = pio0;
uint sm;
double led_buffer[25][3] = {0};                     // Buffer para armazenar o estado dos LEDs

double apagar_leds[25][3] =                         // Apagar LEDs da matriz
 {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

  
double COORDENADA_NIVEL_0[PIXELS][3] = {
    {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}
};

double COORDENADA_NIVEL_1[PIXELS][3] = {
    {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}
};

double COORDENADA_NIVEL_2[PIXELS][3] = {
    {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}
};

double COORDENADA_NIVEL_3[PIXELS][3] = {
    {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0},
    {1, 0, 0}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {1, 0, 0}
};

struct http_state
{
    char response[4096];
    size_t len;
    size_t sent;
};

QueueHandle_t xFilaNivel; // Fila para leitura de nível do reservatório

//-------------------------------------------HTML-------------------------------------------
const char HTML_BODY[] =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Sistema de Monitoramento de Água</title>"
    "<style>"
    "body { font-family: sans-serif; text-align: center; padding: 10px; margin: 0; background: #f9f9f9; }"
    ".botao { font-size: 20px; padding: 10px 30px; margin: 10px; border: none; border-radius: 8px; }"
    ".on { background: #4CAF50; color: white; }"
    ".off { background: #f44336; color: white; }"
    ".barra { width: 30%; background: #ddd; border-radius: 6px; overflow: hidden; margin: 0 auto 15px auto; height: 20px; }"

    ".preenchimento { height: 100%; transition: width 0.3s ease; }"
    "#barra_x { background: #2196F3; }"
    ".label { font-weight: bold; margin-bottom: 5px; display: block; }"
    ".bolinha { width: 20px; height: 20px; border-radius: 50%; display: inline-block; margin-left: 10px; background: #ccc; transition: background 0.3s ease; }"
    "@media (max-width: 600px) { .botaoA { width: 80%; font-size: 18px; } } { .botaoB { width: 80%; font-size: 18px; } }"
    "</style>"
    "<script>"
    "function sendCommand(cmd) { fetch('/bomba/' + cmd); }"
    "function atualizar() {"
    "  fetch('/estado').then(res => res.json()).then(data => {"
    "    document.getElementById('estado').innerText = data.bomba ? 'Ligado' : 'Desligado';"
    "    document.getElementById('x_valor').innerText = data.x;"
    "    document.getElementById('botaoA').innerText = data.botaoA ? 'Ligado' : 'Desligado';"
    "    document.getElementById('botaoB').innerText = data.botaoB ? 'Ligado' : 'Desligado';"
    "    document.getElementById('joy').innerText = data.joy ? 'Ligado' : 'Desligado';"
    "    document.getElementById('bolinha_a').style.background = data.botaoA ? '#2126F3' : '#ccc';"
    "    document.getElementById('bolinha_b').style.background = data.botaoB ? '#2126F3' : '#ccc';"
    "    document.getElementById('bolinha_joy').style.background = data.joy ? '#2126F3' : '#ccc';"
    "    document.getElementById('barra_x').style.width = Math.round(data.x / 4095 * 100) + '%';"
    "  });"
    "}"
    "setInterval(atualizar, 1000);"
    "</script></head><body>"

    "<h1>Controle da Bomba</h1>"

    "<p>Estado da Bomba: <span id='estado'>--</span></p>"

    "<p class='label'>Nível de Água no Reservatório: <span id='x_valor'>--</span></p>"
    "<div class='barra'><div id='barra_x' class='preenchimento'></div></div>"

    "<p class='label'>Botão A (Mudar Infos Display): <span id='botaoA'>--</span> <span id='bolinha_a' class='bolinha'></span></p>"
    "<p class='label'>Botão B: <span id='botaoB'>--</span> <span id='bolinha_b' class='bolinha'></span></p>"
    "<p class='label'>Botão do Joystick (Acionamento da Bomba): <span id='joy'>--</span> <span id='bolinha_joy' class='bolinha'></span></p>"
    "<button class='botao on' onclick=\"sendCommand('on')\">Ligar</button>"
    "<button class='botao off' onclick=\"sendCommand('off')\">Desligar</button>"

    "<hr style='margin-top: 20px;'>"
    "<p style='font-size: 15px; color: #336699; font-style: italic; max-width: 90%; margin: 10px auto;'>"
    "Sistema de Controle de Bomba de Água e Monitoramento de Nível de Reservatório<br>"
    "</p>"

    "</body></html>";
//---------------------------------------------Protótipos---------------------------------------------

// Função de configuração inicial
void setup(void);

void acionar_bomba();

// Função de callback para enviar dados HTTP
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);

// Função de callback para receber dados HTTP
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

// Função de callback para aceitar novas conexões TCP
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);

// Função para iniciar o servidor HTTP
static void start_http_server(void);

// Configura PWM para um GPIO específico
void pwm_setup(uint8_t GPIO);

// Função para converter RGB em um valor de 32 bits
uint matrix_rgb(float r, float g, float b);

// Função para desenhar na matriz
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

// Task para exibir informações no display LCD
void vDisplayTask(void *pvParameters)
{
    bool cor = true;                                                        // Variável para alternar a cor do display
    char *ip_str = (char *)pvParameters;                                    // Recebe o IP como parâmetro
    char volume_str[8];
    snprintf(volume_str, sizeof(volume_str), "%d", *(int *)(&volume_agua));

    while (true)
    {
        if (estado_display)     // Verifica se a flag está ativa, exibe informações sobre a rede
        {
            ssd1306_fill(&ssd, !cor);
            ssd1306_line(&ssd, 0, 0, WIDTH - 1, 0, cor);                    // Linha no topo
            ssd1306_line(&ssd, 0, HEIGHT - 1, WIDTH - 1, HEIGHT - 1, cor);  // Linha na base
            ssd1306_line(&ssd, 0, 0, 0, HEIGHT - 1, cor);                   // Linha na esquerda
            ssd1306_line(&ssd, WIDTH - 1, 0, WIDTH - 1, HEIGHT - 1, cor);   // Linha na direita
            ssd1306_line(&ssd, 0, 12, WIDTH - 1, 12, cor);                  // Linha horizontal
            ssd1306_line(&ssd, 0, 29, WIDTH - 1, 29, cor);                  // Linha horizontal
            ssd1306_line(&ssd, 0, 43, WIDTH - 1, 43, cor);                  // Linha horizontal
            ssd1306_draw_string(&ssd, "WiFi:", 5, 3);
            ssd1306_draw_string(&ssd, WIFI_SSID, 5, 19);
            ssd1306_draw_string(&ssd, "Endereco IP:", 5, 33);
            ssd1306_draw_string(&ssd, ip_str, 5, 52);
            ssd1306_send_data(&ssd);

            vTaskDelay(1000);                                               // Aguarda 1s para atualizar o display
        }
        else                    // Se a flag estiver desativada, exibe o estado da bomba e o volume de água
        {
            ssd1306_fill(&ssd, !cor);
            ssd1306_line(&ssd, 0, 0, WIDTH - 1, 0, cor);                    // Linha no topo
            ssd1306_line(&ssd, 0, HEIGHT - 1, WIDTH - 1, HEIGHT - 1, cor);  // Linha na base
            ssd1306_line(&ssd, 0, 0, 0, HEIGHT - 1, cor);                   // Linha na esquerda
            ssd1306_line(&ssd, WIDTH - 1, 0, WIDTH - 1, HEIGHT - 1, cor);   // Linha na direita
            ssd1306_line(&ssd, 0, 12, WIDTH - 1, 12, cor);                  // Linha horizontal
            ssd1306_line(&ssd, 0, 29, WIDTH - 1, 29, cor);                  // Linha horizontal
            ssd1306_line(&ssd, 0, 43, WIDTH - 1, 43, cor);                  // Linha horizontal
            ssd1306_draw_string(&ssd, "Est. da Bomba:", 5, 3);
            if (estado_bomba)   // Se a bomba estiver ligada, exibe "Ligada", caso contrário, exibe "Desligada"
            {
                ssd1306_draw_string(&ssd, "Ligada", 5, 19);
            }
            else
            {
                ssd1306_draw_string(&ssd, "Desligada", 5, 19);
            }
            ssd1306_draw_string(&ssd, "Vol. de Agua:", 5, 33); 
            ssd1306_draw_string(&ssd, volume_str, 5, 52);                   // Exibe o volume de água em porcentagem
            ssd1306_draw_string(&ssd, "/100", 22, 52);
            ssd1306_send_data(&ssd);

            vTaskDelay(200);                                                // Aguarda 200 ms para atualizar o display
        }
    }
}

void vLeituraNivelTask(void *pvParameters) {
    adc_select_input(2); // Canal 2 = GPIO28
    while (1) {
        volume_agua = adc_read() / 4095.0 * 100; // Lê o valor do ADC e converte para porcentagem (0-100%)
        vTaskDelay(pdMS_TO_TICKS(100)); // Leitura a cada 100 ms
    }
}

void vLedsRGBTask(void *pvParameters) {
    while (1) {
        if (volume_agua <= 40) { // Nível Baixo - Verde
            gpio_put(LED_PIN_GREEN, 1);
            gpio_put(LED_PIN_RED, 0);
        } else if (volume_agua <= 80) { // Nível Médio - Amarelo
            gpio_put(LED_PIN_GREEN, 1);
            gpio_put(LED_PIN_RED, 1);
        } else { // Nível Alto - Vermelho (>80%)
            gpio_put(LED_PIN_GREEN, 0);
            gpio_put(LED_PIN_RED, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Aguarda 100 ms antes da próxima verificação
    }
}

/* Tarefa para tocar o buzzer com pwm */
void vBuzzerTask()
{
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint chan = pwm_gpio_to_channel(BUZZER_PIN);
    uint wrap = 125000000 / 3500; // Frequência base: 3.5kHz (ajuste conforme o buzzer)

    pwm_set_wrap(slice, wrap);
    pwm_set_enabled(slice, true);

    while (true)
    {
        adc_select_input(2);  // Canal do sensor de nível de água
        uint16_t nivel_da_agua = adc_read();  // Valor de 0 a 4095

        // Converta para percentual
        float percentual = (nivel_da_agua / 4095.0f) * 100.0f;

        if (percentual < 30.0f)
        {
            pwm_set_gpio_level(BUZZER_PIN, 0);  // Buzzer desligado
            vTaskDelay(pdMS_TO_TICKS(500));     // Espera meio segundo
        }
        else if (percentual < 60.0f)
        {
            pwm_set_gpio_level(BUZZER_PIN, wrap / 2);  // Liga o buzzer
            vTaskDelay(pdMS_TO_TICKS(500));            // Liga por 500ms
            pwm_set_gpio_level(BUZZER_PIN, 0);         // Desliga
            vTaskDelay(pdMS_TO_TICKS(500));            // Espera
        }
        else if (percentual < 90.0f)
        {
            pwm_set_gpio_level(BUZZER_PIN, wrap / 2);  // Liga
            vTaskDelay(pdMS_TO_TICKS(100));            // Liga por 100ms
            pwm_set_gpio_level(BUZZER_PIN, 0);         // Desliga
            vTaskDelay(pdMS_TO_TICKS(100));            // Espera
        }
        else
        {
            pwm_set_gpio_level(BUZZER_PIN, wrap / 2);  // Liga o buzzer contínuo
            vTaskDelay(pdMS_TO_TICKS(100));            // Mantém
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Aguarda 100 ms antes da próxima leitura
    }
}

void vBotaoBombaTask(void *pvParameters)
{
    while (true)
    {
        acionar_bomba();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void vButton_task() {
    while (true) {
        if (!gpio_get(BOTAO_A)) {
            estado_display = !estado_display;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (!gpio_get(BOTAO_B)) {
            estado_bomba = !estado_bomba;
             vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vMatriz_led_task() {
/*
    sm = pio_claim_unused_sm(pio, true); // Requisita um estado de máquina livre
    matriz_LED_program_init(pio, sm, 0, 0, 1); // Inicializa o PIO com o programa da matriz LED
    pio_sm_set_enabled(pio, sm, true); // Habilita o estado de máquina
    pio_sm_put_blocking(pio, sm, matrix_rgb(r, g, b)); // Envia o valor RGB para o PIO
*/
    while (true) {
        switch (volume_agua) {
        case 0:
            desenho_pio(COORDENADA_NIVEL_0, 0, pio, sm);
             vTaskDelay(pdMS_TO_TICKS(500));
            break;

        case 1:
            desenho_pio(COORDENADA_NIVEL_1, 0, pio, sm);
             vTaskDelay(pdMS_TO_TICKS(500));
            break;

        case 2:
            desenho_pio(COORDENADA_NIVEL_2, 0, pio, sm);
             vTaskDelay(pdMS_TO_TICKS(500));
            break;

        case 3:
            desenho_pio(COORDENADA_NIVEL_3, 0, pio, sm);
             vTaskDelay(pdMS_TO_TICKS(500));
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//------------------------------------------------MAIN------------------------------------------------
int main()
{
    stdio_init_all();                                   // Inicializa a saída padrão (UART)
    sleep_ms(2000);                                     // Aguarda 2 segundos para estabilização

    setup();                                            // Configurações iniciais

    if (cyw43_arch_init())                              // Inicializa o Wi-fi
    {
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi => FALHA", 0, 0);
        ssd1306_send_data(&ssd);
        return 1;
    }

    cyw43_arch_enable_sta_mode();                       // Habilita o modo Station do Wi-Fi
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

    start_http_server();                                // Inicia o servidor HTTP

    char *ip_str_param = malloc(strlen(ip_str) + 1);    // Aloca memória para o IP string
    if (!ip_str_param) {
        printf("Erro ao alocar memória para o IP string\n");
        return 1; // Retorna erro se não conseguir alocar memória
    }
    strcpy(ip_str_param, ip_str);   // Copia o IP string para a memória alocada

    //Tasks
    xTaskCreate(vPollingTask, "Polling Task", 256, NULL, 1, NULL); 
    xTaskCreate(vDisplayTask, "Display Task", 256, ip_str_param, 1, NULL); // Cria a task de display
    xTaskCreate(vLeituraNivelTask, "LeituraNivel", 256, NULL, 2, NULL);
    xTaskCreate(vLedsRGBTask, "ControleRGB", 256, NULL, 2, NULL);
    xTaskCreate(vBuzzerTask, "Task para o buzzer", 256, NULL, 1, NULL); 
    xTaskCreate(vBotaoBombaTask, "Task para acionar a bomba", 256, NULL, 1, NULL);
    xTaskCreate(vButton_task, "Task para botões", 256, NULL, 1, NULL);
    xTaskCreate(vMatriz_led_task, "Task para matriz de LEDs", 256, NULL, 1, NULL);

    vTaskStartScheduler();          // Inicia o escalonador do FreeRTOS
    panic_unsupported();            // Se o escalonador falhar, entra em pânico
}

//----------------------------------------------Funções------------------------------------------------
// Função de configuração inicial
void setup(void){

    gpio_init(BOMBA);                // Inicializa o GPIO da bomba
    gpio_set_dir(BOMBA, GPIO_OUT);   // Define o GPIO como saída
    gpio_put(BOMBA, 0);              // Desliga a bomba inicialmente
    
    gpio_init(LED_MATRIX);          // Inicializa o GPIO da matriz de LEDs
    gpio_set_dir(LED_MATRIX, GPIO_OUT); // Define o GPIO como saída

    gpio_init(LED_PIN_GREEN);
    gpio_set_dir(LED_PIN_GREEN, GPIO_OUT);

    gpio_init(LED_PIN_BLUE);
    gpio_set_dir(LED_PIN_BLUE, GPIO_OUT);

    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);

    adc_gpio_init(SENSOR_NIVEL); 

    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);

    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);

    gpio_init(BOTAO_JOY);
    gpio_set_dir(BOTAO_JOY, GPIO_IN);
    gpio_pull_up(BOTAO_JOY);
    
    pwm_setup(BUZZER_PIN);

    adc_init();
    adc_gpio_init(JOYSTICK_X);
    adc_gpio_init(JOYSTICK_Y);

    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Iniciando Wi-Fi", 0, 0);
    ssd1306_draw_string(&ssd, "Aguarde...", 0, 30);    
    ssd1306_send_data(&ssd);
}

// Faz a leitura da GPIO 22 para acionar a bomba
void acionar_bomba()
{
    if (!gpio_get(BOTAO_JOY))  
    {
        estado_bomba = true;    
    }
    else
    {
        estado_bomba = false;
    }
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

    if (strstr(req, "GET /bomba/on"))
    {
        gpio_put(BOMBA, 1);
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
    else if (strstr(req, "GET /bomba/off"))
    {
        gpio_put(BOMBA, 0);
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
        int botaoA = !gpio_get(BOTAO_A);
        int botaoB = !gpio_get(BOTAO_B);
        int joy = !gpio_get(BOTAO_JOY);

        char json_payload[96];
        int json_len = snprintf(json_payload, sizeof(json_payload),
                                "{\"bomba\":%d,\"x\":%d,\"botaoA\":%d,\"botaoB\":%d,\"joy\":%d}\r\n",
                                gpio_get(BOMBA), volume_agua, botaoA, botaoB, joy);

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
void desenho_pio(double desenho[25][3], uint32_t valor_led, PIO pio, uint sm)
{

  for (int16_t i = 0; i < PIXELS; i++)
  {
    valor_led = matrix_rgb(desenho[i][0], desenho[i][1], desenho[i][2]);
    pio_sm_put_blocking(pio, sm, valor_led);
  };
}

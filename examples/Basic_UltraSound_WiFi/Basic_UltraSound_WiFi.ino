#include "SensEdu.h"
#include "SineLUT.h"
#include <WiFi.h>

/* -------------------------------------------------------------------------- */
/*                                  Settings                                  */
/* -------------------------------------------------------------------------- */
#define WIFI_SSID   "TestWiFi"
#define WIFI_PASS   "TestWiFi"
#define WIFI_PORT   80

/* DAC */
// lut settings are in SineLUT.h
#define DAC_SINE_FREQ     	32000                           // 32kHz
#define DAC_SAMPLE_RATE     DAC_SINE_FREQ * sine_lut_size   // 64 samples per one sine cycle

DAC_Channel* dac_ch = DAC_CH1;
SensEdu_DAC_Settings dac_settings = {
    .dac_channel = dac_ch, 
    .sampling_freq = DAC_SAMPLE_RATE,
    .mem_address = (uint16_t*)sine_lut,
    .mem_size = sine_lut_size,
    .wave_mode = SENSEDU_DAC_MODE_BURST_WAVE,
    .burst_num = dac_cycle_num
};

/* ADC */
const uint16_t mic_data_size = 2000;
SENSEDU_ADC_BUFFER(mic_data, mic_data_size);

ADC_TypeDef* adc = ADC1;
const uint8_t mic_num = 1;
uint8_t mic_pins[mic_num] = {A1};
SensEdu_ADC_Settings adc_settings = {
    .adc = adc,
    .pins = mic_pins,
    .pin_num = mic_num,

    .conv_mode = SENSEDU_ADC_MODE_CONT_TIM_TRIGGERED,
    .sampling_freq = 250000,
    
    .dma_mode = SENSEDU_ADC_DMA_CONNECT,
    .mem_address = (uint16_t*)mic_data,
    .mem_size = mic_data_size
};

/* WiFi */
int status = WL_IDLE_STATUS;
WiFiServer server(WIFI_PORT);

/* errors */
uint32_t lib_error = 0;
uint8_t error_led = D86;

/* -------------------------------------------------------------------------- */
/*                                    Setup                                   */
/* -------------------------------------------------------------------------- */
void setup() {

    Serial.begin(115200);

    SensEdu_DAC_Init(&dac_settings);

    SensEdu_ADC_Init(&adc_settings);
    SensEdu_ADC_Enable(adc);

    pinMode(error_led, OUTPUT);
    digitalWrite(error_led, HIGH);

    lib_error = SensEdu_GetError();
    while (lib_error != 0) {
        handle_error();
    }

    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(WIFI_SSID);

        // connect to WPA/WPA2 network (change this if youre using open / WEP network)
        status = WiFi.begin(WIFI_SSID, WIFI_PASS);

        // wait 10 seconds for connection:
        delay(10000);
    }
    server.begin();
    print_wifi_status();
}

/* -------------------------------------------------------------------------- */
/*                                    Loop                                    */
/* -------------------------------------------------------------------------- */
void loop() {
    lib_error = SensEdu_GetError();
    while (lib_error != 0) {
        handle_error();
    }

    WiFiClient client = server.available();
    if (!client) {
        return;
    }
    Serial.println("Client connected!");

    // Measurement is initiated by the signal from computing device (matlab script)
    static char buf = 0;

    while(client.connected()) {
        if (!client.available()) {
            continue;
        }

        buf = client.read();
        if (buf != 't') { // trigger not detected
            continue;
        }
            
        // start dac->adc sequence
        SensEdu_DAC_Enable(dac_ch);
        while(!SensEdu_DAC_GetBurstCompleteFlag(dac_ch));
        SensEdu_DAC_ClearBurstCompleteFlag(dac_ch);
        SensEdu_ADC_Start(adc);
        
        // wait for the data and send it
        while(!SensEdu_ADC_GetTransferStatus(adc));
        SensEdu_ADC_ClearTransferStatus(adc);
        wifi_send_array(client, (const uint8_t *) & mic_data, mic_data_size << 1);

        // check errors
        lib_error = SensEdu_GetError();
        while (lib_error != 0) {
            handle_error();
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                                  Functions                                 */
/* -------------------------------------------------------------------------- */
void handle_error() {
    digitalWrite(error_led, LOW);
    Serial.print("Error: 0x");
    Serial.println(lib_error, HEX);
    delay(1000);
}

void wifi_send_array(WiFiClient client, const uint8_t* data, size_t size) {
    client.write(data, size);
}

void print_wifi_status(void) {
    // print the SSID of the network youre connected to
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your boards local IP address
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received WiFi signal strength
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

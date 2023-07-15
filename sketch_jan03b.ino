// ESD DESIGN PROJECT
// GROUP MEMBERS: ASAD RAZA (301592) & TALHA NAEEM (242062)
// SUBMITTED TO: DR. USMAN ZABIT
// SUBMITTED ON: 03/01/2023

#include <Arduino.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"
#include <WiFi.h>
#include <WiFiClient.h>

const char* host = "maker.ifttt.com";
const int httpsPort = 80;
const char* ssid = "ZEETEE";
const char* password = "hello123";

// size of noise sample
#define SAMPLES 1024

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = SAMPLES;

#define OCTAVES 9

// our FFT data
static float real[SAMPLES];
static float imag[SAMPLES];
static arduinoFFT fft(real, imag, SAMPLES, SAMPLES);
static float energy[OCTAVES];
// A-weighting curve from 31.5 Hz ... 8000 Hz
static const float aweighting[] = {-39.4, -26.2, -16.1, -8.6, -3.2, 0.0, 1.2, 1.0, -1.1};

static unsigned int bell = 0;
static unsigned int fireAlarm = 0;
static unsigned long ts = millis();
static unsigned long last = micros();
static unsigned int sum = 0;
static unsigned int mn = 9999;
static unsigned int mx = 0;
static unsigned int cnt = 0;
static unsigned long lastTrigger[2] = {0, 0};

//FreeRTOS + ESP-32 Globals
const uint8_t PIN = 32; //pin at which microphone data will arrive
hw_timer_t * timer = NULL;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED; //define spinlock
QueueHandle_t xIsrQueue; //define ISR data queue handle
QueueHandle_t xPeaksQueue; //define handle for queue that  will hold peak value after FFT
SemaphoreHandle_t semaphore; //define counting semaphore handle

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//REGULAR FUNCTIONS TO BE CALLED BY TASKS/ISR (for FFT)

static void integerToFloat(int32_t *integer, float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++)
    {
        vReal[i] = (integer[i] >> 16) / 10.0;
        vImag[i] = 0.0;
    }
}

// calculates energy from Re and Im parts and places it back in the Re part (Im part is zeroed)
static void calculateEnergy(float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++)
    {
        vReal[i] = sq(vReal[i]) + sq(vImag[i]);
        vImag[i] = 0.0;
    }
}

// sums up energy in bins per octave
static void sumEnergy(const float *bins, float *energies, int bin_size, int num_octaves)
{
    // skip the first bin
    int bin = bin_size;
    for (int octave = 0; octave < num_octaves; octave++)
    {
        float sum = 0.0;
        for (int i = 0; i < bin_size; i++)
        {
            sum += real[bin++];
        }
        energies[octave] = sum;
        bin_size *= 2;
    }
}

static float decibel(float v)
{
    return 10.0 * log(v) / log(10);
}

// converts energy to logaritmic, returns A-weighted sum
static float calculateLoudness(float *energies, const float *weights, int num_octaves, float scale)
{
    float sum = 0.0;
    for (int i = 0; i < num_octaves; i++)
    {
        float energy = scale * energies[i];
        sum += energy * pow(10, weights[i] / 10.0);
        energies[i] = decibel(energy);
    }
    return decibel(sum);
}

unsigned int countSetBits(unsigned int n)
{
    unsigned int count = 0;
    while (n)
    {
        count += n & 1;
        n >>= 1;
    }
    return count;
}

//detecting 2 frequencies. Set wide to true to match the previous and next bin as well
bool detectFrequency(unsigned int *mem, unsigned int minMatch, double peak, unsigned int bin1, unsigned int bin2, bool wide)
{
    *mem = *mem << 1;
    if (peak == bin1 || peak == bin2 || (wide && (peak == bin1 + 1 || peak == bin1 - 1 || peak == bin2 + 1 || peak == bin2 - 1)))
    {
        *mem |= 1;
    }

    if (countSetBits(*mem) >= minMatch)
    {
        return true;
    }

    return false;
}

void sendAlarm(unsigned int index, char *topic, unsigned int timeout)
{
    // do not publish if last trigger was earlier than timeout ms
    if (abs(millis() - lastTrigger[index]) < timeout)
    {
        return;
    }

    lastTrigger[index] = millis();
    //publish to mqtt
    //publish(topic, "1");
}

void sendMetrics(char * topic, unsigned int mn, unsigned int mx, unsigned int avg)
{
    String payload = "{\"min\": ";
    payload += mn;
    payload += ", \"max\":";
    payload += mx;
    payload += ", \"value\":";
    payload += avg;
    payload += "}";

    Serial.println(payload);
    //publish to mqtt
    //publish(topic, (char *)payload.c_str());
}

void calculateMetrics(int val) {
  cnt++;
  sum += val;

  if (val > mx)
  {
      mx = val;
  }

  if (val < mn)
  {
      mn = val;
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//ISR

//ISR for when data is received at ESP32 GPIO32 port (data in from microphone)
void IRAM_ATTR onMicrophone() {
    
  static int32_t samples[BLOCK_SIZE]; //Local variable to read microphone data into
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE; //This parameter must be set to pdFalse initially

  // Read multiple samples at once and calculate the sound pressure
  size_t num_bytes_read;
  esp_err_t err = i2s_read(I2S_PORT,
                          (char *)samples,
                          BLOCK_SIZE, // the doc says bytes, but its elements.
                          &num_bytes_read,
                          portMAX_DELAY); // no timeout
  int samples_read = num_bytes_read / 8;

  portENTER_CRITICAL_ISR(&spinlock);
  //Send read sample data to queue to be processed by deferred task
  xQueueSendFromISR(xIsrQueue, &samples, &pxHigherPriorityTaskWoken);
  portEXIT_CRITICAL_ISR(&spinlock);

  // Give a semaphore that we can check in the Task
  // This enables Synchronization of our ISR with the corresponding Task
  // to which the non-critical part of the ISR is deferred.
  xSemaphoreGiveFromISR(semaphore, NULL);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//MAIN (SETUP AND LOOP)

void setup(void)
{
    Serial.begin(115200);
    Serial.print("Connecting to ");
    Serial.println(ssid);
    //WiFi.mode(WIFI_STA);
    WiFi.begin(ssid,password);
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }
    Serial.println("");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("Configuring I2S...");
    esp_err_t err;

    // The I2S config as per the example
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
        .sample_rate = 22627,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // for old esp-idf versions use RIGHT
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
        .dma_buf_count = 8,                       // number of buffers
        .dma_buf_len = BLOCK_SIZE,                // samples per buffer
        .use_apll = true};

    // The pin config as per the setup
    const i2s_pin_config_t pin_config = {
        .bck_io_num = 14,   // BCKL
        .ws_io_num = 15,    // LRCL
        .data_out_num = -1, // not used (only for speakers)
        .data_in_num = 32   // DOUTESP32-INMP441 wiring
    };

    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK)
    {
        Serial.printf("Failed installing driver: %d\n", err);
        while (true)
            ;
    }
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK)
    {
        Serial.printf("Failed setting pin: %d\n", err);
        while (true)
            ;
    }
    Serial.println("I2S driver installed.");

    // Creating the ISR queue of size 10
    xIsrQueue = xQueueCreate(10, sizeof(int32_t));

    //Creating Peaks queue of size 10
    xPeaksQueue = xQueueCreate(10, sizeof(unsigned int));

    // Creating a counting semaphore with width 10 and initial count 0
    semaphore = xSemaphoreCreateCounting(10,0);

    //Task Instantiation
    xTaskCreatePinnedToCore( vDeferredTask, "Deferred Task", 1000, NULL, 2, NULL, 0); //DIP task created with higher priority
    xTaskCreatePinnedToCore( vOutputTask, "Output Task", 1000, NULL , 1, NULL, 0); //Output task created with lower priority

    //Associate ISR with change in value at GPIO pin 32
    //attachInterrupt(PIN, onMicrophone, CHANGE);

  // Use 1st timer from 4 available timers (0,1,2,3).
  // Set Prescaler to 80, i.e. Clk freq / 80. 3rd argument enables up-counting.
  timer = timerBegin(0, 80, true);  

  // Attach onTimer function (i.e. our Interrupt Handler or ISR) to our timer (hardware timer causing interrupt).
  timerAttachInterrupt(timer, &onMicrophone, true); //3rd argument, true, indicates that we want the interrupt to be edge-type interrupt

  // Set alarm to call onTimer function every second (value in microseconds).
  timerAlarmWrite(timer, 2*100000, true);  // Repeat the alarm (3rd parameter)
  timerAlarmEnable(timer);   // Start an alarm



}

void loop() {
  //Nothing here. All is done in tasks.
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//TASK FUNCTIONS

static void vDeferredTask( void *pvParameters) {
    static int32_t DipVar[BLOCK_SIZE]; //create variable to read data from ISR queue

    for( ;; ){
      //If interrupt has occured
      if (xSemaphoreTake(semaphore, 10) == pdTRUE){
       
        //Read sample data from ISR queue
        portENTER_CRITICAL(&spinlock);
        xQueueReceive(xIsrQueue, &DipVar, portMAX_DELAY);
        portEXIT_CRITICAL(&spinlock);

        //HERE WE CARRY OUT FFT PROCESSING
        // integer to float
        integerToFloat(DipVar, real, imag, SAMPLES);

        // apply flat top window, optimal for energy calculations
        fft.Windowing(FFT_WIN_TYP_FLT_TOP, FFT_FORWARD);
        fft.Compute(FFT_FORWARD);

        // calculate energy in each bin
        calculateEnergy(real, imag, SAMPLES);

        // sum up energy in bin for each octave
        sumEnergy(real, energy, 1, OCTAVES);

        // calculate loudness per octave + A weighted loudness
        float loudness = calculateLoudness(energy, aweighting, OCTAVES, 1.0);

        unsigned int peak = (int)floor(fft.MajorPeak());
        //Serial.println(peak);

        calculateMetrics(loudness);

        portENTER_CRITICAL(&spinlock);
        xQueueSend(xPeaksQueue, &peak, portMAX_DELAY); //send peak value to queue to be processed and outputed by another task
        portEXIT_CRITICAL(&spinlock);

        if (micros() - last < 45200) {
          // send mqtt metrics every 10s while waiting and no trigger is detected
          if (millis() - ts >= 10000 && bell == 0 && fireAlarm == 0)
          {
            //Serial.println(cnt[0]);
            sendMetrics("home/noise", mn, mx, sum / cnt);

            portENTER_CRITICAL_ISR(&spinlock); //Enter critical section.
            cnt = 0;
            sum = 0;
            mn = 9999;
            mx = 0;
            ts = millis();
            portEXIT_CRITICAL_ISR(&spinlock); //Exit critical section.

          }
        }

          portENTER_CRITICAL_ISR(&spinlock);
          last = micros();
          portEXIT_CRITICAL_ISR(&spinlock);

      }
    }
    

}

void vOutputTask( void *pvParameters ){
  unsigned int received_peak; //variable to store data read from queue

  for( ;; ){

    WiFiClient client; 
    const int httpPort = 80;  
    if (!client.connect(host, httpPort)) {  
      Serial.println("connection failed");  
      break;} 


    //Read peaks from queue
    portENTER_CRITICAL(&spinlock);
    xQueueReceive(xPeaksQueue, &received_peak, portMAX_DELAY);
    portEXIT_CRITICAL(&spinlock);

    // detecting 1kHz and 1.5kHz (doorbell)
    if (detectFrequency(&bell, 15, received_peak, 45, 68, true))
    {
      Serial.println("Detected bell");
      sendAlarm(0, "home/alarm/doorbell", 2000);
    }

    //detecting frequencies around 3kHz (fire alarm)
    if (detectFrequency(&fireAlarm, 15, received_peak, 140, 141, true))
    {
      Serial.println("Detected fire alarm");
      sendAlarm(1, "home/alarm/fire", 10000);

      String url = "/trigger/Alarm/with/key/beWjLWrIy3roSfOo_E7h3"; 
      Serial.print("Requesting URL: ");
      Serial.println(url);
      client.print(String("POST ") + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");
      vTaskDelay(pdMS_TO_TICKS(5000));

    }
  }
}

 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>
 #include <math.h>
 
 /* SensorTile.box PRO specific headers */
 #include "SensorTile.box_audio.h"
 #include "SensorTile.box_bc.h"
 #include "SensorTile.box_bus.h"
 #include "SensorTile.box_env_sensors.h"
 #include "SensorTile.box_motion_sensors.h"
 
 /* BlueNRG stack headers */
 #include "bluenrg1_types.h"
 #include "bluenrg1_gap.h"
 #include "bluenrg1_aci.h"
 #include "bluenrg1_hci_le.h"
 
 /* Private defines */
 #define TRANSMITTER_PIN                GPIO_PIN_8    /* PA8 - PWM for transmitter */
 #define RECEIVER_PIN                   GPIO_PIN_0    /* PA0 - ADC for receiver */
 #define ADC_BUFFER_SIZE                1024          /* Size of ADC buffer */
 #define SAMPLING_FREQUENCY             10000         /* 10 kHz sampling frequency */
 #define SIGNAL_FREQUENCY               40000         /* 40 kHz ultrasonic frequency */
 #define BURST_DURATION                 5             /* Duration of ultrasound burst in milliseconds */
 #define SAMPLING_WINDOW                100           /* Number of samples to analyze */
 #define DELAY_BETWEEN_TESTS            2000          /* Delay between tests in milliseconds */
 #define NUM_TESTS                      10            /* Number of tests to average */
 #define FFT_SIZE                       1024          /* Size of FFT */
 
 /* Private enums */
 typedef enum {
   INTACT_STATE = 0,
   CRACKED_STATE = 1,
   UNKNOWN_STATE = 2
 } MaterialState;
 
 /* Private structures */
 typedef struct {
   float meanAmplitude;
   float stdDeviation;
   float peakFrequency;
   float signalEnergy;
   int zeroCrossings;
   float harmonicDistortion;
   float signalKurtosis;
   float signalSkewness;
 } SignalFeatures;
 
 /* Private variables */
 static uint16_t adc_buffer[ADC_BUFFER_SIZE];
 static int samples[SAMPLING_WINDOW];
 static TIM_HandleTypeDef htim1;        /* Timer for PWM generation */
 static ADC_HandleTypeDef hadc1;        /* ADC for signal acquisition */
 static DMA_HandleTypeDef hdma_adc1;    /* DMA for ADC */
 static SignalFeatures features;
 
 /* Bluetooth variables */
 uint8_t bdaddr[6];
 uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
 uint16_t crack_detection_service_handle, crack_feature_char_handle;
 extern uint8_t bnrg_expansion_board;
 
 /* Private function prototypes */
 static void SystemClock_Config(void);
 static void MX_GPIO_Init(void);
 static void MX_DMA_Init(void);
 static void MX_ADC1_Init(void);
 static void MX_TIM1_Init(void);
 static void generateUltrasonicSignal(void);
 static void collectSamples(void);
 static void extractFeatures(void);
 static MaterialState detectCrack(void);
 static void displayResults(MaterialState state);
 static void sendDataBluetooth(MaterialState state);
 static void setLED(uint32_t LED, uint8_t state);
 static uint8_t configureBluetooth(void);
 
 /* Main function */
 int main(void) 
 {
   /* Initialize the system */
   HAL_Init();
   SystemClock_Config();
   
   /* Initialize peripherals */
   MX_GPIO_Init();
   MX_DMA_Init();
   MX_ADC1_Init();
   MX_TIM1_Init();
   
   /* Initialize SensorTile.box PRO board */
   BSP_LED_Init(LED_GREEN);
   BSP_LED_Init(LED_RED);
   BSP_LED_Init(LED_BLUE);
   BSP_LED_Init(LED_ORANGE);
   
   /* Initialize buttons */
   BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
   
   /* Initialize Bluetooth */
   if (configureBluetooth() != 0) {
     /* Error in Bluetooth configuration */
     while (1) {
       BSP_LED_On(LED_RED);
       HAL_Delay(100);
       BSP_LED_Off(LED_RED);
       HAL_Delay(100);
     }
   }
   
   /* Indicate system is ready */
   BSP_LED_On(LED_GREEN);
   printf("Structural Health Monitoring System\r\n");
   printf("-----------------------------------\r\n");
   printf("System initialized. Place the test object between transmitter and receiver.\r\n");
   
   while (1) {
     printf("\n--- Starting new test sequence ---\r\n");
     
     /* Reset accumulation variables for averaging */
     float totalMeanAmplitude = 0;
     float totalStdDeviation = 0;
     float totalZeroCrossings = 0;
     float totalSignalEnergy = 0;
     
     /* Perform multiple tests and average the results */
     for (int test = 0; test < NUM_TESTS; test++) {
       /* Generate ultrasonic signal */
       generateUltrasonicSignal();
       
       /* Collect samples from receiving piezo */
       collectSamples();
       
       /* Extract signal features */
       extractFeatures();
       
       /* Accumulate features for averaging */
       totalMeanAmplitude += features.meanAmplitude;
       totalStdDeviation += features.stdDeviation;
       totalZeroCrossings += features.zeroCrossings;
       totalSignalEnergy += features.signalEnergy;
       
       /* Small delay between individual tests */
       HAL_Delay(100);
     }
     
     /* Average the results */
     features.meanAmplitude = totalMeanAmplitude / NUM_TESTS;
     features.stdDeviation = totalStdDeviation / NUM_TESTS;
     features.zeroCrossings = (int)(totalZeroCrossings / NUM_TESTS);
     features.signalEnergy = totalSignalEnergy / NUM_TESTS;
     
     /* Detect cracks based on extracted features */
     MaterialState materialState = detectCrack();
     
     /* Display results on LEDs and UART */
     displayResults(materialState);
     
     /* Send data over Bluetooth */
     sendDataBluetooth(materialState);
     
     /* Delay before next test */
     HAL_Delay(DELAY_BETWEEN_TESTS);
   }
 }
 
 /**
   * @brief  Generates a 40 kHz ultrasonic signal on the transmitter pin using PWM
   * @param  None
   * @retval None
   */
 static void generateUltrasonicSignal(void)
 {
   printf("Generating ultrasonic signal...\r\n");
   
   /* Start PWM generation at 40 kHz */
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   
   /* Keep the signal active for the burst duration */
   HAL_Delay(BURST_DURATION);
   
   /* Stop PWM generation */
   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
 }
 
 /**
   * @brief  Collects samples from the receiving piezo using ADC and DMA
   * @param  None
   * @retval None
   */
 static void collectSamples(void)
 {
   printf("Collecting samples...\r\n");
   
   /* Wait a small amount for the signal to propagate */
   HAL_Delay(1);
   
   /* Start ADC conversion with DMA */
   HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
   
   /* Wait for DMA to complete */
   while (HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_BUSY_REG);
   
   /* Copy the required number of samples for processing */
   for (int i = 0; i < SAMPLING_WINDOW; i++) {
     samples[i] = adc_buffer[i];
   }
   
   /* Stop ADC and DMA */
   HAL_ADC_Stop_DMA(&hadc1);
 }
 
 /**
   * @brief  Extracts features from the collected samples
   * @param  None
   * @retval None
   */
 static void extractFeatures(void)
 {
   printf("Extracting signal features...\r\n");
   
   /* Calculate mean amplitude */
   int32_t sum = 0;
   for (int i = 0; i < SAMPLING_WINDOW; i++) {
     sum += samples[i];
   }
   features.meanAmplitude = (float)sum / SAMPLING_WINDOW;
   
   /* Calculate standard deviation */
   float sumOfSquares = 0;
   for (int i = 0; i < SAMPLING_WINDOW; i++) {
     float difference = samples[i] - features.meanAmplitude;
     sumOfSquares += difference * difference;
   }
   features.stdDeviation = sqrt(sumOfSquares / SAMPLING_WINDOW);
   
   /* Count zero crossings */
   features.zeroCrossings = 0;
   for (int i = 1; i < SAMPLING_WINDOW; i++) {
     if ((samples[i] > features.meanAmplitude && samples[i-1] < features.meanAmplitude) || 
         (samples[i] < features.meanAmplitude && samples[i-1] > features.meanAmplitude)) {
       features.zeroCrossings++;
     }
   }
   
   /* Calculate signal energy */
   features.signalEnergy = 0;
   for (int i = 0; i < SAMPLING_WINDOW; i++) {
     features.signalEnergy += samples[i] * samples[i];
   }
   features.signalEnergy /= SAMPLING_WINDOW;
   
   /* Note: For more advanced features like peak frequency, harmonic distortion,
      kurtosis, and skewness, we would need to implement FFT and statistical
      calculations, which would be included in a more complete implementation. */
      
   /* For demonstration, we'll use placeholder values */
   features.peakFrequency = 40000.0f; /* Expected frequency */
   features.harmonicDistortion = 0.1f;
   features.signalKurtosis = 3.0f;
   features.signalSkewness = 0.0f;
 }
 
 /**
   * @brief  Detects cracks based on the extracted features
   * @param  None
   * @retval MaterialState: INTACT_STATE if no crack detected, CRACKED_STATE if crack detected
   */
 static MaterialState detectCrack(void)
 {
   printf("Analyzing for crack detection...\r\n");
   
   /* Simple detection algorithm based on amplitude and energy
      In a real system, this would be more sophisticated, potentially using ML */
   
   /* Thresholds for crack detection - these would be calibrated for your specific setup */
   const float THRESHOLD_AMPLITUDE = 2000.0f;
   const float THRESHOLD_STD_DEV = 500.0f;
   const int THRESHOLD_ZERO_CROSSINGS = 20;
   const float THRESHOLD_ENERGY = 5000000.0f;
   
   /* Cracked materials typically have:
      - Lower mean amplitude
      - Higher standard deviation
      - More zero crossings
      - Lower signal energy */
   
   /* Decision logic */
   if (features.meanAmplitude < THRESHOLD_AMPLITUDE &&
       features.stdDeviation > THRESHOLD_STD_DEV &&
       features.zeroCrossings > THRESHOLD_ZERO_CROSSINGS &&
       features.signalEnergy < THRESHOLD_ENERGY) {
     return CRACKED_STATE;
   } else if (features.meanAmplitude > THRESHOLD_AMPLITUDE &&
              features.stdDeviation < THRESHOLD_STD_DEV &&
              features.zeroCrossings < THRESHOLD_ZERO_CROSSINGS &&
              features.signalEnergy > THRESHOLD_ENERGY) {
     return INTACT_STATE;
   } else {
     return UNKNOWN_STATE; /* Uncertain result */
   }
 }
 
 /**
   * @brief  Displays the results via LEDs and serial output
   * @param  state: The detected material state
   * @retval None
   */
 static void displayResults(MaterialState state)
 {
   /* Reset LEDs */
   BSP_LED_Off(LED_GREEN);
   BSP_LED_Off(LED_RED);
   BSP_LED_Off(LED_ORANGE);
   
   switch (state) {
     case INTACT_STATE:
       printf("MATERIAL INTACT\r\n");
       BSP_LED_On(LED_GREEN);
       break;
       
     case CRACKED_STATE:
       printf("CRACK DETECTED!\r\n");
       BSP_LED_On(LED_RED);
       break;
       
     case UNKNOWN_STATE:
     default:
       printf("UNDETERMINED STATE\r\n");
       BSP_LED_On(LED_ORANGE);
       break;
   }
   
   /* Print feature values */
   printf("--- Signal Features ---\r\n");
   printf("Mean Amplitude: %.2f\r\n", features.meanAmplitude);
   printf("Standard Deviation: %.2f\r\n", features.stdDeviation);
   printf("Zero Crossings: %d\r\n", features.zeroCrossings);
   printf("Signal Energy: %.2f\r\n", features.signalEnergy);
   printf("Peak Frequency: %.2f\r\n", features.peakFrequency);
   printf("Harmonic Distortion: %.2f\r\n", features.harmonicDistortion);
 }
 
 /**
   * @brief  Sends data over Bluetooth to the smartphone app
   * @param  state: The detected material state
   * @retval None
   */
 static void sendDataBluetooth(MaterialState state)
 {
   /* Prepare data to send */
   uint8_t data[20]; /* Buffer for the BLE characteristic value */
   uint8_t length = 0;
   
   /* First byte is the material state */
   data[length++] = (uint8_t)state;
   
   /* Convert float features to bytes (simplified approach) */
   /* Mean amplitude (4 bytes) */
   memcpy(&data[length], &features.meanAmplitude, sizeof(float));
   length += sizeof(float);
   
   /* Standard deviation (4 bytes) */
   memcpy(&data[length], &features.stdDeviation, sizeof(float));
   length += sizeof(float);
   
   /* Zero crossings (1 byte) */
   data[length++] = (uint8_t)features.zeroCrossings;
   
   /* Signal energy (4 bytes) */
   memcpy(&data[length], &features.signalEnergy, sizeof(float));
   length += sizeof(float);
   
   /* Update the BLE characteristic value */
   aci_gatt_update_char_value(crack_detection_service_handle, 
                              crack_feature_char_handle,
                              0, 
                              length, 
                              data);
   
   printf("Data sent over Bluetooth\r\n");
 }
 
 /**
   * @brief  Configures Bluetooth for communication with the smartphone app
   * @param  None
   * @retval Status (0=Success, non-zero=Error)
   */
 static uint8_t configureBluetooth(void)
 {
   uint8_t ret;
   uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
   
   /* Initialize the BlueNRG */
   if (hci_init(NULL, NULL) != BLE_STATUS_SUCCESS) {
     return 1;
   }
   
   /* Reset the BlueNRG hardware */
   hci_reset();
   HAL_Delay(100);
   
   /* Configure device address */
   bdaddr[0] = 0x12;
   bdaddr[1] = 0x34;
   bdaddr[2] = 0x00;
   bdaddr[3] = 0xE1;
   bdaddr[4] = 0x80;
   bdaddr[5] = 0x02;
   ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
   if (ret != BLE_STATUS_SUCCESS) {
     return 2;
   }
   
   /* Configure GAP */
   ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
   if (ret != BLE_STATUS_SUCCESS) {
     return 3;
   }
   
   /* Set device name */
   const char *device_name = "SHM_System";
   ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen(device_name), (uint8_t *)device_name);
   if (ret != BLE_STATUS_SUCCESS) {
     return 4;
   }
   
   /* Create custom service for crack detection */
   uint8_t uuid[16] = {0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1, 0xe0, 0xf2, 0x73, 0xd9};
   ret = aci_gatt_add_service(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 7, &crack_detection_service_handle);
   if (ret != BLE_STATUS_SUCCESS) {
     return 5;
   }
   
   /* Add characteristic for crack detection features */
   uint8_t char_uuid[16] = {0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x96, 0x9e, 0xe2, 0x11, 0x9e, 0xb1, 0xe1, 0xf2, 0x73, 0xd9};
   ret = aci_gatt_add_char(crack_detection_service_handle, UUID_TYPE_128, char_uuid, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0, 16, 1, &crack_feature_char_handle);
   if (ret != BLE_STATUS_SUCCESS) {
     return 6;
   }
   
   /* Start advertising */
   uint8_t local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME, 'S', 'H', 'M', '_', 'S', 'Y', 'S'};
   ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);
   if (ret != BLE_STATUS_SUCCESS) {
     return 7;
   }
   
   printf("Bluetooth configured successfully\r\n");
   BSP_LED_On(LED_BLUE); /* Indicate BLE is active */
   return 0;
 }
 
 /**
   * @brief  System Clock Configuration
   * @param  None

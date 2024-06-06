#include <mbed.h>
#include <motion_dection_project_inferencing.h>

//variables to communicate with ADXL345 via I2C
#define I2C_SDA p8
#define I2C_SCL p9
#define addr7bit 0x53
#define addr8bit 0xA6 //0x53 << 1 for using mbed.h
#define ADXL_DEVID       0x00
#define ADXL_POWER_CTL   0x2D
#define ADXL_DATAX0      0x32
#define ADXL_DATA_FORMAT 0x31
#define DEVID  0xE5
float SENSITYVITY_2G = 1.0/256;
float EARTH_GRAVITY = 9.80665;
mbed::I2C i2c(I2C_SDA, I2C_SCL);

#define FREQUENCY_HZ 50
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))
#define INTERVAL_US INTERVAL_MS * 1000

//variables to run edge impulse inferences
#define INPUT_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
static bool debug_nn = false;
static uint32_t run_inference_every_ms = 200;
static rtos::Thread inference_thread(osPriorityLow);
static float buffer[INPUT_SIZE] = {0};
static float inference_buffer[INPUT_SIZE];

//utility function for i2c communication
void write_reg(int addr_i2c, int addr_reg, char v) {
  char data[2] = {addr_reg, v};
  i2c.write(addr_i2c, data, 2);
  return;
}
void read_reg(int addr, int reg, char *buf, int length){
  char data = reg;
  i2c.write(addr, &data, 1);
  i2c.read(addr, buf, length);
  return;
}

// void run_inference_background();

void setup() {
  i2c.frequency(400000);

  Serial.begin(115200);
  while(!Serial);

  char id;
  read_reg(addr8bit, ADXL_DEVID, &id, 1);
  if(id == DEVID){
    Serial.println("ADXL found");
    write_reg(addr8bit, ADXL_POWER_CTL, 0x08);
    // write_reg(addr8bit, ADXL_DATA_FORMAT, 0x00);
  }
  else{
    Serial.println("ADXL not found");
  }

  inference_thread.start(mbed::callback(&run_inference_background));
}

//RTOS thread for inferecing in parallel
void run_inference_background(){
  // wait until we have a full buffer
  delay((EI_CLASSIFIER_INTERVAL_MS * EI_CLASSIFIER_RAW_SAMPLE_COUNT) + 100);

  //This is a structure that smoothens the output result
  ei_classifier_smooth_t smooth;
  ei_classifier_smooth_init(&smooth, 10, 7);

  while(1){
    //copy the buffer
    memcpy(inference_buffer, buffer, INPUT_SIZE * sizeof(float));

    //turn the raw buffer in a signal which we can classify
    signal_t signal;
    int err = numpy::signal_from_buffer(inference_buffer, INPUT_SIZE, &signal);
    if(err != 0){
      ei_printf("Failed to create signal from buffer (%d)\n", err);
      return;
    }

    //run the classifier
    ei_impulse_result_t result = {0};

    err = run_classifier(&signal, &result, debug_nn);
    if(err != EI_IMPULSE_OK){
      ei_printf("ERR: Failed to run classifier (%d)\n", err);
      return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": ");

    //ei_classifier_smooth_update yields the predicted label
    const char *prediction = ei_classifier_smooth_update(&smooth, &result);
    ei_printf("%s ", prediction);
    //print the cumulative results
    ei_printf(" [ ");
    for(size_t ix=0; ix < smooth.count_size; ix++){
      ei_printf("%u", smooth.count[ix]);
      if(ix != smooth.count_size + 1){
        ei_printf(", ");
      }
      else{
        ei_printf(" ");
      }
    }
    ei_printf("]\n");

    delay(run_inference_every_ms);
  }

  ei_classifier_smooth_free(&smooth);
}

void loop() {
  mbed::Timer timer;
  timer.start();
  //accel reading
  char data[6];
  read_reg(addr8bit, ADXL_DATAX0, data, 6);

  int16_t ax16 = (int16_t)(data[1] << 8 | data[0]);
  int16_t ay16 = (int16_t)(data[3] << 8 | data[2]);
  int16_t az16 = (int16_t)(data[5] << 8 | data[4]);

  float ax = ax16 * SENSITYVITY_2G * EARTH_GRAVITY;
  float ay = ay16 * SENSITYVITY_2G * EARTH_GRAVITY;
  float az = az16 * SENSITYVITY_2G * EARTH_GRAVITY;
  // Serial.print(ax);
  // Serial.print(",");
  // Serial.print(ay);
  // Serial.print(",");
  // Serial.println(az);

  numpy::roll(buffer, INPUT_SIZE, -3);

  buffer[INPUT_SIZE - 3] = ax;
  buffer[INPUT_SIZE - 2] = ay;
  buffer[INPUT_SIZE - 1] = az;

  timer.stop();

  //Calculated how much time the program needs to wait before the next accel reading
  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  auto t0 = timer.elapsed_time();
  auto t_diff = duration_cast<microseconds>(t0);
  uint64_t t_wait_us = INTERVAL_US - t_diff.count();
  int32_t t_wait_ms = (t_wait_us / 1000);
  int32_t t_wait_leftover_us = (t_wait_us % 1000);
  delay(t_wait_ms);
  delayMicroseconds(t_wait_leftover_us);

}

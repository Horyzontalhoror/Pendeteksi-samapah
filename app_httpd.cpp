#include <sampah_inferencing.h>

#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "image_util.h"
#include "camera_index.h"
//#include "camera_index_html.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "fd_forward.h"
//#include "dl_lib.h"
#include "fr_forward.h"

/* Modify the following line according to your project name
   Do not forget to import the library using "Sketch">"Include Library">"Add .ZIP Library..."
*/


uint8_t *out_buf;
uint8_t *ei_buf;
double jenis[2];
double anorganik;
double organik;
static int8_t ei_activate = 0;
dl_matrix3du_t *resized_matrix = NULL;
size_t out_len = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
ei_impulse_result_t result = {0};

typedef struct
{
  size_t size;  //number of values used for filtering
  size_t index; //current value index
  size_t count; //value count
  int sum;
  int *values; //array to be filled with values
} ra_filter_t;

typedef struct
{
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size)
{
  memset(filter, 0, sizeof(ra_filter_t));

  filter->values = (int *)malloc(sample_size * sizeof(int));
  if (!filter->values)
  {
    return NULL;
  }
  memset(filter->values, 0, sample_size * sizeof(int));

  filter->size = sample_size;
  return filter;
}

static int ra_filter_run(ra_filter_t *filter, int value)
{
  if (!filter->values)
  {
    return value;
  }
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index++;
  filter->index = filter->index % filter->size;
  if (filter->count < filter->size)
  {
    filter->count++;
  }
  return filter->sum / filter->count;
}

int raw_feature_get_data(size_t offset, size_t out_len, float *signal_ptr)
{
  size_t pixel_ix = offset * 3;
  size_t bytes_left = out_len;
  size_t out_ptr_ix = 0;

  // read byte for byte
  while (bytes_left != 0) {
    // grab the values and convert to r/g/b
    uint8_t r, g, b;
    r = ei_buf[pixel_ix];
    g = ei_buf[pixel_ix + 1];
    b = ei_buf[pixel_ix + 2];

    // then convert to out_ptr format
    float pixel_f = (r << 16) + (g << 8) + b;
    signal_ptr[out_ptr_ix] = pixel_f;

    // and go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    bytes_left--;
  }
  return 0;
}

void classify()
{
  Serial.println("Getting signal...");
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_WIDTH;
  signal.get_data = &raw_feature_get_data;

  Serial.println("Run classifier...");
  // Feed signal to the classifier
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false /* debug */);

  // Returned error variable "res" while data object.array in "result"
  ei_printf("run_classifier returned: %d\n", res);
  if (res != 0)
    return;

  // print the predictions
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: \t%f\r\n", result.classification[ix].label, result.classification[ix].value);

    jenis[ix] = result.classification[ix].value;

    //      for(int i = 0; i < *jenis; ++i)
    //      {
    //        Serial.println(jenis[i]);
    //       }
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %f\r\n", result.anomaly);
#endif
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index)
  {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
  {
    return 0;
  }
  j->len += len;
  return len;
}

int deteksi() {
  int detek = 0;
  camera_fb_t *fb = NULL;
  int64_t fr_start = esp_timer_get_time();
  size_t out_len, out_width, out_height;
  size_t ei_len;

  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
  }

  bool s;
  bool detected = false;

  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!image_matrix)
  {
    esp_camera_fb_return(fb);
    Serial.println("dl_matrix3du_alloc failed");
  }

  out_buf = image_matrix->item;
  out_len = fb->width * fb->height * 3;
  out_width = fb->width;
  out_height = fb->height;

  Serial.println("Converting to RGB888...");
  int64_t time_start = esp_timer_get_time();
  s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
  int64_t time_end = esp_timer_get_time();
  Serial.printf("Done in %ums\n", (uint32_t)((time_end - time_start) / 1000));

  esp_camera_fb_return(fb);
  if (!s)
  {
    dl_matrix3du_free(image_matrix);
    Serial.println("to rgb888 failed");
  }

  dl_matrix3du_t *ei_matrix = dl_matrix3du_alloc(1, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3);
  if (!ei_matrix)
  {
    esp_camera_fb_return(fb);
    Serial.println("dl_matrix3du_alloc failed");
  }

  ei_buf = ei_matrix->item;
  ei_len = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;

  Serial.println("Resizing the frame buffer...");
  time_start = esp_timer_get_time();
  image_resize_linear(ei_buf, out_buf, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, 3, out_width, out_height);
  time_end = esp_timer_get_time();
  Serial.printf("Done in %ums\n", (uint32_t)((time_end - time_start) / 1000));

  dl_matrix3du_free(image_matrix);

  classify();

  dl_matrix3du_free(ei_matrix);
  anorganik = jenis[0];
  organik = jenis[1];
  if (organik > anorganik) {
    detek = 1;
    Serial.println("sampah organik");
  }
  if (organik < anorganik) {
    detek = 0;
    Serial.println("sampah anorganik");
  }
  ei_impulse_result_t result = {0};
  return detek;
}

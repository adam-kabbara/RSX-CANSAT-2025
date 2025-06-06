
// General utilities not specific to this app to support:
// - wifi
// - NTP
// - remote logging
// - base64 encoding
// - device sleep
//
// s60sc 2021, 2023
// some functions based on code contributed by gemi254

#include "appGlobals.h"

bool dbgVerbose = false;
bool timeSynchronized = false;
bool monitorOpen = true;
bool dataFilesChecked = false;
// allow any startup failures to be reported via browser for remote devices
char startupFailure[SF_LEN] = {0};
size_t alertBufferSize = 0;
byte* alertBuffer = NULL; // buffer for telegram / smtp alert image
RTC_NOINIT_ATTR uint32_t crashLoop;
RTC_NOINIT_ATTR char brownoutStatus;
static void initBrownout(void);
int wakePin; // if wakeUse is true
bool wakeUse = false; // true to allow app to sleep and wake

char hostName[MAX_HOST_LEN] = "";
/************************** NTP  **************************/

// Needs to be a time zone string from: https://raw.githubusercontent.com/nayarsystems/posix_tz_db/master/zones.csv
char timezone[FILE_NAME_LEN] = "GMT0";
char ntpServer[MAX_HOST_LEN] = "pool.ntp.org";
uint8_t alarmHour = 1;

time_t getEpoch() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec;
}

void dateFormat(char* inBuff, size_t inBuffLen, bool isFolder) {
  // construct filename from date/time
  time_t currEpoch = getEpoch();
  if (isFolder) strftime(inBuff, inBuffLen, "/%Y%m%d", localtime(&currEpoch));
  else strftime(inBuff, inBuffLen, "/%Y%m%d/%Y%m%d_%H%M%S", localtime(&currEpoch));
}

static void showLocalTime(const char* timeSrc) {
  time_t currEpoch = getEpoch();
  char timeFormat[20];
  strftime(timeFormat, sizeof(timeFormat), "%d/%m/%Y %H:%M:%S", localtime(&currEpoch));
  LOG_INF("Got current time from %s: %s with tz: %s", timeSrc, timeFormat, timezone);
  timeSynchronized = true;
}

/********************** misc functions ************************/

bool changeExtension(char* fileName, const char* newExt) {
  // replace original file extension with supplied extension (buffer must be large enough)
  size_t inNamePtr = strlen(fileName);
  // find '.' before extension text
  while (inNamePtr > 0 && fileName[inNamePtr] != '.') inNamePtr--;
  inNamePtr++;
  size_t extLen = strlen(newExt);
  memcpy(fileName + inNamePtr, newExt, extLen);
  fileName[inNamePtr + extLen] = 0;
  return (inNamePtr > 1) ? true : false;
}

void showProgress(const char* marker) {
  // show progess as dots 
  static uint8_t dotCnt = 0;
  logPrint(marker); // progress marker
  if (++dotCnt >= DOT_MAX) {
    dotCnt = 0;
    logLine();
  }
}

bool calcProgress(int progressVal, int totalVal, int percentReport, uint8_t &pcProgress) {
  // calculate percentage progress, only report back on percentReport boundary
  uint8_t percentage = (progressVal * 100) / totalVal;
  if (percentage >= pcProgress + percentReport) {
    pcProgress = percentage;
    return true;
  } else return false;
}

bool urlEncode(const char* inVal, char* encoded, size_t maxSize) {
  int encodedLen = 0;
  char hexTable[] = "0123456789ABCDEF";
  while (*inVal) {
    if (isalnum(*inVal) || strchr("$-_.+!*'(),:@~#", *inVal)) *encoded++ = *inVal;
    else {
      encodedLen += 3; 
      if (encodedLen >= maxSize) return false;  // Buffer overflow
      *encoded++ = '%';
      *encoded++ = hexTable[(*inVal) >> 4];
      *encoded++ = hexTable[*inVal & 0xf];
    }
    inVal++;
  }
  *encoded = 0;
  return true;
}

void urlDecode(char* inVal) {
  // replace url encoded characters
  std::string decodeVal(inVal); 
  std::string replaceVal = decodeVal;
  std::smatch match; 
  while (regex_search(decodeVal, match, std::regex("(%)([0-9A-Fa-f]{2})"))) {
    std::string s(1, static_cast<char>(std::strtoul(match.str(2).c_str(),nullptr,16))); // hex to ascii 
    replaceVal = std::regex_replace(replaceVal, std::regex(match.str(0)), s);
    decodeVal = match.suffix().str();
  }
  strcpy(inVal, replaceVal.c_str());
}

void listBuff (const uint8_t* b, size_t len) {
  // output buffer content as hex, 16 bytes per line
  if (!len || !b) LOG_WRN("Nothing to print");
  else {
    for (size_t i = 0; i < len; i += 16) {
      int linelen = (len - i) < 16 ? (len - i) : 16;
      for (size_t k = 0; k < linelen; k++) logPrint(" %02x", b[i+k]);
      puts(" ");
    }
  }
}

size_t isSubArray(uint8_t* haystack, uint8_t* needle, size_t hSize, size_t nSize) {
  // find a subarray (needle) in another array (haystack)
  size_t h = 0, n = 0; // Two pointers to traverse the arrays
  // Traverse both arrays simultaneously
  while (h < hSize && n < nSize) {
    // If element matches, increment both pointers
    if (haystack[h] == needle[n]) {
      h++;
      n++;
      // If needle is completely traversed
      if (n == nSize) return h; // position of end of needle
    } else {
      // if not, increment h and reset n
      h = h - n + 1;
      n = 0;
    }
  }
  return 0; // not found
}

void removeChar(char* s, char c) {
  // remove specified character from string
  int writer = 0, reader = 0;
  while (s[reader]) {
    if (s[reader] != c) s[writer++] = s[reader];
    reader++;       
  }
  s[writer] = 0;
}

void replaceChar(char* s, char c, char r) {
  // replace specified character in string
  int reader = 0;
  while (s[reader]) {
    if (s[reader] == c) s[reader] = r;
    reader++;       
  }
}

char* fmtSize (uint64_t sizeVal) {
  // format size according to magnitude
  // only one call per format string
  static char returnStr[20];
  if (sizeVal < 50 * 1024) sprintf(returnStr, "%llu bytes", sizeVal);
  else if (sizeVal < ONEMEG) sprintf(returnStr, "%lluKB", sizeVal / 1024);
  else if (sizeVal < ONEMEG * 1024) sprintf(returnStr, "%0.1fMB", (double)(sizeVal) / ONEMEG);
  else sprintf(returnStr, "%0.1fGB", (double)(sizeVal) / (ONEMEG * 1024));
  return returnStr;
}

void checkMemory(const char* source ) {
  LOG_INF("%s Free: heap %u, block: %u, min: %u, pSRAM %u", source, ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap(), ESP.getFreePsram());
  if (ESP.getFreeHeap() < WARN_HEAP) LOG_WRN("Free heap only %u, min %u", ESP.getFreeHeap(), ESP.getMinFreeHeap());
  if (ESP.getMaxAllocHeap() < WARN_ALLOC) LOG_WRN("Max allocatable heap block is only %u", ESP.getMaxAllocHeap());
}

uint32_t checkStackUse(TaskHandle_t thisTask, int taskIdx) {
  // get minimum free stack size for task since started
  static uint32_t minStack[20]; 
  uint32_t freeStack = 0;
  if (thisTask != NULL) {
    freeStack = (uint32_t)uxTaskGetStackHighWaterMark(thisTask);
    if (!minStack[taskIdx]) {
      minStack[taskIdx] = freeStack; // initialise
      LOG_INF("Task %s on core %d, initial stack space %u", pcTaskGetTaskName(thisTask), xPortGetCoreID(), freeStack);
    }
    if (freeStack < minStack[taskIdx]) {
      minStack[taskIdx] = freeStack;
      if (freeStack < MIN_STACK_FREE) LOG_WRN("Task %s on core %d, stack space only: %u", pcTaskGetTaskName(thisTask), xPortGetCoreID(), freeStack);
      else LOG_INF("Task %s on core %d, stack space reduced to %u", pcTaskGetTaskName(thisTask), xPortGetCoreID(), freeStack);
    }
  }
  return freeStack;
}

void debugMemory(const char* caller) {
  if (DEBUG_MEM) {
    logPrint("%s > Free: heap %u, block: %u, min: %u, pSRAM %u\n", caller, ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap(), ESP.getFreePsram());
    delay(FLUSH_DELAY);
  }
}

void doRestart(const char* restartStr) {
  LOG_ALT("Controlled restart: %s", restartStr);
// #ifdef ISCAM
//   appShutdown();
// #endif
  flush_log(true);
  delay(2000);
  ESP.restart();
}

uint16_t smoothAnalog(int analogPin, int samples) {
  // get averaged analog pin value 
  uint32_t level = 0; 
  if (analogPin > 0) {
    for (int j = 0; j < samples; j++) level += analogRead(analogPin); 
    level /= samples;
  }
  return level;
}

void setupADC() {
  analogSetAttenuation(ADC_ATTEN);
  analogReadResolution(ADC_BITS);
}

float smoothSensor(float latestVal, float smoothedVal, float alpha) {
  // simple Exponential Moving Average filter 
  // where alpha between 0.0 (max smooth) and 1.0 (no smooth)
  return (latestVal * alpha) + smoothedVal * (1.0 - alpha);
}

// onboard chip temperature sensor
#if CONFIG_IDF_TARGET_ESP32
extern "C" {
// Use internal on chip temperature sensor (if present)
uint8_t temprature_sens_read(); // sic
}
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
#include "driver/temperature_sensor.h"
static temperature_sensor_handle_t temp_sensor = NULL;
#endif

static void prepInternalTemp() {
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
  // setup internal sensor
  temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
  temperature_sensor_install(&temp_sensor_config, &temp_sensor);
  temperature_sensor_enable(temp_sensor);
#endif
}

float readInternalTemp() {
  float intTemp = NULL_TEMP;
#if CONFIG_IDF_TARGET_ESP32
  // convert on chip raw temperature in F to Celsius degrees
  intTemp = (temprature_sens_read() - 32) / 1.8;  // value of 55 means not present
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
    temperature_sensor_get_celsius(temp_sensor, &intTemp); 
#endif
  return intTemp;
}
/*********************** Remote loggging ***********************/
/*
 * Log mode selection in user interface: 
 * false : log to serial / web monitor only
 * true  : also saves log on SD card. To download the log generated, either:
 *  - To view the log, press Show Log button on the browser
 * - To clear the log file contents, on log web page press Clear Log link
 */
 
#define MAX_OUT 200
static va_list arglist;
static char fmtBuf[MAX_OUT];
static char outBuf[MAX_OUT];
char alertMsg[MAX_OUT];
TaskHandle_t logHandle = NULL;
static SemaphoreHandle_t logSemaphore = NULL;
static SemaphoreHandle_t logMutex = NULL;
static int logWait = 100; // ms
bool useLogColors = false;  // true to colorise log messages (eg if using idf.py, but not arduino)
bool wsLog = false;

#define WRITE_CACHE_CYCLE 5

bool sdLog = false; // log to SD
int logType = 0; // which log contents to display (0 : ram, 1 : sd, 2 : ws)
static FILE* log_remote_fp = NULL;
static uint32_t counter_write = 0;

// RAM memory based logging in RTC slow memory (cannot init)
RTC_NOINIT_ATTR char messageLog[RAM_LOG_LEN];
RTC_NOINIT_ATTR uint16_t mlogEnd;

static void ramLogClear() {
  mlogEnd = 0;
  memset(messageLog, 0, RAM_LOG_LEN);
}
  
static void ramLogStore(size_t msgLen) {
  // save log entry in ram buffer
  if (mlogEnd + msgLen >= RAM_LOG_LEN) {
    // log needs to roll around cyclic buffer
    uint16_t firstPart = RAM_LOG_LEN - mlogEnd;
    memcpy(messageLog + mlogEnd, outBuf, firstPart);
    msgLen -= firstPart;
    memcpy(messageLog, outBuf + firstPart, msgLen);
    mlogEnd = 0;
  } else memcpy(messageLog + mlogEnd, outBuf, msgLen);
  mlogEnd += msgLen;
}

void flush_log(bool andClose) {
  if (log_remote_fp != NULL) {
    fsync(fileno(log_remote_fp));  
    fflush(log_remote_fp);
    if (andClose) {
      LOG_INF("Closed SD file for logging");
      fclose(log_remote_fp);
      log_remote_fp = NULL;
    } else delay(1000);
  }  
}

static void remote_log_init_SD() {
#if !CONFIG_IDF_TARGET_ESP32C3
  STORAGE.mkdir(DATA_DIR);
  // Open remote file
  log_remote_fp = NULL;
  log_remote_fp = fopen("/sdcard" LOG_FILE_PATH, "a");
  if (log_remote_fp == NULL) {LOG_WRN("Failed to open SD log file %s", LOG_FILE_PATH);}
  else {
    logPrint(" \n");
    LOG_INF("Opened SD file for logging");
  }
#endif
}

void reset_log() {
  if (logType == 0) ramLogClear();
  if (logType == 2) {
    if (log_remote_fp != NULL) flush_log(true); // Close log file
    STORAGE.remove(LOG_FILE_PATH);
    remote_log_init_SD();
  }
  if (logType != 1) LOG_INF("Cleared %s log file", logType == 0 ? "RAM" : "SD"); 
}

void remote_log_init() {
  // setup required log mode
  if (sdLog) {
    flush_log(false);
    remote_log_init_SD(); // store log on sd card
  } else flush_log(true);
}

static void logTask(void *arg) {
  // separate task to reduce stack size in other tasks
  while(true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vsnprintf(outBuf, MAX_OUT, fmtBuf, arglist);
    va_end(arglist);
    xSemaphoreGive(logSemaphore);
  }
}

void logPrint(const char *format, ...) {
  // feeds logTask to format message, then outputs as required
  if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(logWait)) == pdTRUE) {
    strncpy(fmtBuf, format, MAX_OUT);
    va_start(arglist, format); 
    vTaskPrioritySet(logHandle, uxTaskPriorityGet(NULL) + 1);
    xTaskNotifyGive(logHandle);
    outBuf[MAX_OUT - 2] = '\n'; 
    outBuf[MAX_OUT - 1] = 0; // ensure always have ending newline
    xSemaphoreTake(logSemaphore, portMAX_DELAY); // wait for logTask to complete        
    // output to monitor console if attached
    size_t msgLen = strlen(outBuf);
    if (outBuf[msgLen - 2] == '~') {
      // set up alert message for browser
      outBuf[msgLen - 2] = ' ';
      strncpy(alertMsg, outBuf, MAX_OUT - 1);
      alertMsg[msgLen - 2] = 0;
    }
    ramLogStore(msgLen); // store in rtc ram 
    if (monitorOpen) Serial.print(outBuf); 
    else delay(10); // allow time for other tasks
    if (sdLog) {
      if (log_remote_fp != NULL) {
        // output to SD if file opened
        fwrite(outBuf, sizeof(char), msgLen, log_remote_fp); // log.txt
        // periodic sync to SD
        if (counter_write++ % WRITE_CACHE_CYCLE == 0) fsync(fileno(log_remote_fp));
      } 
    }
    // output to web socket if open
    if (msgLen > 1) {
      outBuf[msgLen - 1] = 0; // lose final '/n'
    }
    xSemaphoreGive(logMutex);
  } 
}

void logLine() {
  logPrint(" \n");
}

void logSetup() {
  // prep logging environment
  Serial.begin(115200);
  Serial.setDebugOutput(DBG_ON);
  printf("\n\n");
  if (DEBUG_MEM) printf("init > Free: heap %lu\n", ESP.getFreeHeap()); 
  if (!DBG_ON) esp_log_level_set("*", ESP_LOG_NONE); // suppress ESP_LOG_ERROR messages
  if (crashLoop == MAGIC_NUM) snprintf(startupFailure, SF_LEN, STARTUP_FAIL "Crash loop detected, check log %s", (brownoutStatus == 'B' || brownoutStatus == 'R') ? "(brownout)" : " ");
  crashLoop = MAGIC_NUM;
  logSemaphore = xSemaphoreCreateBinary(); // flag that log message formatted
  logMutex = xSemaphoreCreateMutex(); // control access to log formatter
  xSemaphoreGive(logSemaphore);
  xSemaphoreGive(logMutex);
  xTaskCreate(logTask, "logTask", LOG_STACK_SIZE, NULL, LOG_PRI, &logHandle);
  if (mlogEnd >= RAM_LOG_LEN) ramLogClear(); // init
  LOG_INF("Setup RAM based log, size %u, starting from %u\n\n", RAM_LOG_LEN, mlogEnd);
  LOG_INF("=============== %s %s ===============", APP_NAME, APP_VER);
  initBrownout();
  prepInternalTemp();
  LOG_INF("Compiled with arduino-esp32 v%s", ESP_ARDUINO_VERSION_STR);
  wakeupResetReason();
  if (alertBuffer == NULL) alertBuffer = (byte*)ps_malloc(MAX_ALERT); 
  if (jsonBuff == NULL) jsonBuff = psramFound() ? (char*)ps_malloc(JSON_BUFF_LEN) : (char*)malloc(JSON_BUFF_LEN); 
  debugMemory("logSetup"); 
}

void formatHex(const char* inData, size_t inLen) {
  // format data as hex bytes for output
  char formatted[(inLen * 3) + 1];
  for (int i=0; i<inLen; i++) sprintf(formatted + (i*3), "%02x ", inData[i]);
  formatted[(inLen * 3)] = 0; // terminator
  LOG_INF("Hex: %s", formatted);
}

const char* espErrMsg(esp_err_t errCode) {
  // convert esp error code to text
  static char errText[100];
  esp_err_to_name_r(errCode, errText, 100);
  return errText;
}

void forceCrash() {
  // force crash for testing purposes
  delay(5000);
#pragma GCC diagnostic ignored "-Wdiv-by-zero"
  printf("%u\n", 1/0);
#pragma GCC diagnostic warning "-Wdiv-by-zero"
}

/****************** base 64 ******************/

#define BASE64 "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

const uint8_t* encode64chunk(const uint8_t* inp, int rem) {
  // receive 3 byte input buffer and return 4 byte base64 buffer
  rem = 3 - rem; // last chunk may be less than 3 bytes 
  uint32_t buff = 0; // hold 3 bytes as shifted 24 bits
  static uint8_t b64[4];
  // shift input into buffer
  for (int i = 0; i < 3 - rem; i++) buff |= inp[i] << (8*(2-i)); 
  // shift 6 bit output from buffer and encode
  for (int i = 0; i < 4 - rem; i++) b64[i] = BASE64[buff >> (6*(3-i)) & 0x3F]; 
  // filler for last chunk if less than 3 bytes
  for (int i = 0; i < rem; i++) b64[3-i] = '='; 
  return b64;
}

const char* encode64(const char* inp) {
  // helper to base64 encode strings up to 90 chars long
  static char encoded[121]; // space for 4/3 expansion + terminator
  encoded[0] = 0;
  int len = strlen(inp);
  if (len > 90) {
    LOG_WRN("Input string too long: %u chars", len);
    len = 90;
  }
  for (int i = 0; i < len; i += 3) 
    strncat(encoded, (char*)encode64chunk((uint8_t*)inp + i, min(len - i, 3)), 4);
  return encoded;
}


/************** qualitive core idle time monitoring *************/

// not working properly
#include "esp_freertos_hooks.h"

#define INTERVAL_TIME 100 // reporting interval in ms
#define TICKS_PER_INTERVAL (pdMS_TO_TICKS(INTERVAL_TIME))

static uint32_t idleCalls[portNUM_PROCESSORS] = {0};
static uint32_t idleCnt[portNUM_PROCESSORS];

static bool hookCallback() {
  idleCalls[xPortGetCoreID()]++;
  return true;
}

uint32_t* reportIdle() {
  static uint32_t idlePercent[portNUM_PROCESSORS];
  for (int i = 0; i < portNUM_PROCESSORS; i++)
    idlePercent[i] = (100 * idleCnt[i]) / TICKS_PER_INTERVAL;
  return idlePercent;
}

static void idleMonTask(void* p) {
  while (true) {
    for (int i = 0; i < portNUM_PROCESSORS; i++) {
      idleCnt[i] = idleCalls[i];
      idleCalls[i] = 0;
    }
    vTaskDelay(TICKS_PER_INTERVAL);
  }
  vTaskDelete(NULL);
}

void startIdleMon() {
  // report on each core idle time per interval
  // Core 0: wifi, Core 1: Arduino
  LOG_INF("Start core idle time monitoring @ interval %ums", INTERVAL_TIME);
  for (int i = 0; i < portNUM_PROCESSORS; i++) 
    esp_register_freertos_idle_hook_for_cpu(hookCallback, i);
  xTaskCreatePinnedToCore(idleMonTask, "idlemon", 1024, NULL, IDLEMON_PRI, NULL, 0);
}


/****************** send device to sleep (light or deep) & watchdog ******************/

#include <esp_wifi.h>
#include <driver/gpio.h>

static esp_sleep_wakeup_cause_t printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : LOG_INF("Wakeup by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : LOG_INF("Wakeup by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : LOG_INF("Wakeup by internal timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : LOG_INF("Wakeup by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : LOG_INF("Wakeup by ULP program"); break;
    case ESP_SLEEP_WAKEUP_GPIO: LOG_INF("Wakeup by GPIO"); break;    
    case ESP_SLEEP_WAKEUP_UART: LOG_INF("Wakeup by UART"); break; 
    default : LOG_INF("Wakeup by reset"); break;
  }
  return wakeup_reason;
}


static esp_reset_reason_t printResetReason() {
  esp_reset_reason_t bootReason = esp_reset_reason();
  switch (bootReason) {
    case ESP_RST_UNKNOWN: LOG_INF("Reset for unknown reason"); break;
    case ESP_RST_POWERON: {
      LOG_INF("Power on reset");
      brownoutStatus = 0;
      messageLog[0] = 0;
      break;
    }
    case ESP_RST_EXT: LOG_INF("Reset from external pin"); break;
    case ESP_RST_SW: LOG_INF("Software reset via esp_restart"); break;
    case ESP_RST_PANIC: LOG_INF("Software reset due to exception/panic"); break;
    case ESP_RST_INT_WDT: LOG_INF("Reset due to interrupt watchdog"); break;
    case ESP_RST_TASK_WDT: LOG_INF("Reset due to task watchdog"); break;
    case ESP_RST_WDT: LOG_INF("Reset due to other watchdogs"); break;
    case ESP_RST_DEEPSLEEP: LOG_INF("Reset after exiting deep sleep mode"); break;
    case ESP_RST_BROWNOUT: LOG_INF("Software reset due to brownout"); break;
    case ESP_RST_SDIO: LOG_INF("Reset over SDIO"); break;
    default: LOG_WRN("Unhandled reset reason"); break;
  }
  return bootReason;
}

esp_sleep_wakeup_cause_t wakeupResetReason() {
  printResetReason();
  esp_sleep_wakeup_cause_t wakeupReason = printWakeupReason();
  return wakeupReason;
}

void goToSleep(int wakeupPin, bool deepSleep) {
#if !CONFIG_IDF_TARGET_ESP32C3
  // if deep sleep, restarts with reset
  // if light sleep, restarts by continuing this function
  LOG_INF("Going into %s sleep", deepSleep ? "deep" : "light");
  delay(100);
  if (deepSleep) { 
    if (wakeupPin >= 0) {
      // wakeup on pin low
      pinMode(wakeupPin, INPUT_PULLUP);
      esp_sleep_enable_ext0_wakeup((gpio_num_t)wakeupPin, 0);
    }
    esp_deep_sleep_start();
  } else {
    // light sleep
    esp_wifi_stop();
    // wakeup on pin high
    if (wakeupPin >= 0) gpio_wakeup_enable((gpio_num_t)wakeupPin, GPIO_INTR_HIGH_LEVEL); 
    esp_light_sleep_start();
  }
  // light sleep restarts here
  LOG_INF("Light sleep wakeup");
  esp_wifi_start();
#else
  LOG_WRN("This function not compatible with ESP32-C3");
#endif
}


// catch software resets due to brownouts
//https://github.com/espressif/esp-idf/blob/master/components/esp_system/port/brownout.c

#include "esp_private/system_internal.h"
#include "esp_private/rtc_ctrl.h"
#include "hal/brownout_ll.h"

#include "soc/rtc_periph.h"
#include "hal/brownout_hal.h"

#define BROWNOUT_DET_LVL 7

IRAM_ATTR static void notifyBrownout(void *arg) {
  esp_cpu_stall(!xPortGetCoreID());  // Stop the other core.
  esp_reset_reason_set_hint(ESP_RST_BROWNOUT);
  brownoutStatus = 'B';
  esp_restart_noos(); // dirty reboot
}

static void initBrownout(void) {
  // brownout warning only output once to prevent bootloop
  if (brownoutStatus == 'R') LOG_WRN("Brownout warning previously notified");
  else if (brownoutStatus == 'B') {
    LOG_WRN("Brownout occurred due to inadequate power supply");
    brownoutStatus = 'R';
  } else {
    brownout_hal_config_t cfg = {
      .threshold = BROWNOUT_DET_LVL,
      .enabled = true,
      .reset_enabled = false,
      .flash_power_down = true,
      .rf_power_down = true,
    };
    brownout_hal_config(&cfg);
    brownout_ll_intr_clear();
    rtc_isr_register(notifyBrownout, NULL, RTC_CNTL_BROWN_OUT_INT_ENA_M, RTC_INTR_FLAG_IRAM);
    brownout_ll_intr_enable(true);
    brownoutStatus = 0; 
  }
}

// Global MJPEG2SD declarations
//
// s60sc 2021, 2022, 2024

#pragma once
#include "globals.h"

#define CAMERA_MODEL_FREENOVE_ESP32S3_CAM

/**************************************************************************/

#define ALLOW_SPACES false  // set true to allow whitespace in configs.txt key values

/*********************** Fixed defines leave as is ***********************/ 
/** Do not change anything below here unless you know what you are doing **/

#include "esp_camera.h"
#include "camera_pins.h"

//#define DEV_ONLY // leave commented out
#define STATIC_IP_OCTAL "132" // dev only
#define DEBUG_MEM false // leave as false
#define FLUSH_DELAY 0 // for debugging crashes
#define DBG_ON false // esp debug output
#define DOT_MAX 50
#define HOSTNAME_GRP 99
//#define REPORT_IDLE // core processor idle time monitoring
 
#define APP_VER "10.4.2"

#define APP_NAME "ESP-CAM_MJPEG" // max 15 chars
#define INDEX_PAGE_PATH DATA_DIR "/MJPEG2SD" HTML_EXT
#define FILE_NAME_LEN 64
#define IN_FILE_NAME_LEN (FILE_NAME_LEN * 2)
#define JSON_BUFF_LEN (32 * 1024) // set big enough to hold all file names in a folder
#define MAX_CONFIGS 200 // must be > number of entries in configs.txt
#define MAX_JPEG (ONEMEG / 2) // UXGA jpeg frame buffer at highest quality 375kB rounded up
#define MIN_RAM 8 // min object size stored in ram instead of PSRAM default is 4096
#define MAX_RAM 4096 // max object size stored in ram instead of PSRAM default is 4096
#define TLS_HEAP (64 * 1024) // min free heap for TLS session
#define WARN_HEAP (32 * 1024) // low free heap warning
#define WARN_ALLOC (16 * 1024) // low free max allocatable free heap block
#define MAX_FRAME_WAIT 1200
#define RGB888_BYTES 3 // number of bytes per pixel
#define GRAYSCALE_BYTES 1 // number of bytes per pixel 
#define MAX_ALERT MAX_JPEG

#ifdef NO_SD
#define STORAGE LittleFS
#else
#define STORAGE SD_MMC
#endif
#define GITHUB_PATH "/s60sc/ESP32-CAM_MJPEG2SD/master"
#define RAMSIZE (1024 * 8) // set this to multiple of SD card sector size (512 or 1024 bytes)
#define CHUNKSIZE (1024 * 4)
#define ISCAM // cam specific code in generics

// to determine if newer data files need to be loaded
#define CFG_VER 23

#define AVI_EXT "avi"
#define CSV_EXT "csv"
#define SRT_EXT "srt"
#define AVI_HEADER_LEN 310 // AVI header length
#define CHUNK_HDR 8 // bytes per jpeg hdr in AVI 
#define WAVTEMP "/current.wav"
#define AVITEMP "/current.avi"
#define TLTEMP "/current.tl"
#define TELETEMP "/current.csv"
#define SRTTEMP "/current.srt"

#define DMA_BUFF_LEN 512 // used for I2S buffer size
#define DMA_BUFF_CNT 4
#define MIC_GAIN_CENTER 3 // mid point

#ifdef CONFIG_IDF_TARGET_ESP32S3 
#define SERVER_STACK_SIZE (1024 * 8)
#define DS18B20_STACK_SIZE (1024 * 2)
#else
#define SERVER_STACK_SIZE (1024 * 4)
#define DS18B20_STACK_SIZE (1024)
#endif
#define STICK_STACK_SIZE (1024 * 4)
#define BATT_STACK_SIZE (1024 * 2)
#define CAPTURE_STACK_SIZE (1024 * 4)
#define EMAIL_STACK_SIZE (1024 * 6)
#define FS_STACK_SIZE (1024 * 4)
#define LOG_STACK_SIZE (1024 * 3)
#define AUDIO_STACK_SIZE (1024 * 4)
#define MICREM_STACK_SIZE (1024 * 2)
#define MQTT_STACK_SIZE (1024 * 4)
#define PING_STACK_SIZE (1024 * 5)
#define PLAYBACK_STACK_SIZE (1024 * 2)
#define SERVO_STACK_SIZE (1024)
#define SUSTAIN_STACK_SIZE (1024 * 4)
#define TGRAM_STACK_SIZE (1024 * 6)
#define TELEM_STACK_SIZE (1024 * 4)
#define HB_STACK_SIZE (1024 * 2)
#define UART_STACK_SIZE (1024 * 2)
#define INTERCOM_STACK_SIZE (1024 * 2)

// task priorities
#define CAPTURE_PRI 6
#define SUSTAIN_PRI 5
#define HTTP_PRI 5
#define STICK_PRI 5
#define AUDIO_PRI 5
#define INTERCOM_PRI 5
#define PLAY_PRI 4
#define TELEM_PRI 3
#define TGRAM_PRI 1
#define EMAIL_PRI 1
#define FTP_PRI 1
#define LOG_PRI 1
#define MQTT_PRI 1
#define LED_PRI 1
#define SERVO_PRI 1
#define HB_PRI 1
#define UART_PRI 1
#define DS18B20_PRI 1
#define BATT_PRI 1
#define IDLEMON_PRI 5

/******************** Function declarations *******************/

struct mjpegStruct {
  size_t buffLen;
  size_t buffOffset;
  size_t jpegSize;
};

struct fnameStruct {
  uint8_t recFPS;
  uint32_t recDuration;
  uint16_t frameCnt;
};

enum audioAction {NO_ACTION, UPDATE_CONFIG, RECORD_ACTION, PLAY_ACTION, PASS_ACTION, WAV_ACTION, STOP_ACTION};

// global app specific functions

void applyFilters();
void applyVolume();
void appShutdown();
void browserMicInput(uint8_t* wsMsg, size_t wsMsgLen);
void buildAviHdr(uint8_t FPS, uint8_t frameType, uint16_t frameCnt, bool isTL = false);
void buildAviIdx(size_t dataSize, bool isVid = true, bool isTL = false);
size_t buildSubtitle(int srtSeqNo, uint32_t sampleInterval);
void buzzerAlert(bool buzzerOn);
bool checkMotion(camera_fb_t* fb, bool motionStatus, bool lightLevelOnly = false);
int8_t checkPotVol(int8_t adjVol);
bool checkSDFiles();
void currentStackUsage();
void displayAudioLed(int16_t audioSample);
void finalizeAviIndex(uint16_t frameCnt, bool isTL = false);
void finishAudioRecord(bool isValid);
float* getBMx280();
float* getMPU9250();
mjpegStruct getNextFrame(bool firstCall = false);
int getInputPeripheral(uint8_t cmd);
bool getPIRval();
bool haveWavFile(bool isTL = false);
bool identifyBMx();
void intercom();
bool isNight(uint8_t nightSwitch);
void keepFrame(camera_fb_t* fb);
void micTaskStatus();
void motorSpeed(int speedVal, bool leftMotor = true);
void notifyMotion(camera_fb_t* fb);
void openSDfile(const char* streamFile);
void prepAudio();
void prepAviIndex(bool isTL = false);
bool prepCam();
void prepI2Ccam(int camSda, int camScl);
bool prepI2Cdevices();
bool prepRecording();
void prepTelemetry();
void prepMic();
void prepMotors();
void prepUart();
void setCamPan(int panVal);
void setCamTilt(int tiltVal);
uint8_t setFPS(uint8_t val);
uint8_t setFPSlookup(uint8_t val);
void setInputPeripheral(uint8_t cmd, uint32_t controlVal);
void setLamp(uint8_t lampVal);
void setLightsRC(bool lightsOn);
bool setOutputPeripheral(uint8_t cmd, uint32_t rxValue);
void setSteering(int steerVal);
void setStepperPin(uint8_t pinNum, uint8_t pinPos);
void setStickTimer(bool restartTimer, uint32_t interval = 0);
void startAudioRecord();
void startHeartbeat();
void startSustainTasks();
bool startTelemetry();
void stepperDone();
void stepperRun(float RPM, float revFraction, bool _clockwise);
void stopPlaying();
void stopSustainTask(int taskId);
void stopTelemetry(const char* fileName);
void storeSensorData(bool fromStream);
void takePhotos(bool startPhotos);
void trackSteeering(int controlVal, bool steering);
size_t updateWavHeader();
size_t writeAviIndex(byte* clientBuf, size_t buffSize, bool isTL = false);
bool writeUart(uint8_t cmd, uint32_t outputData);
size_t writeWavFile(byte* clientBuf, size_t buffSize);

/******************** Global app declarations *******************/

// // motion detection parameters
// extern int moveStartChecks; // checks per second for start motion
// extern int moveStopSecs; // secs between each check for stop, also determines post motion time
extern int maxFrames; // maximum number of frames in video before auto close 

// // motion recording parameters
// extern int detectMotionFrames; // min sequence of changed frames to confirm motion 
// extern int detectNightFrames; // frames of sequential darkness to avoid spurious day / night switching
// extern int detectNumBands;
// extern int detectStartBand;
// extern int detectEndBand; // inclusive
// extern int detectChangeThreshold; // min difference in pixel comparison to indicate a change
// extern bool mlUse; // whether to use ML for motion detection, requires INCLUDE_TINYML to be true
// extern float mlProbability; // minimum probability (0.0 - 1.0) for positive classification

// status & control fields 
extern const char* appConfig;
extern bool autoUpload;
extern bool dbgMotion;
extern bool doPlayback;
extern bool doRecording; // whether to capture to SD or not
extern bool forceRecord; // Recording enabled by rec button
extern bool forcePlayback; // playback enabled by user
extern uint8_t FPS;
extern uint8_t fsizePtr; // index to frameData[] for record
extern bool isCapturing;
extern uint8_t lightLevel;  
extern uint8_t lampLevel;  
extern int micGain;
extern int8_t ampVol;
extern uint8_t minSeconds; // default min video length (includes moveStopSecs time)
extern float motionVal;  // motion sensitivity setting - min percentage of changed pixels that constitute a movement
extern uint8_t nightSwitch; // initial white level % for night/day switching
extern bool nightTime; 
extern bool stopPlayback;
extern bool useMotion; // whether to use camera for motion detection (with motionDetect.cpp)  
extern uint8_t colorDepth;
extern bool timeLapseOn; // enable time lapse recording
extern int maxFrames;
extern uint8_t xclkMhz;
extern char camModel[];
extern bool doKeepFrame;
extern int alertMax; // too many could cause account suspension (daily emails)
extern bool streamNvr;
extern bool streamSnd;
extern bool streamSrt;
extern uint8_t numStreams;
extern uint8_t vidStreams;

// buffers
extern uint8_t iSDbuffer[];
extern uint8_t aviHeader[];
extern const uint8_t dcBuf[]; // 00dc
extern const uint8_t wbBuf[]; // 01wb
extern byte* streamBuffer[]; // buffer for stream frame
extern size_t streamBufferSize[];
extern uint8_t* motionJpeg;
extern size_t motionJpegLen;
extern uint8_t* audioBuffer;
extern size_t audioBytes;
extern char srtBuffer[];
extern size_t srtBytes;

// peripherals used
extern bool pirUse; // true to use PIR or radar sensor (RCWL-0516) for motion detection
extern bool lampAuto; // if true in conjunction with usePir, switch on lamp when PIR activated
extern bool lampNight;
extern int lampType;
extern bool voltUse; // true to report on ADC pin eg for for battery
extern bool wakeUse;
extern bool buzzerUse; // true to use active buzzer
extern int buzzerPin; 
extern int buzzerDuration; 
extern int relayPin;
extern bool relayMode;

// External Heartbeat
extern bool external_heartbeat_active;
extern char external_heartbeat_domain[]; //External Heartbeat domain/IP  
extern char external_heartbeat_uri[];    //External Heartbeat uri (i.e. /myesp32-cam-hub/index.php)
extern int external_heartbeat_port;      //External Heartbeat server port to connect.  
extern char external_heartbeat_token[];  //External Heartbeat server auth token.  

// task handling
extern TaskHandle_t battHandle;
extern TaskHandle_t captureHandle;
extern TaskHandle_t DS18B20handle;
extern TaskHandle_t emailHandle;
extern TaskHandle_t fsHandle;
extern TaskHandle_t logHandle;
extern TaskHandle_t mqttTaskHandle;
extern TaskHandle_t playbackHandle;
extern esp_ping_handle_t pingHandle;
extern TaskHandle_t servoHandle;
extern TaskHandle_t stickHandle;
extern TaskHandle_t sustainHandle[];
extern TaskHandle_t telegramHandle;
extern TaskHandle_t telemetryHandle;
extern TaskHandle_t uartRxHandle;
extern TaskHandle_t audioHandle;
extern SemaphoreHandle_t frameSemaphore[];
extern SemaphoreHandle_t motionSemaphore;


/************************** structures ********************************/

struct frameStruct {
  const char* frameSizeStr;
  const uint16_t frameWidth;
  const uint16_t frameHeight;
  const uint16_t defaultFPS;
  const uint8_t scaleFactor; // (0..4)
  const uint8_t sampleRate; // (1..N)
};

// indexed by frame size - needs to be consistent with sensor.h framesize_t enum
const frameStruct frameData[] = {
  {"96X96", 96, 96, 30, 1, 1},   // 2MP sensors
  {"QQVGA", 160, 120, 30, 1, 1},
  {"QCIF", 176, 144, 30, 1, 1}, 
  {"HQVGA", 240, 176, 30, 2, 1}, 
  {"240X240", 240, 240, 30, 2, 1}, 
  {"QVGA", 320, 240, 30, 2, 1}, 
  {"CIF", 400, 296, 30, 2, 1},  
  {"HVGA", 480, 320, 30, 2, 1}, 
  {"VGA", 640, 480, 20, 3, 1}, 
  {"SVGA", 800, 600, 20, 3, 1}, 
  {"XGA", 1024, 768, 5, 3, 1},   
  {"HD", 1280, 720, 5, 3, 1}, 
  {"SXGA", 1280, 1024, 5, 3, 1}, 
  {"UXGA", 1600, 1200, 5, 4, 1},  
  {"FHD", 920, 1080, 5, 3, 1},    // 3MP Sensors
  {"P_HD", 720, 1280, 5, 3, 1},
  {"P_3MP", 864, 1536, 5, 3, 1},
  {"QXGA", 2048, 1536, 5, 4, 1},
  {"QHD", 2560, 1440, 5, 4, 1},   // 5MP Sensors
  {"WQXGA", 2560, 1600, 5, 4, 1},
  {"P_FHD", 1080, 1920, 5, 4, 1},
  {"QSXGA", 2560, 1920, 4, 4, 1}
};

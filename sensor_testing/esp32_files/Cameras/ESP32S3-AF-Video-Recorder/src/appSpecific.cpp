// mjpeg2sd app specific functions

#include "appGlobals.h"

static char variable[FILE_NAME_LEN]; 
static char value[FILE_NAME_LEN];
static char alertCaption[100];
static bool alertReady = false;
static bool depthColor = true;
static bool devHub = false;
char AuxIP[MAX_IP_LEN];
bool useUart = false; 
volatile audioAction THIS_ACTION = PASS_ACTION;
static void stopRC();

/************************ webServer callbacks *************************/

bool updateAppStatus(const char* variable, const char* value, bool fromUser) {
  // update vars from browser input
  esp_err_t res = ESP_OK; 
  sensor_t* s = esp_camera_sensor_get();
  int intVal = atoi(value);
  float fltVal = atof(value);
  if (!strcmp(variable, "custom")) return res;
#ifndef AUXILIARY
  else if (!strcmp(variable, "stopPlaying")) stopPlaying();
  else if (!strcmp(variable, "minf")) minSeconds = intVal; 
  else if (!strcmp(variable, "motionVal")) motionVal = intVal;
  // else if (!strcmp(variable, "moveStartChecks")) moveStartChecks = intVal;
  // else if (!strcmp(variable, "moveStopSecs")) moveStopSecs = intVal;
  // else if (!strcmp(variable, "maxFrames")) maxFrames = intVal;
  // else if (!strcmp(variable, "detectMotionFrames")) detectMotionFrames = intVal;
  // else if (!strcmp(variable, "detectNightFrames")) detectNightFrames = intVal;
  // else if (!strcmp(variable, "detectNumBands")) detectNumBands = intVal;
  // else if (!strcmp(variable, "detectStartBand")) detectStartBand = intVal;
  // else if (!strcmp(variable, "detectEndBand")) detectEndBand = intVal;
  // else if (!strcmp(variable, "detectChangeThreshold")) detectChangeThreshold = intVal;
  // else if (!strcmp(variable, "mlUse")) mlUse = (bool)intVal;
  // else if (!strcmp(variable, "mlProbability")) mlProbability = fltVal < 0 ? 0.0 : (fltVal > 1.0 ? 1.0 : fltVal);
  else if (!strcmp(variable, "depthColor")) {
    depthColor = (bool)intVal;
    colorDepth = depthColor ? RGB888_BYTES : GRAYSCALE_BYTES;
  }
  else if (!strcmp(variable, "enableMotion")) {
    // Turn on/off motion detection 
    useMotion = (intVal) ? false : false; //SUPPOSED TO BE TRUE TRUE
    if (fsizePtr > 16 && useMotion) {
      useMotion = false;
      updateConfigVect("enableMotion", "0");
      LOG_WRN("Motion detection disabled as frame size %s is too large", frameData[fsizePtr].frameSizeStr);
    } else LOG_INF("%s motion detection", useMotion ? "Enabling" : "Disabling");
  }
  else if (!strcmp(variable, "timeLapseOn")) timeLapseOn = intVal;
  else if (!strcmp(variable, "lswitch")) nightSwitch = intVal;
#endif
  else if (!strcmp(variable, "delete")) {
    stopPlayback = true;
    deleteFolderOrFile(value);
  }
  else if (!strcmp(variable, "record")) doRecording = (intVal) ? true : false;   
  else if (!strcmp(variable, "forceRecord")) forceRecord = (intVal) ? true : false; 
  else if (!strcmp(variable, "dbgMotion")) {
    // only enable show motion if motion detect enabled
    dbgMotion = (intVal && useMotion) ? true : false;
    doRecording = !dbgMotion;
  }
  else if (!strcmp(variable, "devHub")) devHub = (bool)intVal;   
  else if (!strcmp(variable, "wakeUse")) wakeUse = (bool)intVal;
#ifndef AUXILIARY
  else if (!strcmp(variable, "AuxIP")) strncpy(AuxIP, value, MAX_IP_LEN-1);
#endif
  else if (!strcmp(variable, "useUart")) useUart = (bool)intVal;

#ifndef AUXILIARY
  // camera settings
  else if (!strcmp(variable, "xclkMhz")) xclkMhz = intVal;
  else if (!strcmp(variable, "framesize")) {
    fsizePtr = intVal;
    if (s) {
      if (s->set_framesize(s, (framesize_t)fsizePtr) != ESP_OK) res = false;
      // update default FPS for this frame size
      if (playbackHandle != NULL) {
        setFPSlookup(fsizePtr);
        updateConfigVect("fps", String(FPS).c_str()); 
      }
      if (fsizePtr > 16 && useMotion) {
        useMotion = false;
        updateConfigVect("enableMotion", "0");
        LOG_WRN("Motion detection disabled as frame size %s is too large", frameData[fsizePtr].frameSizeStr);
      }
    }
  }
  else if (!strcmp(variable, "fps")) {
    FPS = intVal;
    if (playbackHandle != NULL) setFPS(FPS);
  }
  else if (s) {
    if (!strcmp(variable, "quality")) res = s->set_quality(s, intVal);
    else if (!strcmp(variable, "contrast")) res = s->set_contrast(s, intVal);
    else if (!strcmp(variable, "brightness")) res = s->set_brightness(s, intVal);
    else if (!strcmp(variable, "saturation")) res = s->set_saturation(s, intVal);
    else if (!strcmp(variable, "denoise")) res = s->set_denoise(s, intVal);    
    else if (!strcmp(variable, "sharpness")) res = s->set_sharpness(s, intVal);    
    else if (!strcmp(variable, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)intVal);
    else if (!strcmp(variable, "colorbar")) res = s->set_colorbar(s, intVal);
    else if (!strcmp(variable, "awb")) res = s->set_whitebal(s, intVal);
    else if (!strcmp(variable, "agc")) res = s->set_gain_ctrl(s, intVal);
    else if (!strcmp(variable, "aec")) res = s->set_exposure_ctrl(s, intVal);
    else if (!strcmp(variable, "hmirror")) res = s->set_hmirror(s, intVal);
    else if (!strcmp(variable, "vflip")) res = s->set_vflip(s, intVal);
    else if (!strcmp(variable, "awb_gain")) res = s->set_awb_gain(s, intVal);
    else if (!strcmp(variable, "agc_gain")) res = s->set_agc_gain(s, intVal);
    else if (!strcmp(variable, "aec_value")) res = s->set_aec_value(s, intVal);
    else if (!strcmp(variable, "aec2")) res = s->set_aec2(s, intVal);
    else if (!strcmp(variable, "dcw")) res = s->set_dcw(s, intVal);
    else if (!strcmp(variable, "bpc")) res = s->set_bpc(s, intVal);
    else if (!strcmp(variable, "wpc")) res = s->set_wpc(s, intVal);
    else if (!strcmp(variable, "raw_gma")) res = s->set_raw_gma(s, intVal);
    else if (!strcmp(variable, "lenc")) res = s->set_lenc(s, intVal);
    else if (!strcmp(variable, "special_effect")) res = s->set_special_effect(s, intVal);
    else if (!strcmp(variable, "wb_mode")) res = s->set_wb_mode(s, intVal);
    else if (!strcmp(variable, "ae_level")) res = s->set_ae_level(s, intVal);
    else res = ESP_FAIL;
  }
#endif
  return res == ESP_OK ? true : false;
}

static bool extractKeyVal(const char* wsMsg) {
  // extract key 
  strncpy(variable, wsMsg, FILE_NAME_LEN - 1); 
  char* endPtr = strchr(variable, '=');
  if (endPtr != NULL) {
    *endPtr = 0; // split variable into 2 strings, first is key name
    strcpy(value, variable + strlen(variable) + 1); // value is now second part of string
    return true;
  } else LOG_ERR("Invalid query string: %s", wsMsg);
  return false;
} 

esp_err_t appSpecificWebHandler(httpd_req_t *req, const char* variable, const char* value) {
  // update handling requiring response specific to mjpeg2sd
  if (!strcmp(variable, "sfile")) {
    // get folders / files on SD, save received filename if has required extension
    strcpy(inFileName, value);
    if (!forceRecord) doPlayback = listDir(inFileName, jsonBuff, JSON_BUFF_LEN, AVI_EXT); // browser control
    else strcpy(jsonBuff, "{}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, jsonBuff);
  } 
  else if (!strcmp(variable, "updateFPS")) {
    // requires response with updated default fps
    sprintf(jsonBuff, "{\"fps\":\"%u\"}", setFPSlookup(fsizePtr));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, jsonBuff);
  } 
  else if (!strcmp(variable, "still")) {
    // send single jpeg to browser
    uint32_t startTime = millis();
    doKeepFrame = true;
    while (doKeepFrame && millis() - startTime < MAX_FRAME_WAIT) delay(100);
    if (!doKeepFrame && alertBufferSize) {
      httpd_resp_set_type(req, "image/jpeg");
      httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
      httpd_resp_send(req, (const char*)alertBuffer, alertBufferSize);   
      uint32_t jpegTime = millis() - startTime;
      LOG_INF("JPEG: %uB in %ums", alertBufferSize, jpegTime);
      alertBufferSize = 0;
    } else LOG_WRN("Failed to get still");
  } else if (!strcmp(variable, "svg")) {
    // build svg image for use by another app's hub instead of image
    const char* svgHtml = R"~(
        <svg width="200" height="200" xmlns="http://www.w3.org/2000/svg">
          <rect width="100%" height="100%" fill="lightgray"/>
          <text x="50%" y="50%" text-anchor="middle" alignment-baseline="middle" font-size="30">
    )~";
    
    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.svg");
    httpd_resp_sendstr_chunk(req, svgHtml);
    httpd_resp_sendstr_chunk(req, "MJPE2SD");
    httpd_resp_sendstr_chunk(req, "°C</text></svg>");
    httpd_resp_sendstr_chunk(req, NULL);
  } else return ESP_FAIL;
  return ESP_OK;
}

static bool setPeripheral(char cmd, int controlVal, bool fromUart) {
  bool res = true;
  switch (cmd) {
    case 'K': 
      // cam browser conn closed
        stopRC();
    break;
    default:
      res = false;
    break;
  }
  return res;
}

void appSpecificWsHandler(const char* wsMsg) {
  // message from web socket
  int wsLen = strlen(wsMsg) - 1;
  char cmd = (char)wsMsg[0];
  int controlVal = atoi(wsMsg + 1); // skip first char
  bool aux = false;
  if (useUart && !aux) {
  } else {
    if (!setPeripheral(cmd, controlVal, false)) {
      switch (cmd) {
        case 'X':
        break;
        case 'C': 
          // control request
          if (extractKeyVal(wsMsg + 1)) updateStatus(variable, value);
        break;
        case 'S': 
          // status request
          buildJsonString(wsLen); // required config number 
          logPrint("%s\n", jsonBuff);
        break;   
        case 'U': 
          // update or control request
          memcpy(jsonBuff, wsMsg + 1, wsLen); // remove 'U'
          parseJson(wsLen);
        break;
        case 'H': 
          // browser keepalive heartbeat
          heartBeatDone = true;
        break;
        case 'K': 
          // kill websocket connection
          killSocket();
        break;
        default:
          LOG_WRN("unknown command %s", wsMsg);
        break;
      }
    }
  }
}

void appSpecificWsBinHandler(uint8_t* wsMsg, size_t wsMsgLen) {

}

void buildAppJsonString(bool filter) {
  // build app specific part of json string
  char* p = jsonBuff + 1;
  p += sprintf(p, "\"llevel\":%u,", lightLevel);
  p += sprintf(p, "\"night\":%s,", nightTime ? "\"Yes\"" : "\"No\"");
  float aTemp = readTemperature(true);
  if (aTemp > -127.0) p += sprintf(p, "\"atemp\":\"%0.1f\",", aTemp);
  else p += sprintf(p, "\"atemp\":\"n/a\",");
  float currentVoltage = readVoltage();
  if (currentVoltage < 0) p += sprintf(p, "\"battv\":\"n/a\",");
  else p += sprintf(p, "\"battv\":\"%0.1fV\",", currentVoltage); 
  if (forcePlayback && !doPlayback) {
    // switch off playback 
    forcePlayback = false;
    p += sprintf(p, "\"forcePlayback\":0,");  
  }
  p += sprintf(p, "\"showRecord\":%u,", (uint8_t)((isCapturing && doRecording) || forceRecord));
  p += sprintf(p, "\"camModel\":\"%s\",", camModel);
  p += sprintf(p, "\"sustainId\":\"%u\",", sustainId);     
  // Extend info
  uint8_t cardType = 99; // not MMC
  if ((fs::SDMMCFS*)&STORAGE == &SD_MMC) cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) p += sprintf(p, "\"card\":\"%s\",", "NO card");
  else {
    if (!filter) {
      if (cardType == CARD_MMC) p += sprintf(p, "\"card\":\"%s\",", "MMC"); 
      else if (cardType == CARD_SD) p += sprintf(p, "\"card\":\"%s\",", "SDSC");
      else if (cardType == CARD_SDHC) p += sprintf(p, "\"card\":\"%s\",", "SDHC"); 
      else if (cardType == 99) p += sprintf(p, "\"card\":\"%s\",", "LittlrFS"); 
    }
    if ((fs::SDMMCFS*)&STORAGE == &SD_MMC) p += sprintf(p, "\"card_size\":\"%s\",", fmtSize(SD_MMC.cardSize()));
    p += sprintf(p, "\"used_bytes\":\"%s\",", fmtSize(STORAGE.usedBytes()));
    p += sprintf(p, "\"free_bytes\":\"%s\",", fmtSize(STORAGE.totalBytes() - STORAGE.usedBytes()));
    p += sprintf(p, "\"total_bytes\":\"%s\",", fmtSize(STORAGE.totalBytes()));
  }
  p += sprintf(p, "\"free_psram\":\"%s\",", fmtSize(ESP.getFreePsram()));     
  //p += sprintf(p, "\"vcc\":\"%i V\",", ESP.getVcc() / 1023.0F; ); 
  *p = 0;
}

/******************************************************************/

void externalAlert(const char* subject, const char* message) {
  // alert any configured external servers
}

void displayAudioLed(int16_t audioSample) {
}

void setupAudioLed() {
}

int8_t checkPotVol(int8_t adjVol) {
  return adjVol; // dummy
}

void applyFilters() {
  applyVolume();
}

#if !INCLUDE_PERIPH
float readVoltage() {
  return -1.0;
}
float readTemperature(bool isCelsius, bool onlyDS18) {
  return readInternalTemp();
}
#endif

bool appDataFiles() {
  // callback from setupAssist.cpp, for any app specific files 
  return true;
}

void currentStackUsage() {
  checkStackUse(captureHandle, 0);
  checkStackUse(fsHandle, 3);
  checkStackUse(logHandle, 4);
  // 7: pingtask
  checkStackUse(playbackHandle, 8);
  checkStackUse(servoHandle, 9);
  checkStackUse(stickHandle, 10);
  // 14: http webserver
  for (int i=0; i < numStreams; i++) checkStackUse(sustainHandle[i], 15 + i);
}

static void stopRC() {
}

/************** default app configuration **************/
const char* appConfig = R"~(
ST_SSID~~99~~na
fsPort~21~99~~na
fsServer~~99~~na
ftpUser~~99~~na
fsWd~~99~~na
fsUse~~99~~na
smtp_port~465~99~~na
smtp_server~smtp.gmail.com~99~~na
smtp_login~~99~~na
smtp_email~~99~~na
Auth_Name~~99~~na
useHttps~~99~~na
useSecure~~99~~na
useFtps~~99~~na
extIP~~99~~na
restart~~99~~na
sdLog~0~99~~na
xclkMhz~20~98~~na
ae_level~-2~98~~na
aec~1~98~~na
aec2~1~98~~na
aec_value~204~98~~na
agc~1~98~~na
agc_gain~0~98~~na
autoUpload~0~98~~na
deleteAfter~0~98~~na
awb~1~98~~na
awb_gain~1~98~~na
bpc~1~98~~na
brightness~0~98~~na
colorbar~0~98~~na
contrast~0~98~~na
dcw~1~98~~na
enableMotion~1~98~~na
fps~20~98~~na
framesize~9~98~~na
gainceiling~0~98~~na
hmirror~0~98~~na
lampLevel~0~98~~na
lenc~1~98~~na
lswitch~10~98~~na
micGain~0~98~~na
ampVol~0~98~~na
minf~5~98~~na
motionVal~8~98~~na
quality~12~98~~na
raw_gma~1~98~~na
record~1~98~~na
saturation~0~98~~na
sharpness~0~98~~na
denoise~4~98~~na
special_effect~0~98~~na
timeLapseOn~0~98~~na
timezone~GMT0~98~~na
vflip~0~98~~na
wb_mode~0~98~~na
wpc~1~98~~na
ST_ip~~0~T~Static IP address
ST_gw~~0~T~Router IP address
ST_sn~255.255.255.0~0~T~Router subnet
ST_ns1~~0~T~DNS server
ST_ns2~~0~T~Alt DNS server
AP_Pass~~0~T~AP Password
AP_ip~~0~T~AP IP Address if not 192.168.4.1
AP_sn~~0~T~AP subnet
AP_gw~~0~T~AP gateway
allowAP~1~0~C~Allow simultaneous AP 
doGetExtIP~1~0~C~Enable get external IP
wifiTimeoutSecs~30~0~N~WiFi connect timeout (secs)
logType~0~99~N~Output log selection
ntpServer~pool.ntp.org~0~T~NTP Server address
alarmHour~1~2~N~Hour of day for daily actions
refreshVal~5~2~N~Web page refresh rate (secs)
responseTimeoutSecs~10~2~N~Server response timeout (secs)
useUart~0~3~C~Use UART for Auxiliary connection
uartTxdPin~~3~N~UART TX pin
uartRxdPin~~3~N~UART RX pin
tlSecsBetweenFrames~600~1~N~Timelapse interval (secs)
tlDurationMins~720~1~N~Timelapse duration (mins)
tlPlaybackFPS~1~1~N~Timelapse playback FPS
moveStartChecks~5~1~N~Checks per second for start motion
moveStopSecs~2~1~N~Non movement to stop recording (secs)
maxFrames~20000~1~N~Max frames in recording
detectMotionFrames~5~1~N~Num changed frames to start motion
detectNightFrames~10~1~N~Min dark frames to indicate night
detectNumBands~10~1~N~Total num of detection bands
detectStartBand~3~1~N~Top band where motion is checked
detectEndBand~8~1~N~Bottom band where motion is checked
detectChangeThreshold~15~1~N~Pixel difference to indicate change
mlUse~0~1~C~Use Machine Learning
mlProbability~0.8~1~N~ML minimum positive probability 0.0 - 1.0
depthColor~0~1~C~Color depth for motion detection: Gray <> RGB

smtpUse~0~2~C~Enable email sending
smtpMaxEmails~10~2~N~Max daily alerts
sdMinCardFreeSpace~100~2~N~Min free MBytes on SD before action
sdFreeSpaceMode~1~2~S:No Check:Delete oldest:Ftp then delete~Action mode on SD min free
formatIfMountFailed~0~2~C~Format file system on failure
pirUse~0~3~C~Use PIR for detection
lampType~0~3~S:Manual:PIR~How lamp activated
SVactive~0~3~C~Enable servo use
pirPin~~3~N~Pin used for PIR
lampPin~~3~N~Pin used for Lamp
servoPanPin~~6~N~Pin used for Pan Servo
servoTiltPin~~6~N~Pin used for Tilt Servo
ds18b20Pin~~3~N~Pin used for DS18B20 temperature sensor
AudActive~0~3~C~Show audio configuration
micSckPin~-1~7~N~Microphone I2S SCK pin
micSWsPin~-1~7~N~Microphone I2S WS, PDM CLK pin
micSdPin~-1~7~N~Microphone I2S SD, PDM DAT pin
mampBckIo~-1~7~N~Amplifier I2S BCLK (SCK) pin
mampSwsIo~-1~7~N~Amplifier I2S LRCLK (WS) pin
mampSdIo~-1~7~N~Amplifier I2S DIN pin
servoDelay~0~6~N~Delay between each 1 degree change (ms)
servoMinAngle~0~6~N~Set min angle for servo model
servoMaxAngle~180~6~N~Set max angle for servo model
servoMinPulseWidth~544~6~N~Set min pulse width for servo model (usecs)
servoMaxPulseWidth~2400~6~N~Set max pulse width for servo model (usecs)
servoCenter~90~6~N~Angle at which servo centered
voltDivider~2~3~N~Voltage divider resistor ratio
voltLow~3~3~N~Warning level for low voltage
voltInterval~5~3~N~Voltage check interval (mins)
voltPin~~3~N~ADC Pin used for battery voltage
voltUse~0~3~C~Use Voltage check
wakePin~~3~N~Pin used for to wake app from sleep
wakeUse~0~3~C~Deep sleep app during night
mqtt_active~0~2~C~Mqtt enabled
mqtt_broker~~2~T~Mqtt server ip to connect
mqtt_port~1883~2~N~Mqtt server port
mqtt_user~~2~T~Mqtt user name
mqtt_user_Pass~~2~T~Mqtt user password
mqtt_topic_prefix~homeassistant/~2~T~Mqtt topic path prefix
external_heartbeat_active~0~2~C~External Heartbeat Server enabled
external_heartbeat_domain~~2~T~Heartbeat receiver domain or IP (eg. www.espsee.com)
external_heartbeat_uri~~2~T~Heartbeat receiver URI (eg. /heartbeat/)
external_heartbeat_port~443~2~N~Heartbeat receiver port
external_heartbeat_token~~2~T~Heartbeat receiver auth token
usePing~1~0~C~Use ping
teleUse~0~3~C~Use telemetry recording
teleInterval~1~3~N~Telemetry collection interval (secs)
RCactive~0~3~C~Enable remote control
servoSteerPin~~4~N~Pin used for steering servo
motorRevPin~~4~N~Pin used for motor reverse / left track 
motorFwdPin~~4~N~Pin used for motor forward / left track 
motorRevPinR~~4~N~Pin used for right track reverse
motorFwdPinR~~4~N~Pin used for right track forward
lightsRCpin~~4~N~Pin used for RC lights output
heartbeatRC~5~4~N~RC connection heartbeat time (secs)
AuxIP~~3~T~Send RC / Servo / PG commands to auxiliary IP
stickXpin~~4~N~Pin used for joystick steering
stickYpin~~4~N~Pin used for joystick motor
stickzPushPin~~4~N~Pin used for joystick lights
stickUse~0~4~C~Use joystick
pwmFreq~50~4~N~RC Motor PWM frequency
maxSteerAngle~45~4~N~Max steering angle from straightahead
maxTurnSpeed~50~4~N~Max tracked turn speed differential 
maxDutyCycle~100~4~N~Max motor duty cycle % (speed)
minDutyCycle~10~4~N~Min motor duty cycle % (stop)
allowReverse~1~4~C~Reverse motion required
autoControl~1~4~C~Stop motor or center steering if control inactive
waitTime~20~4~N~Min wait (ms) between RC updates to app
tgramUse~0~2~C~Use Telegram Bot
tgramToken~~2~T~Telegram Bot token
tgramChatId~~2~T~Telegram chat identifier
devHub~0~2~C~Show Camera Hub tab
buzzerUse~0~3~C~Use active buzzer
buzzerPin~~3~N~Pin used for active buzzer
buzzerDuration~~3~N~Duration of buzzer sound in secs
stepIN1pin~-1~5~N~Stepper IN1 pin number
stepIN2pin~-1~5~N~Stepper IN2 pin number
stepIN3pin~-1~5~N~Stepper IN3 pin number
stepIN4pin~-1~5~N~Stepper IN4 pin number
PGactive~0~3~C~Enable photogrammetry
numberOfPhotos~20~5~N~Number of photos
RPM~1~5~N~Turntable revolution speed as RPM
gearing~5.7~5~N~Turntable / motor gearing ratio
clockwise~1~5~C~Clockwise turntable if true
timeForFocus~0~5~N~Time allocated to auto focus (secs)
timeForPhoto~2~5~N~Time allocated to take photo (secs)
pinShutter~-1~5~N~Pin connected to camera shutter
pinFocus~-1~5~N~Pin connected to camera focus
extCam~0~5~C~Use external camera
AtakePhotos~Start~5~A~Start photogrammetry
BabortPhotos~Abort~5~A~Abort photogrammetry
relayPin~-1~3~N~Pin to switch relay 
relayMode~0~3~S:Manual:Night~How relay activated
relaySwitch~0~3~C~Switch relay off / on
I2Csda~-1~3~N~I2C SDA pin if unshared
I2Cscl~-1~3~N~I2C SCL pin if unshared
)~";

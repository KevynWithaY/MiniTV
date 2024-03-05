/***
 * Required libraries:
 * Arduino_GFX: https://github.com/moononournation/Arduino_GFX.git
 * libhelix: https://github.com/pschatzmann/arduino-libhelix.git
 * JPEGDEC: https://github.com/bitbank2/JPEGDEC.git
 */

#include <WiFi.h>
#include <FS.h>

//#include <FFat.h>
//#include <LittleFS.h>
#include <SD.h>
#include <SPI.h>
//#include <SD_MMC.h>
//#include <SPIFFS.h>

/* audio */
#include "esp32_audio_task.h"

/* MJPEG Video */
#include "mjpeg_decode_draw_task.h"


// Filenames
// (auto fall back to MP3 if AAC file not available)
#define BASE_AUDIO_FILENAME "/44100"
#define MP3_EXTENSION ".mp3"
#define AAC_EXTENSION ".aac"

#define AAC_FILENAME BASE_AUDIO_FILENAME AAC_EXTENSION // "/44100.aac"
#define MP3_FILENAME BASE_AUDIO_FILENAME MP3_EXTENSION // "/44100.mp3"

//#define MJPEG_FILENAME "/128_30fps.mjpeg" 
#define MJPEG_FILENAME "/320_24fps.mjpeg"
//#define MJPEG_FILENAME "/280_24fps.mjpeg"
//#define MJPEG_FILENAME "/320_30fps.mjpeg"
//#define MJPEG_FILENAME "/288_30fps.mjpeg"
// #define MJPEG_FILENAME "/320_30fps.mjpeg"

// Frames per second
#define FPS 24 //30

// Video size
#define VID_WIDTH 320 //288 // 128
#define VID_HEIGHT 240 // 240 // 128


// Buffer size (based on video size) [width * height * 2 / 8]
//#define MJPEG_BUFFER_SIZE (288 * 240 * 2 / 8)
//#define MJPEG_BUFFER_SIZE (320 * 240 * 2 / 8)
//#define MJPEG_BUFFER_SIZE (128 * 128 * 2 / 8)
#define MJPEG_BUFFER_SIZE ((VID_WIDTH * VID_HEIGHT * 2 / 8) + 256)

// Multi-tasking:
#define AUDIOASSIGNCORE 1
#define DECODEASSIGNCORE 0
#define DRAWASSIGNCORE 1

// Legend display:
#define CHART_MARGIN 64
#define LEGEND_A_COLOR 0x1BB6
#define LEGEND_B_COLOR 0xFBE1
#define LEGEND_C_COLOR 0x2D05
#define LEGEND_D_COLOR 0xD125
#define LEGEND_E_COLOR 0x9337
#define LEGEND_F_COLOR 0x8AA9
#define LEGEND_G_COLOR 0xE3B8
#define LEGEND_H_COLOR 0x7BEF
#define LEGEND_I_COLOR 0xBDE4
#define LEGEND_J_COLOR 0x15F9


/*******************************************************************************
 * Pin Definitions
 ******************************************************************************/

// I2S Pins:
// ---------
//#define USE_I2S       // Priority audio, will be used if not commented out
#define USE_A2DP_OUT    // Secondary, only used if USE_I2S is commented out and this one is not commented out

//#ifdef USE_I2S
// --
#define I2S_BCLK 26   // A0
#define I2S_LRCLK 25  // A1
#define I2S_DOUT 27   // A2
// --
//#else
// --
//#ifdef USE_A2DP_OUT

#include "AudioTools.h"
#include "AudioLibs/AudioA2DP.h"
#include "AudioLibs/AudioSourceSDFAT.h"
#include "AudioCodecs/CodecMP3Helix.h"

const char* BLUETOOTH_REC_NAME = "Motorola S305";  // The name of the bluetooth receiver to which we'll try to connect

A2DPStream out;
MP3DecoderHelix decoder;
AudioPlayer player;
//#endif
// --
//#endif



// SD Card Pins:
// -------------
#define SD_CS 5 //15 
#define SD_MOSI 23 //13 
#define SD_CLK 18 //14 
#define SD_MISO 19 //12 

#define SD_SPIMODE  VSPI
//#define SD_FREQUENCY 40000000  // comment out to use default of 4 Mhz (4000000)

// TFT Pins:
// ---------
#define TFT_RST -1 //33
#define GFX_BL 21 // DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin
#define TFT_DC 2
#define TFT_SCK 14
#define TFT_MOSI 13
#define TFT_CS 15
#define TFT_MISO 12

#define TFT_SPIMODE HSPI
#define TFT_FREQUENCY 80000000  // comment out to not specify the frequency
#define TFT_INVERTDISPLAY false

/*******************************************************************************
 * Start of Arduino_GFX setting
 ******************************************************************************/

#include <Arduino_GFX_Library.h>

#define TFT_BRIGHTNESS 128

// -- Specify bus --

//Arduino_DataBus *bus = create_default_Arduino_DataBus();
//Arduino_GFX *gfx = new Arduino_ST7789(bus, DF_GFX_RST, 1 /* rotation */, true /* IPS */, 240 /* width */, 288 /* height */, 0 /* col offset 1 */, 20 /* row offset 1 */, 0 /* col offset 2 */, 12 /* row offset 2 */);

// This "DMA" bus worked, but skipped 4.1% of frame compared to 0% for the Arduino_ESP32SPI on the 24fps 320x240 video I tried...
// Arduino_DataBus *bus = new Arduino_ESP32SPIDMA(TFT_DC /* DC */, TFT_CS /* CS */, TFT_SCK /* SCK */, TFT_MOSI /* MOSI */, TFT_MISO /* MISO */, 
//   TFT_SPIMODE /* VSPI or HSPI */, false /* Shared Bus? */);

// ESP32 SPI bus:
Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(TFT_DC /* DC */, TFT_CS /* CS */, TFT_SCK /* SCK */, TFT_MOSI /* MOSI */, TFT_MISO /* MISO */,
  TFT_SPIMODE /* VSPI or HSPI */, false /* Shared Bus? */);

// -- Specify display --

// ST7789 on CYD2USB
Arduino_GFX *gfx = new Arduino_ST7789(
  bus, TFT_RST /* RST */, 1 /* rotation */);


/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

/* variables */
static int next_frame = 0;
static int skipped_frames = 0;
static unsigned long start_ms, curr_ms, next_frame_ms;


/*******************************************************************************
 * Functions
 ******************************************************************************/

// Pixel drawing callback
static int drawMCU(JPEGDRAW *pDraw)
{
  // Serial.printf("Draw pos = (%d, %d), size = %d x %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  unsigned long s = millis();
  gfx->draw16bitRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  total_show_video_ms += millis() - s;
  return 1;
} /* drawMCU() */

// List directory of filesystem
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}


/*******************************************************************************
 * Setup
 ******************************************************************************/

void setup()
{
  WiFi.mode(WIFI_OFF);

  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("MJPEG_2task_Audio_1task");

  Serial.println("Init display");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

#ifdef TFT_FREQUENCY
  if (!gfx->begin(TFT_FREQUENCY)) {
    Serial.println("Init display failed!");
  }
#else
  if (!gfx->begin()) {
    Serial.println("Init display failed!");
  }
#endif

gfx->invertDisplay(TFT_INVERTDISPLAY);

  // draw splash screen
  gfx->fillScreen(BLACK);
  
#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  /*******************
  * Initialize I2S
  *******************/

#ifdef USE_I2S
  Serial.println("Init I2S");
  //gfx->println("Init I2S");
  esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, -1 /* MCLK */, I2S_BCLK /* SCLK */, I2S_LRCLK /* LRCK */, I2S_DOUT /* DOUT */, -1 /* DIN */);

  if (ret_val != ESP_OK) {
    Serial.printf("i2s_init failed: %d\n", ret_val);
  }
  if (!(i2s_zero_dma_buffer(I2S_NUM_0) == ESP_OK)) {
    Serial.printf("Error clearing the I2S DMA buffer\n");
  }
#endif


  /************************************
  * Initialize SD card / file system
  ************************************/

  Serial.println("Init FS");
  gfx->println("Init FS");
  // if (!LittleFS.begin(false, "/root"))
  // if (!SPIFFS.begin(false, "/root"))
  // if (!FFat.begin(false, "/root"))

  bool useFiles = true;

  SPIClass sdspi = SPIClass(SD_SPIMODE);    
  sdspi.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
 
 #ifdef SD_FREQUENCY
  if (!SD.begin(SD_CS /* SS */, sdspi /* SPIClass */, 80000000)) {
    Serial.println("ERROR: File system mount failed!");
    gfx->println("ERROR: File system mount failed!");
    useFiles = false;
  }
#else
  if (!SD.begin(SD_CS /* SS */, sdspi /* SPIClass */)) {
    Serial.println("ERROR: File system mount failed!");
    gfx->println("ERROR: File system mount failed!");
    useFiles = false;
  }
#endif

  if (useFiles) {
    // Print information about SD card:
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE) {
      Serial.println("No SD card attached");
      return;
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
      Serial.println("MMC");
    } else if (cardType == CARD_SD) {
      Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
      Serial.println("SDHC");
    } else {
      Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    // List files
    listDir(SD, "/", 0);

    // Check for expected files:
    bool aac_file_available = false;
    Serial.println("Open AAC file: " AAC_FILENAME);
    gfx->println("Open AAC file: " AAC_FILENAME);
    File aFile = SD.open(AAC_FILENAME);

    if (aFile) {
      aac_file_available = true;
    } else {
      Serial.println("Open MP3 file: " MP3_FILENAME);
      gfx->println("Open MP3 file: " MP3_FILENAME);
      aFile = SD.open(MP3_FILENAME);
    }

    if (!aFile || aFile.isDirectory()) {
      Serial.println("ERROR: Failed to open " AAC_FILENAME " or " MP3_FILENAME " file for reading");
      gfx->println("ERROR: Failed to open " AAC_FILENAME " or " MP3_FILENAME " file for reading");
      useFiles = false;
    } else {
      Serial.println("Open MJPEG file: " MJPEG_FILENAME);
      gfx->println("Open MJPEG file: " MJPEG_FILENAME);
      File vFile = SD.open(MJPEG_FILENAME);

      if (!vFile || vFile.isDirectory()) {
        Serial.println("ERROR: Failed to open " MJPEG_FILENAME " file for reading");
        gfx->println("ERROR: Failed to open " MJPEG_FILENAME " file for reading");
        useFiles = false;
      } else {
        Serial.println("Init video");
        gfx->println("Init video");
        mjpeg_setup(&vFile, MJPEG_BUFFER_SIZE, drawMCU,
                    false /* useBigEndian */, DECODEASSIGNCORE, DRAWASSIGNCORE);

#ifdef USE_I2S
        Serial.println("Audio - USE I2S");      
        Serial.println("Start play audio task");
        gfx->println("Start play audio task");
        BaseType_t ret_val;
        if (aac_file_available) {
          Serial.println("Audio - AAC file is available");      
          ret_val = aac_player_task_start(&aFile, AUDIOASSIGNCORE);
        } else {
          Serial.println("Audio - Fallback to MP3 file");      
          ret_val = mp3_player_task_start(&aFile, AUDIOASSIGNCORE);
        }
        if (ret_val != pdPASS) {
          Serial.printf("Audio player task start failed: %d\n", ret_val);
          gfx->printf("Audio player task start failed: %d\n", ret_val);
        }
#else

#ifdef USE_A2DP_OUT
  Serial.println("Audio - USE A2DP");      
  // A2DPStream out;
  // MP3DecoderHelix decoder;
  // AudioPlayer player;

  if (aac_file_available) {
    Serial.println("Audio - AAC file is available");  
    aFile.close();
    AudioSourceSDFAT source(A2DP_AAC_FILENAME, ".aac");
    //AudioPlayer player(source, out, decoder);
    player = AudioPlayer(source, out, decoder);
    auto cfg = out.defaultConfig(TX_MODE);
    cfg.name = BLUETOOTH_REC_NAME;  // set the device here. Otherwise the next available device is used for output
    //cfg.auto_reconnect = true;  // if this is use we just quickly connect to the last device ignoring cfg.name
    out.begin(cfg);

    // setup player
    player.setVolume(0.1);
    player.begin();
  } else {
    Serial.println("Audio - Fallback to MP3 file");      
    aFile.close();
    AudioSourceSDFAT source(A2DP_MP3_FILENAME, ".mp3");
    //AudioPlayer player(source, out, decoder);
    player = AudioPlayer(source, out, decoder);
    auto cfg = out.defaultConfig(TX_MODE);
    cfg.name = BLUETOOTH_REC_NAME;  // set the device here. Otherwise the next available device is used for output
    //cfg.auto_reconnect = true;  // if this is use we just quickly connect to the last device ignoring cfg.name
    out.begin(cfg);

    // setup player
    player.setVolume(0.1);
    player.begin();
  }
#endif
#endif

        Serial.println("Start play video");
        gfx->println("Start play video");
        start_ms = millis();
        curr_ms = millis();
        next_frame_ms = start_ms + (++next_frame * 1000 / FPS / 2);
        while (vFile.available() && mjpeg_read_frame())  // Read video
        {
          total_read_video_ms += millis() - curr_ms;
          curr_ms = millis();

          if (millis() < next_frame_ms)  // check show frame or skip frame
          {
            // Play video
            mjpeg_draw_frame();
            total_decode_video_ms += millis() - curr_ms;
            curr_ms = millis();
          } else {
            ++skipped_frames;
            Serial.println("Skip frame");
          }

          while (millis() < next_frame_ms) {
            vTaskDelay(pdMS_TO_TICKS(1));
          }

          curr_ms = millis();
          next_frame_ms = start_ms + (++next_frame * 1000 / FPS);
        }
        int time_used = millis() - start_ms;
        int total_frames = next_frame - 1;
        Serial.println("AV end");
        vFile.close();
#ifndef USE_A2DP_OUT
        aFile.close();
#endif
        delay(200);

  /************************************
  *   End of video, show statistics
  ************************************/
        
        gfx->fillScreen(BLACK);

        int played_frames = total_frames - skipped_frames;
        float fps = 1000.0 * played_frames / time_used;
        total_decode_audio_ms -= total_play_audio_ms;
        // total_decode_video_ms -= total_show_video_ms;
        Serial.printf("Played frames: %d\n", played_frames);
        Serial.printf("Skipped frames: %d (%0.1f %%)\n", skipped_frames, 100.0 * skipped_frames / total_frames);
        Serial.printf("Time used: %d ms\n", time_used);
        Serial.printf("Expected FPS: %d\n", FPS);
        Serial.printf("Actual FPS: %0.1f\n", fps);
        Serial.printf("Read audio: %lu ms (%0.1f %%)\n", total_read_audio_ms, 100.0 * total_read_audio_ms / time_used);
        Serial.printf("Decode audio: %lu ms (%0.1f %%)\n", total_decode_audio_ms, 100.0 * total_decode_audio_ms / time_used);
        Serial.printf("Play audio: %lu ms (%0.1f %%)\n", total_play_audio_ms, 100.0 * total_play_audio_ms / time_used);
        Serial.printf("Read video: %lu ms (%0.1f %%)\n", total_read_video_ms, 100.0 * total_read_video_ms / time_used);
        Serial.printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video_ms, 100.0 * total_decode_video_ms / time_used);
        Serial.printf("Show video: %lu ms (%0.1f %%)\n", total_show_video_ms, 100.0 * total_show_video_ms / time_used);


        // gfx->setCursor(0, 0);
        gfx->setTextColor(WHITE);
        gfx->printf("Played frames: %d\n", played_frames);
        gfx->printf("Skipped frames: %d (%0.1f %%)\n", skipped_frames, 100.0 * skipped_frames / total_frames);
        gfx->printf("Time used: %d ms\n", time_used);
        gfx->printf("Expected FPS: %d\n", FPS);
        gfx->printf("Actual FPS: %0.1f\n\n", fps);

        int16_t r1 = ((gfx->height() - CHART_MARGIN - CHART_MARGIN) / 2);
        int16_t r2 = r1 / 2;
        int16_t cx = gfx->width() - r1 - 10;
        int16_t cy = r1 + CHART_MARGIN;

        float arc_start1 = 0;
        float arc_end1 = arc_start1 + max(2.0, 360.0 * total_read_audio_ms / time_used);
        for (int i = arc_start1 + 1; i < arc_end1; i += 2) {
          gfx->fillArc(cx, cy, r1, r2, arc_start1 - 90.0, i - 90.0, LEGEND_A_COLOR);
        }
        gfx->fillArc(cx, cy, r1, r2, arc_start1 - 90.0, arc_end1 - 90.0, LEGEND_A_COLOR);
        gfx->setTextColor(LEGEND_A_COLOR);
        gfx->printf("Read audio: %lu ms (%0.1f %%)\n", total_read_audio_ms, 100.0 * total_read_audio_ms / time_used);

        float arc_start2 = arc_end1;
        float arc_end2 = arc_start2 + max(2.0, 360.0 * total_decode_audio_ms / time_used);
        for (int i = arc_start2 + 1; i < arc_end2; i += 2) {
          gfx->fillArc(cx, cy, r1, r2, arc_start2 - 90.0, i - 90.0, LEGEND_B_COLOR);
        }
        gfx->fillArc(cx, cy, r1, r2, arc_start2 - 90.0, arc_end2 - 90.0, LEGEND_B_COLOR);
        gfx->setTextColor(LEGEND_B_COLOR);
        gfx->printf("Decode audio: %lu ms (%0.1f %%)\n", total_decode_audio_ms, 100.0 * total_decode_audio_ms / time_used);
        gfx->setTextColor(LEGEND_J_COLOR);
        gfx->printf("Play audio: %lu ms (%0.1f %%)\n", total_play_audio_ms, 100.0 * total_play_audio_ms / time_used);

        float arc_start3 = arc_end2;
        float arc_end3 = arc_start3 + max(2.0, 360.0 * total_read_video_ms / time_used);
        for (int i = arc_start3 + 1; i < arc_end3; i += 2) {
          gfx->fillArc(cx, cy, r1, r2, arc_start3 - 90.0, i - 90.0, LEGEND_C_COLOR);
        }
        gfx->fillArc(cx, cy, r1, r2, arc_start3 - 90.0, arc_end3 - 90.0, LEGEND_C_COLOR);
        gfx->setTextColor(LEGEND_C_COLOR);
        gfx->printf("Read video: %lu ms (%0.1f %%)\n", total_read_video_ms, 100.0 * total_read_video_ms / time_used);

        float arc_start4 = arc_end3;
        float arc_end4 = arc_start4 + max(2.0, 360.0 * total_show_video_ms / time_used);
        for (int i = arc_start4 + 1; i < arc_end4; i += 2) {
          gfx->fillArc(cx, cy, r1, r2, arc_start4 - 90.0, i - 90.0, LEGEND_D_COLOR);
        }
        gfx->fillArc(cx, cy, r1, r2, arc_start4 - 90.0, arc_end4 - 90.0, LEGEND_D_COLOR);
        gfx->setTextColor(LEGEND_D_COLOR);
        gfx->printf("Show video: %lu ms (%0.1f %%)\n", total_show_video_ms, 100.0 * total_show_video_ms / time_used);

        float arc_start5 = 0;
        float arc_end5 = arc_start5 + max(2.0, 360.0 * total_decode_video_ms / time_used);
        for (int i = arc_start5 + 1; i < arc_end5; i += 2) {
          gfx->fillArc(cx, cy, r2, 0, arc_start5 - 90.0, i - 90.0, LEGEND_E_COLOR);
        }
        gfx->fillArc(cx, cy, r2, 0, arc_start5 - 90.0, arc_end5 - 90.0, LEGEND_E_COLOR);
        gfx->setTextColor(LEGEND_E_COLOR);
        gfx->printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video_ms, 100.0 * total_decode_video_ms / time_used);
      }

      // delay(60000);
#ifdef GFX_BL
      // digitalWrite(GFX_BL, LOW);
#endif
      // gfx->displayOff();
      // esp_deep_sleep_start();
    }
  }
}

void loop()
{
#ifdef USE_A2DP_OUT
  player.copy();
#endif
}

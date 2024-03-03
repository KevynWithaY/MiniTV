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
#define AAC_FILENAME "/44100.aac"
#define MP3_FILENAME "/44100.mp3"
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

#define TFT_WIDTH 320
#define TFT_HEIGHT 240

// Buffer size (based on video size) [width * height * 2 / 8]
//#define MJPEG_BUFFER_SIZE (288 * 240 * 2 / 8)
//#define MJPEG_BUFFER_SIZE (320 * 240 * 2 / 8)
//#define MJPEG_BUFFER_SIZE (128 * 128 * 2 / 8)
#define MJPEG_BUFFER_SIZE ((VID_WIDTH * VID_HEIGHT * 2 / 8) + 256)

// Multi-tasking:
#define AUDIOASSIGNCORE 1
#define DECODEASSIGNCORE 0
#define DRAWASSIGNCORE 1

// SD Card Pins
#define SDCS 5 //15 
#define SDMOSI 23 //13 
#define SDCLK 18 //14 
#define SDMISO 19 //12 


/*******************************************************************************
 * Start of Arduino_GFX setting
 ******************************************************************************/
#define TFT_BRIGHTNESS 128
#define GFX_BL 21 // DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin
#define TFT_DC 2
#define TFT_CS 15
#define TFT_SCK 14
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_RST -1 //33
#define TFT_SPI HSPI

#include <Arduino_GFX_Library.h>

//Arduino_DataBus *bus = create_default_Arduino_DataBus();
// Arduino_GFX *gfx = new Arduino_ILI9341(bus, DF_GFX_RST, 3 /* rotation */, false /* IPS */);
//Arduino_GFX *gfx = new Arduino_ST7789(bus, DF_GFX_RST, 1 /* rotation */, true /* IPS */, 240 /* width */, 288 /* height */, 0 /* col offset 1 */, 20 /* row offset 1 */, 0 /* col offset 2 */, 12 /* row offset 2 */);

// ESP32 SPI bus:
Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(TFT_DC /* DC */, TFT_CS /* CS */, TFT_SCK /* SCK */, TFT_MOSI /* MOSI */, TFT_MISO /* MISO */,
  TFT_SPI /* VSPI or HSPI */, false /* Shared Bus? */);

// This "DMA" bus worked, but skipped 4.1% of frame compared to 0% for the Arduino_ESP32SPI on the 24fps 320x240 video I tried...
// Arduino_DataBus *bus = new Arduino_ESP32SPIDMA(TFT_DC /* DC */, TFT_CS /* CS */, TFT_SCK /* SCK */, TFT_MOSI /* MOSI */, TFT_MISO /* MISO */, 
//   TFT_SPI /* VSPI or HSPI */, false /* Shared Bus? */);

// ST7735 128x128 display:
// 1.5" GREENTAB B 128x128
// Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST, 
//   0 /* rotation */, false /* IPS */, 
//   128 /* width */, 128 /* height */, 
//   2 /* col offset 1 */, 3 /* row offset 1 */, 
//   2 /* col offset 2 */, 1 /* row offset 2 */);

// // ILI9341 320x240 display
// Arduino_GFX *gfx = new Arduino_ILI9341(
//   bus, TFT_RST /* RST */, 3 /* rotation */, false /* IPS */);

// // ILI9342 320x240 display on CYD
// Arduino_GFX *gfx = new Arduino_ILI9341(
//   bus, TFT_RST /* RST */, 1 /* rotation */, false /* IPS */);



// ST7789 1.69" 280x240 rounded-corner display from Waveshare (using video size 288x240 due to MJPEG preference for sizes divisible by 16)
// 1.69" IPS LCD ST7789
// Arduino_GFX *gfx = new Arduino_ST7789(
//   bus, TFT_RST /* RST */, 1 /* rotation */, true /* IPS */,
//   240 /* width */, 280 /* height */,
//   20 /* col offset 1 */, 20 /* row offset 1 */,
//   0 /* col offset 2 */, 20 /* row offset 2 */);


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



// pixel drawing callback
static int drawMCU(JPEGDRAW *pDraw)
{
  // Serial.printf("Draw pos = (%d, %d), size = %d x %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  unsigned long s = millis();
  gfx->draw16bitRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  total_show_video_ms += millis() - s;
  return 1;
} /* drawMCU() */


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


void setup()
{
  WiFi.mode(WIFI_OFF);

  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("MJPEG_2task_Audio_1task");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  Serial.println("Init display");
  if (!gfx->begin(80000000))
  //if (!gfx->begin(40000000))
  {
    Serial.println("Init display failed!");
  }

gfx->invertDisplay(false);

  // draw splash screen
  gfx->fillScreen(BLACK);
  
#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif




  
  // // Draw rainbow gradient
  // int32_t w, h, n, n1, cx, cy, cx1, cy1, cn, cn1;
  // w = gfx->width();
  // h = gfx->height();
  // n = min(w, h);
  // n1 = n - 1;
  // cx = w / 2;
  // cy = h / 2;
  // cx1 = cx - 1;
  // cy1 = cy - 1;
  // cn = min(cx1, cy1);

  // int16_t i, r = 360 / cn;

  // for (i = 6; i < cn; i += 6)
  // {
  //   int hue = map(i, 0, (cn/6)), 0, 255);
  //   uint16_t color = gfx->colorHSV(hue, 255, 255);
  //   gfx->fillArc(cx1, cy1, i, i - 3, 0, i * r, color);
  // }

  // delay(1000);





//   Serial.println("Init I2S");
//   gfx->println("Init I2S");
// #if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32)
//   esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, -1 /* MCLK */, 25 /* SCLK */, 26 /* LRCK */, 32 /* DOUT */, -1 /* DIN */);
// #elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S2)
//   esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, -1 /* MCLK */, 4 /* SCLK */, 5 /* LRCK */, 18 /* DOUT */, -1 /* DIN */);
// #elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)
//   esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, 42 /* MCLK */, 46 /* SCLK */, 45 /* LRCK */, 43 /* DOUT */, 44 /* DIN */);
// #elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32C3)
//   esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, -1 /* MCLK */, 10 /* SCLK */, 19 /* LRCK */, 18 /* DOUT */, -1 /* DIN */);
// #endif
//   if (ret_val != ESP_OK)
//   {
//     Serial.printf("i2s_init failed: %d\n", ret_val);
//   }
//   if (!(i2s_zero_dma_buffer(I2S_NUM_0) == ESP_OK)) {
//     Serial.printf("Error clearing the I2S DMA buffer\n");
//   }


  // Initialize SD card / file system:
  // ---------------------------------

  Serial.println("Init FS");
  gfx->println("Init FS");
  // if (!LittleFS.begin(false, "/root"))
  // if (!SPIFFS.begin(false, "/root"))
  // if (!FFat.begin(false, "/root"))

  bool useFiles = true;

  SPIClass sdspi = SPIClass(VSPI);    
  sdspi.begin(SDCLK, SDMISO, SDMOSI, SDCS);
 
  if (!SD.begin(SDCS /* SS */, sdspi /* SPIClass */, 80000000)) {
    Serial.println("ERROR: File system mount failed!");
    gfx->println("ERROR: File system mount failed!");
    useFiles = false;
  }
  else {
    // Print information about SD card:
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
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
    // File aFile = LittleFS.open(AAC_FILENAME);
    // File aFile = SPIFFS.open(AAC_FILENAME);
    // File aFile = FFat.open(AAC_FILENAME);
    File aFile = SD.open(AAC_FILENAME);
    // File aFile = SD_MMC.open(AAC_FILENAME);
    if (aFile)
    {
      aac_file_available = true;
    }
    else
    {
      Serial.println("Open MP3 file: " MP3_FILENAME);
      gfx->println("Open MP3 file: " MP3_FILENAME);
      // aFile = LittleFS.open(MP3_FILENAME);
      // aFile = SPIFFS.open(MP3_FILENAME);
      // aFile = FFat.open(MP3_FILENAME);
      aFile = SD.open(MP3_FILENAME);
      // aFile = SD_MMC.open(MP3_FILENAME);
    }

    if (!aFile || aFile.isDirectory())
    {
      Serial.println("ERROR: Failed to open " AAC_FILENAME " or " MP3_FILENAME " file for reading");
      gfx->println("ERROR: Failed to open " AAC_FILENAME " or " MP3_FILENAME " file for reading");
      useFiles = false;
    }
    else
    {
      Serial.println("Open MJPEG file: " MJPEG_FILENAME);
      gfx->println("Open MJPEG file: " MJPEG_FILENAME);
      // File vFile = LittleFS.open(MJPEG_FILENAME);
      // File vFile = SPIFFS.open(MJPEG_FILENAME);
      // File vFile = FFat.open(MJPEG_FILENAME);
      File vFile = SD.open(MJPEG_FILENAME);
      // File vFile = SD_MMC.open(MJPEG_FILENAME);
      if (!vFile || vFile.isDirectory())
      {
        Serial.println("ERROR: Failed to open " MJPEG_FILENAME " file for reading");
        gfx->println("ERROR: Failed to open " MJPEG_FILENAME " file for reading");
        useFiles = false;
      }
      else
      {
        Serial.println("Init video");
        gfx->println("Init video");
        mjpeg_setup(&vFile, MJPEG_BUFFER_SIZE, drawMCU,
                    false /* useBigEndian */, DECODEASSIGNCORE, DRAWASSIGNCORE);

        
        
        
        // Serial.println("Start play audio task");
        // gfx->println("Start play audio task");
        // BaseType_t ret_val;
        // if (aac_file_available)
        // {
        //   ret_val = aac_player_task_start(&aFile, AUDIOASSIGNCORE);
        // }
        // else
        // {
        //   ret_val = mp3_player_task_start(&aFile, AUDIOASSIGNCORE);
        // }
        // if (ret_val != pdPASS)
        // {
        //   Serial.printf("Audio player task start failed: %d\n", ret_val);
        //   gfx->printf("Audio player task start failed: %d\n", ret_val);
        // }




        Serial.println("Start play video");
        gfx->println("Start play video");
        start_ms = millis();
        curr_ms = millis();
        next_frame_ms = start_ms + (++next_frame * 1000 / FPS / 2);
        while (vFile.available() && mjpeg_read_frame()) // Read video
        {
          total_read_video_ms += millis() - curr_ms;
          curr_ms = millis();

          if (millis() < next_frame_ms) // check show frame or skip frame
          {
            // Play video
            mjpeg_draw_frame();
            total_decode_video_ms += millis() - curr_ms;
            curr_ms = millis();
          }
          else
          {
            ++skipped_frames;
            Serial.println("Skip frame");
          }

          while (millis() < next_frame_ms)
          {
            vTaskDelay(pdMS_TO_TICKS(1));
          }

          curr_ms = millis();
          next_frame_ms = start_ms + (++next_frame * 1000 / FPS);
        }
        int time_used = millis() - start_ms;
        int total_frames = next_frame - 1;
        Serial.println("AV end");
        vFile.close();
        aFile.close();

        delay(200);


        // End of video, show statistics:
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
        for (int i = arc_start1 + 1; i < arc_end1; i += 2)
        {
          gfx->fillArc(cx, cy, r1, r2, arc_start1 - 90.0, i - 90.0, LEGEND_A_COLOR);
        }
        gfx->fillArc(cx, cy, r1, r2, arc_start1 - 90.0, arc_end1 - 90.0, LEGEND_A_COLOR);
        gfx->setTextColor(LEGEND_A_COLOR);
        gfx->printf("Read audio: %lu ms (%0.1f %%)\n", total_read_audio_ms, 100.0 * total_read_audio_ms / time_used);

        float arc_start2 = arc_end1;
        float arc_end2 = arc_start2 + max(2.0, 360.0 * total_decode_audio_ms / time_used);
        for (int i = arc_start2 + 1; i < arc_end2; i += 2)
        {
          gfx->fillArc(cx, cy, r1, r2, arc_start2 - 90.0, i - 90.0, LEGEND_B_COLOR);
        }
        gfx->fillArc(cx, cy, r1, r2, arc_start2 - 90.0, arc_end2 - 90.0, LEGEND_B_COLOR);
        gfx->setTextColor(LEGEND_B_COLOR);
        gfx->printf("Decode audio: %lu ms (%0.1f %%)\n", total_decode_audio_ms, 100.0 * total_decode_audio_ms / time_used);
        gfx->setTextColor(LEGEND_J_COLOR);
        gfx->printf("Play audio: %lu ms (%0.1f %%)\n", total_play_audio_ms, 100.0 * total_play_audio_ms / time_used);

        float arc_start3 = arc_end2;
        float arc_end3 = arc_start3 + max(2.0, 360.0 * total_read_video_ms / time_used);
        for (int i = arc_start3 + 1; i < arc_end3; i += 2)
        {
          gfx->fillArc(cx, cy, r1, r2, arc_start3 - 90.0, i - 90.0, LEGEND_C_COLOR);
        }
        gfx->fillArc(cx, cy, r1, r2, arc_start3 - 90.0, arc_end3 - 90.0, LEGEND_C_COLOR);
        gfx->setTextColor(LEGEND_C_COLOR);
        gfx->printf("Read video: %lu ms (%0.1f %%)\n", total_read_video_ms, 100.0 * total_read_video_ms / time_used);

        float arc_start4 = arc_end3;
        float arc_end4 = arc_start4 + max(2.0, 360.0 * total_show_video_ms / time_used);
        for (int i = arc_start4 + 1; i < arc_end4; i += 2)
        {
          gfx->fillArc(cx, cy, r1, r2, arc_start4 - 90.0, i - 90.0, LEGEND_D_COLOR);
        }
        gfx->fillArc(cx, cy, r1, r2, arc_start4 - 90.0, arc_end4 - 90.0, LEGEND_D_COLOR);
        gfx->setTextColor(LEGEND_D_COLOR);
        gfx->printf("Show video: %lu ms (%0.1f %%)\n", total_show_video_ms, 100.0 * total_show_video_ms / time_used);

        float arc_start5 = 0;
        float arc_end5 = arc_start5 + max(2.0, 360.0 * total_decode_video_ms / time_used);
        for (int i = arc_start5 + 1; i < arc_end5; i += 2)
        {
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
}

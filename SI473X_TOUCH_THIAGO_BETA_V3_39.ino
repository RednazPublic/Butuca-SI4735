// BUTUCA 2021
// -----------------------------------------------------------------------
// - Seek - and Seek + run out of band range, it does not respect the limits Ex. Press 41m and, Press Seek + and it stop > 7440 kHz
// - What is the best delay after press a button (layer First, second, thirf, fourth): 400, 300, 200 ... None (BETA 200)
// - Calibration for 3.5" ILI 9488 Touch (OK!)
// - MEM Problema fixa freq até que outro botão seja pressionado. É possivel acionar o filtro e steps. Stereo fica fixo em FM F?????
// - Terceira linha esta flicando ao precionar RDS ou AGC+
// - Corrigir cor de YELLOWUNSET para um cinza mais claro no DaylightMode (ok!)
// - Em FM selecionar 76.00, entrar em BAND, volta pelo encoder - 108.00 ou 76.07 sobreescrevem o 76.00.
// - apresentar o volume constantemente (OK!)
//
//
//
//
//
//
//
//
// -----------------------------------------------------------------------
// UNDER CONSTRUCTION - It is a Beta version and can no work properly.
//
// The sketches and documentation available here are based on Gert's sketch that made his work available for avery one.TFT_RED
// I would like to thank to [Gert Baak](https://github.com/pe0mgb/SI4735-Radio-ESP32-Touchscreen-Arduino).
//
//  Thi sketch was made  by
//  V3.0 27-07-2020 Bug-support and little improvements.
//  This sketch is based on the SI4735 Library of Ricardo PU2CLR. Thanks for the very nice work.
//  This sketch uses  a 2.8 inch 240*320 touch-screen with ILI9341, ESP32 WROOM-32 and Rotary Encoder.
//  The radio is fully controlled by the (Touch)Screen and Rotary Encoder
//  This sketch uses the Rotary Encoder Class implementation from Ben Buxton (the source code is included
//  together with this sketch).
//  For the touch-screen the library TFT_eSPI is used. The configuration setup-file: setup1_ILI9341 is also
//  included.
//  Also a schematic drawing is available.
//  ABOUT SSB PATCH:
//  This sketch will download a SSB patch to your SI4735 device (patch_init.h). It will take about 8KB of the Arduino memory.
//  In this context, a patch is a piece of software used to change the behavior of the SI4735 device.
//  There is little information available about patching the SI4735. The following information is the understanding of the author of
//  this project and it is not necessarily correct. A patch is executed internally (run by internal MCU) of the device.
//  Usually, patches are used to fixes bugs or add improvements and new features of the firmware installed in the internal ROM of the device.
//  Patches to the SI4735 are distributed in binary form and have to be transferred to the internal RAM of the device by
//  the host MCU (in this case Arduino). Since the RAM is volatile memory, the patch stored into the device gets lost when you turn off the system.
//  Consequently, the content of the patch has to be transferred again to the device each time after turn on the system or reset the device.
//
//  ATTENTION: The author of this project does not guarantee that procedures shown here will work in your development environment.
//  Given this, it is at your own risk to continue with the procedures suggested here.
//  This library works with the I2C communication protocol and it is designed to apply a SSB extension PATCH to CI SI4735-D60.
//  Once again, the author disclaims any liability for any damage this procedure may cause to your SI4735 or other devices that you are using.
//
//  Library TFT_eSPI you may download from here : https://github.com/Bodmer/TFT_eSPI
//  Library Rotary is provided with the program
//  Library SI4735 you may download from here   : https://github.com/pu2clr/SI4735
//
//  *********************************
//  **   Display connections etc.  **
//  *********************************
//  |------------|------------------|------------|------------|------------|
//  |Display 2.8 |      ESP32       |   Si4735   |  Encoder   |  Beeper    |
//  |  ILI9341   |                  |            |            |            |        Encoder        1,2,3
//  |------------|------------------|------------|------------|------------|        Encoder switch 4,5
//  |   Vcc      |     3V3     | 01 |    Vcc     |            |            |        pin 33 with 18K to 3.3 volt and 18K to ground.
//  |   GND      |     GND     | 02 |    GND     |     2,4    |            |        pin 32 (Beeper) via 2K to base V1  BC547
//  |   CS       |     15      | 03 |            |            |            |        Collector via beeper to 5v
//  |   Reset    |      4      | 04 |            |            |            |        Emmitor to ground
//  |   D/C      |      2      | 05 |            |            |            |
//  |   SDI      |     23      | 06 |            |            |            |        Encoder        1,2,3
//  |   SCK      |     18      | 07 |            |            |            |        Encoder switch 4,5
//  |   LED Coll.|     14 2K   | 08 |            |            |            |        Display LED
//  |   SDO      |             | 09 |            |            |            |        Emmitor  V2 BC557 to 3.3 V
//  |   T_CLK    |     18      | 10 |            |            |            |        Base with 2K to pin 14 (Display_Led)
//  |   T_CS     |      5      | 11 |            |            |            |        Collector to led pin display
//  |   T_DIN    |     23      | 12 |            |            |            |
//  |   T_DO     |     19      | 13 |            |            |            |
//  |   T_IRQ    |     34      | 14 |            |            |            |
//  |            |     12      |    |   Reset    |            |            |
//  |            |     21      |    |    SDA     |            |            |
//  |            |     22      |    |    SCL     |            |            |
//  |            |     16      |    |            |      1     |            |
//  |            |     17      |    |            |      3     |            |
//  |            |     33      |    |            |      5     |            |
//  |            |     32 2K   |    |            |            |     In     |
//  |            |     27 Mute |    |see schematics           |            |
//  |------------|-------------|----|------------|------------|------------|

#define VersionText "V3.9"

#include <SPIFFS.h>
#include <SI4735.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "EEPROM.h"
#include "Rotary.h"
#include "Noise.h"

#include "NotoSans_ExtraBold6pt7b.h"
#include "NotoSans_ExtraCondensedBold8pt7b.h"
#include "FreeSansBold10pt7b.h"
#include "NotoSans_ExtraCondensedBold13pt7b.h"
#include "NotoSans_ExtraCondensedSemiBold56pt7b.h"

#define RDSSTATION NotoSans_ExtraBold6pt7b
#define FMTYPE NotoSans_ExtraBold6pt7b
#define ERRORTYPE NotoSans_ExtraCondensedBold8pt7b
#define SETUPLINE1 NotoSans_ExtraCondensedBold8pt7b
#define BandText FreeSansBold10pt7b
#define SMeterText "NotoSans-CB11"
#define ButtonText1 NotoSans_ExtraCondensedBold13pt7b
#define SETUPLINE "NotoSans-ECB18"
#define FREQNUMBER NotoSans_ExtraCondensedSemiBold56pt7b


// Fonts - https://rop.nl/truetype2gfx/
// https://wiki.seeedstudio.com/Wio-Terminal-LCD-Anti-aliased-Fonts/ - How to proceed
// https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/ - Load SPIFF with .vlw
// http://www.perbang.dk/rgb/E3B717/
// http://javl.github.io/image2cpp/

// =================================================
//
// =================================================

// Test it with patch_init.h or patch_full.h. Do not try load both.
#include "patch_init.h" // SSB patch for whole SSBRX initialization string
//#include "patch_full.h"    // SSB patch for whole SSBRX full download

const uint16_t size_content = sizeof ssb_patch_content; // see ssb_patch_content in patch_full.h or patch_init.h

#define ESP32_I2C_SDA    21  // I2C bus pin on ESP32
#define ESP32_I2C_SCL    22  // I2C bus pin on ESP32
#define RESET_PIN        12
#define ENCODER_PIN_A    17  // http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
#define ENCODER_PIN_B    16
#define ENCODER_SWITCH   33
#define Display_Led      14
#define displayon         0
#define displayoff        1
#define AUDIO_MUTE       27

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

#define MIN_ELAPSED_TIME             100
#define MIN_ELAPSED_RSSI_TIME        150
#define MIN_ELAPSED_DISPL_TIME      1000
//#define MIN_ELAPSED_RDS_TIME         5
#define DEFAULT_VOLUME                45  // change it for your favorite start sound volume
#define MIN_ELAPSED_VOLbut_TIME     1000

#define LSB         0
#define USB         1
#define AM          2
#define SYNC        3
#define FM          4

bool bfoOn          = false;
bool ssbLoaded      = false;

bool FirstLayer = true;
bool SecondLayer = false;
bool ThirdLayer = false;
bool ForthLayer = false;
bool HamBand = false;
bool Modebut = false;
bool FREQbut = false;
bool Decipoint = false;
bool STEPbut = false;
bool BroadBand;
bool BandWidth;
bool MISCbut = false;
bool PRESbut = false;
bool VOLbut = false;
bool DISplay = false;
bool Mutestat = false;
bool AGCgainbut = false;
bool writingEeprom = false;
bool Secondline = false;

bool NightLight = true;
bool pressed;
bool press1;
bool audioMuteOn = true;
bool audioMuteOff = false;
bool RDS = true; // RDS on  or  off
bool SEEK =  false;

int currentBFO;
int previousBFO = 0;
int nrbox       = 0;
int OldRSSI;
int NewRSSI;
int NewSNR;
int encBut;
int AGCgain     = 0;

long elapsedRSSI        = millis();
long elapsedRDS         = millis();
long stationNameElapsed = millis();
long DisplayOnTime      = millis();
long VOLbutOnTime       = millis();

volatile int encoderCount  = 0;

uint16_t previousFrequency;
uint16_t currentStep        =  1;

uint8_t currentBFOStep     = 25;

uint8_t currentPRES        =  0;
uint8_t previousPRES       =  0;
uint8_t currentPRESStep    =  1;

int currentAGCgain     =  1;
int previousAGCgain    =  1;
uint8_t currentAGCgainStep =  1;
uint8_t MaxAGCgain;
uint8_t MaxAGCgainFM       = 26;
uint8_t MaxAGCgainAM       = 37;
uint8_t MinAGCgain         =  1;

uint8_t RoundCorner         =  8;

int currentVOL         =  0;
int previousVOL        =  0;
uint8_t currentVOLStep     =  1;
uint8_t MaxVOL             = 63;
uint8_t MinVOL             = 0;
uint32_t BackColor;
uint32_t UNSETColor;
uint32_t SETColor;
uint32_t REDMETER;

uint8_t currentAGCAtt      =  0;
uint8_t bwIdxSSB;
uint8_t bwIdxAM;
uint8_t bandIdx;
uint8_t currentMode = FM;
uint8_t previousMode;
uint16_t x = 0, y = 0; // To store the touch coordinates
uint8_t encoderStatus;
uint16_t freqstep;
uint8_t freqstepnr = 0; //1kHz
int   freqDec = 0;
float Displayfreq      = 0;
float currentFrequency = 0;
float dpfrq            = 0;
float fact             = 1;
float Oldfrequency = currentFrequency;

String BWtext;
String RDSbuttext;
String AGCgainbuttext;

const char *bandwidthSSB[] = {"1.2", "2.2", "3.0", "4.0", "0.5", "1.0"};
const char *bandwidthAM[]  = {"6.0", "4.0", "3.0", "2.0", "1.0", "1.8", "2.5"};
const char *Keypathtext[]  = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", ".", "ENTER", "Clear"};
const char *bandModeDesc[] = {"LSB", "USB", "AM ", "SYN"};

char buffer[64]; // Useful to handle string
//char buffer1[64];

const char *stationName;
char bufferStatioName[40];

char bufferFrequency[15];
char bufferVFO[15];
char bufferUnit[5];
char bufferBandName[10];
char bufferVolume[10];
char bufferAgcGain[10];
char bufferRDS[65];
char bufferAux[15];

const int ledChannel = 0;
const int resolution = 1;

//=======================================================   Buttons First and Third Layer   ==========================
typedef struct // Buttons first layer
{
  const char *ButtonNam;

  int    ButtonNum;       // Button location at display from 0 to 11. To move around buttons freely at first layer.
  const char *ButtonNam1;
  int    ButtonNum1;      // Button location at display from 0 to 11. To move around buttons freely at third layer.
  int    XButos;          // Xoffset
  long   YButos;          // Yoffset
} Button;

int ytotoffset = 0;

int Xinfo = 17; //Inicial 17
int Yinfo = 15; //Inicial 15

int XFreq = Xinfo;
int YFreq = Yinfo - 10;

//  Button table
int Xbutst  =   1;               // X Start location Buttons
int Ybutst  = 220 + ytotoffset;  // Y Start location Buttons

int Xsmtr   =   0;
int Ysmtr   =  85 + ytotoffset;  // S meter

int XVolInd =   0;
int YVolInd = 135 + ytotoffset;  // Volume indicator

int XFreqDispl  =   0;
int YFreqDispl  =   0 + ytotoffset;  // display

int Xbutsiz =  80;  //size of buttons first & third layer
int Ybutsiz =  50;
int YbutsizExtra =  45;

int Xbut0  = 0 * Xbutsiz ; int Ybut0   = 0 * Ybutsiz; // location calqualation for 12 first layer buttons
int Xbut1  = 1 * Xbutsiz ; int Ybut1   = 0 * Ybutsiz;
int Xbut2  = 2 * Xbutsiz ; int Ybut2   = 0 * Ybutsiz;
int Xbut3  = 3 * Xbutsiz ; int Ybut3   = 0 * Ybutsiz;
int Xbut4  = 4 * Xbutsiz ; int Ybut4   = 0 * Ybutsiz;
int Xbut5  = 5 * Xbutsiz ; int Ybut5   = 0 * Ybutsiz;
int Xbut6  = 0 * Xbutsiz ; int Ybut6   = 1 * Ybutsiz;
int Xbut7  = 1 * Xbutsiz ; int Ybut7   = 1 * Ybutsiz;
int Xbut8  = 2 * Xbutsiz ; long Ybut8  = 1 * Ybutsiz;
int Xbut9  = 3 * Xbutsiz ; long Ybut9  = 1 * Ybutsiz;
int Xbut10 = 4 * Xbutsiz ; long Ybut10 = 1 * Ybutsiz;
int Xbut11 = 5 * Xbutsiz ; long Ybut11 = 1 * Ybutsiz;
// Extra Buttons
int Xbut12 = 5 * Xbutsiz ; long Ybut12 = -3.5 * Ybutsiz;
int Xbut13 = 5 * Xbutsiz ; long Ybut13 = -2.5 * Ybutsiz;
int Xbut14 = 5 * Xbutsiz ; long Ybut14 = -1.5 * Ybutsiz;

#define HAM       0
#define FREQ      1
#define AGC       2
#define STEP      3
#define VOL       4
#define PRESET    5
#define BROAD     6
#define MODE      7
#define BFO       8
#define BANDW     9
#define MUTE     10
#define NEXT     11
#define SEEKUP   12
#define SEEKDOWN 13
#define SPCTR    14

#define AGCset    0
#define INFO      1
#define ONOFF     2
#define NR3       3
#define NR4       4
#define NR5       5
#define FMRDS     6
#define STORE     7
#define COLOR     8
#define NR9       9
#define NR10     10
#define BACK     11
#define NR12     12
#define NR13     13
#define NR14     14
//

Button bt[] = {       //
  { "HAM"   ,  0 , "ACG+"   ,  0 , Xbut0 , Ybut0  },
  { "FREQ"  ,  1 , "INFO"   ,  1 , Xbut1 , Ybut1  },
  { "AGC"   ,  2 , "ONOFF"  ,  2 , Xbut2 , Ybut2  },       //                             |----|
  { "STEP"  ,  3 , ""       ,  3 , Xbut3 , Ybut3  },       //                             | 12 |
  { "VOL"   ,  4 , ""       ,  4 , Xbut4 , Ybut4  },       //                             |----|
  { "MEM"   ,  5 , ""       ,  5 , Xbut5 , Ybut5  },       //                             | 13 |
  { "BAND"  ,  6 , "RDS"    ,  6 , Xbut6 , Ybut6  },       //                             |----|
  { "MODE"  ,  7 , "STORE"  ,  7 , Xbut7 , Ybut7  },       //                             | 14 |
  { "BFO"   ,  8 , "COLOR"  ,  8 , Xbut8 , Ybut8  },       //                             |----|
  { "FILTER",  9 , ""       ,  9 , Xbut9 , Ybut9  },       //
  { "MUTE"  , 10 , ""       , 10 , Xbut10, Ybut10 },       //    |----|----|----|----|----|----|
  { "NEXT"  , 11 , "BACK"   , 11 , Xbut11, Ybut11 },       //    |  0 |  1 |  2 |  3 |  4 |  5 |
  { "SEEK +", 12 , "SEEK +" , 12 , Xbut12, Ybut12 },       //    |----|----|----|----|----|----|
  { "SEEK -", 13 , "SEEK -" , 13 , Xbut13, Ybut13 },       //    |  6 |  7 |  8 |  9 | 10 | 11 |
  { "SPCTR",  14 , "SPCTR"  , 14 , Xbut14, Ybut14 },       //    |----|----|----|----|----|----|
};

// You may freely move around the button (blue) position on the display to your flavour by changing the position in ButtonNum and ButtonNum1
// You have to stay in the First or Third Layer
//======================================================= End  Buttons First  and Third Layer   ======================

// Default color definitions
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFE19      /* 255, 192, 203 */
#define TFT_BROWN       0x9A60      /* 150,  75,   0 */
#define TFT_GOLD        0xFEA0      /* 255, 215,   0 */
#define TFT_SILVER      0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
#define TFT_VIOLET      0x915C      /* 180,  46, 226 */

#define TextColor       TFT_BLACK

//======================================================= Tunings Steps     ===============================
typedef struct // Tuning steps
{
  uint16_t stepFreq;
  uint16_t Xstepos;          //Xoffset
  uint16_t Xstepsr;          //X size rectang
  uint16_t Ystepos;          //Yoffset
  uint16_t Ystepsr;          //Y size rectang
  uint16_t Ystepnr;          //Y next rectang
} Step;

//  Tuning steps table

uint16_t Xfstep = 60;
uint16_t Yfstep = 20;

Step sp[] = {
  { 1 , Xfstep, 200, Yfstep, 50,  0},
  { 5 , Xfstep, 200, Yfstep, 50, 50},
  { 10 , Xfstep, 200, Yfstep, 50, 100},
  { 50, Xfstep, 200, Yfstep, 50, 150},
  { 100, Xfstep, 200, Yfstep, 50, 200},
  { 500, Xfstep, 200, Yfstep, 50, 250},
  { 1000, Xfstep, 200, Yfstep, 50, 200},
  { 9, Xfstep, 200, Yfstep, 50, 250}
};
//======================================================= End Tunings Steps     ===============================

//======================================================= Modulation Types     ================================
typedef struct // MODULATION
{
  uint16_t Modenum;
  uint16_t Xmodos;          //Xoffset
  uint16_t Xmodsr;          //X size rectang
  uint16_t Ymodos;          //Yoffset
  uint16_t Ymodsr;          //Y size rectang
  uint16_t Ymodnr;          //Y next rectang
} Mode;

//  Modulation table

uint16_t Xfmod = 60;
uint16_t Yfmod = 0;

Mode md[] = {
  { 0  , Xfmod, 200, Yfmod, 50,  0},
  { 1  , Xfmod, 200, Yfmod, 50, 50},
  { 2  , Xfmod, 200, Yfmod, 50, 100},
  { 3  , Xfmod, 200, Yfmod, 50, 150},
  { 4  , Xfmod, 200, Yfmod, 50, 200}
};
//======================================================= End Modulation Types     ============================

//======================================================= Keypath     =========================================
typedef struct // Keypath
{
  uint16_t KeypNum;
  uint16_t Xkeypos;          //Xoffset
  uint16_t Xkeypsr;          //X size rectang
  uint16_t Xkeypnr;          //Y next rectang
  uint16_t Ykeypos;          //Yoffset
  uint16_t Ykeypsr;          //X size rectang
  uint16_t Ykeypnr;          //Y next rectang
} Keypath;

//  Keypath table
uint16_t Xpath = 10;
uint16_t Ypath = 30;

Keypath kp[] = {
  {  0 , Xpath,  100 ,   0 , Ypath , 50 ,   0},
  {  1 , Xpath,  100 , 100 , Ypath , 50 ,   0},
  {  2 , Xpath,  100 , 200 , Ypath , 50 ,   0},
  {  3 , Xpath,  100 ,   0 , Ypath , 50 ,  50},
  {  4 , Xpath,  100 , 100 , Ypath , 50 ,  50},
  {  5 , Xpath,  100 , 200 , Ypath , 50 ,  50},
  {  6 , Xpath,  100 ,   0 , Ypath , 50 , 100},
  {  7 , Xpath,  100 , 100 , Ypath , 50 , 100},
  {  8 , Xpath,  100 , 200 , Ypath , 50 , 100},
  {  9 , Xpath,  100 ,   0 , Ypath , 50 , 150},
  { 10 , Xpath,  100 , 100 , Ypath , 50 , 150},
  { 11 , Xpath,  100 , 200 , Ypath , 50 , 150},
};

//======================================================= End Keypath     =====================================

//======================================================= Bandwidth AM & FM     ===============================
typedef struct // Bandwidth AM & SSB
{
  uint16_t BandWidthAM;
  uint16_t BandWidthSSB;
  uint16_t Xos;          //Xoffset
  uint16_t Xsr;          //X size rectang
  uint16_t Yos;          //Yoffset
  uint16_t Ysr;          //X size rectang
  uint16_t Ynr;          //Y next rectang
} Bandwidth;

//  Bandwidth table
uint16_t XfBW = 60;
uint16_t YfBW = 15;

Bandwidth bw[] = {
  { 4 , 4 , XfBW, 200, YfBW, 30,   0},
  { 5 , 5 , XfBW, 200, YfBW, 30,  30},
  { 3 , 0 , XfBW, 200, YfBW, 30,  60},
  { 6 , 1 , XfBW, 200, YfBW, 30,  90},
  { 2 , 2 , XfBW, 200, YfBW, 30, 120},
  { 1 , 3 , XfBW, 200, YfBW, 30, 150},
  { 0 , 0 , XfBW, 200, YfBW, 30, 180},
};
//======================================================= End Bandwidth AM & FM     ===========================

//======================================================= Broad Band Definitions     ==========================
typedef struct // Broad-Band switch
{
  uint16_t BbandNum; // bandIdx
  uint16_t Xbbandos;          //Xoffset
  uint16_t Xbbandsr;          //X size rectang
  uint16_t Xbbandnr;          //X next rectang
  uint16_t Ybbandos;          //Yoffset
  uint16_t Ybbandsr;          //X size rectang
  uint16_t Ybbandnr;          //Y next rectang
} BBandnumber;

//  Bandnumber table for the broad-bands
uint16_t Xfbband = 10;
uint16_t Yfbband = 15;

BBandnumber bb[] = {
  {  0 , Xfbband, 100 ,  0 ,  Yfbband , 35 ,   0}, // 0
  {  1 , Xfbband, 100 ,  0 ,  Yfbband , 35 ,  35}, // 1
  {  2 , Xfbband, 100 ,  0 ,  Yfbband , 35 ,  70}, // 2
  {  6 , Xfbband, 100 ,  0 ,  Yfbband , 35 , 105}, // 3
  {  7 , Xfbband, 100 ,  0 ,  Yfbband , 35 , 140}, // 4
  {  9 , Xfbband, 100 ,  0 ,  Yfbband , 35 , 175}, // 5
  { 11 , Xfbband, 100 , 100 ,  Yfbband , 35 ,   0}, // 6
  { 13 , Xfbband, 100 , 100 ,  Yfbband , 35 ,  35}, // 7
  { 14 , Xfbband, 100 , 100 ,  Yfbband , 35 ,  70}, // 8
  { 16 , Xfbband, 100 , 100 ,  Yfbband , 35 , 105}, // 9
  { 17 , Xfbband, 100 , 100 ,  Yfbband , 35 , 140}, //10
  { 19 , Xfbband, 100 , 100 ,  Yfbband , 35 , 175}, //11
  { 21 , Xfbband, 100 , 200 ,  Yfbband , 35 ,   0}, //12
  { 22 , Xfbband, 100 , 200 ,  Yfbband , 35 ,  35}, //13
  { 24 , Xfbband, 100 , 200 ,  Yfbband , 35 ,  70}, //14
  { 26 , Xfbband, 100 , 200 ,  Yfbband , 35 , 105}, //15
  { 27 , Xfbband, 100 , 200 ,  Yfbband , 35 , 140}, //16
  { 29 , Xfbband, 100 , 200 ,  Yfbband , 35 , 175}, //17

};
//======================================================= End Broad Band Definitions     ======================

//======================================================= Ham Band Definitions     ============================
typedef struct // Ham Band switch
{
  uint16_t BandNum; // bandIdx
  uint16_t HamBandTxt;
  uint16_t Xbandos;          //Xoffset
  uint16_t Xbandsr;          //X size rectang
  uint16_t Xbandnr;          //X next rectang
  uint16_t Ybandos;          //Yoffset
  uint16_t Ybandsr;          //Y size rectang
  uint16_t Ybandnr;          //Y next rectang
} Bandnumber;

//  Bandnumber table for the hambands
uint16_t Xfband = 0;
uint16_t Yfband = 0;

Bandnumber bn[] = {
  {  3 , 0 , Xfband, 150 ,   0 , Yfband , 35 ,   0},
  {  4 , 6 , Xfband, 150 ,   0 , Yfband , 35 ,  35},
  {  5 , 4 , Xfband, 150 ,   0 , Yfband , 35 ,  70},
  {  8 , 8 , Xfband, 150 ,   0 , Yfband , 35 , 105},
  { 10 , 1 , Xfband, 150 ,   0 , Yfband , 35 , 140},
  { 12 , 5 , Xfband, 150 ,   0 , Yfband , 35 , 175},
  { 15 , 2 , Xfband, 150 , 150 , Yfband , 35 ,   0},
  { 18 , 7 , Xfband, 150 , 150 , Yfband , 35 ,  35},
  { 20 , 3 , Xfband, 150 , 150 , Yfband , 35 ,  70},
  { 23 , 9 , Xfband, 150 , 150 , Yfband , 35 , 105},
  { 25 , 10 , Xfband, 150 , 150 , Yfband , 35 , 140},
  { 28 , 11 , Xfband, 150 , 150 , Yfband , 35 , 175}
};


//======================================================= End Ham Band Definitions     ========================

//======================================================= THE Band Definitions     ============================
typedef struct // Band data
{
  const char *bandName; // Bandname
  uint8_t  bandType;    // Band type (FM, MW or SW)
  uint16_t prefmod;     // Pref. modulation
  uint16_t minimumFreq; // Minimum frequency of the band
  uint16_t maximumFreq; // maximum frequency of the band
  uint16_t currentFreq; // Default frequency or current frequency
  uint16_t currentStep; // Default step (increment and decrement)
  //float BFOf1;            // BFO set for f1
  //float F1b;              // Freq1 in kHz
  //float BFOf2;            // BFO set for f2
  //float F2b;              // Freq2 in kHz
} Band;

//   Band table

Band band[] = {
  {   "FM", FM_BAND_TYPE,  FM,  7600, 10800,  9890, 10}, //  FM           0
  {   "OL", LW_BAND_TYPE,  AM,   153,   279,   198, 1}, //  LW            1
  {   "OM", MW_BAND_TYPE,  AM,   520,  1710,  1000, 10}, //  MW           2
  {   "SW", LW_BAND_TYPE,  AM,   150, 30000,   284, 1}, // All/LW/MW/SW   3
  {   "630 m", SW_BAND_TYPE, LSB,   472,   479,   475, 1}, // Ham  630M   4
  {   "160 m", SW_BAND_TYPE, LSB,  1800,  1910,  1899, 1}, // Ham  160M   5
  {   "OT", SW_BAND_TYPE,  AM,  1920,  3200,  2400, 5}, //      120M      6
  {   "90 m", SW_BAND_TYPE,  AM,  3200,  3400,  3300, 5}, //       90M    7
  {   "80 m", SW_BAND_TYPE, LSB,  3500,  3800,  3700, 1}, // Ham   80M    8
  {   "75 m", SW_BAND_TYPE,  AM,  3900,  4000,  3950, 5}, //       75M    9
  {   "60 m", SW_BAND_TYPE, USB,  5330,  5410,  5375, 1}, // Ham   60M   10
  {   "49 m", SW_BAND_TYPE,  AM,  5900,  6200,  6000, 5}, //       49M   11
  {   "40 m", SW_BAND_TYPE, LSB,  7000,  7200,  7132, 1}, // Ham   40M   12
  {   "41 m", SW_BAND_TYPE,  AM,  7200,  7450,  7210, 5}, //       41M   13
  {   "31 m", SW_BAND_TYPE,  AM,  9400,  9900,  9600, 5}, //       31M   14
  {   "30 m", SW_BAND_TYPE, USB, 10100, 10150, 10125, 1}, // Ham   30M   15
  {   "25 m", SW_BAND_TYPE,  AM, 11600, 12100, 11700, 5}, //       25M   16
  {   "22 m", SW_BAND_TYPE,  AM, 13570, 13870, 13700, 5}, //       22M   17
  {   "20 m", SW_BAND_TYPE, USB, 14000, 14350, 14200, 1}, // Ham   20M   18
  {   "19 m", SW_BAND_TYPE,  AM, 15100, 15830, 15700, 5}, //       19M   19
  {   "17 m", SW_BAND_TYPE, USB, 18068, 18168, 18100, 1}, // Ham   17M   20
  {   "16 m", SW_BAND_TYPE,  AM, 17480, 17900, 17600, 5}, //       16M   21
  {   "15 m", SW_BAND_TYPE,  AM, 18900, 19020, 18950, 5}, //       15M   22
  {   "15 m", SW_BAND_TYPE, USB, 21000, 21450, 21350, 1}, // Ham   15M   23
  {   "13 m", SW_BAND_TYPE,  AM, 21450, 21850, 21500, 5}, //       13M   24
  {   "12 m", SW_BAND_TYPE, USB, 24890, 24990, 24940, 1}, // Ham   12M   25
  {   "11 m", SW_BAND_TYPE,  AM, 25670, 26100, 25800, 5}, //       11M   26
  {   "PX", SW_BAND_TYPE,  AM, 26200, 27990, 27200, 1}, // CB band       27
  {   "10 m", SW_BAND_TYPE, USB, 28000, 30000, 28500, 1}, // Ham   10M   28
  {   "OC", SW_BAND_TYPE,  AM,  1730, 30000, 15500, 5}  // Whole SW      29
};
//======================================================= End THE Band Definitions     ========================

//======================================================= FM Presets     ======================================
typedef struct // Preset data
{
  float      presetIdx;
  const char *PresetName;
} FM_Preset ;

FM_Preset preset[] = {

  9370  , "Itapema",      // 00

};


//======================================================= END FM Presets     ======================================

const int lastButton = (sizeof bt / sizeof(Button)) - 1;
const int lastBand   = (sizeof band / sizeof(Band)) - 1;
const int lastHam    = (sizeof bn / sizeof(Bandnumber)) - 1;
const int lastBroad  = (sizeof bb / sizeof(BBandnumber)) - 1;
const int lastMod    = (sizeof md / sizeof(Mode)) - 1;
const int lastBW     = (sizeof bw / sizeof(Bandwidth)) - 1;
const int lastStep   = (sizeof sp / sizeof(Step)) - 1;
const int lastKPath  = (sizeof kp / sizeof(Keypath)) - 1;
const int lastPreset = (sizeof preset / sizeof (FM_Preset)) - 1;

#define offsetEEPROM       0x30
#define EEPROM_SIZE        150

struct StoreStruct {
  byte     chkDigit;
  byte     bandIdx;
  uint16_t Freq;
  uint8_t  currentMode;
  uint8_t  bwIdxSSB;
  uint8_t  bwIdxAM;
  uint8_t  currentStep;
  int      currentBFO;
  uint8_t  currentAGCAtt;
  uint8_t  currentVOL;
  uint8_t  currentBFOStep;
  uint8_t  RDS;
};

StoreStruct storage = {
  '#',  //First time check
  0,  //bandIdx
  8930,  //Freq
  0,  //mode
  1,  //bwIdxSSB
  3,  //bwIdxAM
  9,  //currentStep
  -125,  //currentBFO
  2,  //currentAGCAtt
  45,  //currentVOL
  25,  //currentBFOStep
  1   //RDS
};

uint8_t rssi = 0;
uint8_t stereo = 1;
uint8_t volume = DEFAULT_VOLUME;

// Devices class declarations
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);

TFT_eSPI tft = TFT_eSPI();

SI4735 si4735;

//=======================================================================================
void IRAM_ATTR RotaryEncFreq() { //ICACHE_RAM_ATTR ?????
  //=======================================================================================
  // rotary encoder events
  if (!writingEeprom) {
    encoderStatus = encoder.process();

    if (encoderStatus)
    {
      if (encoderStatus == DIR_CW)// Direction clockwise
      {
        encoderCount = -1;
      }
      else
      {
        encoderCount = 1;
      }
    }
  }
}

/**
   Cleans the EEPROM
*/
void resetEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  for (int k = 0; k < EEPROM_SIZE; k++) {
    EEPROM.write(k, 0);
  }
  EEPROM.end();
}

//=======================================================================================
void setup() {
  //=======================================================================================
  pinMode(Display_Led, OUTPUT);
  digitalWrite(Display_Led, displayoff);
  DISplay = false;

  uint16_t calData[5] = { 257, 3678, 209, 3607, 7 }; // 3.5" - ILI9488
  tft.setTouch(calData);

  //  uint16_t calData[5] = { 271, 3658, 188, 3673, 7 }; // 4.0" - ST7796S
  //  tft.setTouch(calData);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  SPIFFS.begin();
  delay(100);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(&FREQNUMBER);
  tft.drawString("BUTUCA", 150, 120);
  tft.drawString(VersionText, 300, 220);
  tft.setFreeFont(NULL);

  tft.drawBitmap (0, 0, Noise, 480, 320, TFT_WHITE);

  digitalWrite(Display_Led, displayon);
  DISplay = true;
  UNSETColor = tft.color565(102, 80, 00);
  SETColor = tft.color565(102, 80, 00);
  BackColor  = tft.color565(145, 116, 07); // XXXXXXXXXX
  REDMETER = tft.color565(150, 21, 9);
  tft.setTextColor(TFT_BLACK, BackColor);
  // if (!SPIFFS.begin()) {
  //    Serial.println("SPIFFS initialisation failed!");
  //    while (1) yield(); // Stay here twiddling thumbs waiting
  //  }
  //  Serial.println("\r\nSPIFFS available!");
  //
  //  // ESP32 will crash if any of the fonts are missing
  //  bool font_missing = false;
  //  if (SPIFFS.exists("/NotoSansBold15.vlw")    == false) font_missing = true;
  //  if (SPIFFS.exists("/NotoSansBold36.vlw")    == false) font_missing = true;
  //
  //  if (font_missing)
  //  {
  //    Serial.println("\r\nFont missing in SPIFFS, did you upload it?");
  //    while(1) yield();
  //  }
  //  else Serial.println("\r\nFonts found OK.");
  //tft.setRotation(0); // Rotate 0
  //tft.setRotation(1); // Rotate 90
  //tft.setRotation(2); // Rotate 180
  //tft.setRotation(3); // Rotate 270

  // Cleans the EEPROM content.
  // if the encoder push button is pressed during the system initialization, the EEPROM will be clened.
  for (int i = 0; i < 10; i++) {
    encBut = analogRead(ENCODER_SWITCH);
    if (encBut < 500) {
      resetEEPROM(); // Cleans the EEPROM.
      tft.fillScreen(BackColor);
      tft.setCursor(0, 0);
      tft.println(F("A EEPROM FOI LIMPA!"));
      while (1); // Stops the System. It is needed to turn it off.
    }
    delay(100);
  }

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    tft.fillScreen(TFT_YELLOW);
    tft.setCursor(0, 0);
    tft.println(F("failed to initialise EEPROM"));
    // Serial.println(F("failed to initialise EEPROM"));
    while (1);
  }

  // RESET the EEPROM
  if (EEPROM.read(offsetEEPROM) != storage.chkDigit) {
    // Serial.println(F("Writing defaults...."));
    saveConfig();
  }
  loadConfig();
  printConfig();


  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL); //I2C for SI4735

  // Serial.println(F("I2C - Inicio"));

  // Encoder pins
  pinMode(ENCODER_PIN_A , INPUT_PULLUP); //Rotary encoder Freqency/bfo/preset
  pinMode(ENCODER_PIN_B , INPUT_PULLUP);
  // Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), RotaryEncFreq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), RotaryEncFreq, CHANGE);

  si4735.setAudioMuteMcuPin(AUDIO_MUTE);
  int16_t si4735Addr = si4735.getDeviceI2CAddress(RESET_PIN);

  // Serial.println(F("I2C - ADDR"));
  // Serial.println(si4735Addr);

  //  tft.fillScreen(BackColor);

  //  delay(100);
  //  tft.setCursor(7, 25);
  //  tft.setFreeFont(FSB12);
  //  tft.setTextSize(1);
  //  tft.setTextColor(TextColor, BackColor);
  //  // Serial.println("    SI4732 RADIO");
  //  // Serial.println("Version 1");
  //  tft.println("RADIO DSP");
  //  //  tft.setCursor(7, 75);
  //  tft.println("SOFT PE0MGB-PU2CLR");
  //  //  tft.setCursor(7, 125);
  //  tft.println("PCB BY THIAGO");
  //  tft.setCursor(7, 200);
  //  tft.setTextColor(TextColor, BackColor);
  if ( si4735Addr == 0 ) {
    tft.print("Si4735 not detected");
    //    tft.setFreeFont(NULL);
    // Serial.println("Si4735 not detected");
    while (1);
  } else {
    tft.setTextColor(TextColor, BackColor);
    //    tft.setFreeFont(FSB9);
    //    tft.setTextSize(1);
    //    tft.print("SI4732 :  ");
    //    tft.println(si4735Addr, HEX);
    //    tft.setFreeFont(NULL);
    // Serial.print("Si473X addr :  ");
    // Serial.println(si4735Addr, HEX);
  }


  delay(3000);
  digitalWrite(Display_Led, displayoff);
  DISplay = false;
  tft.fillScreen(BackColor);
  BaselineDisplay ();



  // Setup the radio from last setup in EEPROM

  // Serial.println(F("Iniciando VARIAVEIS"));

  bandIdx                   = storage.bandIdx;
  band[bandIdx].currentFreq = storage.Freq;
  currentMode               = storage.currentMode;
  bwIdxSSB                  = storage.bwIdxSSB;
  bwIdxAM                   = storage.bwIdxAM;
  currentStep               = storage.currentStep;
  currentBFO                = storage.currentBFO;
  currentAGCAtt             = storage.currentAGCAtt;
  currentVOL                = storage.currentVOL;
  currentBFOStep            = storage.currentBFOStep;
  RDS                       = storage.RDS;

  // Serial.println(F("Iniciando O SI4732"));

  if (bandIdx == 0)  si4735.setup(RESET_PIN, 0); // Start in FM
  else si4735.setup(RESET_PIN, 1); // Start in AM
  if (bandIdx != 0) si4735.setAM();

  // Serial.println(F("SI4732 Iniciado"));

  freqstep = 1000;//hz
  previousBFO = -1;
  si4735.setVolume(currentVOL);
  previousVOL = currentVOL;

  BandSet();
  if (currentStep != band[bandIdx].currentStep ) band[bandIdx].currentStep = currentStep;
  currentFrequency = previousFrequency = si4735.getFrequency();

  encBut = 600;
  x = y = 0;
  DrawFila();
  si4735.setSeekFmSpacing(1);
  si4735.setSeekFmLimits(8750, 10790);
  si4735.setSeekAmRssiThreshold(20);
  si4735.setSeekAmSrnThreshold(10);
  si4735.setSeekFmRssiThreshold(5);
  si4735.setSeekFmSrnThreshold(5);

  xTaskCreate(SaveInEeprom, "SaveInEeprom", 2048, NULL, 1, NULL);
  ColdStart ();
  digitalWrite(Display_Led, displayon);
  DISplay = true;

}// end setup


/**
   Cleans the buffer contents

*/
void cleanBuffer() {
  bufferVolume[0] = bufferAgcGain[0] = bufferFrequency[0] = bufferUnit[0] = bufferBandName[0] = bufferVFO[0] = '\0';
}

//=======================================================================================
void BaselineDisplay ()  {
  //=======================================================================================

  //----------------------------------------------------//
  //   Limpa display apresenta as teclas sem texto
  //----------------------------------------------------//
  //  digitalWrite(Display_Led, displayon);
  tft.fillScreen(BackColor);
  clearKeyPathFila (); // teclas sem texto na base (12)


  //-------------------------------------------------------------------//
  //   Apresenta linha, texto e numeros do SMETER e SNR METER
  //-------------------------------------------------------------------//

  Xsmtr = Xinfo;
  Ysmtr = 155 + Yinfo;
  tft.setTextColor(TextColor, BackColor);
  tft.loadFont(SMeterText);
  tft.drawString("S METER", Xsmtr, Ysmtr);
  tft.drawString("SNR METER",  Xsmtr + 224, Ysmtr);

  for (int i = 0; i < 10; i++) {
    tft.drawRoundRect(Xsmtr + (i * 12), Ysmtr + 11, 11, 13, 2, TFT_BLACK);
    tft.setCursor((Xsmtr + 2 + (i * 12)), Ysmtr + 26);
    if ((i == 0)) tft.print("S");
    else tft.print(i);
  }
  for (int i = 1; i <= 7; i++) {
    tft.drawRoundRect((Xsmtr + 108 + (i * 12)), Ysmtr + 11, 11, 13, 2, TFT_BLACK);
    tft.setCursor((Xsmtr + 102 + (i * 12)), Ysmtr + 26);
    if ((i == 2) || (i == 4) || (i == 6))
    {
      tft.setTextColor(REDMETER, BackColor);
      tft.print("+");
      tft.print(i * 10);
      tft.setTextColor(TextColor, BackColor);
    }
  }

  tft.setCursor(Xsmtr + 227, Ysmtr + 26);
  tft.print("0");
  for (int i = 0; i < 11; i++)
  {
    tft.drawRoundRect(Xsmtr + 224 + (i * 12), Ysmtr + 11, 11, 13, 2, TFT_BLACK);
    tft.setCursor((Xsmtr + 224 + (i * 12)), Ysmtr + 26);
    if ((i % 2 == 0) && (i != 0)) {
      if (i == 10) {
        tft.setCursor(Xsmtr + 337, Ysmtr + 26);
        tft.print("100");
      }
      else tft.print(i * 10);
    }
  }
  tft.unloadFont();

  //-------------------------------------------------------------------//
  //   Apresenta Frequencia
  //-------------------------------------------------------------------//
  int XFreq = Xinfo;
  int YFreq = Yinfo - 10;
  tft.setTextColor(TextColor, BackColor); // xxxxxxxxxxxxxx
  bufferFrequency[0] = '\0';
  showFrequency();
  //-------------------------------------------------------------------//
  //   Apresenta Informações
  //-------------------------------------------------------------------//


  tft.loadFont(SETUPLINE);

  tft.setTextColor(TFT_BLACK, BackColor);

  tft.drawString("AGC", Xinfo , Yinfo);

  tft.fillRect(Xinfo + 32, Yinfo + 1, 2, 30, TFT_BLACK);
  tft.drawString("BFO", Xinfo + 38 , Yinfo);

  tft.fillRect(Xinfo + 89 , Yinfo + 1, 2, 30, TFT_BLACK);
  tft.drawString("STEP", Xinfo + 95, Yinfo);

  tft.fillRect(Xinfo + 152, Yinfo + 1, 2, 30, TFT_BLACK);
  tft.drawString("FILTER", Xinfo + 158, Yinfo);

  tft.fillRect(Xinfo + 210, Yinfo + 1, 2, 30, TFT_BLACK);
  tft.drawString("VOL", Xinfo + 216, Yinfo);

  tft.fillRect(Xinfo + 248, Yinfo + 1, 2, 30, TFT_BLACK);
  tft.drawString("RSSI", Xinfo + 254, Yinfo);

  tft.fillRect(Xinfo + 308, Yinfo + 1, 2, 30, TFT_BLACK);
  tft.drawString("SNR", Xinfo + 314, Yinfo);

  //  tft.setTextColor(UNSETColor, BackColor);
  tft.fillRect(Xinfo + 352, Yinfo + 1, 2, 30, TFT_BLACK);
  tft.drawString("RDS", Xinfo + 358, Yinfo);

  tft.unloadFont();

}

//=======================================================================================
void SaveInEeprom (void* arg)  {
  //=======================================================================================
  while (1) {
    storage.bandIdx = bandIdx;
    storage.Freq =  band[bandIdx].currentFreq;
    storage.currentMode = currentMode;
    storage.bwIdxSSB = bwIdxSSB;
    storage.bwIdxAM = bwIdxAM;
    storage.currentStep = currentStep;
    storage.currentBFO = currentBFO;
    storage.currentAGCAtt = currentAGCAtt;
    storage.currentVOL = currentVOL;
    storage.currentBFOStep = currentBFOStep;
    storage.RDS = RDS;
    for (unsigned int t = 0; t < sizeof(storage); t++) {
      delay(1);
      if (EEPROM.read(offsetEEPROM + t) != *((char*)&storage + t)) {
        delay(1);
        EEPROM.write(offsetEEPROM + t, *((char*)&storage + t));
      }
    }
    writingEeprom = true;
    EEPROM.commit();
    writingEeprom = false;
    vTaskDelay(5000 / portTICK_RATE_MS);
  }
}

//=======================================================================================
void saveConfig() {
  //=======================================================================================
  delay(10);
  for (unsigned int t = 0; t < sizeof(storage); t++) {
    if (EEPROM.read(offsetEEPROM + t) != *((char*)&storage + t)) {
      EEPROM.write(offsetEEPROM + t, *((char*)&storage + t));
    }
  }
  EEPROM.commit();
}

//=======================================================================================
void loadConfig() {
  //=======================================================================================
  if (EEPROM.read(offsetEEPROM + 0) == storage.chkDigit) {
    for (unsigned int t = 0; t < sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(offsetEEPROM + t);
    // Serial.println("Load config done");
  }
}

//=======================================================================================
void printConfig() {
  //=======================================================================================
  // Serial.println(sizeof(storage));
  // if (EEPROM.read(offsetEEPROM) == storage.chkDigit){
  //  for (unsigned int t = 0; t < sizeof(storage); t++)
  //    Serial.write(EEPROM.read(offsetEEPROM + t));
  //  Serial.println();
  //setSettings(0);
  //}
}

//=======================================================================================
void BandSet()  {
  //=======================================================================================
  if (bandIdx == 0) currentMode = 0;// only mod FM in FM band
  if ((currentMode == AM) || (currentMode == FM)) ssbLoaded = false;

  if ((currentMode == LSB) || (currentMode == USB) || (currentMode == SYNC))
  {
    if (ssbLoaded == false) {
      loadSSB();
    }
  }
  tft.setFreeFont(&NotoSans_ExtraCondensedSemiBold56pt7b); //Apaga a frequencia anterior
  tft.setTextPadding(tft.textWidth("10804"));
  tft.drawString("", XFreq + 50, YFreq + 54);
  tft.setTextPadding(0);
  tft.setFreeFont(NULL);

  useBand();
  setBandWidth();
}

//=======================================================================================
void useBand()  {
  //=======================================================================================
  //  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  cleanBuffer();

  if (band[bandIdx].bandType == FM_BAND_TYPE)
  {
    bfoOn = false;
    si4735.setTuneFrequencyAntennaCapacitor(0);
    delay(100);
    si4735.setFM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
    si4735.setFMDeEmphasis(1);
    ssbLoaded = false;
    si4735.RdsInit();
    si4735.setRdsConfig(1, 2, 2, 2, 2);
    currentMode = FM; // necessario setar, houve algum problema XXXX
  }
  else
  {
    if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
      si4735.setTuneFrequencyAntennaCapacitor(0);
    } else { //SW_BAND_TYPE
      si4735.setTuneFrequencyAntennaCapacitor(1);
    }
    if (ssbLoaded)
    {
      si4735.setSSB(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep, currentMode);
      si4735.setSSBAutomaticVolumeControl(1);
      //si4735.setSsbSoftMuteMaxAttenuation(0); // Disable Soft Mute for SSB
      //si4735.setSSBDspAfc(0);
      //si4735.setSSBAvcDivider(3);
      //si4735.setSsbSoftMuteMaxAttenuation(8); // Disable Soft Mute for SSB
      //si4735.setSBBSidebandCutoffFilter(0);

      si4735.setSSBBfo(currentBFO);
    }
    else
    {
      si4735.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
      //si4735.setAutomaticGainControl(1, 0);
      //si4735.setAmSoftMuteMaxAttenuation(0); // // Disable Soft Mute for AM
      bfoOn = false;
    }

  }
  delay(100);
}// end useband

//=======================================================================================
void setBandWidth()  {
  //=======================================================================================
  if (currentMode == LSB || currentMode == USB)
  {
    si4735.setSSBAudioBandwidth(bwIdxSSB);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bwIdxSSB == 0 || bwIdxSSB == 4 || bwIdxSSB == 5)
      si4735.setSBBSidebandCutoffFilter(0);
    else
      si4735.setSBBSidebandCutoffFilter(1);
  }
  else if (currentMode == AM)
  {
    si4735.setBandwidth(bwIdxAM, 0);
  }
}

//=======================================================================================
void loadSSB()  {
  //=======================================================================================
  // si4735.reset();
  si4735.queryLibraryId(); // Is it really necessary here? I will check it.
  si4735.patchPowerUp();
  delay(50);
  //  si4735.setI2CFastMode(); // Recommended  XXX REtirar e ver se da ruido.
  //si4735.setI2CFastModeCustom(500000); // It is a test and may crash.
  si4735.downloadPatch(ssb_patch_content, size_content);
  si4735.setI2CStandardMode(); // goes back to default (100kHz)

  // delay(50);
  // Parameters
  // AUDIOBW - SSB Audio bandwidth; 0 = 1.2kHz (default); 1=2.2kHz; 2=3kHz; 3=4kHz; 4=500Hz; 5=1kHz;
  // SBCUTFLT SSB - side band cutoff filter for band passand low pass filter ( 0 or 1)
  // AVC_DIVIDER  - set 0 for SSB mode; set 3 for SYNC mode.
  // AVCEN - SSB Automatic Volume Control (AVC) enable; 0=disable; 1=enable (default).
  // SMUTESEL - SSB Soft-mute Based on RSSI or SNR (0 or 1).
  // DSP_AFCDIS - DSP AFC Disable or enable; 0=SYNC MODE, AFC enable; 1=SSB MODE, AFC disable.
  if (currentMode == SYNC) {
    si4735.setSSBConfig(bwIdxSSB, 1, 3, 0, 0, 0); // SYNC MODE
    currentBFO = 0;
  }
  else
    si4735.setSSBConfig(bwIdxSSB, 1, 0, 0, 0, 1);
  delay(25);
  ssbLoaded = true;
}

//=======================================================================================
void Freqcalq(int keyval)  { // apresenta a frequencia a ser inserida
  //=======================================================================================
  char tmpFreq [7];
  if (Decipoint)
  {
    dpfrq = dpfrq + keyval / fact;
  }
  else
  {
    Displayfreq = (Displayfreq + keyval) * 10;
  }
  fact = fact * 10;
  tft.setFreeFont(&ERRORTYPE);
  tft.setTextColor(REDMETER, BackColor);
  if (Decipoint)
  {
    tft.setTextPadding(tft.textWidth("108.00"));
    snprintf(tmpFreq, sizeof(tmpFreq), "%3.3f", (Displayfreq / 10) + dpfrq);
  }
  else
  {
    tft.setTextPadding(tft.textWidth("29999"));
    snprintf(tmpFreq, sizeof(tmpFreq) - 1, "%5.0f", ((Displayfreq / 10) + dpfrq));
  }
  tft.drawString(tmpFreq, XFreq + 260 , YFreq + 145);
}

//=======================================================================================
void Smeter() {
  //=======================================================================================
  int spoint;
  if (currentMode != FM) {
    //dBuV to S point conversion HF
    if ((rssi >= 0) and (rssi <=  1)) spoint =  0;        // S0
    if ((rssi >  1) and (rssi <=  1)) spoint =  1;        // S1
    if ((rssi >  2) and (rssi <=  3)) spoint =  2;        // S2
    if ((rssi >  3) and (rssi <=  4)) spoint =  3;        // S3
    if ((rssi >  4) and (rssi <= 10)) spoint =  4;        // S4
    if ((rssi > 10) and (rssi <= 16)) spoint =  5;        // S5
    if ((rssi > 16) and (rssi <= 22)) spoint =  6;        // S6
    if ((rssi > 22) and (rssi <= 28)) spoint =  7;        // S7
    if ((rssi > 28) and (rssi <= 34)) spoint =  8;        // S8
    if ((rssi > 34) and (rssi <= 44)) spoint =  9;        // S9
    if ((rssi > 44) and (rssi <= 54)) spoint = 10;        // S9 +10
    if ((rssi > 54) and (rssi <= 64)) spoint = 11;        // S9 +20
    if ((rssi > 64) and (rssi <= 74)) spoint = 12;        // S9 +30
    if ((rssi > 74) and (rssi <= 84)) spoint = 13;        // S9 +40
    if ((rssi > 84) and (rssi <= 94)) spoint = 14;        // S9 +50
    if ((rssi > 94) and (rssi <= 95)) spoint = 15;        // S9 +60
    if  (rssi > 95)                   spoint = 16;        //>S9 +60
  }
  else
  {
    //dBuV to S point conversion FM
    if  (rssi <  1) spoint = 5;
    if ((rssi >  1) and (rssi <=  2)) spoint =  6;        // S6
    if ((rssi >  2) and (rssi <=  8)) spoint =  7;        // S7
    if ((rssi >  8) and (rssi <= 14)) spoint =  8;        // S8
    if ((rssi > 14) and (rssi <= 24)) spoint =  9;        // S9
    if ((rssi > 24) and (rssi <= 34)) spoint = 10;        // S9 +10
    if ((rssi > 34) and (rssi <= 44)) spoint = 11;        // S9 +20
    if ((rssi > 44) and (rssi <= 54)) spoint = 12;        // S9 +30
    if ((rssi > 54) and (rssi <= 64)) spoint = 13;        // S9 +40
    if ((rssi > 64) and (rssi <= 74)) spoint = 14;        // S9 +50
    if ((rssi > 74) and (rssi <= 76)) spoint = 15;        // S9 +60
    if  (rssi > 76)                   spoint = 16;        //>S9 +60
  }

  for (int i = 0; i <= spoint; i++)
  {
    tft.fillRoundRect(Xsmtr + 1 + (i * 12), Ysmtr + 12, 9, 11, 2, ((i > 9) ? REDMETER : UNSETColor));
  }
  for (int i = spoint + 1; i <= 17; i++)
  {
    tft.fillRoundRect(Xsmtr + 1 + (i * 12), Ysmtr + 12, 9, 11, 2, BackColor);
  }
}

//=======================================================================================
void SNRmeter() {
  //=======================================================================================
  int spoint;

  //dBuV to S point conversion FM
  if  (NewSNR <  10) spoint = 0;                            // SNR = >=0  & <10
  if ((NewSNR >= 10) and (NewSNR < 20)) spoint =  1;        // SNR = >=10 & <20
  if ((NewSNR >= 20) and (NewSNR < 30)) spoint =  2;        // SNR = >=20 & <30
  if ((NewSNR >= 30) and (NewSNR < 40)) spoint =  3;        // SNR = >=30 & <40
  if ((NewSNR >= 40) and (NewSNR < 50)) spoint =  4;        // SNR = >=40 & <50
  if ((NewSNR >= 50) and (NewSNR < 60)) spoint =  5;        // SNR = >=50 & <60
  if ((NewSNR >= 60) and (NewSNR < 70)) spoint =  6;        // SNR = >=60 & <70
  if ((NewSNR >= 70) and (NewSNR < 80)) spoint =  7;        // SNR = >=70 & <80
  if ((NewSNR >= 80) and (NewSNR < 90)) spoint =  8;        // SNR = >=80 & <90
  if ((NewSNR >= 90) and (NewSNR < 100)) spoint =  9;       // SNR = >=90 & <100
  if  (NewSNR >= 100)spoint = 10;                           // SNR = >=100

  for (int i = 0; i <= spoint; i++)
  {
    tft.fillRoundRect(Xsmtr + 225 + (i * 12), Ysmtr + 12, 9, 11, 2, ((i > 9) ? TFT_BLACK : UNSETColor));
  }
  for (int i = spoint + 1; i <= 10; i++)
  {
    tft.fillRoundRect(Xsmtr + 225 + (i * 12), Ysmtr + 12, 9, 11, 2, BackColor);
  }
}
//=======================================================================================
void VolumeIndicator(int vol) {
  //=======================================================================================
  vol = map(vol, 20, 63, 0, 212);
}

//=======================================================================================
void ShowVolume () {
  //=======================================================================================
  tft.setFreeFont(&SETUPLINE1); //Apresenta Volume
  tft.setTextPadding(tft.textWidth("100"));
  tft.drawString(String((int) map(currentVOL, MinVOL, MaxVOL, 0, 100)), Xinfo + 216, Yinfo + 17); // PRINT VOL
  tft.setTextPadding(0);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void ColdStart () {
  //=======================================================================================
  ShowVolume ();
  showRSSI();
}

//=======================================================================================
void loop() {
  //=======================================================================================

  if ((FirstLayer == true) or (ThirdLayer == true))  map(si4735.getVolume(), 20, 63, 0, 212);

  // Pressed will be set true is there is a valid touch on the screen
  while (((pressed == false) and (encoderCount == 0) and (encBut > 500)) or (writingEeprom)) {  // wait loop
    pressed = tft.getTouch(&x, &y);
    encBut = analogRead(ENCODER_SWITCH);
    showtimeRSSI();
    DisplayRDS();
    Dispoff();
  }

  encoderCheck();        // Check if the encoder has moved.
  encoderButtonCheck();  // Check if encoderbutton is pressed

  if (pressed) {
    pressed = false;
    PRESbut = false; // Preset stopped after other button is pressed

    DisplayOnTime = millis();
    if (DISplay == false) {
      delay(200);
      digitalWrite(Display_Led, displayon);
      DISplay = true;
      x = y = 0; // no valid touch only for switch on display led
    }
    if (FirstLayer) { //==================================================
      //Check which button is pressed in First Layer.
      for (int n = 0 ; n <= lastButton; n++) {
        if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz)) {
          delay(200);
          x = 0;
          y = 0;
          //tft.print(n);
          if ((VOLbut == true) and (n != VOL)) {
            VOLbut = false;
            drawVOL();
            DrawDispl ();
          }

          if ((Mutestat == true) and (n != MUTE)) {
            Mutestat = false;
            drawMUTE();
          }

          if ((bfoOn == true) and (n == VOL)) {
            bfoOn = false;
            drawBFO();
          }


          if (n == HAM) {
            delay(200);  //HamBand button
            HamBand = true;
            HamBandlist();
            FirstLayer = false;
            SecondLayer = true;
          }

          if (n == BFO) {           //==============================  // BFO button
            delay(200);

            if (currentMode == LSB || currentMode == USB)  {
              if (bfoOn == false) {
                bfoOn = true;
              } else {
                bfoOn = false;
              }
              cleanDispl();
              drawBFO();
              DrawDispl ();
            }
          }

          if (n == FREQ) {          //============================  // Frequency input
            delay(200);
            FREQbut = true;
            Decipoint = false;
            Displayfreq = 0;
            dpfrq = 0;
            drawKeyFreq();
            FirstLayer = false;
            SecondLayer = true;
          }

          if (n == AGC) {           //============================//AGC switch
            si4735.getAutomaticGainControl();
            AGCgain = 0;
            if  (si4735.isAgcEnabled()) {
              si4735.setAutomaticGainControl(1, 0);     //    disabled
            } else {
              AGCgainbut = false;
              si4735.setAutomaticGainControl(0, 0);      //   enabled
            }
            cleanDispl();
            drawAGC();
            DrawDispl ();
          }

          if (n == MODE) {    //============================= MODE
            if (currentMode != FM)  {
              delay(200);// Mode
              Modebut = true;
              Modelist();
              FirstLayer = false;
              SecondLayer = true;
            }
          }


          if (n == BANDW) {        //=========================BANDWIDTH
            delay(200);
            if (currentMode != FM)  {
              BandWidth = true;
              BWList();
              FirstLayer = false;
              SecondLayer = true;
            }
          }


          if (n == STEP) {            //========================= STEPS for tune and bfo
            delay(200);
            if (currentMode != FM)  {
              if (bfoOn) setStepBFO ();
              else {
                cleanDispl();
                STEPbut = true;
                Steplist();
                FirstLayer = false;
                SecondLayer = true;
              }
            }
          }

          if (n == BROAD)  {
            delay(200);
            BroadBand = true;
            Secondline = false;
            BroadBandlist();
            FirstLayer = false;
            SecondLayer = true;
          }

          if (n == PRESET) {
            //            if (currentMode != FM)
            //            {
            //              currentBFO = 0;
            //              if (!ssbLoaded)
            //              {
            //                loadSSB();
            //              }
            //              currentMode = (currentMode == LSB) ? USB : LSB;
            //              band[bandIdx].currentFreq = currentFrequency;
            //              band[bandIdx].currentStep = currentStep;
            //              useBand();
            //              si4735.setSSBDspAfc(0);
            //              si4735.setSSBAvcDivider(3);
            //            }
            delay(200);
            x = 0;
            y = 0;
            PRESbut = true;
            //tft.fillScreen(TFT_BLACK);
            FirstLayer = false;
            SecondLayer = true;
          }

          if (n == VOL) {
            delay(200);
            x = 0;
            y = 0;
            if (VOLbut == false) {
              VOLbut = true;
              currentVOL = si4735.getVolume();
              previousVOL = currentVOL;
              FirstLayer = false;
              SecondLayer = true;
            }
            else {
              VOLbut = false;
              FirstLayer = true;
              SecondLayer = false;
            }
            cleanDispl();
            FreqDispl();
            drawVOL();
          }

          if (n == MUTE) {
            delay(200);
            x = 0;
            y = 0;
            if (Mutestat == false)  {
              Mutestat = true;
            }
            else  {
              Mutestat = false;
            }
            drawMUTE();
          }
          if (n == SEEKUP) {
            delay(200);
            x = 0;
            y = 0;
            SEEK = true;
            if ((currentMode != LSB) and (currentMode != USB))   {
              if (currentMode != FM) {     // No FM
                if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     //9 kHz
                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
                }
                else {
                  bandIdx = 29;// all sw
                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     // 5 kHz
                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
                }
              }
              si4735.frequencyUp();
              si4735.seekStationProgress(SeekFreq, checkStopSeeking,  SEEK_UP);
              // si4735.seekStationProgress(SeekFreq,1);// 1 is up
              delay(300);
              currentFrequency = si4735.getFrequency();
              band[bandIdx].currentFreq = currentFrequency ;
              if (currentFrequency != previousFrequency)
              {
                previousFrequency = currentFrequency;
                DrawDispl();
                delay(300);
              }
            }
            cleanDispl();
            showFrequency();
            SEEK = false;
          }

          if (n == SEEKDOWN) {
            delay(200);
            x = 0;
            y = 0;
            SEEK = true;
            if ((currentMode != LSB) and (currentMode != USB))   {
              if (currentMode != FM) {     // No FM
                if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     //9 kHz
                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
                } else {
                  bandIdx = 29;// all sw
                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     // 5 kHz
                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
                }
              }
              si4735.frequencyDown();
              // si4735.seekStationProgress(SeekFreq,0);
              si4735.seekStationProgress(SeekFreq, checkStopSeeking,  SEEK_DOWN);
              delay(300);
              currentFrequency = si4735.getFrequency();
              band[bandIdx].currentFreq = currentFrequency ;
              if (currentFrequency != previousFrequency)
              {
                previousFrequency = currentFrequency;
                DrawDispl();
                delay(300);
              }
            }
            cleanDispl();
            showFrequency();
            SEEK = false;
          }

          if (n == SPCTR) {
            delay(200);
            x = 0;
            y = 0;
            SpectrumRF ();
            while (!tft.getTouch(&x, &y));
            tft.fillScreen(BackColor);
            FirstLayer  = true;
            SecondLayer = false;
            ThirdLayer  = false;
            ForthLayer  = false;
            BaselineDisplay ();
            DrawFila();
          }

          if (n == NEXT) {
            delay(200);
            x = 0;
            y = 0;
            FirstLayer  = false;
            SecondLayer = false;
            ThirdLayer  = true;
            ForthLayer  = false;
            DrawThla();
            cleanDispl();
            showFrequency();
            //            SpectrumRF (); // Teste
          }
        }
      }
      delay (150); // wait before read touch again;
    } // end FirstLayer

    if (SecondLayer) {  //===============================================================
      if (Modebut == true) {
        for (int n = 0 ; n <= lastMod - 1; n++) {
          if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz)) {

            delay(200);
            Modebut = false;
            currentMode = n;
            BandSet();
            DrawFila();
          }
        }
      }

      if (BandWidth == true) {
        if ( currentMode == AM) {
          for (int n = 0 ; n < 7; n++) {
            if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz)) {

              delay(200);
              bwIdxAM = bw[n].BandWidthAM;
              BandWidth = false;
              BandSet();
              //setBandWidth();
              DrawFila();

            }
          }
        }
        else {
          for (int n = 0 ; n < 6; n++) {
            if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz)) {

              delay(200);
              bwIdxSSB = bw[n].BandWidthSSB;
              BandWidth = false;
              BandSet();
              DrawFila();
            }
          }
        }
      }

      if (STEPbut == true) {
        for (int n = 0 ; n < lastStep + 1; n++) {
          if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz)) {

            delay(200);
            STEPbut = false;
            currentStep = sp[n].stepFreq;
            setStepBFO ();
            DrawFila();
          }
        }
      }

      if (BroadBand == true) {
        for (int n = 0 ; n <= lastBroad; n++)
        {
          if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz))
          {

            delay(200);

            if (n == 11 && Secondline)
            {
              BroadBandlist ();
              FirstLayer  = false;
              SecondLayer = false;
              ThirdLayer  = false;
              ForthLayer  = true;
            }
            else
            {
              BroadBand = false;
              bandIdx = bb[n].BbandNum;
              if ((bandIdx == 0) and (currentAGCgain >= 28)) currentAGCgain = previousAGCgain = 26; // currentAGCgain in FM max. 26
              si4735.setAM();
              delay(50);
              currentMode = band[bandIdx].prefmod;
              bwIdxAM =  3;
              BandSet();
              DrawFila(); //Draw first layer
            }
          }
        }
      }
      if (HamBand == true) {
        for (int n = 0 ; n <= lastHam; n++) {
          if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz)) {

            delay(200);
            HamBand = false;
            bandIdx = bn[n].BandNum;
            if (ssbLoaded == false) {
              si4735.setAM();
              delay(50);
            }
            currentMode = band[bandIdx].prefmod;
            bwIdxSSB = 1;
            BandSet();
            DrawFila();
          }
        }
      }


      if (PRESbut == true) {
        delay(200);
        if (currentMode != 0) { // geen fm ?
          bandIdx = 0;
          currentMode = 0;
          bfoOn = false;
          drawBFO();
          previousPRES = -1;
        }
        FirstLayer  =  true;
        SecondLayer = false;
        previousPRES = -2;
      }

      if (VOLbut == true) {
        delay(200);
        currentVOL = si4735.getVolume();
        previousVOL = currentVOL;
        FirstLayer  =  true;
        SecondLayer = false;
        cleanDispl();
        FreqDispl();
      }

      if (FREQbut == true) {
        //        Oldfrequency = currentFrequency;
        for (int n = 0 ; n < 12; n++) { // which keys are pressed?
          if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz)) {
            delay(200);
            if ((n >= 0) and (n <= 8)) Freqcalq(n + 1);
            if (n == 9) Freqcalq(0);
            if (n == 10) {
              Decipoint = true;
              fact = 10;
            }
            if (n == 11) {// Send button
              delay (200); //Aguarda na tela a frequencia digitada
              tft.drawString("", XFreq + 260 , YFreq + 145); //Limpa a frequencia digitada
              tft.setTextPadding(0); //Retira o Padding da freq (kHz ou MHz)
              tft.setFreeFont(NULL); //Retira fonte anterior
              FREQbut = false;
              Displayfreq = (Displayfreq / 10) + dpfrq;
              tft.setFreeFont(&ERRORTYPE);
              tft.setTextColor(BackColor, REDMETER);
              tft.setTextPadding(tft.textWidth("XERRORX"));

              if (Displayfreq < 1) {
                tft.drawString("  ERROR", XFreq + 260 , YFreq + 145);
              } else {
                if ((Displayfreq > 30.0) and (Displayfreq < 76.0 )) {
                  tft.drawString("  ERROR", XFreq + 260 , YFreq + 145);
                } else {
                  if ((Displayfreq >= 108) and (Displayfreq < 153 )) {
                    tft.drawString("  ERROR", XFreq + 260 , YFreq + 145);
                  } else {
                    if (Displayfreq > 30000) Displayfreq = Displayfreq / 1000000;
                    if ((Displayfreq <= 30000) and (Displayfreq >= 153) and (Decipoint == false )) Displayfreq = Displayfreq / 1000;
                    if ((Displayfreq >= 87.5) and (Displayfreq <= 108)) {
                      currentFrequency = Displayfreq * 100;
                      bandIdx = 0;
                      band[bandIdx].currentFreq = currentFrequency;
                    } else {
                      currentFrequency = Displayfreq * 1000;
                      for (int q = 1 ; q <= lastBand; q++) {
                        if (((currentFrequency) >= band[q].minimumFreq) and ((currentFrequency) <= band[q].maximumFreq)) {
                          bandIdx = q;
                          currentMode = band[q].prefmod;
                          currentStep = band[bandIdx].currentStep;
                          break;
                        }
                      }
                      delay(200);
                      band[bandIdx].currentFreq = currentFrequency;

                    }
                  }
                }
              }
              //              showContent(XFreq, YFreq + 142, (String(Oldfrequency)), (String(currentFrequency)), &FREQNUMBER, TFT_BLACK, 50, 1); //Frequencia
              delay (500); //Aguarda na tela a frequencia digitada
              tft.setTextColor(REDMETER, BackColor);
              tft.drawString("", XFreq + 260 , YFreq + 145); //Limpa a frequencia digitada
              tft.setFreeFont(NULL);
              tft.setTextDatum(TL_DATUM);
              tft.setTextPadding(0);
              BandSet();
              DrawFila();
            }//   End   n=11 Send button
          }
        }
      }//end freq
      delay (150); // wait before read touch again;
    }// end second layer

    if (ThirdLayer) { //==================================================
      //Check which button is pressed in Third Layer.
      for (int n = 0 ; n <= lastButton; n++) {
        if ((x > (Xbutst + (bt[bt[n].ButtonNum1].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum1].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum1].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum1].YButos) + Ybutsiz)) {

          delay(200);
          x = 0;
          y = 0;
          //tft.print(n);
          //          if (n == SEEKUP) {
          //            delay(200);
          //            x = 0;
          //            y = 0;
          //            SEEK = true;
          //            if ((currentMode != LSB) and (currentMode != USB))   {
          //              if (currentMode != FM) {     // No FM
          //                if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
          //                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     //9 kHz
          //                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
          //                }
          //                else {
          //                  bandIdx = 29;// all sw
          //                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     // 5 kHz
          //                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
          //                }
          //              }
          //              si4735.frequencyUp();
          //              si4735.seekStationProgress(SeekFreq, checkStopSeeking,  SEEK_UP);
          //              // si4735.seekStationProgress(SeekFreq,1);// 1 is up
          //              delay(300);
          //              currentFrequency = si4735.getFrequency();
          //              band[bandIdx].currentFreq = currentFrequency ;
          //              if (currentFrequency != previousFrequency)
          //              {
          //                previousFrequency = currentFrequency;
          //                DrawDispl();
          //                delay(300);
          //              }
          //            }
          //            cleanDispl();
          //            showFrequency();
          //            SEEK = false;
          //          }

          //          if (n == SEEKDN) {
          //            delay(200);
          //            x = 0;
          //            y = 0;
          //            SEEK = true;
          //            if ((currentMode != LSB) and (currentMode != USB))   {
          //              if (currentMode != FM) {     // No FM
          //                if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
          //                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     //9 kHz
          //                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
          //                } else {
          //                  bandIdx = 29;// all sw
          //                  si4735.setSeekAmSpacing(band[bandIdx].currentStep);     // 5 kHz
          //                  si4735.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
          //                }
          //              }
          //              si4735.frequencyDown();
          //              // si4735.seekStationProgress(SeekFreq,0);
          //              si4735.seekStationProgress(SeekFreq, checkStopSeeking,  SEEK_DOWN);
          //              delay(300);
          //              currentFrequency = si4735.getFrequency();
          //              band[bandIdx].currentFreq = currentFrequency ;
          //              if (currentFrequency != previousFrequency)
          //              {
          //                previousFrequency = currentFrequency;
          //                DrawDispl();
          //                delay(300);
          //              }
          //            }
          //            cleanDispl();
          //            showFrequency();
          //            SEEK = false;
          //          }

          if (n == INFO) {
            delay(200);
            x = 0;
            y = 0;
            subrstatus();
            //            while (!tft.getTouch(&x, &y));
            tft.fillScreen(BackColor);
            FirstLayer  = true;
            SecondLayer = false;
            ThirdLayer  = false;
            ForthLayer  = false;
            BaselineDisplay ();
            DrawFila();
          }

          if (n == BACK) {
            delay(200);
            x = 0;
            y = 0;
            AGCgainbut = false;
            FirstLayer  = true;
            SecondLayer = false;
            ThirdLayer  = false;
            ForthLayer  = false;
            cleanDispl();
            DrawFila();
          }

          if (n == FMRDS && currentMode == FM) {
            delay(200);
            x = 0;
            y = 0;
            if (RDS) RDS = false;
            else RDS = true;
            //DrawRDSbut();
            DrawThla();
            //DrawButThla();
          }

          if (n == AGCset) {
            delay(200);
            x = 0;
            y = 0;
            if (AGCgainbut) AGCgainbut = false;
            else {
              bfoOn = false; // only AGC function at the rotory encoder
              AGCgainbut = true;
              si4735.getAutomaticGainControl();
              previousAGCgain = 37; // force to setup AGC gain
            }
            FreqDispl();
            DrawThla();
          }
          if (n == ONOFF) {
            delay(200);
            x = 0;
            y = 0;
            DispoffUser();
            //            while (!tft.getTouch(&x, &y));
            tft.fillScreen(BackColor);
            FirstLayer  = true;
            SecondLayer = false;
            ThirdLayer  = false;
            ForthLayer  = false;
            BaselineDisplay ();
            DrawFila();
          }
          //          if (n == SPCTR1) {
          //            delay(200);
          //            x = 0;
          //            y = 0;
          //            SpectrumRF ();
          //            while (!tft.getTouch(&x, &y));
          //            tft.fillScreen(BackColor);
          //            FirstLayer  = true;
          //            SecondLayer = false;
          //            ThirdLayer  = false;
          //            ForthLayer  = false;
          //            BaselineDisplay ();
          //            DrawFila();
          //          }
          if (n == COLOR) {
            delay(200);
            x = 0;
            y = 0;
            NightDayBacklight ();
            while (!tft.getTouch(&x, &y));
            tft.fillScreen(BackColor);
            FirstLayer  = true;
            SecondLayer = false;
            ThirdLayer  = false;
            ForthLayer  = false;
            BaselineDisplay ();
            DrawFila();
          }
        }
      }
      delay (150); // wait before read the touch again;
    } // end ThirdLayer

    if (ForthLayer) { //===============================================================
      if (BroadBand == true) {
        for (int n = 0 ; n <= lastBroad - 11; n++)
        {
          if ((x > (Xbutst + (bt[bt[n].ButtonNum].XButos))) and (x < Xbutst + (bt[bt[n].ButtonNum].XButos) + Xbutsiz) and (y > Ybutst + (bt[bt[n].ButtonNum].YButos)) and (y < Ybutst + (bt[bt[n].ButtonNum].YButos) + Ybutsiz))
          {
            delay(200); // XXXXXXXXXXXXXXXXXXXXXXX
            BroadBand = false;
            n = n + 11;
            bandIdx = bb[n].BbandNum;
            if ((bandIdx == 0) and (currentAGCgain >= 28)) currentAGCgain = previousAGCgain = 26; // currentAGCgain in FM max. 26
            si4735.setAM();
            delay(50);
            currentMode = band[bandIdx].prefmod;
            bwIdxAM =  3;
            BandSet();
            DrawFila(); //Draw first layer
          }

        }
      }
      delay (150); // wait before read touch again;
    }
  }// end pressed

  if (currentMode == LSB || currentMode == USB) // set BFO value in si4735
  {
    if (currentBFO != previousBFO)
    {
      previousBFO = currentBFO;
      si4735.setSSBBfo(currentBFO);
      if (bfoOn) FreqDispl();
    }
  }

  if (currentPRES != previousPRES)
  {
    si4735.getCurrentReceivedSignalQuality();
    if (si4735.isCurrentTuneFM() == false) {
      bandIdx = 0;
      band[bandIdx].currentFreq = ((preset[currentPRES].presetIdx));
      BandSet();
    }
    tft.setCursor(0, 20);
    if (currentPRES > lastPreset) currentPRES = 0;
    if (currentPRES < 0) currentPRES = lastPreset;
    previousPRES = currentPRES;
    DrawDispl();
    //    tft.fillRect(XFreqDispl + 6, YFreqDispl + 22 , 228, 32, TFT_BLACK);
    AGCfreqdisp();
    tft.setTextColor(TextColor, BackColor);
    tft.setTextSize(1);
    //tft.setTextDatum(BC_DATUM);

    //   tft.drawString(String(currentPRES) + ") " + String(((preset[currentPRES].presetIdx) / 100), 1), 60, 51);
    tft.setTextColor(TextColor, BackColor);
    //   tft.drawString(String(preset[currentPRES].PresetName), 175, 51);
    bandIdx = 0;
    si4735.setFrequency((preset[currentPRES].presetIdx));
    band[bandIdx].currentFreq = si4735.getFrequency();
  }

  if (currentVOL != previousVOL)
  {
    currentVOL = currentVOL + (currentVOL - previousVOL);
    tft.setCursor(0, 360); // ????
    if (currentVOL > MaxVOL) currentVOL = MaxVOL;
    if (currentVOL < MinVOL) currentVOL = MinVOL;
    previousVOL = currentVOL;
    si4735.setVolume(currentVOL);
    FreqDispl();
  }

  if (currentAGCgain != previousAGCgain)
  {
    AGCgain = 1;
    //currentAGCgain = currentAGCgain + (currentAGCgain - previousAGCgain);
    tft.setCursor(0, 20);
    if (si4735.isCurrentTuneFM())  MaxAGCgain = MaxAGCgainFM;
    else MaxAGCgain = MaxAGCgainAM;

    if (currentAGCgain > MaxAGCgain) currentAGCgain = MaxAGCgain;
    if (currentAGCgain < MinAGCgain) currentAGCgain = MinAGCgain;

    previousAGCgain = currentAGCgain;
    si4735.setAutomaticGainControl(1, currentAGCgain);
    DrawDispl();
    DrawAGCgainbut();
  }



  //=======================================================================================
}// end loop
//=======================================================================================

//=======================================================================================
void DispoffUser()  {
  //=======================================================================================
  DISplay = false;
  digitalWrite(Display_Led, displayoff);
}

//=======================================================================================
void Dispoff()  {
  //=======================================================================================
  if (((millis() - DisplayOnTime) > MIN_ELAPSED_DISPL_TIME * 300) and (DISplay == true)) {
    DISplay = false;
    digitalWrite(Display_Led, displayoff);
    PRESbut = false;
    cleanDispl();
    DrawDispl();
    DisplayOnTime = millis();
  }
}

//=======================================================================================
void VOLbutoff()  {
  //=======================================================================================
  if (((millis() - VOLbutOnTime) > MIN_ELAPSED_VOLbut_TIME * 10) and (VOLbut == true)) {
    VOLbut = false;
    drawVOL();
    cleanDispl();
    FreqDispl();
  }
  if (VOLbut == false) VOLbutOnTime = millis();
}

//=======================================================================================
void DisplayRDS()  {
  //=======================================================================================
  if (( currentMode == FM) and ((FirstLayer) or (ThirdLayer))) {
    if ( currentFrequency != previousFrequency ) {
      previousFrequency = currentFrequency;
      bufferStatioName[0] = '\0';
      stationName = '\0';
    }
    if ((RDS) and  (NewSNR >= 12)) checkRDS();
  }
}

//=======================================================================================
void showtimeRSSI() {
  //=======================================================================================
  // Show RSSI status only if this condition has changed
  if ((millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * 10) // 150 * 10  = 1.5 sec refresh time RSSI
  {
    si4735.getCurrentReceivedSignalQuality();
    NewRSSI = si4735.getCurrentRSSI();
    NewSNR = si4735.getCurrentSNR();
    if (OldRSSI != NewRSSI)
    {
      OldRSSI = NewRSSI;
      showRSSI();
    }
    elapsedRSSI = millis();
  }
}

//=======================================================================================
void showRSSI() {
  //=======================================================================================
  if ((currentMode == FM) and ((FirstLayer) or (ThirdLayer))) {
    tft.setFreeFont(&FMTYPE);
    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(tft.textWidth("STEREO"));
    tft.setTextColor(TextColor, BackColor);
    tft.drawString((si4735.getCurrentPilot()) ? "STEREO" : "MONO", XFreq + 352 , YFreq + 79);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
    tft.setFreeFont(NULL);
  }
  rssi = NewRSSI;
  if ((FirstLayer) or (ThirdLayer))
  {
    Smeter();
    SNRmeter ();
  }
  if ((FirstLayer) or (ThirdLayer)) {  // dBuV and volume at freq. display
    tft.setFreeFont(&SETUPLINE1);
    //tft.loadFont(SETUPLINE);
    tft.setTextColor(TFT_BLACK, BackColor);
    tft.setTextPadding(tft.textWidth("99XdBuV"));
    tft.drawString(String(NewRSSI) + " dBuV", Xinfo + 254, Yinfo + 17); //RSSI
    tft.setTextPadding(tft.textWidth("99XdB"));
    tft.drawString(String(NewSNR) + " dB" , Xinfo + 314, Yinfo + 17); //SNR
    tft.setTextPadding(0);
    //tft.unloadFont();
    tft.setFreeFont(NULL);
  }
  VOLbutoff();
}

//=======================================================================================
void encoderCheck()  {
  //=======================================================================================
  if (encoderCount != 0)
  {
    if (DISplay == false) { //  Wake-up  Display
      DisplayOnTime = millis();
      digitalWrite(Display_Led, displayon);
      DISplay = true;
    }
    int mainpurp = 1;

    if (bfoOn)  {
      currentBFO = (encoderCount == 1) ? (currentBFO + currentBFOStep) : (currentBFO - currentBFOStep);
      mainpurp = 0;
    }

    if (PRESbut) {     // FM preset
      currentPRES = (encoderCount == 1) ? (currentPRES + currentPRESStep) : (currentPRES - currentPRESStep);
      mainpurp = 0;
    }

    if (VOLbut) {     // Volume control
      currentVOL = (encoderCount == 1) ? (currentVOL + currentVOLStep) : (currentVOL - currentVOLStep);
      mainpurp = 0;
    }

    if (AGCgainbut) {     // AGC gain control
      currentAGCgain = (encoderCount == 1) ? (currentAGCgain + currentAGCgainStep) : (currentAGCgain - currentAGCgainStep);
      mainpurp = 0;
    }


    if (mainpurp == 1)
    {

      if (encoderCount == 1) {
        si4735.frequencyUp();
      } else {
        si4735.frequencyDown();
      }
      FreqDispl();
      band[bandIdx].currentFreq = si4735.getFrequency();
    }

    // Beta test
    if ( !FirstLayer && !AGCgainbut)  // If you move the encoder when you are on second leyer, so you will abort the current action.
    {
      cleanDispl();
      DrawFila();
      ThirdLayer = false;
      BroadBand = false;
      STEPbut = false;
      BandWidth = false;
      Modebut = false;
      HamBand = false;
      FREQbut = false;
    }

    encoderCount = 0;
  }
}

//=======================================================================================
void encoderButtonCheck()  {
  //=======================================================================================
  //Encoder button
  encBut = analogRead(ENCODER_SWITCH);
  if (encBut < 500) {

    delay(400);
    if (DISplay == false) { //  Wake-up  Display
      DisplayOnTime = millis();
      digitalWrite(Display_Led, displayon);
      DISplay = true;
      return;
    }
    if (PRESbut == true) {// FM preset selection
      PRESbut = false;
      DrawDispl();
      return;
    }
    else {
      if (ssbLoaded) {  // SSB is on
        if (bfoOn) {
          bfoOn = false;
        }
        else {
          bfoOn = true;
        }
        //if (currentMode == FM) bfoOn = false;
        cleanDispl();  // TIDO --> Is that necessary?
        drawBFO();
        DrawDispl();
      }
    }
  }
}

//=======================================================================================
void setStepBFO() {
  //=======================================================================================
  // This command should work only for SSB mode
  if (bfoOn && (currentMode == LSB || currentMode == USB))
  {
    currentBFOStep = (currentBFOStep == 25) ? 10 : 25;
  }
  else
  {
    si4735.setFrequencyStep(currentStep);
    band[bandIdx].currentStep = currentStep;
  }
  DrawDispl();
}

//=======================================================================================
void DrawFila()   {// Draw of first layer
  //=======================================================================================
  FirstLayer = true;
  SecondLayer = false;
  ThirdLayer = false;
  ForthLayer = false;
  DrawButFila();
  DrawDispl();
  ShowVolume ();
}

//=======================================================================================
void DrawThla()  {  // Draw of Third layer
  //=======================================================================================
  FirstLayer = false;
  SecondLayer = false;
  ThirdLayer = true;
  ForthLayer = false;
  DrawButThla();
  DrawDispl();
  //  DrawRDSbut();
  //  DrawAGCgainbut();
}

//=======================================================================================
void DrawButFila() { // Buttons first layer
  //=======================================================================================
  clearKeyPathFila();
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(BC_DATUM);
  for (int n = 0 ; n <= lastButton; n++)
  {
    if (n == 12 || n == 13 || n == 14)
    {
      tft.drawRoundRect(bt[bt[n].ButtonNum].XButos + Xbutst, bt[bt[n].ButtonNum].YButos + Ybutst - (n - 25), Xbutsiz - 2, YbutsizExtra - 2, RoundCorner, TFT_BLACK);
      tft.drawRoundRect(bt[bt[n].ButtonNum].XButos + Xbutst, bt[bt[n].ButtonNum].YButos + Ybutst - (n - 25), Xbutsiz - 3, YbutsizExtra - 3, RoundCorner - 1, TFT_BLACK);
      tft.drawString((bt[n].ButtonNam), ( bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((YbutsizExtra) / 2 + 14) - (n - 25)));
    }
    else
    {
      tft.drawString((bt[n].ButtonNam), ( bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
    }
  }
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
  drawBFO();
  drawAGC();
}

//=======================================================================================
void DrawButThla() { // Buttons Third layer
  //=======================================================================================
  clearKeyPathFila();
  //    delay (5000); // xxxxxxxxxxx
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(BC_DATUM);
  for (int n = 0 ; n <= lastButton; n++)
  {
    if (n == 12 || n == 13 || n == 14)
    {
      tft.drawRoundRect(bt[bt[n].ButtonNum1].XButos + Xbutst, bt[bt[n].ButtonNum1].YButos + Ybutst - (n - 25), Xbutsiz - 2, YbutsizExtra - 2, RoundCorner, UNSETColor);
      tft.drawRoundRect(bt[bt[n].ButtonNum1].XButos + Xbutst, bt[bt[n].ButtonNum1].YButos + Ybutst - (n - 25), Xbutsiz - 3, YbutsizExtra - 3, RoundCorner - 1, UNSETColor);
      tft.setTextColor(UNSETColor, BackColor);
      tft.drawString((bt[n].ButtonNam1), ( bt[bt[n].ButtonNum1].XButos + Xbutst + (Xbutsiz / 2 - 2) ), (bt[bt[n].ButtonNum1].YButos + Ybutst  + ((YbutsizExtra) / 2 + 14) - (n - 25)));
    }
    else
    {
      tft.drawString((bt[n].ButtonNam1), ( bt[bt[n].ButtonNum1].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[n].ButtonNum1].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
    }
  }
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
  DrawRDSbut();
}

//=======================================================================================
void drawVOL()   {
  //=======================================================================================
  int VOLbutcol;
  if (VOLbut) {
    VOLbutcol = SETColor;
    tft.drawRoundRect(bt[bt[4].ButtonNum].XButos + Xbutst, bt[bt[4].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.fillRoundRect((bt[bt[4].ButtonNum].XButos + Xbutst + 1) , (bt[bt[4].ButtonNum].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, SETColor);
  } else {
    VOLbutcol = BackColor;
    tft.fillRoundRect((bt[bt[4].ButtonNum].XButos + Xbutst + 1) , (bt[bt[4].ButtonNum].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, BackColor);
    tft.drawRoundRect(bt[bt[4].ButtonNum].XButos + Xbutst, bt[bt[4].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.drawRoundRect(bt[bt[4].ButtonNum].XButos + Xbutst, bt[bt[4].ButtonNum].YButos + Ybutst , Xbutsiz - 3, Ybutsiz - 3, RoundCorner - 1, TFT_BLACK);
  }
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TFT_BLACK, VOLbutcol);
  tft.setTextDatum(BC_DATUM);
  tft.drawString((bt[4].ButtonNam), ( bt[bt[4].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[4].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 12)));
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void DrawAGCgainbut()  {
  //=======================================================================================
  //  if (ThirdLayer)  {
  int AGCgainbutcol;
  if (AGCgainbut) {
    AGCgainbutcol = SETColor;
    tft.drawRoundRect(bt[bt[AGCset].ButtonNum1].XButos + Xbutst, bt[bt[AGCset].ButtonNum1].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.fillRoundRect((bt[bt[AGCset].ButtonNum1].XButos + Xbutst + 1) , (bt[bt[AGCset].ButtonNum1].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, SETColor);
  } else {
    AGCgainbutcol = BackColor;
    tft.fillRoundRect((bt[bt[AGCset].ButtonNum1].XButos + Xbutst + 1) , (bt[bt[AGCset].ButtonNum1].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, BackColor);
    tft.drawRoundRect(bt[bt[AGCset].ButtonNum1].XButos + Xbutst, bt[bt[AGCset].ButtonNum1].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.drawRoundRect(bt[bt[AGCset].ButtonNum1].XButos + Xbutst, bt[bt[AGCset].ButtonNum1].YButos + Ybutst , Xbutsiz - 3, Ybutsiz - 3, RoundCorner - 1, TFT_BLACK);
  }
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TFT_BLACK, AGCgainbutcol);
  tft.setTextDatum(BC_DATUM);
  tft.drawString((bt[AGCset].ButtonNam1), ( bt[bt[AGCset].ButtonNum1].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[AGCset].ButtonNum1].YButos + Ybutst  + (Ybutsiz / 2 + 12)));
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
  //  }
}

//=======================================================================================
void DrawRDSbut()  {
  //=======================================================================================
  int RDSbutcol;
  tft.setFreeFont(&SETUPLINE1);
  tft.setTextColor(TFT_BLACK, BackColor);
  tft.setTextPadding(tft.textWidth("OFF"));
  if (RDS) {
    RDSbutcol = SETColor;
    tft.drawRoundRect(bt[bt[FMRDS].ButtonNum1].XButos + Xbutst, bt[bt[FMRDS].ButtonNum1].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.fillRoundRect((bt[bt[FMRDS].ButtonNum1].XButos + Xbutst + 1) , (bt[bt[FMRDS].ButtonNum1].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, SETColor);
    tft.drawString("ON", Xinfo + 358, Yinfo + 17);
  } else {
    RDSbutcol = BackColor;
    tft.fillRoundRect((bt[bt[FMRDS].ButtonNum1].XButos + Xbutst + 1) , (bt[bt[FMRDS].ButtonNum1].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, BackColor);
    tft.drawRoundRect(bt[bt[FMRDS].ButtonNum1].XButos + Xbutst, bt[bt[FMRDS].ButtonNum1].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.drawRoundRect(bt[bt[FMRDS].ButtonNum1].XButos + Xbutst, bt[bt[FMRDS].ButtonNum1].YButos + Ybutst , Xbutsiz - 3, Ybutsiz - 3, RoundCorner - 1, TFT_BLACK);
    tft.drawString("OFF", Xinfo + 358, Yinfo + 17);
  }
  tft.setFreeFont(NULL);
  tft.setTextPadding(0);
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TFT_BLACK, RDSbutcol);
  tft.setTextDatum(BC_DATUM);
  tft.drawString((bt[FMRDS].ButtonNam1), ( bt[bt[FMRDS].ButtonNum1].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[FMRDS].ButtonNum1].YButos + Ybutst  + (Ybutsiz / 2 + 12)));
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void drawMUTE()  {
  //=======================================================================================
  int MUTEbutcol;
  if (Mutestat) {
    MUTEbutcol = SETColor;
    si4735.setAudioMute(audioMuteOn);
    tft.drawRoundRect(bt[bt[10].ButtonNum].XButos + Xbutst, bt[bt[10].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.fillRoundRect((bt[bt[10].ButtonNum].XButos + Xbutst + 1) , (bt[bt[10].ButtonNum].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, SETColor);
  } else {
    MUTEbutcol = BackColor;
    si4735.setAudioMute(audioMuteOff);
    tft.fillRoundRect((bt[bt[10].ButtonNum].XButos + Xbutst + 1) , (bt[bt[10].ButtonNum].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, BackColor);
    tft.drawRoundRect(bt[bt[10].ButtonNum].XButos + Xbutst, bt[bt[10].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.drawRoundRect(bt[bt[10].ButtonNum].XButos + Xbutst, bt[bt[10].ButtonNum].YButos + Ybutst , Xbutsiz - 3, Ybutsiz - 3, RoundCorner - 1, TFT_BLACK);
  }
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TFT_BLACK, MUTEbutcol);
  tft.setTextDatum(BC_DATUM);
  tft.drawString((bt[10].ButtonNam), ( bt[bt[10].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[10].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 12)));
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void drawAGC()  {
  //=======================================================================================
  int AGCbutcol;
  si4735.getAutomaticGainControl();
  if (si4735.isAgcEnabled()) {
    AGCbutcol = SETColor;
    tft.drawRoundRect(bt[bt[2].ButtonNum].XButos + Xbutst, bt[bt[2].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.fillRoundRect((bt[bt[2].ButtonNum].XButos + Xbutst + 1) , (bt[bt[2].ButtonNum].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, SETColor);
  } else {
    AGCbutcol = BackColor;
    tft.fillRoundRect((bt[bt[2].ButtonNum].XButos + Xbutst + 1) , (bt[bt[2].ButtonNum].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, BackColor);
    tft.drawRoundRect(bt[bt[2].ButtonNum].XButos + Xbutst, bt[bt[2].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.drawRoundRect(bt[bt[2].ButtonNum].XButos + Xbutst, bt[bt[2].ButtonNum].YButos + Ybutst , Xbutsiz - 3, Ybutsiz - 3, RoundCorner - 1, TFT_BLACK);
  }
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TFT_BLACK, AGCbutcol);
  tft.setTextDatum(BC_DATUM);
  tft.drawString((bt[2].ButtonNam), ( bt[bt[2].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[2].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 12)));
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void drawBFO ()  {
  //=======================================================================================
  int BFObutcol;
  if (bfoOn) {
    BFObutcol = SETColor;
    tft.drawRoundRect(bt[bt[8].ButtonNum].XButos + Xbutst, bt[bt[8].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.fillRoundRect((bt[bt[8].ButtonNum].XButos + Xbutst + 1) , (bt[bt[8].ButtonNum].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, SETColor);
  } else {
    BFObutcol = BackColor;
    tft.fillRoundRect((bt[bt[8].ButtonNum].XButos + Xbutst + 1) , (bt[bt[8].ButtonNum].YButos + Ybutst + 1), (Xbutsiz - 4) , Ybutsiz - 4, RoundCorner, BackColor);
    tft.drawRoundRect(bt[bt[8].ButtonNum].XButos + Xbutst, bt[bt[8].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.drawRoundRect(bt[bt[8].ButtonNum].XButos + Xbutst, bt[bt[8].ButtonNum].YButos + Ybutst , Xbutsiz - 3, Ybutsiz - 3, RoundCorner - 1, TFT_BLACK);
  }
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TFT_BLACK, BFObutcol);
  tft.setTextDatum(BC_DATUM);
  tft.drawString((bt[8].ButtonNam), ( bt[bt[8].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[8].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 12)));
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void clearKeyPathFila() {
  //=======================================================================================

  for (int n = 0 ; n <= 11; n++) {
    tft.fillRoundRect(bt[bt[n].ButtonNum].XButos + Xbutst, bt[bt[n].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, BackColor);
    tft.drawRoundRect(bt[bt[n].ButtonNum].XButos + Xbutst, bt[bt[n].ButtonNum].YButos + Ybutst , Xbutsiz - 2, Ybutsiz - 2, RoundCorner, TFT_BLACK);
    tft.drawRoundRect(bt[bt[n].ButtonNum].XButos + Xbutst, bt[bt[n].ButtonNum].YButos + Ybutst , Xbutsiz - 3, Ybutsiz - 3, RoundCorner - 1, TFT_BLACK);
  }
}

//=======================================================================================
void drawKeyFreq() {
  //=======================================================================================
  clearKeyPathFila();
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(BC_DATUM);
  for (int n = 0 ; n <= lastKPath; n++) {
    tft.drawString((Keypathtext[kp[n].KeypNum]), (bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[n].ButtonNum].YButos + Ybutst  + (Ybutsiz / 2 + 12)));
  }
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void HamBandlist() {
  //=======================================================================================
  clearKeyPathFila();
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(BC_DATUM);
  for (int n = 0 ; n <= lastHam; n++) {
    tft.drawString(band[bn[n].BandNum].bandName, (bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2)), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
  }
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void BroadBandlist() {
  //=======================================================================================
  clearKeyPathFila();
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(BC_DATUM);
  if (Secondline)
  {
    for (int n = 0 ; n <= lastBroad - 11; n++)
    {
      tft.drawString(band[bb[n + 11].BbandNum].bandName, (bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
    }
    //    delay(4000);
    Secondline = false;
  }
  else
  {
    for (int n = 0 ; n <= (lastBroad - 7); n++) {
      tft.drawString(band[bb[n].BbandNum].bandName, (bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
    }
    int n = lastBroad - 6;
    tft.drawString("NEXT", (bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
    Secondline = true;
  }
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);

}

//=======================================================================================
void Steplist() {
  //=======================================================================================
  clearKeyPathFila();
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(BC_DATUM);
  for (int n = 0 ; n <= lastStep; n++) {
    tft.drawString(String(sp[n].stepFreq), (bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
  }
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void Modelist() {
  //=======================================================================================
  clearKeyPathFila();
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(BC_DATUM);
  for (int n = 0 ; n <= lastMod - 1; n++) {
    tft.drawString(bandModeDesc[md[n].Modenum], (bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
  }
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void BWList()  {
  //=======================================================================================
  clearKeyPathFila();
  tft.setFreeFont(&ButtonText1);
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(BC_DATUM);
  if ( currentMode == AM) nrbox = 7;
  else nrbox = 6;
  for (int n = 0 ; n < nrbox; n++) {
    if ( currentMode == AM) tft.drawString(String (bandwidthAM[bw[n].BandWidthAM]), (bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
    else tft.drawString(String (bandwidthSSB[bw[n].BandWidthSSB]), (bt[bt[n].ButtonNum].XButos + Xbutst + (Xbutsiz / 2 - 2) ), (bt[bt[n].ButtonNum].YButos + Ybutst  + ((Ybutsiz) / 2 + 12)));
  }
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void subrstatus() {
  //=======================================================================================
  tft.fillScreen(BackColor);
  tft.setFreeFont(&SETUPLINE1);
  while (x == 0) {
    currentFrequency = si4735.getFrequency(); // Atualiza a frequencia atual
    tft.setTextColor(TextColor, BackColor);
    tft.drawString("Mod. : " + String(bandModeDesc[band[bandIdx].prefmod]), 10, 5);
    if ( currentMode != FM)  tft.drawString("Freq.: " + String(currentFrequency, 0) + " kHz", 10, 20);
    else tft.drawString("Freq.: " + String(currentFrequency / 100, 1) + " MHz", 10, 20);
    si4735.getCurrentReceivedSignalQuality();
    tft.drawString("RSSI : " + String(si4735.getCurrentRSSI()) + "dBuV", 10, 35); // si4735.getCurrentSNR()
    tft.drawString("SNR : " + String(si4735.getCurrentSNR()) + "uV", 10, 50);
    if (  currentMode == FM ) {
      tft.drawString(si4735.getCurrentPilot() ? "STEREO" : "MONO", 130, 20);
    }
    si4735.getAutomaticGainControl();
    si4735.getCurrentReceivedSignalQuality();
    tft.drawString("LNA GAIN : " + String(si4735.getAgcGainIndex()) + "/" + String(currentAGCAtt), 10, 65);
    tft.drawString("Volume : " + String(si4735.getVolume()), 10, 80);
    tft.drawString(si4735.isAgcEnabled() ? "AGC ON " : "AGC OFF", 10, 95);
    tft.drawString((bfoOn) ? "BFO ON  " : "BFO OFF ", 10, 110);
    tft.drawString("AVC max GAIN : " + String(si4735.getCurrentAvcAmMaxGain()), 10, 125);
    tft.drawString("Ant. Cap = " + String(si4735.getAntennaTuningCapacitor()) , 10, 140);
    tft.drawString("BANDA : " + String(bandIdx) + "  " + String(band[bandIdx].bandName) , 10, 155);
    tft.drawString("FILTRO SSB : " + String(bandwidthSSB[bwIdxSSB]) + " kHz", 10, 170);
    tft.drawString("FILTRO AM : " + String(bandwidthAM[bwIdxAM]) + " kHz", 10, 185);
    tft.drawString("PASSO : " + String(currentStep), 10, 200);
    int vsupply = analogRead(ENCODER_SWITCH);
    tft.drawString("Power Supply : " + String(((1.66 / 1850)*vsupply) * 2) + " V.", 10, 215);
    tft.drawString("SOFTWARE V3 BETA", 10, 245);
    tft.drawString("BY RICARDO - GERT - THIAGO - ZANDER", 10, 260);
    press1 = tft.getTouch(&x, &y);
    delay (500);
  }
  x = y = 0;
  tft.setFreeFont(NULL);
  //  delay(400);
}

//=======================================================================================
void showRDSStation() {
  //=======================================================================================
  if ((FirstLayer) or (ThirdLayer)) {
    //    //tft.loadFont("NotoSans-CB14");
    tft.setFreeFont(&RDSSTATION);
    tft.setTextColor(TFT_BLACK, BackColor);
    tft.setCursor(XFreq, YFreq + 149);
    tft.print(stationName);
    tft.setFreeFont(NULL);
    //    //tft.unloadFont();
  }
  delay(250);
}

//=======================================================================================
void checkRDS() {
  //=======================================================================================
  si4735.getRdsStatus();
  if (si4735.getRdsReceived()) {
    if (si4735.getRdsSync() && si4735.getRdsSyncFound() ) {
      stationName = si4735.getRdsText0A();
      tft.setTextSize(1);
      tft.setTextColor(TextColor, BackColor);
      tft.setTextDatum(BC_DATUM);
      if ( stationName != NULL)   showRDSStation();
    }
  }
  tft.setTextDatum(TL_DATUM);
}

/*
    Prevents blinking during the frequency display.
    Erases the old digits if it has changed and print the new digit values.
*/
//=======================================================================================
void showContentWithoutBlink(int col, int line, char *oldValue, const char *newValue, uint32_t color, uint8_t space, uint8_t textSize) {
  //=======================================================================================
  int c = col;

  char *pOld;
  char *pNew;

  pOld = oldValue;
  pNew = (char *) newValue;

  // prints just changed digits
  while (*pOld && *pNew)
  {
    if (*pOld != *pNew)
    {
      tft.drawChar(c, line, *pOld, BackColor, BackColor, textSize);
      tft.drawChar(c, line, *pNew, color, color, textSize);
    }

    pOld++;
    pNew++;
    c += space;
  }

  // Is there anything else to erase?
  while (*pOld)
  {
    tft.drawChar(c, line, *pOld, BackColor, BackColor, textSize);
    pOld++;
    c += space;
  }

  // Is there anything else to print?
  while (*pNew)
  {
    tft.drawChar(c, line, *pNew, color, color, textSize);
    pNew++;
    c += space;
  }
  strcpy(oldValue, newValue);
}


char *kHz =  "kHz";
char *mhz =  "MHz";
//=======================================================================================
char * formatFrequency(char *strFreq) {
  //=======================================================================================
  char tmp[15];
  char *unt;

  sprintf(tmp, "%5.3u", si4735.getFrequency());  // TODO sprintf(tmp, "%5.3u", si4735.getFrequency());

  strFreq[0] = tmp[0];
  strFreq[1] = tmp[1];
  if (si4735.isCurrentTuneFM())
  {
    strFreq[2] = tmp[2];
    strFreq[3] = tmp[3];
    strFreq[4] = tmp[4];
    strFreq[5] = '\0';
    unt = mhz;
  } else {
    strFreq[2] = tmp[2];
    strFreq[3] = tmp[3];
    strFreq[4] = tmp[4];
    strFreq[5] = '\0';
    unt = kHz;
  }

  return unt;;
}

//=======================================================================================
inline void showContent(uint16_t col, uint16_t lin, char *oldContent, char *newContent, const GFXfont * font, uint16_t color, uint8_t space, byte size) {
  //=======================================================================================
  tft.setFreeFont(font);
  showContentWithoutBlink(col, lin, oldContent, newContent, color, space, size);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void cleanDispl() {
  //=======================================================================================
  cleanBuffer();
}

//=======================================================================================
void showFrequency() {
  //=======================================================================================
  char tmpFrequency[10];
  char tmpVFO[10];
  char *untFreq;
  long tmpcurrentFreq;
  if (bfoOn) {
    tft.setFreeFont(&SETUPLINE1);
    tft.setTextColor(TFT_BLACK, BackColor);
    tft.setTextPadding(tft.textWidth("-9999 Hz"));
    tft.drawString (String(currentBFO) + " Hz", Xinfo + 38 , Yinfo + 17);// BFO FREQUENCY
    tft.setFreeFont(NULL);
    tft.setTextPadding(0);
    return;
  }
  untFreq = formatFrequency(tmpFrequency);
  tft.setTextColor(TextColor, BackColor);
  showContent(XFreq + 50, YFreq + 138 , bufferFrequency, tmpFrequency, &NotoSans_ExtraCondensedSemiBold56pt7b, TFT_BLACK, 50, 1);
  tft.setFreeFont(&ButtonText1);
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(tft.textWidth("MHz"));
  tft.drawString(untFreq, XFreq + 352 , YFreq + 119); //Khz our Mhz
  if (band[bandIdx].bandType == FM_BAND_TYPE)
  {
    tft.fillCircle(XFreq + 198, YFreq + 133, 3, TFT_BLACK);
  }
  tft.setFreeFont(&BandText);
  tft.setTextColor(TFT_BLACK, BackColor);
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(tft.textWidth("630 m"));
  tft.drawString((char *)band[bandIdx].bandName, XFreq + 352 , YFreq + 59);// Band "Meters" - 60 m, 180 m, ...
  tft.setTextColor(TextColor, BackColor);
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(NULL);
  tft.setTextPadding(0);
}

//=======================================================================================
void FreqDispl() {
  //=======================================================================================
  if (!FirstLayer && !ThirdLayer) // Nothing to do if you are on FirstLayer or ThirdLayer
    return;
  AGCfreqdisp();
  BFOfreqdisp();
  if (VOLbut) {
    tft.setFreeFont(&SETUPLINE1);
    tft.setTextPadding(tft.textWidth("100"));
    tft.drawString(String((int) map(currentVOL, MinVOL, MaxVOL, 0, 100)), Xinfo + 216, Yinfo + 17); // PRINT VOL
    tft.setTextPadding(0);
    tft.setFreeFont(NULL);
  }
  else {
    showFrequency();
  }
  tft.setTextPadding(0);
}

/**
   Checks the stop seeking criterias.
   Returns true if the user press the touch or rotates the encoder.
*/
//=======================================================================================
bool checkStopSeeking() {
  //=======================================================================================
  // Checks the touch and encoder
  return (bool) encoderCount || tft.getTouch(&x, &y);   // returns true if the user rotates the encoder or touches on screen
}

//=======================================================================================
void SeekFreq (uint16_t freq) {
  //=======================================================================================
  if ((FirstLayer) || (ThirdLayer))  {
    currentFrequency = freq;
    showFrequency();
  }
}

//=======================================================================================
void DrawDispl() {
  //=======================================================================================
  FreqDispl();
  tft.setFreeFont(&FMTYPE);
  if (band[bandIdx].bandType != FM_BAND_TYPE) {
    tft.setTextColor(TFT_BLACK, BackColor);
    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(tft.textWidth("STEREO"));
    tft.drawString(bandModeDesc[currentMode], XFreq + 352 , YFreq + 79); //PRINT AM USB LSB SYN
    tft.setFreeFont(NULL);
    if (currentMode == AM) BWtext = bandwidthAM[bwIdxAM];
    else BWtext = bandwidthSSB[bwIdxSSB];
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
    Filterfreqdisp();
    Stepfreqdisp();
    tft.setTextPadding(0);

  }
}

//=======================================================================================
void Filterfreqdisp() {
  //=======================================================================================
  tft.setFreeFont(&SETUPLINE1);
  tft.setTextColor(TFT_BLACK, BackColor);
  tft.setTextPadding(tft.textWidth("6.0XkHz"));
  tft.drawString(BWtext + " kHz", Xinfo + 158, Yinfo + 17); // Filter
  tft.setFreeFont(NULL);
}

//=======================================================================================
void Stepfreqdisp() {
  //=======================================================================================
  tft.setFreeFont(&SETUPLINE1);
  tft.setTextColor(TFT_BLACK, BackColor);
  tft.setTextPadding(tft.textWidth("1000 kHz"));
  tft.drawString(String(band[bandIdx].currentStep) + " kHz", Xinfo + 95, Yinfo + 17); // Step
  tft.setFreeFont(NULL);
}

//=======================================================================================
void AGCfreqdisp() {
  //=======================================================================================
  tft.setFreeFont(&SETUPLINE1);
  si4735.getAutomaticGainControl();
  tft.setTextColor(TFT_BLACK, BackColor);
  tft.setTextPadding(tft.textWidth("OFF"));
  if (si4735.isAgcEnabled()) {
    tft.drawString("ON", Xinfo, Yinfo + 17);
  } else {
    if (AGCgain == 0)   {
      tft.drawString("OFF", Xinfo, Yinfo + 17);
    } else {
      tft.drawString(String(currentAGCgain), Xinfo, Yinfo + 17); // Show AGC Gain number
    }
  }
  tft.setTextPadding(0);
  tft.setFreeFont(NULL);
}

//=======================================================================================
void BFOfreqdisp() {
  //=======================================================================================
  tft.setFreeFont(&SETUPLINE1);
  if (band[bandIdx].bandType != FM_BAND_TYPE) {
    if (bfoOn) {
      tft.setTextColor(TFT_BLACK, BackColor);
      tft.setTextPadding(tft.textWidth("9999 kHz"));
      tft.drawString(String(currentBFOStep) + " Hz", Xinfo + 94, Yinfo + 17); // Step BFO
    }
  }
  tft.setFreeFont(NULL);
}

//=======================================================================================
void drawCross(int x, int y, unsigned int color)
//=======================================================================================
{
  tft.drawLine(x - 5, y, x + 5, y, color);
  tft.drawLine(x, y - 5, x, y + 5, color);
}

//=======================================================================================
void SpectrumRF ()
//=======================================================================================
{
  byte DBFilter = 0;
  tft.fillScreen(BackColor);
  tft.setTextColor(TextColor, BackColor);
  for (int i = 220; i > 40; i = i - 30) // Imprime referencia numerica parte inferior (Lateral)
  {
    tft.setCursor (1, i - 3);
    tft.print(DBFilter);
    DBFilter = DBFilter + 10;
  }
  for (int i = 220; i > 60; i = i - 30) // Desenha quadriculado;
  {
    tft.drawLine(20, i, 460, i, TFT_BLACK); // Linhas parte inferior
  }
  for (int i = 60; i > 10; i = i - 10) // Imprime referencia numerica parte superior (Lateral)
  {
    tft.setCursor (1, i - 3);
    tft.print(DBFilter);
    DBFilter = DBFilter + 10;
  }
  for (int i = 60; i > 10; i = i - 10) // Desenha quadriculado;
  {
    tft.drawLine(20, i, 460, i, TFT_BLACK); // Linhas parte superior
  }
  for (int i = 460; i > 0; i = i - 20) // Desenha quadriculado;
  {
    tft.drawLine(i, 220, i, 20, TFT_BLACK); // Colunas
  }

  int   PointRSSI;
  float FrequenciaParcial;
  int Step = round(((band[bandIdx].maximumFreq - band[bandIdx].minimumFreq) / 440.0));
  if (!Step) Step = 1;
  si4735.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].minimumFreq, round(((band[bandIdx].maximumFreq - band[bandIdx].minimumFreq) / 440.0)));

  tft.setTextColor (TFT_BLACK);
  tft.setCursor(2, 230);
  tft.print ("dBuV");
  tft.setFreeFont(&SETUPLINE1);
  tft.setCursor(160, 235);
  tft.print (" Step: ");
  tft.print(Step);
  tft.print(" kHz | Band:");
  tft.print (band[bandIdx].minimumFreq / 1000.0, 2);
  tft.print (" - ");
  tft.print ((Step * 440 + band[bandIdx].minimumFreq) / 1000.0, 2);
  tft.print (" MHz");

  for (int i = 0; i <= 10; i++) // Imprime numeros frequencia topo da tela
  {
    FrequenciaParcial = (band[bandIdx].minimumFreq + (i * Step * 40)) / 1000.0;
    tft.setCursor(24 + i * 40, 14);
    tft.print (FrequenciaParcial, 2);
    if (i <= 9) tft.drawFastVLine((60 + i * 40), 8, 8, TFT_BLACK);
  }
  tft.setCursor(453, 14);
  tft.print ("MHz");
  tft.setFreeFont(NULL);
  tft.setTextSize(1);
  si4735.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].minimumFreq, round(((band[bandIdx].maximumFreq - band[bandIdx].minimumFreq) / 440.0)));
  for (int i = 21; i <= 459; i++)
  {
    si4735.getCurrentReceivedSignalQuality();
    PointRSSI = si4735.getCurrentRSSI();
    delay(50);
    if (PointRSSI > 50)
    {
      tft.drawLine (i, 219, i, 219 - (PointRSSI + 100), REDMETER);
    }
    else
    {
      tft.drawLine (i, 219, i, 219 - 3 * PointRSSI, REDMETER);
    }
    si4735.frequencyUp();
    delay (50); //wait stable frequency
  }
}

//=======================================================================================
void NightDayBacklight ()
//=======================================================================================
{
  if (NightLight)
  {
    NightLight = false; // Night
    BackColor  = tft.color565(247, 203, 24);
    UNSETColor = tft.color565(184, 147, 16);
    SETColor = tft.color565(184, 147, 16);
    REDMETER = tft.color565(204, 17, 00);
  }
  else
  {
    NightLight = true; // Day
    BackColor  = tft.color565(145, 116, 07);
    UNSETColor = tft.color565(102, 80, 00);
    SETColor = tft.color565(102, 80, 00);
    REDMETER = tft.color565(150, 21, 9);
  }
}

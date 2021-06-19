
# BUTUCA

Interface HMI for SI4735 Touch Display 480x320 (4.0" or 3.5") using controller / driver ST7796S or ILI9488 and ESP32.
This sketch is based on Ricardo Caratti library, Gert Baak sketch using Thiago Lima hardware.

## Screenshots

![](https://github.com/RednazPublic/Butuca-SI4735/blob/main/Images/Butuca%20SI4735.png)

  
## Installation 

Follow this article https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/ to upload NotoSans-CB11.vlw and NotoSans-ECB18.vlw into SPIFFS.

Bodmer TFT_eSPI

Include in Arduino\libraries\TFT_eSPI\User_Setups this file: Setup27_RPi_ST7796_ESP32.H (4.0") or Setup21_ILI9488.h (3.5")

Inclkude in Arduino\libraries\TFT_eSPI those files User_Setup_Select.h and User_Setup.h for respective display you have.
    
## Authors

- [Ricardo Caratti](https://github.com/pu2clr/SI4735)
- [Gert Baak](https://github.com/pe0mgb/SI4735-Radio-ESP32-Touchscreen-Arduino)
- [Thiago Lima](https://www.facebook.com/groups/1121785218031286/) - Hardware 
- Carlos E. Zander - Butuca skecth

BUTUCA is a Brazilian expression to "keep eyes on ..."

  

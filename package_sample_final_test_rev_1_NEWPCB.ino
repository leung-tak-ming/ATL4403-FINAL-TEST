#define LDACpin_A  4   //IO A
#define LDACpin_B  0    //IO B
#define LDACpin_C  2    //IO C
#define LDACpin_D  15  //IO D
#define OFFSET_CH1  6/2000  //3
#define OFFSET_CH2  5/2000  //2.4
#define OFFSET_CH3  9/2000  //4.7
#define OFFSET_CH4  15/2000 //7.4
#define OFFSET_CH5  18/2000 //9.2
#define OFFSET_CH6  20/2000 //10.3
#define OFFSET_CH7  4/2000 //1.9
#define OFFSET_CH8  10/2000 //4.7
#define OFFSET_CH9  16/2000  //7.9
#define OFFSET_CH10  19/2000  // 9.4
#define OFFSET_CH11  14/2000 //7.1
#define OFFSET_CH12  5/2000 //2.7
#define OFFSET_D1  -9
#define OFFSET_D2  -9
#define OFFSET_D3  -9
#define OFFSET_D4  -9
//I2C device found at address 0x3C  ! oled
//I2C device found at address 0x40  ! INA3221
//I2C device found at address 0x41  ! INA3221   //BUS VOLTAGE AND CURRENT AND BUS CURRENT
//I2C device found at address 0x48  ! ADS1115 gain phase
//I2C device found at address 0x49  ! ADS1115 vref VINT
//I2C device found at address 0x4c  ! LM75
//I2C device found at address 0x60   microchip MCP4728 DA
//I2C device found at address 0x75  ! Charger IC
//----------------------------LED------------------------------------------------------------------
#define GOOD_LED 13
#define BAD_LED 27
//--------------------------------------------------------------------------------------------------
//#include "SDL_Arduino_INA3221.h"
#include <Adafruit_MCP4728.h>

String getSsid;
String getPass;
String  MAC;
int32_t rssi;           // store WiFi signal strength here
char ssid[] = "AnApp Hardware";
char pass[] = "Aa27882382";
//int bootCount;
int count_unit = 0;
//----------------;-----temperauter sensor------------------------------------------------------
#include <Temperature_LM75_Derived.h>
Generic_LM75 temperature;
//-------------------------------ifttt -------------------------------------------------------
const char* server = "maker.ifttt.com";
const char* resource = "/trigger/FINAL TEST/with/key/dp2ArfF5ZBhh5OyWs20JRVA6RZhdyxx9V2gMl9VEWzO";
//--------------------------------------------------------------------------------------------

#include "EEPROM.h"
#include <Preferences.h>  // WiFi storage
#define EEPROM_COUNT_ADDR 0
Preferences preferences;
#define CONTROL_1 34    //start pin io34 @10M and fuse
//#define CONTROL_5 12    //done signal
//#define CONTROL_2 5    //Start signal for phase measure @ 1MHz
#define DEVICE_EN 33    //Start signal for phase measure @ 1MHz
int  WFstatus;
//const uint8_t _dataPin  = 21;
//const uint8_t _clkPin   = 22;
const uint8_t _dataPin  = 18;   //oscillator ad9833
const uint8_t _clkPin   = 5;    //oscillator ad9833
const uint8_t _fsyncPin = 19;   //oscillator ad9833
float multiplier = 0.125F;      //AD gain
float ma;                       //gain valable

float VOUT;
//float GAIN;
float VREF;
float VINT;
float VOFFSET;
float busvoltage;
float buscurrent;
float vddvoltage;
float current_mA;
float vddcurrent;
float VOUT_200k;

//float GAIN;
float VREF_200k;
float VINT_200k;
float VOFFSET_200k;
float busvoltage_200k;
float buscurrent_200k;
float vddvoltage_200k;
float current_mA_200k;
float vddcurrent_200k;
const  char* rssiSSID;       // NO MORE hard coded set AP, all SmartConfig
const  char* password;
String PrefSSID, PrefPassword;  // used by preferences storage

#include "esp_system.h"
#include <esp_wifi.h>
#include <string.h>
#include <WiFi.h>
#include <Preferences.h>  // WiFi storage
#define VOUT_LOW  5.7    //set lower limit  -5%
#define VOUT_HI   6.3     //set high limit   +5%
#define VOUT_MIN  0.5    //SET detect device threshold     

#define GAIN_LOW  0.95 * GAIN  //set device ga    -5%
#define GAIN 1
#define GAIN_HI   1.05 * GAIN  //set device gain  +5%
//-----------------------------
#define GAIN_75k_LOW  0.95 * GAIN_75k  //set device ga    -5%
#define GAIN_75k      1.15
#define GAIN_75k_HI   1.05 * GAIN_75k  //set device gain  +5%
//-----------------------------
#define GAIN_200k_LOW  0.95 * GAIN_200k  //set device ga    -5%
#define GAIN_200k      1.15
#define GAIN_200k_HI   1.05 * GAIN_200k  //set device gain  +5%

//-----------------------------
#define VREF_LOW  VREF_AVG * 0.95  //set device gain      -5%
#define VREF_LOW  VREF_AVG * 0.95  //set device gain      -5%
#define VREF_HI   VREF_AVG * 1.05  //set device gain      +5%
#define VREF_AVG  1.24
#define VINT_LOW   VINT_AVG * 0.95  //set device gain       -5%
#define VINT_HI    VINT_AVG * 1.05  //set device gain       +5%
#define VINT_AVG  3.0
#define VOFFSET_LOW  -0.03  //set device gain   +5%
#define VOFFSET_HI   0.03  //set device gain    -5%
#define busvoltage_hi  busvoltage * 1.05      //max
#define busvolage      15
#define busvoltage_low busvoltage * 0.95      //min
#define busoffsetcurrent     0.92*0.87
#define current_mA_hi  current_avg * 1.1      //max  typ 16.8
#define current_avg    26.8
#define current_mA_low current_avg * 0.9    //min
#define current_vdd_hi  current_vdd * 1.10      //max  typ 16.8
#define current_vdd    45.2
#define current_vdd_low current_vdd * 0.90    //min
#include <SDL_Arduino_INA3221.h>
#define freq_hi    500000   //500k  set freq Hi 19mA @gain =2 
#define freq_low   200000    //200k  set freq Low 29mA@Gain =2 
//---------------------------------------------------------------------------
SDL_Arduino_INA3221 ina3221 = SDL_Arduino_INA3221(0x41, 0.1) ;//0x41
SDL_Arduino_INA3221 ina3221a = SDL_Arduino_INA3221(0x40, 0.1) ;
//----------------------------------------------------------------------------
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define NUMFLAKES     10 // Number of snowflakes in the animation example
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

#include <Wire.h>
//--------------------------------------Analog to I2C setting---------------------------
#include <Adafruit_ADS1015.h>           //adc setup library files
// ads.setGain(GAIN_TWOTHIRDS);

Adafruit_ADS1115 ads(0x48);     /* Use this for the 16-bit version   ADD =GND  before and after trim */
Adafruit_ADS1115 ads1(0x49);   /* Use this for the 16-bit version   ADD =VCC   phase and gain */
// send raw 16-bit word
int n;
float   ADC_A01;   //ADC channelA A0-A1   address (0x48)           //
int16_t ADC_A23;   //ADC ChannelA A2-A3   address (0x48)          //
int16_t ADC_B01;   //ADC channelA B0-B1   address (0x49)          //
int16_t ADC_B23;   //ADC channelA B2-B3   address (0x49)          //
float   ADC_A01_200k;   //ADC channelA A0-A1   address (0x48)           //
int16_t ADC_A23_200k;   //ADC ChannelA A2-A3   address (0x48)          //
int16_t ADC_B01_200k;   //ADC channelA B0-B1   address (0x49)          //
int16_t ADC_B23_200k;   //ADC channelA B2-B3   address (0x49)          //
float shuntvoltage = 0;
float current_mA_1 = 0;
float bin = 0;
float loadvoltage = 0;
float gain_200k;
float gain_75k;
//  ad3

//----------------------I2C to DC output for tester setup ----------------------------------------------------
bool setChannelValue(MCP4728_channel_t channel, uint16_t new_value,
                     MCP4728_vref_t new_vref = MCP4728_VREF_INTERNAL,
                     MCP4728_gain_t new_gain = MCP4728_GAIN_1X,
                     MCP4728_pd_mode_t new_pd_mode = MCP4728_PD_MODE_NORMAL,
                     bool udac = 1);
Adafruit_MCP4728 mcp;
//-------------------------------------------------------------------------------------
void spiSend(const uint16_t data)
{
  digitalWrite(_fsyncPin, LOW);

  uint16_t m = 1UL << 15;
  for (uint8_t i = 0; i < 16; i++)
  {
    digitalWrite(_dataPin, data & m ? HIGH : LOW);
    digitalWrite(_clkPin, LOW); //data is valid on falling edge
    digitalWrite(_clkPin, HIGH);
    m >>= 1;
  }

  digitalWrite(_dataPin, LOW); //idle low
  digitalWrite(_fsyncPin, HIGH);
}
void display_init()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    //  for(;;); // Don't proceed, loop forever
  }
  display.display();
  // delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(24, 24);            // Start at top-left corner
  display.println("Final TEST BOARD");
  display.setCursor(24, 34);
  display.println("  REV 1.1 ");
  display.setCursor(24, 44);
  display.println(" 16/09/2020");
  display.display();
  delay(1000); // Pause for 2 seconds
}
void setFreq(double f)
{
  const uint16_t b28  = (1UL << 13);
  const uint16_t freq = (1UL << 14);
  const double   f_clk = 25e6;
  const double   scale = 1UL << 28;
  const uint32_t n_reg = f * scale / f_clk;
  const uint16_t f_low  = n_reg         & 0x3fffUL;
  const uint16_t f_high = (n_reg >> 14) & 0x3fffUL;

  spiSend(b28);
  spiSend(f_low  | freq);
  spiSend(f_high | freq);
  delay(200);
}

void setup(void)
{ Wire.begin();
  pinMode(_clkPin,   OUTPUT);
  pinMode(_fsyncPin, OUTPUT);
  pinMode(_dataPin,  OUTPUT);
  pinMode(DEVICE_EN, OUTPUT);
  pinMode(GOOD_LED, OUTPUT);
  pinMode(BAD_LED, OUTPUT);
  digitalWrite(DEVICE_EN, LOW);
  digitalWrite(_fsyncPin, HIGH);
  digitalWrite(_clkPin,   LOW);
  delay(10);
  EEPROM.begin(128);
  setFreq(75000.0);
 
  display_init();
  display.println("Connecting wifi...");
  display.display();
  wifiInit();       // get WiFi connected
  pinMode(CONTROL_1, INPUT_PULLUP);
  //  MAC = getMacAddress();
  //---------------set oscillator
  Serial.begin(115200);
  // IP_info();
  //  digitalWrite(_fsyncPin, HIGH);
  //  digitalWrite(_clkPin,   LOW);
  digitalWrite(_fsyncPin, HIGH);
  digitalWrite(_clkPin,   LOW);
  delay(10);
  setFreq(100000.0);
  //----set adc
  ads.setGain(GAIN_ONE);
  ads1.setGain(GAIN_ONE);
  ads.begin();  //init adc=1
  ads1.begin(); //init adc 2
  ina3221.begin();
  //    if (! ina3221.begin()) {
  //      Serial.println("Failed to find INA3221 chip");
  //      while (1) {
  //        (10);
  //      }
  //    }
  ina3221a.begin();
  //    if (! ina3221a.begin()) {
  //      Serial.println("Failed to find INA3221b chip");
  //      while (1) {
  //        (10);
  //      }
  //    }
  do {
    // warning();
    IP_info();
    busvoltage = 0;
    busvoltage = ina3221.getBusVoltage_V(1);
    current_mA_1 = ina3221.getCurrent_mA(1);
    if (current_mA_1 > 100) {
      display.print("OVER CURRENT");
      digitalWrite(DEVICE_EN, LOW);
    }
    display.print("VDD =");
    display.println(busvoltage, 4);
    display.display();
    delay(100);

  } while ((busvoltage > busvoltage_hi) || (busvoltage < busvoltage_low));
  if (!mcp.begin(0x60)) {
    Serial.println("Failed to find MCP4728 chip");
    while (1) {
      delay(10);
    }
     Serial.println("init MCP4728 chip  i2C to DC ouput ");  //BG 1-6 and CONTROL 1-6
   
  }
   setio();
//---------------write fornat -------------------------
//  write_adc_B(c, c, c, c);
//  write_adc_A(c/2, c/2, c/2, c/2);
//  write_adc_C(c/4, c/4, c/4, c/4);
//-----------------------------------------------------   
}

void loop()
{ Serial.print("Temperature = ");
  Serial.print(temperature.readTemperatureC());
  Serial.println(" C");

  unsigned long start, finished, elapsed;
  start = millis();
  do {
    busvoltage = ina3221.getBusVoltage_V(1);
  }
  while (busvoltage < 14);

  //  measure();

  digitalWrite(DEVICE_EN, HIGH);
  // unsigned long time1;

  //  ADC_B01 = ads1.readADC_Differential_0_1();
  //  VOUT = -ADC_B01 * multiplier / 1000 * 4.93;
  //  finished = millis();
  //  Serial.print("T = 1  "); Serial.print(VOUT); Serial.print(" "); Serial.println(finished - start);
  //  ADC_B01 = ads1.readADC_Differential_0_1();
  //  VOUT = -ADC_B01 * multiplier / 1000 * 4.93;  finished = millis();
  //  Serial.print("T = 2  "); Serial.print(VOUT); Serial.print(" "); Serial.println(finished - start);
  //  ADC_B01 = ads1.readADC_Differential_0_1();
  //  VOUT = -ADC_B01 * multiplier / 1000 * 4.93;  finished = millis();
  //  Serial.print("T = 3  "); Serial.print(VOUT); Serial.print(" "); Serial.println(finished - start);
  //  ADC_B01 = ads1.readADC_Differential_0_1();
  //  VOUT = -ADC_B01 * multiplier / 1000 * 4.93;  finished = millis();
  //  Serial.print("T = 4 "); Serial.print(VOUT); Serial.print(" "); Serial.println(finished - start);

  measure();

  int val = digitalRead(CONTROL_1);   // esp32 board pin 39
  if (val == 0) {
    //  digitalWrite(AD9851_RESET_PIN, LOW);
    makeIFTTTRequest(); display.display();
    // digitalWrite(DEVICE_EN, LOW);
    //  digitalWrite(AD9851_RESET_PIN, HIGH); //disable dds
  }
  //  CHECK_GAIN();
}
//void CHECK_GAIN(int gain){
//   setFreq(200000.0);
//
//   ADC_A01 = 0;
//  for (int a = 0; a < count; a++)
//  {
//    ADC_A01 = ADC_A01 + abs(-ads.readADC_Differential_0_1());
//  }
//  ADC_A01 = (ADC_A01 / (count));
//}
//return (ADC_01);
//}

void measure() {

  setFreq(freq_hi);
  busvoltage_200k = ina3221.getBusVoltage_V(1);
  buscurrent_200k = ina3221.getCurrent_mA(1) * busoffsetcurrent;
  vddvoltage_200k = ina3221.getBusVoltage_V(2);
  vddcurrent_200k = ina3221.getCurrent_mA(2);
  ADC_A01_200k = averagegain(10);
  float mag = (ADC_A01_200k * multiplier / 1000 );
  //  Serial.print ("MAG  NEW = ");
  gain_200k = ( pow(10, ((mag - 0.922) / 1.57)) ) * GAIN; //@
  ADC_A23_200k = -ads.readADC_Differential_2_3(); delay(10);   //GPIO10 PHASE
  ADC_B01_200k = ads1.readADC_Differential_0_1(); delay(10);   //GPIO09 VINT // VOUT
  ADC_B23_200k = -ads1.readADC_Differential_2_3(); delay(10);   //GPIO10 VREF
  VINT_200k = ADC_B01_200k * multiplier / 1000 * 2;
  VREF_200k = -ADC_B23_200k * multiplier / 1000;
  //=======================================================================================
  setFreq(freq_low);
  ADC_A23 = -ads.readADC_Differential_2_3(); delay(10);   //GPIO10 PHASE
  ADC_B01 = ads1.readADC_Differential_0_1(); delay(10);   //GPIO09 VINT // VOUT
  ADC_B23 = ads1.readADC_Differential_2_3(); delay(10);   //GPIO10 VREF
  shuntvoltage = ina3221.getShuntVoltage_mV(1);
  busvoltage = ina3221.getBusVoltage_V(1); delay(10);
  buscurrent = ina3221.getCurrent_mA(1) * busoffsetcurrent; delay(10);
  vddvoltage = ina3221.getBusVoltage_V(2); delay(10);
  vddcurrent = ina3221.getCurrent_mA(2); delay(10);
  ADC_A01 = averagegain(10);
  // ADC_A01 = averagegain(10);
  mag = (ADC_A01 * multiplier / 1000 );
  //  Serial.print ("MAG  NEW = ");
  gain_75k = (pow(10, ((mag - 0.922) / 1.57)) ) * GAIN; //@
  VINT = ADC_B01 * multiplier / 1000 * 2;
  VREF = ADC_B23 * multiplier / 1000;
  Serial.print("Busvoltage_75k\t200k\t"); Serial.print(busvoltage, 3);  Serial.print("\t"); Serial.println(busvoltage_200k, 3);
  Serial.print("Buscurrent_75k\t200k\t"); Serial.print(buscurrent, 3);  Serial.print("\t"); Serial.println(buscurrent_200k, 3);
  Serial.print("Vddvoltage_75k\t200k\t"); Serial.print(vddvoltage, 3); Serial.print("\t"); Serial.println(vddvoltage_200k, 3);
  Serial.print("Vddcurrent_75k\t200k\t"); Serial.print(vddcurrent, 3); Serial.print("\t"); Serial.println(vddcurrent_200k, 3);
  Serial.print("Gain 75k\t200k\t"); Serial.print(gain_75k, 3); Serial.print("\t"); Serial.println(gain_200k, 3);
  Serial.print("RAW Gain 75k\t200k\t"); Serial.print(ADC_A01 * multiplier / 1000); Serial.print("\t"); Serial.println(ADC_A01_200k * multiplier / 1000);
  Serial.print("RAW Phase 75k\t200k\t"); Serial.print(ADC_A23 * multiplier / 1000); Serial.print("\t"); Serial.println(ADC_A23_200k * multiplier / 1000);
  Serial.print("V_INT 75k\t200k\t"); Serial.print(VINT, 4); Serial.print("\t"); Serial.println(VINT_200k, 4);
  Serial.print("VREF 75k\t200k\t"); Serial.print(VREF, 4); Serial.print("\t"); Serial.println(VREF_200k, 4);
  CHECK_GAIN();
  CHECK_VREF();
  CHECK_VINT();
  // CHECK_OFFSET();
  CHECK_VOUT();
  CHECK_IVDDH();
  CHECK_IVDD();
  DISPLAY_SUMMARY();
} //--------End Summary----------------//
float averagegain(int count)
{
  // int count = 10;
  ADC_A01 = 0;
  for (int a = 0; a < count; a++)
  {
    ADC_A01 = ADC_A01 + abs(-ads.readADC_Differential_0_1()); delay(5);
  }
  ADC_A01 = (ADC_A01 / (count));  //AVERAGE GAIN

  return ADC_A01;

}
void DISPLAY_SUMMARY() {
  display.setTextSize(2);
  if (n == 0) {

    display.print("GOOD :) "); bin = 0; display.println(count_unit + 1);
    digitalWrite(GOOD_LED, HIGH);
    digitalWrite(BAD_LED, LOW);
  } else {
    bin = n;

    if (VOUT < VOUT_MIN) {

      display.println ("NOT DETECT");

    } else {
      display.print("B");
      display.print(n);
      display.println(" BAD  :( ")  ;
      digitalWrite(GOOD_LED, LOW);
      digitalWrite(BAD_LED, HIGH);
    }
  }


  // -------End Summary--------//

  display.setTextSize(1);
  display.display();
  digitalWrite(DEVICE_EN, LOW);
}

void warning()
{ display.clearDisplay();
  display.setCursor(24, 12);            // Start at top-left corner
  display.println("Warning ");
  display.setCursor(0, 24);
  display.println("Plase SET VDD = 15V");
  display.setCursor(24, 36                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          );
  display.println("            ");
  display.display();
  delay(1000); // Pause for 2 seconds

}
void IP_info()
{
  getSsid = WiFi.SSID();
  getPass = WiFi.psk();
  rssi = getRSSI(  rssiSSID );
  WFstatus = getWifiStatus( WFstatus );
  MAC = getMacAddress();

  Serial.printf( "\n\n\tSSID\t%s, ", getSsid.c_str() );
  Serial.print( rssi);  Serial.printf(" dBm\n" );  // printf??
  Serial.printf( "\tPass:\t %s\n", getPass.c_str() );
  Serial.print( "\n\n\tIP address:\t" );  Serial.print(WiFi.localIP() );
  Serial.print( " / " );
  Serial.println( WiFi.subnetMask() );
  Serial.print( "\tGateway IP:\t" );  Serial.println( WiFi.gatewayIP() );
  Serial.print( "\t1st DNS:\t" );     Serial.println( WiFi.dnsIP() );
  Serial.printf( "\tMAC:\t\t%s\n", MAC.c_str() );
  display.setCursor(0, 0);
  display.clearDisplay();
  display.printf( "SSID: %s,", getSsid.c_str() );
  // display.println( rssi);  display.printf(" dBm\n" );  // printf??
  display.printf( "\nPass: %s\n", getPass.c_str() );
  display.println( "IP address:\ " );  display.println(WiFi.localIP() );
                   // display.print( " / " );
                   // display.println( WiFi.subnetMask() );
                   // display.print( "\tGateway IP:\t" );  display.println( WiFi.gatewayIP() );
                   // display.print( "\t1st DNS:\t" );     display.println( WiFi.dnsIP() );
                   display.printf( "\tMAC:\n\t\t%s\n", MAC.c_str() );
                   display.display();
                 }

                   int getWifiStatus( int WiFiStatus  )
                   {
                   WiFiStatus = WiFi.status();
                   Serial.printf("\tStatus %d",  WiFiStatus );
                   switch ( WiFiStatus )
                   {
                   case WL_IDLE_STATUS :                         // WL_IDLE_STATUS     = 0,
                   Serial.printf(", WiFi IDLE \n");
                   break;
                   case WL_NO_SSID_AVAIL:                        // WL_NO_SSID_AVAIL   = 1,
                   Serial.printf(", NO SSID AVAIL \n");
                   break;
                   case WL_SCAN_COMPLETED:                       // WL_SCAN_COMPLETED  = 2,
                   Serial.printf(", WiFi SCAN_COMPLETED \n");
                   break;
                   case WL_CONNECTED:                            // WL_CONNECTED       = 3,
                   Serial.printf(", WiFi CONNECTED \n");
                   break;
                   case WL_CONNECT_FAILED:                       // WL_CONNECT_FAILED  = 4,
                   Serial.printf(", WiFi WL_CONNECT FAILED\n");
                   break;
                   case WL_CONNECTION_LOST:                      // WL_CONNECTION_LOST = 5,
                   Serial.printf(", WiFi CONNECTION LOST\n");
                   WiFi.persistent(false);                 // don't write FLASH
                   break;
                   case WL_DISCONNECTED:                         // WL_DISCONNECTED    = 6
                   Serial.printf(", WiFi DISCONNECTED ==\n");
                   WiFi.persistent(false);                 // don't write FLASH when reconnecting
                   break;
                 }
                   return WiFiStatus;
                 }
                   // END getWifiStatus()
                   // Get the station interface MAC address.
                   // @return String MAC
                   String getMacAddress(void)
                   {
                   WiFi.mode(WIFI_AP_STA);                    // required to read NVR before WiFi.begin()
                   uint8_t baseMac[6];
                   esp_read_mac( baseMac, ESP_MAC_WIFI_STA ); // Get MAC address for WiFi station
                   char macStr[18] = { 0 };
                   sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
                   return String(macStr);
                 }
                   // END getMacAddress()
                   // Return RSSI or 0 if target SSID not found
                   // const char* SSID = "YOUR_SSID";  // declare in GLOBAL space
                   // call:  int32_t rssi = getRSSI( SSID );
                   int32_t getRSSI( const char* target_ssid )
                   {
                   byte available_networks = WiFi.scanNetworks();

                   for (int network = 0; network < available_networks; network++)
                   {
                   if ( strcmp(  WiFi.SSID( network).c_str(), target_ssid ) == 0)
                   {
                   return WiFi.RSSI( network );
                 }
                 }
                   return 0;
                 } //  END  getRSSI()
                   // Requires; #include <esp_wifi.h>
                   // Returns String NONE, ssid or pass arcording to request
                   // ie String var = getSsidPass( "pass" );
                   String getSsidPass( String s )
                   {
                   String val = "NONE";  // return "NONE" if wrong key sent
                   s.toUpperCase();
                   if ( s.compareTo("SSID") == 0 )
                   {
                   wifi_config_t conf;
                   esp_wifi_get_config( WIFI_IF_STA, &conf );
                   val = String( reinterpret_cast<const char*>(conf.sta.ssid) );
                 }
                   if ( s.compareTo("PASS") == 0 )
                   {
                   wifi_config_t conf;
                   esp_wifi_get_config( WIFI_IF_STA, &conf );
                   val = String( reinterpret_cast<const char*>(conf.sta.password) );
                 }
                   return val;
                 }

                   void wifiInit()  //
                   {
                   WiFi.mode(WIFI_AP_STA);   // required to read NVR before WiFi.begin()
                   // load credentials from NVR, a little RTOS code here
                   wifi_config_t conf;
                   esp_wifi_get_config(WIFI_IF_STA, &conf);  // load wifi settings to struct comf
                   rssiSSID = reinterpret_cast<const char*>(conf.sta.ssid);
                   password = reinterpret_cast<const char*>(conf.sta.password);

                   //  Serial.printf( "%s\n", rssiSSID );
                   //  Serial.printf( "%s\n", password );

                   // Open Preferences with "wifi" namespace. Namespace is limited to 15 chars
                   preferences.begin("wifi", false);
                   PrefSSID          =  preferences.getString("ssid", "none");      //NVS key ssid
                   PrefPassword  =  preferences.getString("password", "none");  //NVS key password
                   preferences.end();
                   int val = digitalRead(CONTROL_1);   // esp32 board pin 39
                   if (val == 1) {
                   Serial.print ("RUN SMART CONFIG START");
                   initSmartConfig();
                   delay( 3000);
                   ESP.restart();
                 }
                   // keep from rewriting flash if not needed
                   if ( !checkPrefsStore() )      // see is NV and Prefs are the same
                   { // not the same, setup with SmartConfig
                   if ( PrefSSID == "none" ) // New...setup wifi
                   {
                   initSmartConfig();
                   delay( 3000);
                   ESP.restart();   // reboot with wifi configured
                 }
                 }
                   // I flash LEDs while connecting here
                   WiFi.begin( PrefSSID.c_str() , PrefPassword.c_str() );
                   int WLcount = 0;
                   while (WiFi.status() != WL_CONNECTED && WLcount < 200 ) // can take > 100 loops depending on router settings
                   {
                   delay( 100 );
                   Serial.printf(".");
                   ++WLcount;
                 }
                   delay( 3000 );
                   //  stop the led flasher here
                 }  // END wifiInit()
                   // match WiFi IDs in NVS to Pref store,  assumes WiFi.mode(WIFI_AP_STA);  was executed
                   bool checkPrefsStore()
                   {
                   bool val = false;
                   String NVssid, NVpass, prefssid, prefpass;

                   NVssid = getSsidPass( "ssid" );
                   NVpass = getSsidPass( "pass" );

                   // Open Preferences with my-app namespace. Namespace name is limited to 15 chars
                   preferences.begin("wifi", false);
                   prefssid  =  preferences.getString("ssid", "none");     //NVS key ssid
                   prefpass  =  preferences.getString("password", "none"); //NVS key password
                   preferences.end();
                   if ( NVssid.equals(prefssid) && NVpass.equals(prefpass) )
                   {
                   val = true;
                 }
                   return val;
                 }

                   // optionally call this function any way you want in your own code
                   // to remap WiFi to another AP using SmartConfig mode.   Button, condition etc..
                   void initSmartConfig()
                   {
                   // start LED flasher
                   int loopCounter = 0;
                   WiFi.mode( WIFI_AP_STA );       //Init WiFi, start SmartConfig
                   Serial.printf( "Entering SmartConfig\n" );
                   WiFi.beginSmartConfig();
                   while (!WiFi.smartConfigDone())
                   {
                   // flash led to indicate not configured
                   Serial.printf( "." );
                   if ( loopCounter >= 40 ) // keep from scrolling sideways forever
                   {
                   loopCounter = 0;
                   Serial.printf( "\n" );
                 }
                   delay(600);
                   ++loopCounter;
                 }
                   loopCounter = 0;

                   // stopped flasher here

                   Serial.printf("\nSmartConfig received.\n Waiting for WiFi\n\n");
                   delay(2000 );

                   while ( WiFi.status() != WL_CONNECTED )     // check till connected
                   {
                   delay(101);
                 }
                   IP_info();  // connected lets see IP info

                   preferences.begin("wifi", false);      // put it in storage
                   preferences.putString( "ssid"         , getSsid);
                   preferences.putString( "password", getPass);
                   preferences.end();

                   delay(300);
                 }
                   // Make an HTTP request to the IFTTT web service
                   void makeIFTTTRequest() {
                   display.clearDisplay();
                   display.setCursor(24, 24);
                   display.setTextSize(1);
                   display.println("Update to wifi");
                   display.display();
                   int bootCount;
                   count_unit++;
                   EEPROM.get(EEPROM_COUNT_ADDR, bootCount);
                   bootCount++;
                   EEPROM.put(EEPROM_COUNT_ADDR, bootCount);
                   EEPROM.commit();
                   if (bootCount % 1 == 0)
                   {
                   Serial.print("Connecting to ");
                   Serial.print(server);
                   WiFiClient client;
                   int retries = 2;
                   while (!!!client.connect(server, 80) && (retries-- > 0)) {
                   Serial.print(".");
                 }
                   Serial.println();
                   if (!!!client.connected()) {
                   Serial.println("Failed to connect...");
                 }
                   Serial.print("Request resource: ");
                   Serial.println(resource);
                   String stringOne = String('|') + String('|') + String('|');
                   //Serial.println(stringOne);
                   //  int dev = 1;
                   //float VOUT;GAIN;VREF;VINT;VOFFSET;;current_mA;
                   //"A" | VOUT| GAIN | VREF | VINT | VOFFSET | BUSVOLTAGE  | BUSCURR | VDD VOLTAGE | VDDCURRENT|ADCA01|ADCA23|ADCB01|ADCB23| BIN |count|
                   //VOUT_200k| GAIN_200k | VREF_200k | VINT_200k| BUSVOLTAGE_200k  | BUSCURR_200k | VDD VOLTAGE_200k | VDDCURRENT_200k|ADCA01_200k|ADCA23_200k|ADCB01_200k|ADCB23_200k|
                   String stringtwo  = String( 'A' + stringOne + VOUT + stringOne + GAIN + stringOne + VREF + stringOne + VINT  + stringOne + VOFFSET * 1000 + stringOne + busvoltage + stringOne + current_mA + stringOne + vddvoltage + stringOne + vddcurrent + stringOne
                   +  ADC_A01 + stringOne + ADC_A23 + stringOne + ADC_B01 +  stringOne + ADC_B23 +  stringOne + bin + stringOne + count_unit + stringOne + VOUT_200k +

                   stringOne + gain_200k + stringOne + VREF_200k + stringOne + VINT_200k + stringOne + busvoltage_200k + stringOne + buscurrent_200k + stringOne + vddvoltage_200k + stringOne + vddcurrent_200k
                   + stringOne + ADC_A01_200k + stringOne + ADC_A23_200k + stringOne + ADC_B01_200k +  stringOne + ADC_B23_200k
                   );
                   String jsonObject = String("{\"value1\":\"") + stringtwo + "\",\"value2\":\"" +
                   "\",\"value3\":\"" +
                   '^' +    stringOne + bootCount + "\"}";
                   //  String jsonObject = String("{\"value1\":\"") + h + "\",\"value2\":\"" + t + "\",\"value3\":\"" + f + "\"}";
                   client.println(String("POST ") + resource + " HTTP/1.1");
                   client.println(String("Host: ") + server);
                   client.println("Connection: close\r\nContent-Type: application/json");
                   client.print("Content-Length: ");
                   client.println(jsonObject.length());
                   client.println();
                   client.println(jsonObject);
                   // int timeout = 5 * 10; // 5 seconds
                   int timeout = 20 ; // 5 seconds
  while (!!!client.available() && (timeout-- > 0)) {
    delay(100);
    }
  if (!!!client.available()) {
  Serial.println("No response...");
    display.println("NOT SUCESS");
    display.println("Please upload again");
    EEPROM.get(EEPROM_COUNT_ADDR, bootCount);
    bootCount--;
    EEPROM.put(EEPROM_COUNT_ADDR, bootCount);
    delay(1000);
    display.display();

  }
  while (client.available()) {
  Serial.write(client.read());
  }
  Serial.println("\nclosing connection");
  client.stop();
  display.println("Finished");
  display.display();
                 }
  else
  {}

}
void CHECK_OFFSET() {
  display.print("Offset:");
  VOFFSET = (ADC_B23 + ADC_B01) * multiplier / 1000;
  display.print(VOFFSET, 4);
  if ((VOFFSET > VOFFSET_LOW) && (VOFFSET < VOFFSET_HI)) {
    display.println ("  PASS");
  }
  else
  {
    display.println ("  Fault"); n++;
  }
  //-------------------end offset---------------------//
}
void CHECK_VINT()
{


  //----------------------VINT----------------------//
  display.print("VINT:   ");
  VINT = ADC_B01 * multiplier / 1000;
  display.print(VINT, 4);
  if ((VINT > VINT_LOW) && (VINT < VINT_HI)) {
    display.println ("  PASS");
  }
  else
  {
    display.println ("  Fault"); n++;
  }
  //-------------------VINT end---------------------//
}
void CHECK_IVDD()
{


  //----------------------IVDD----------------------//
  display.print("IVDD:  ");
  vddcurrent = ina3221.getCurrent_mA(2);   //VDD current
  display.print(vddcurrent, 2);
  if ((vddcurrent > current_vdd_low) && (vddcurrent < current_vdd_hi)) {
    display.println ("  PASS");
  }
  else
  {
    display.println ("  Fault"); n++;
  }
  //-------------------IVDD end---------------------//
}

void CHECK_IVDDH()
{
  // busvoltage = ina3221.getBusVoltage_V(1);

  //-------------------end vout---------------------//
  current_mA_1 = 0;
  float current = 0;
  int count = 5;
  for (int a = 0; a < count; a++)
  {
    current_mA_1 = ina3221.getCurrent_mA(1);   //VDDH current
    current = current + current_mA_1;
  }
  //  Serial.println(current / count);
  current_mA = (current / count) * busoffsetcurrent;
  //  display.print("VDD:");
  //  display.print(busvoltage);
  display.print("I_VDDH:");
  display.print(current_mA, 3);

  if ((current_mA > current_mA_low) && (current_mA < current_mA_hi)) {
    display.println ("  PASS");
  }
  else
  {
    display.println ("  Fault"); n++;
  }
}
void CHECK_VREF()
{
  display.print("VREF:  ");

  VREF = ADC_B23 * multiplier / 1000;
  display.print(VREF, 4);
  if ((VREF > VREF_LOW) && (VREF < VREF_HI)) {
    display.println ("  PASS");
  }
  else
  {
    display.println ("  Fault"); n++;
  }
}
void CHECK_VOUT()
{
  display.print("VOUT:  ");
  VOUT =  vddvoltage = ina3221.getBusVoltage_V(2);
  display.print(VOUT, 4);
  if ((VOUT > VOUT_LOW) && (VOUT < VOUT_HI)) {
    display.println ("  PASS");
  }
  else
  {
    display.println ("  Fault"); n++;
  }
}
void CHECK_GAIN() {
  //  float mag = (ADC_A01 * multiplier / 1000 );
  //  //  Serial.print ("MAG  NEW = ");
  //  ma = (1 / pow(10, ((mag - 0.922) / 0.57)) ) * GAIN; //@
  //  //  Serial.println(1 / ma, 4);
  //  //delay (500);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  n = 0;
  display.print("Gain:  ");
  //int GAIN = ma;
  display.print(gain_75k, 4);
  if ((gain_75k > GAIN_75k_LOW) && (gain_75k < GAIN_75k_HI)) {
    display.println ("  PASS");
  }
  else
  {
    display.println ("  Fault"); n++;
  }
}
void setio() {

  pinMode(LDACpin_A, OUTPUT);
  digitalWrite(LDACpin_A, HIGH);
  pinMode(LDACpin_B, OUTPUT);
  digitalWrite(LDACpin_B, HIGH);
  pinMode(LDACpin_C, OUTPUT);
  digitalWrite(LDACpin_C, HIGH);
  pinMode(LDACpin_D, OUTPUT);
  digitalWrite(LDACpin_D, HIGH);
  delay(100);
}
void write_adc_A(int CHA, int CHB, int CHC, int CHD)
{

  mcp.setChannelValue(MCP4728_CHANNEL_A, CHA-CHA*OFFSET_CH1);
  mcp.setChannelValue(MCP4728_CHANNEL_B, CHB-CHB*OFFSET_CH2);
  mcp.setChannelValue(MCP4728_CHANNEL_C, CHC-CHC*OFFSET_CH3);
  mcp.setChannelValue(MCP4728_CHANNEL_D, CHD-CHD*OFFSET_CH4);
  digitalWrite(LDACpin_A, LOW);
  digitalWrite(LDACpin_A, HIGH);
}
void write_adc_B(int CHA, int CHB, int CHC, int CHD)
{

  mcp.setChannelValue(MCP4728_CHANNEL_A, CHA-CHA*OFFSET_CH5);
  mcp.setChannelValue(MCP4728_CHANNEL_B, CHB-CHB*OFFSET_CH6);
  mcp.setChannelValue(MCP4728_CHANNEL_C, CHC-CHC*OFFSET_CH7);
  mcp.setChannelValue(MCP4728_CHANNEL_D, CHD-CHD*OFFSET_CH8);
  digitalWrite(LDACpin_B, LOW);
  digitalWrite(LDACpin_B, HIGH);
}
void write_adc_C(int CHA, int CHB, int CHC, int CHD)
{

  mcp.setChannelValue(MCP4728_CHANNEL_A, CHA-CHA*OFFSET_CH9);
  mcp.setChannelValue(MCP4728_CHANNEL_B, CHB-CHB*OFFSET_CH10);
  mcp.setChannelValue(MCP4728_CHANNEL_C, CHC-CHC*OFFSET_CH11);
  mcp.setChannelValue(MCP4728_CHANNEL_D, CHD-CHD*OFFSET_CH12);
  digitalWrite(LDACpin_C, LOW);
  digitalWrite(LDACpin_C, HIGH);
}
void write_adc_D(int CHA, int CHB, int CHC, int CHD)
{

  mcp.setChannelValue(MCP4728_CHANNEL_A, CHA);
  mcp.setChannelValue(MCP4728_CHANNEL_B, CHB);
  mcp.setChannelValue(MCP4728_CHANNEL_C, CHC);
  mcp.setChannelValue(MCP4728_CHANNEL_D, CHD);
  digitalWrite(LDACpin_D, LOW);
  digitalWrite(LDACpin_D, HIGH);
}

#include <gtt.h>
#include <gtt_device.h>
#include <gtt_enum.h>
#include <gtt_events.h>
#include <gtt_ext_types.h>
#include <gtt_packet_builder.h>
#include <gtt_parser.h>
#include <gtt_protocol.h>
#include <gtt_text.h>

#include <Wire.h>
#include <OneWire.h>

#include "GTT_Arduino_Thermometer_Demo_Stage_2.c"
#include "GTT_Arduino_Thermometer_Demo_Stage_2.h"
#include <stdlib.h>
#define I2C_Address 0x28  //Define default 8bit I2C address of 0x50 >> 1 for 7bit Arduino

OneWire ds18s20(2); //sensor on pin 2
gtt_device gtt; //Declare the GTT device

byte addr[8]; //Buffer to store One wire Address
bool probeConnected; //Bool to determine indicate if the DS18S20 is connected
bool tempSensorReady = 1;
uint8_t maxTemp = 90; //Default Max limit
uint8_t minTemp = 60; //Default Low limit
unsigned long tempTimer;

// Buffer for incoming data
uint8_t rx_buffer[64] = {0};

// Buffer for outgoing data
uint8_t tx_buffer[64] = {0};  

void setup() {
  //Setup I2C bus  
  gtt.Write = i2cWrite; //Set the write function
  gtt.Read = i2cRead; //Set the read function
  gtt.rx_buffer = rx_buffer; //Declare a buffer for input data
  gtt.rx_buffer_size = sizeof(rx_buffer); //Declare the size of the input buffer
  gtt.tx_buffer = tx_buffer; //Declare a buffer for output data
  gtt.tx_buffer_size = sizeof(tx_buffer); //Declare the size of the output buffer
  Wire.begin(); //Begin I2C communication
  Serial.begin(9600);
  delay(100);  
  resetDisplay();
  //Wait for display to reset  
  delay(3000);            
  gtt_set_screen1_image_toggle_2_state(&gtt, 1); //If the Arduino can establish communication with the GTT, toggle the Arduino connection indicator appropriately       
  gtt25_set_button_clickhandler(&gtt, MyButtonClick); //Configure the button click handler
  setCommunicationChannel(2); //set the communication channel to i2c so button clicks can be returned to the Arduino
  tempSensorReady = 1; //Indicate that the DS18S20 is ready for communication
}

void loop() {
  if(tempSensorReady){ //If the DS18S20 is ready to start a conversion
    probeConnected = searchForTempProbe(); //Search for the DS18S20
    startTempConversion(); //Start the temperature sensor reading process
    tempTimer = millis(); //Start conversion timer
    tempSensorReady = 0; //Toggle SensorReady bool
  }
  if((millis()-tempTimer)>=800){ //800+ mS after starting the conversion, read the temp
    if(probeConnected){ //If the probe is connected    
        int16_t temp = readTempProbe(); //Read the temperature            
        char buf[4] = {0};
        sprintf(buf,"%d",temp); //Convert the temperature value to a string
        gtt_set_screen1_dynamic_label_2_text(&gtt, gtt_make_text_ascii(buf)); //Update the GTT label
        gtt_set_screen1_bar_graph_1_value(&gtt, temp); //Update the GTT bar graph              

        //If the temperature exceeds either the high or low limit, activate the buzzer and motor
        if(temp >= maxTemp){ 
          Serial.println("Buzzing High");
          activateBuzzer(3000, 500);
        }
        if(temp <= minTemp){        
          Serial.println("Buzzing low");
          activateBuzzer(500, 500);
        }        
    }   
    else { //If the probe isn't connected
      gtt_set_screen1_image_toggle_3_state(&gtt, 0); //Set the probe indicator to "Disconnected"  
      gtt_set_screen1_bar_graph_1_value(&gtt, 0); //Set the GTT bar graph to 0     
      gtt_set_screen1_dynamic_label_2_text(&gtt, gtt_make_text_ascii("NA")); //Update the GTT label to "NA"          
    }
    tempSensorReady = 1; //Temperature sensor is ready for another conversion    
  }      
  gtt_parser_process(&gtt); //Parse for touch events
    
}

void MyButtonClick(gtt_device* gtt, uint16_t ObjectID, uint8_t State)
{    
  Serial.println(State);  
  if (State == 1)
  {    
    char buf[4] = {0};
    if (ObjectID == id_screen1_circle_button_1)
    {      
      if((maxTemp-1)>minTemp){
        maxTemp--;            
        Serial.print("maxTemp = ");  
        Serial.println(maxTemp);      
        sprintf(buf,"%d",maxTemp); //Convert the temperature value to a string
        gtt_set_screen1_dynamic_label_3_text(gtt, gtt_make_text_ascii(buf)); //Update the High limit        
      }            
    }
    if (ObjectID == id_screen1_circle_button_2)
    { 
      if(minTemp >0)
      {     
        minTemp--;              
        Serial.print("minTemp = ");  
        Serial.println(minTemp);  
        sprintf(buf,"%d",minTemp); //Convert the temperature value to a string              
        gtt_set_screen1_dynamic_label_1_text(gtt, gtt_make_text_ascii(buf)); //Update the Low limit
      }
    }
    if (ObjectID == id_screen1_circle_button_3)
    {      
      if(maxTemp <110)
      {
        maxTemp++;                
        Serial.print("maxTemp = ");  
        Serial.println(maxTemp);
        sprintf(buf,"%d",maxTemp); //Convert the temperature value to a string              
        gtt_set_screen1_dynamic_label_3_text(gtt, gtt_make_text_ascii(buf)); //Update the High limit
      }      
    }
    if (ObjectID == id_screen1_circle_button_4)
    {      
      if((minTemp+1)<maxTemp)
      {
        minTemp++;          
        Serial.print("minTemp = ");  
        Serial.println(minTemp);
        sprintf(buf,"%d",minTemp); //Convert the temperature value to a string   
        gtt_set_screen1_dynamic_label_1_text(gtt, gtt_make_text_ascii(buf)); //Update the Low limit
      }      
   }  
  }    
}

void resetDisplay() {
  Serial.println("resetting display");
  char command[] = { 254, 1 };
  i2cWrite(&gtt, command, sizeof(command));
}

void setCommunicationChannel(byte channel) {
  Serial.println("Setting Communication Channel");
  char command[] = { 254, 5, channel };
  i2cWrite(&gtt, command, sizeof(command));
}

void activateBuzzer(short frequency, short duration) {
  Serial.println("Setting Communication Channel");
  char command[] = { 254, 187, (byte)(frequency >>8), (byte)(frequency && 0xFF), (byte)(duration >>8), (byte)(duration && 0xFF)};
  i2cWrite(&gtt, command, sizeof(command));
}

/* The Code used in the searchorTempProbe and readTempProbe function is 
 * sample code taken from https://playground.arduino.cc/Learning/OneWire */
byte searchForTempProbe(){
  
  ds18s20.reset_search();
  
  if ( !ds18s20.search(addr)) {
    Serial.print("No more addresses.\n");
    ds18s20.reset_search();
    delay(250);
    return 0;
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      gtt_set_screen1_image_toggle_3_state(&gtt, 0); //Set the probe indicator to "Disconnected"      
      return 0;
  }
  if ( addr[0] == 0x10) {
      Serial.print("Device is a DS18S20 family device.\n");
      gtt_set_screen1_image_toggle_3_state(&gtt, 1); //Set the probe indicator to "Connected"
      return 1;
  }     
  else {
      Serial.print("Device family is not recognized: 0x");
      Serial.println(addr[0],HEX);
      gtt_set_screen1_image_toggle_3_state(&gtt, 0); //Set the probe indicator to "Disconnected"      
      return 0;
  }
}

void startTempConversion(){
  ds18s20.reset();
  ds18s20.select(addr);
  ds18s20.write(0x44,1);// start conversion, with parasite power on at the end
}

uint16_t readTempProbe(){
  byte present = 0;
  byte data[12];
  uint16_t temp;
     
  present = ds18s20.reset();
  ds18s20.select(addr);    
  ds18s20.write(0xBE);// Read Scratchpad
  
  for ( byte i = 0; i < 9; i++) {// we need 9 bytes
    data[i] = ds18s20.read();        
  }    
  temp = ( (data[1] << 8) + data[0] ) * 0.5;    
  temp= (temp * 9/5) + 32;
  Serial.print("Temp = ");
  Serial.println(temp);
  return temp;
}     

int i2cWrite(gtt_device* gtt_device, char* data, byte data_length) {//Write an array of bytes over i2c
  Wire.beginTransmission(I2C_Address);  
  for (int i = 0; i < data_length; i++) {
    Wire.write(data[i]);        
  }
  Wire.endTransmission();  
  return 0;
}

byte i2cRead(gtt_device* gtt_device) { //Wait for one byte to be read over i2c  
  byte data;  
  Wire.beginTransmission(I2C_Address);  
  Wire.requestFrom(I2C_Address, 1);     
  if(Wire.available()<1) 
  {
    return -1;
  }
  else{
    data = Wire.read();      
    return data;
  }
}


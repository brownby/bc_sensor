#include<Arduino.h>
#include<SD.h>
#include<SPI.h>
#include<Wire.h>
#include<SHT31.h>
#include<HoneywellZephyrI2C.h>

ZephyrFlowRateSensor frs(0x49, 200);
File data_file;
SHT31 sht;

#define dim_   0 // PA22
#define frs_   1 // PA23
#define trh_   2 // PA10
#define ler_   3 // PA11
#define leg_   4 // PB10
#define leb_   5 // PB11
#define stat_  7 // PA21
#define en_    8 // PA16, MOSI (!)
#define cd_   30 // PA27

#define dac_   A0 // PA02
#define v_bc_  A1 // PB02
#define v_ref_ A2 // PB03
#define usb_   A3 // PA04

#define led_   LED_BUILTIN  // PB08
#define bat_   ADC_BATTERY  // PB09

// states

bool usb;       
bool stat;       
bool en = false; 

int bat;  

// parameters

int interval = 30; // in seconds
int dim = 20;      // set LED brightness, from 1 to 32   
int rate = 0;      // in ccm 
int flow_l = 0;    // ccm, at DAC min
int flow_h = 0;    // ccm, at DAC max
   
// measurements
   
int v_bc;  
int v_ref;

float temp;  // in celcius 
float rhum;  // in %
float flow;  // in ccm

// other

bool flag = 0;
unsigned int rgb[3]; 

void setup() {

  //Serial.begin(115200);
  //Serial.println("setup");

  pinMode(dim_, OUTPUT);
  pinMode(frs_, OUTPUT);
  pinMode(trh_, OUTPUT);
  pinMode(ler_, OUTPUT);
  pinMode(leg_, OUTPUT);
  pinMode(leb_, OUTPUT);
  pinMode(en_,  OUTPUT);
  pinMode(dac_, OUTPUT);
  pinMode(led_, OUTPUT);
  pinMode(cd_, INPUT_PULLUP);

  digitalWrite(frs_, HIGH);
  digitalWrite(trh_, HIGH);
  digitalWrite(led_, HIGH);

  digitalWrite(en_, LOW); // disable blower
  digitalWrite(dac_, LOW);

  Wire.begin();
  Wire.setClock(100000);
  sht.begin(0x44);
  frs.begin();

  SD.begin();

  if (!digitalRead(cd_)){
    data_file = SD.open("data.txt", FILE_WRITE);
    if (!data_file){
    //Serial.println("error opening file");
    }
    data_file.close(); 
  }
  else{
    //Serial.println("no SD card detected");
  }
  
  analogReadResolution(12);
  analogWriteResolution(8);

  digitalWrite(frs_, LOW);
  digitalWrite(trh_, LOW);
  digitalWrite(led_, LOW);

}

void set_dim(byte level) {

  // if (level < 1 || level > 32) {
  //   Serial.println("LED level argument must be between 1 and 32");
  // }
  
  digitalWrite(dim_, HIGH);

  for (byte i = 1; i < (32-level); i++) {
    digitalWrite(dim_, LOW);
    digitalWrite(dim_, HIGH);
  }
}

void set_flow(int rate) {

  int dac = map(rate, flow_l, flow_h, 0, 4095);
  analogWrite(dac_, dac);
  en = true;
  digitalWrite(en_, en);

}

void set_ring(int red, int green, int blue) {

  rgb[0] = 255-red; 
  rgb[1] = 255-green;
  rgb[2] = 255-blue;

  analogWrite(ler_, rgb[0]);
  analogWrite(leg_, rgb[1]);
  analogWrite(leb_, rgb[2]);

}

void get_pd() {

  v_bc = analogRead(v_bc_);
  v_ref = analogRead(v_ref_);

}

void get_flow() {

  flow = frs.flow();

  
  digitalWrite(frs_, HIGH);
  delay(50);
  digitalWrite(frs_, LOW);  

}

void get_trh() {

  sht.read();
  temp = sht.getTemperature();
  rhum = sht.getHumidity();

  digitalWrite(trh_, HIGH);
  delay(50);
  digitalWrite(trh_, LOW);
  
}

void write_data(){
  //if (!digitalRead(cd_)){
    digitalWrite(frs_, HIGH);
    data_file = SD.open("data.txt", FILE_WRITE);
    if (data_file){
      char buffer[50];
      //sprintf(buffer, "%d V, %d V, %f C, %f RH, %f ccm", v_bc, v_ref, temp, rhum, flow); 
      //sprintf(buffer, "%f C, %f RH", temp_raw, rhum_raw); 
      data_file.println(buffer);
    }
    data_file.close();
  //}
  //else{
    digitalWrite(frs_, LOW);
  //}
}

void parse_cmd(){

  if(Serial.available() > 0){
    char incoming = Serial.read();
    if(incoming == 'b'){
      Serial.print("battery voltage is: ");
      Serial.print((8.4/4095.0)*analogRead(ADC_BATTERY));
      Serial.println(" volts");
    }
    else if(incoming == 'c'){
      Serial.print("usb is ");
      usb = digitalRead(usb_);
      if(usb){
        Serial.print("plugged in and the battery is ");
        stat = digitalRead(stat_);
        if(!stat){
          Serial.println("charging");
        }
        else{
          Serial.println("not charging");
        }
      }
      else{
       Serial.println("not plugged in"); 
      }
    }
  }
}

void check_state(){

  usb = digitalRead(usb_);
  stat = digitalRead(stat_);

}

void loop() {

  get_trh();
  get_flow();
  set_ring(200, 0, 255);

  if (!flag){
    set_dim(32);
    flag = true;
  }

  get_pd();

  delay(1000);

  // if (digitalRead(stat_) && !digitalRead(usb_)){
  //   set_flow(rate);
  //   set_dim(dim);
  //   get_flow();

  //   if (abs(flow-rate)<10){
  //     delay(10);
  //     get_trh();
  //     get_pd();
  //     delay(10);
  //     write_data();
  //     en = false;
  //     digitalWrite(en_, en);        
  //   }
  // }
  
  // for (byte i = 0; i < interval; i++) {
  //   check_state();
  //   parse_cmd();
  //   delay(1000);
  // }
}

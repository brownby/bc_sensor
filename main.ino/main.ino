#include<Wire.h>
#include<SPI.h>
#include<SD.h>

File data_file;

#define SDA 0  // PA08
#define SCL 0  // PA09

#define SD_MOSI 0 // PA12
#define SD_MISO 0 // PA15
#define SD_SCK  0 // PA13
#define SD_NSS  0 // PA14
#define SD_CD   0 // PA27

#define dim_   D0 // PA22
#define frs_   D1 // PA23
#define trh_   D2 // PA10
#define ler_   D3 // PA11
#define leg_   D4 // PB10
#define leb_   D5 // PB11
#define stat_  D7 // PA21
#define en_    D8 // PA16, MOSI (!)

#define v_bc_  A1 // PB02
#define v_ref_ A2 // PB03
#define usb_   A3 // PA04

#define dac_   DAC0         // PA02 (aka A0)
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
int temp;  // in celcius 
int rhum;  // in %
int flow;  // in ccm

void setup() {

  Serial.begin(115200);
  Serial.println("setup");
  digitalWrite(led_,HIGH);

  Wire.begin();
  SD.begin();

  data_file = SD.open("data.txt", O_CREATE);
  if (!data_file){
    Serial.println("error opening file");
  }
  file.close()  

  pinMode(dim_, OUTPUT);
  pinMode(frs_, OUTPUT);
  pinMode(trh_, OUTPUT);
  pinMode(ler_, OUTPUT);
  pinMode(leg_, OUTPUT);
  pinMode(leb_, OUTPUT);
  pinMode(en_,  OUTPUT);
  pinMode(dac_, OUTPUT);
  pinMode(led_, OUTPUT);

  analogReadResolution(12);
  analogWriteResolution(10);

  digitalWrite(en_, en); // disable blower
  digitalWrite(led_, LOW);

}

void set_dim(byte level) {

  if (level < 1 || level > 32) {
    Serial.println("LED level argument must be between 1 and 32");
  }
  
  digitalWrite(dim_, HIGH);
  
  for (byte i = 0; i < (32-level); i++) {
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

  analogWrite(ler_, red);
  analogWrite(leg_, green);
  analogWrite(leb_, blue);

}

void get_pd() {

  v_bc = analogRead(v_bc_);
  v_ref = analogRead(v_ref_);

}

int get_flow() {

  Wire.requestFrom(byte(0x49), 2);

  delay(100);

  if (2 <= Wire.available()) {
    int flow_raw = Wire.read();
    flow_raw = flow_raw << 8; 
    flow_raw |= Wire.read();
  }

  flow = 200*((flow_raw/16384-0.5)/0.4);

  digitalWrite(frs_, HIGH);
  delay(250);
  digitalWrite(frs_, LOW);

  return flow;

}

void get_trh() {

  Wire.beginTransmission(byte(0x44));
  Wire.write(byte(0x2C)); // clock stretching, vs no = 0x24
  Wire.write(byte(0x0D)); // medium repeatability (or 0x0B if no clk stretch)
  Wire.endTransmission();

  delay(100);

  if (2 <= Wire.available()) {
    int temp_raw = Wire.read();
    temp_raw = temp_raw << 8; 
    temp_raw |= Wire.read();
  }

  if (2 <= Wire.available()) {
    int rhum_raw = Wire.read();
    rhum_raw = rhum_raw << 8; 
    rhum_raw |= Wire.read();
  }

  temp = -45 + 175*(temp_raw/65535);
  rhum = 100*(rhum_raw/65535);

  digitalWrite(trh_, HIGH);
  delay(250);
  digitalWrite(trh_, LOW);

}

void write_data(){
  
  data_file = SD.open(data.txt, O_APPEND);
  if (data_file){
    char buffer[50];
    sprintf(buffer, "%d V, %d V, %d C, %d RH, %d ccm", v_bc, v_ref, temp, rhum, flow); 
    data_file.println(buffer);
  }
  data_file.close();
}

void parse_cmd(){

  if(Serial.available > 0){
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

  if (digitalRead(stat_) && !digitalRead(usb_)){
    set_flow(rate);
    set_dim(dim);
    flow = get_flow();

    if (abs(flow-rate)<10){
      delay(10);
      get_trh();
      get_pd();
      delay(10);
      write_data();
      en = false;
      digitalWrite(en_, en);        
    }
  }
  
  for (byte i = 0; i < interval; i++) {
    check_state();
    parse_cmd();
    delay(1000);
  }
}

// WiP versione con OLED 1.3 agg_05.03.2024

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <PID_v1.h>
#include <U8glib.h> 
#include <Adafruit_MAX31865.h>
#include <RTClib.h>
#include <DHT_U.h>

//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);	// I2C / TWI 
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_FAST);	// Dev 0, Fast I2C / TWI
//U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);	// Display which does not send ACK

//  Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
RTC_DS3231 rtc;  //3231

DateTime now;

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      419.2  // calibrare sonda
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
#define isterUm 1.5  // isteresi valore umidità

void run();
void set1();
void set2();
void set3();
void set4();
void set5();
void set6();
void toEsc();
void temposcaduto();
int Piu(int vapiu);
int Meno(int vameno);

typedef void (*fDiStato) ();
int stato = 0;
unsigned long te_ris, T_esc;

DHT dht(3, DHT21);

fDiStato stati[] {run, set1, set2, set3, set4, set5, set6};

double kp=25, ki=3, kd=1.5;  //kp=20, ki=5, kd=1 parametri PID
double ingresso, uscita, setpoint = 20;
uint16_t winsize = 5000; // tempo finestra rele PID
uint32_t winstart, t1, t2;
uint8_t SSR = 4, LUX = 5, VENT = 6;  // PIN rele SSR riscaldatore tappeto, PIN rele luce, PIN ventola estraz. aria
uint8_t Pset = 7, PSetP = 8, PSetM = 9;  // PIN rele SSR riscaldatore tappeto, PIN rele luce 

uint8_t Alba = 6, Tram = 18; // ora inizio e fine luce
uint8_t Dly = 100;  // delay pulsanti set
uint16_t S_Um = 75;  // set umidità MAX
float Um, Tm; // Umidità e temperatura DHT 21
bool Giorno = false; 
int8_t TUmPlus = 0;


char buffer1[12];

PID pid(&ingresso, &uscita, &setpoint, kp, ki, kd, DIRECT);

void setup() {

  Serial.begin(9600);

  Wire.begin();
  dht.begin();
  rtc.begin();
  u8g.begin();


  if (rtc.lostPower()) {  // 1302 (!rtc.isrunning())
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
    delay(300);

  // definizione PIN IN-OUT
  pinMode(SSR, OUTPUT);  // rele tappeto
  pinMode(LUX, OUTPUT);  // rele luca Grow
  pinMode(VENT, OUTPUT);  // rele estraz. aria
  pinMode(Pset, INPUT);  // Pulsante SET
  pinMode(PSetP, INPUT);  // Pulsante SET +
  pinMode(PSetM, INPUT);  // Pulsante SET -
  attachInterrupt(digitalPinToInterrupt(2), toEsc, FALLING); // DEFINIZ. interrupt per tasto ESC premuto 

  thermo.begin(MAX31865_2WIRE);  // set to 3WIRE or 4WIRE as necessary
  pid.SetOutputLimits(0, winsize);
  pid.SetMode(AUTOMATIC);

}

void loop() {

/*
  Serial.print("  ");  
  Serial.println(stato);
 */
stati[stato]();
  
}

void run(){

  DateTime now = rtc.now();

    if ((now.hour() >= Alba)  && (now.hour() < Tram)) {
      digitalWrite(LUX, HIGH);
      //Giorno = true;      
    } else {
      digitalWrite(LUX, LOW);  // luce grove spenta
      //Giorno = false;
    }

Um = dht.readHumidity();  // legge valore umidità
Tm = dht.readTemperature();  // legge valore temperatura DHT 21

if ((now.hour() >= Alba)  && (now.hour() < Tram + TUmPlus)) {  // controllo umidità solo di giorno

  if( Um >= (S_Um + isterUm)){
    digitalWrite(VENT, HIGH);  // ventilatore on
  }
  if ( Um  <= (S_Um - isterUm)){
    digitalWrite(VENT, LOW);  // ventilatore off
  }
} else {
    digitalWrite(VENT, LOW);  // ventilatore off
}

ingresso = thermo.temperature(RNOMINAL, RREF);
pid.Compute();
  
  if ((millis() - t1) >= 1500) {   //rinfresca LCD non bloccante ogni 1 secondo

    //T_U = !T_U;  // ogni secondo inverte valore x temperatura/ umiditàù

    u8g.firstPage();  

    do {  // inizio loop print LCD
      u8g.setFont(u8g_font_6x13);  //  u8g_font_unifont
      u8g.drawStr(30, 10, "MINI SERRA" );
     
      u8g.setPrintPos(0, 22);
      u8g.print(" T. Tappeto: " + String(ingresso) + " C");

      u8g.setPrintPos(0, 35);
      u8g.print("Air T " + String(Tm) + "C U " + String(Um) + "%");


      u8g.setPrintPos(0,48);
      u8g.print("SetP. T. " + String(int(setpoint)) + "C - U. " + String(S_Um) + "%");
   
      u8g.setPrintPos(0,64);
      u8g.print("RTC " );  
      if(now.hour() <= 9){
        u8g.print("0" + String(now.hour()));
      } else {
        u8g.print(now.hour());      
      }
      u8g.print(":");
        if(now.minute() <= 9){
        u8g.print("0" + String(now.minute()));
      } else {
        u8g.print(now.minute());      
      }
      u8g.setPrintPos(58,64);
      u8g.print(" Vent:");
        if(digitalRead(VENT)){
          u8g.print(" ON");
      } else {
        u8g.print("OFF");      
      } 

    } while( u8g.nextPage() ); // FINE LOOP PRINT LCD

     t1 = millis(); 
  }

  if((now.hour() >= Alba)  && (now.hour() < 24)){ // Tappeto riscaldante solo dall'alba (impostazione T_alba) alle 24
    if ((millis() - winstart) > winsize) {
      winstart += winsize;
    }
    if (uscita < (millis() - winstart)) {
      digitalWrite(SSR, LOW);
    } else {
      digitalWrite(SSR, HIGH);
   }
     // Serial.println(digitalRead(SSR));
  }

  if(digitalRead(Pset)){ // legge pulsante SET

    delay(Dly);
    T_esc = millis(); 
    stato = 1;
  }   

 delay(10);
}


void set1(){
  // codice x set temperatura letto 

  u8g.firstPage();  

    do {  // inizio loop print LCD
      u8g.setFont(u8g_font_7x13);  
      u8g.setPrintPos(0,10);
      u8g.println("Set Temp. TAPPETO" );
      u8g.setPrintPos(0,26);
      u8g.println("Setpoint: " + String(setpoint) + " C" );
}   while( u8g.nextPage() ); // FINE LOOP PRINT LCD

  if(digitalRead(Pset)){
    delay(Dly);
    T_esc = millis();
    stato = 2;  // va a stato oraio luce grow
  }

if(digitalRead(PSetP)){  //aumento valore + 1
    delay(Dly);
    T_esc = millis();
    setpoint = constrain(Piu(setpoint), 15, 30);  // RANGE TEMPERATURE
  }     

if(digitalRead(PSetM)){  //diminuisce valore - 1
    delay(Dly);
    T_esc = millis();
    setpoint = constrain(Meno(setpoint), 15, 30);  // RANGE TEMPERATURE
  }  
temposcaduto();  // controlla se non pressati tasti
}

void set3(){ 
// codice per set orari lampada 

  u8g.firstPage();  

    do {  // inizio loop print LCD
      u8g.setFont(u8g_font_7x13);  
      u8g.setPrintPos(0,10);
      u8g.println("Set Ora LUX (6-18)" );
      u8g.setPrintPos(0,26);
      u8g.println("Time: " + String(Alba) + " - " + String(Tram));
}   while( u8g.nextPage() ); // FINE LOOP PRINT LCD

if(digitalRead(Pset)){
    delay(Dly);
    T_esc = millis();
    stato = 4;  // va a stato set umidità MAX
  }
if(digitalRead(PSetP)){  //aumento valore + 1
    delay(Dly);
    T_esc = millis();
    Alba = constrain(Meno(Alba), 4, 8);  // RANGE inizio lux
    Tram = constrain(Piu(Tram), 16, 20);  // RANGE fine lux
  }     

if(digitalRead(PSetM)){  //diminuisce valore - 1
    delay(Dly);
    T_esc = millis();
    Alba = constrain(Piu(Alba), 4, 8);  // RANGE inizio lux
    Tram = constrain(Meno(Tram), 16, 20);  // RANGE fine lux
  }  

temposcaduto();  // controlla se non pressati tasti
}

void set2(){
  // codice per set umidità 
   
 u8g.firstPage();  

    do {  // inizio loop print LCD
      u8g.setFont(u8g_font_7x13);  
      u8g.setPrintPos(0,10);
      u8g.println("Set UMIDITA' MAX" );
      u8g.setPrintPos(0,26);
      u8g.println("MAX UM.: " + String(S_Um) + " %" );
}   while( u8g.nextPage() ); // FINE LOOP PRINT LCD

  if(digitalRead(Pset)){
    delay(Dly);
    T_esc = millis();
    stato = 3;  // va a stato reg. ore
  }

if(digitalRead(PSetP)){  //aumento valore + 1
    delay(Dly);
    T_esc = millis();
    S_Um = constrain(Piu(S_Um), 50, 95);  // RANGE Umidità
  }     

if(digitalRead(PSetM)){  //diminuisce valore - 1
    delay(Dly);
    T_esc = millis();
    S_Um = constrain(Meno(S_Um), 50, 95);  // RANGE Umidità
  }  
temposcaduto();  // controlla se non pressati tasti
}

void set4(){
  // codice per set time (regolazione ore) 
  now = rtc.now();
  int Seth = now.hour();

 u8g.firstPage();  

    do {  // inizio loop print LCD
      u8g.setFont(u8g_font_7x13);  
      u8g.setPrintPos(0,10); 
      u8g.println("Set ORA Corrente " );
      u8g.setPrintPos(0,26);
      u8g.println("ORE: " + String(Seth));
}   while( u8g.nextPage() ); // FINE LOOP PRINT LCD 

  if(digitalRead(Pset)){
    delay(Dly);
    T_esc = millis();
    stato = 5;  // va a stato reg. minuti
  }

if(digitalRead(PSetP)){  //aumento valore + 1
    delay(Dly);
    T_esc = millis();
    Seth++;  // set ORE +
    if(Seth >= 24) Seth = 0;
    rtc.adjust (DateTime(now.year(), now.month(), now.day(), Seth, now.minute(), 0));
  }     

if(digitalRead(PSetM)){  //diminuisce valore - 1
    delay(Dly);
    T_esc = millis();
    Seth--;  // set ORE +
    if(Seth < 0) Seth = 23;
    rtc.adjust (DateTime(now.year(), now.month(), now.day(), Seth, now.minute(), 0));

  }  
temposcaduto();  // controlla se non pressati tasti
}

void set5(){
  // codice per set time (regolazione minuti) 
  now = rtc.now();
  int Setm = now.minute();
 u8g.firstPage();  

    do {  // inizio loop print LCD
      u8g.setFont(u8g_font_7x13);  
      u8g.setPrintPos(0,10); 
      u8g.println("Set MINUTI Correnti " );
      u8g.setPrintPos(0,26);
      u8g.println("MINUTI: " + String(Setm));
}   while( u8g.nextPage() ); // FINE LOOP PRINT LCD 

  if(digitalRead(Pset)){
    delay(Dly);
    T_esc = millis();
    stato = 6;  // va a stato reg. tempo plus umidita
  }

if(digitalRead(PSetP)){  //aumento valore + 1
    delay(Dly);
    T_esc = millis();
    Setm++;  // set MINUTI +
    if(Setm >= 60) Setm = 0;
    rtc.adjust (DateTime(now.year(), now.month(), now.day(), now.hour(), Setm, 0));
  }     

if(digitalRead(PSetM)){  //diminuisce valore - 1
    delay(Dly);
    T_esc = millis();
    Setm--;  // set MINUTI -
    if(Setm < 0) Setm = 59;
    rtc.adjust (DateTime(now.year(), now.month(), now.day(), now.hour(), Setm, 0));

  }  
temposcaduto();  // controlla se non pressati tasti
}

void set6(){
  // codice per set tempo plus controllo umidità 
   
 u8g.firstPage();  

    do {  // inizio loop print LCD
      u8g.setFont(u8g_font_7x13);  
      u8g.setPrintPos(0,10);
      u8g.println("Time +/- fine CTRL " );
      u8g.setPrintPos(0,22);
      u8g.println("umidita' " );
      u8g.setPrintPos(0,36);
      if(TUmPlus >= 0) {
      u8g.println("Tramonto = " + String(Tram) + " + " + String(TUmPlus) + " ");
      } else {
      u8g.println("Tramonto = " + String(Tram) + " " + String(TUmPlus) + " ");  
      }
}   while( u8g.nextPage() ); // FINE LOOP PRINT LCD

  if(digitalRead(Pset)){
    delay(Dly);
    T_esc = millis();
    stato = 0;  // va a stato run
  }

if(digitalRead(PSetP)){  //aumento valore + 1
    delay(Dly);
    T_esc = millis();
    TUmPlus = constrain(Piu(TUmPlus), -4, 6);  // RANGE Umidità
  }     

if(digitalRead(PSetM)){  //diminuisce valore - 1
    delay(Dly);
    T_esc = millis();
    TUmPlus = constrain(Meno(TUmPlus), -4, 6);  // RANGE Umidità
  }  
temposcaduto();  // controlla se non pressati tasti
}

// FUNZIONI 
//___________________________________________
int Piu(int vapiu){

  vapiu ++; 
  return vapiu;
    
}

int Meno(int vameno){
  
   vameno --; 
  return vameno;
    
}

void temposcaduto() {
  if(millis() - T_esc > 20000) {
    delay(100);
    stato = 0;
  }

}

void toEsc() {
  stato = 0;
}
/* // reader 1
 wiegand      esp32
 D0 --------- 36
 D1 --------- 39

***********************
   //  reader 2
 wiegand      esp32
 D0 --------- 34
 D1 --------- 35

 ***********************
   // reader 3
 wiegand      esp32
 D0 --------- 32
 D1 --------- 33
 ***********************
    // reader 4
 wiegand      esp32
 D0 --------- 25
 D1 --------- 26

 ***********************

   // ENC28j60 ETHERNET 
  ENC28J60  ------- ESP32 (VSPI)
  CS ---------- D5
  SCK --------- D18
  MISO(SO)------ D19
  MOSI(SI) -------- D23
  VCC ------ 3.3v
  ***************************
  
  // RS485     **** RS485 transceiver. MAX485
   MAX485(TTL to RS485 converter)----- ESP32 (UART2) // RX_16 TX_17
  RO ----------------------------- RX PIN (GPIO 16)  D(RECIVER OUTPUT)
  DI -------------------------- TX PIN (GPIO 17 ) (DRIVER INPUT )
  DE -------------------------  ENABLE PIN (GDPIO 4) (RECIVER OUTPUT)
  RE -------------------------- ENABLE PIN (GDPIO 2)  (DRIVER OUTPUT)
  VCC-------------------- 5V 

*************************************
 ///// PCF8574   /// I2C Communication
  PCF8574 (I2C Device) ---------- ESP32   
  SDA ----------------------  SDA GPIO 21
  SCL --------------------- SCL GPIO 22
  VCC ------------------- 3.3V OR 5V
  A0 --------------------- GND
  A1----------------------- GND
  A2-------------------------GND
  P0---->P3 4 BOUTTON
  P4 ---->P7 4 relais  
 //// INT -----------> pin 15 interrupt
  ****************************
  
  SN74HC595N Shifting register
  SN74HC595N --------------------------------------------------------------ESP32 
  DS/SER /(DATA) /INPUT/Serial data input ------------------------------- dataPin = 13
  ST_CP/ RCLK/REFRESH_OUTPUT/Storage register clock pin (latch pin)------ latchPin =12
  SH_CP/SRCLK/CLOCK/ Shift register clock pin ----------------------------clockPin = 14
  Les broches Q0 – Q3 --------------------------------------------------->  4 LED 
  OE : DISABLE/ENABLE_OUTPUT/active low
  MR/SECLR : REGISTER CLEAR //Master Reclear, active low
  OE (pin 13) ------------------------------------------------------------to ground
  MR (pin 10)------------------------------------------------------------- to 5V
 VCC ----------------------------------------------------------------------- 5V
 ***********************************
bibliotheques  :
EthernetENC : https://github.com/JAndrassy/EthernetENC
PCF8574_library :https://github.com/xreef/PCF8574_library/tree/master
SSLClient :https://github.com/OPEnSLab-OSU/SSLClient
Yet_Another_Arduino_Wiegand_Library "Wiegand.h" :https://github.com/paulo-raca/YetAnotherArduinoWiegandLibrary
*/

#include <EthernetENC.h>
#include <Wiegand.h>
#include <SSLClient.h>
#include "trust_anchors.h"
#include <SPI.h>
#include <HardwareSerial.h>
#include "PCF8574.h"

HardwareSerial SerialPort(2);

int slave_1_id = 1;
int slave_2_id = 2;
int slave_3_id = 3;
int slave_4_id = 4;

const int Enable =  2;

char server[] = "192.168.100.8";
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

const String serail_number= "1re3tg6po" ;

// Set i2c address
PCF8574 pcf8574(0x27,21,22); // SDA 21 , SCL 22

#define intPin  15                   // interrupt input 

//SN74HC595N Shifting register
int dataPin = 13; //DS
int latchPin =12; //ST_CP   
int clockPin = 14; //SH_CP

#define PIN_D0_READER1 36
#define PIN_D1_READER1 39
#define PIN_D0_READER2 34
#define PIN_D1_READER2 35
#define PIN_D0_READER3 32
#define PIN_D1_READER3 33
#define PIN_D0_READER4 25
#define PIN_D1_READER4 26

Wiegand wiegandReader1;
Wiegand wiegandReader2;
Wiegand wiegandReader3;
Wiegand wiegandReader4;
EthernetClient client;

bool etatActuel ;
bool etatPrecedent = false;
bool cardRead1 = false;
bool cardRead2 = false;
bool cardRead3 = false;
bool cardRead4 = false;
bool porte_1 = false;
bool porte_2 = false;
bool porte_3 = false;
bool porte_4 = false;
bool disconnected_1 =false;
bool disconnected_2 =false;
bool disconnected_3 =false;
bool disconnected_4 =false;
String valueStr_1 = "" ;
String valueStr_2 = "" ;
String valueStr_3 = "" ;
String valueStr_4 = "" ;
String data = "";  //reponse de serveur 
String porte=""; // variable pour le num porte requet https
int Num_lecteur ; //variable pour le num lecteur  requet https disconnect 

// intterup push bouton 
volatile bool buttonPressed = false;   
byte buttonReg;                        

void setup()
{ 
pinMode(27,OUTPUT);
digitalWrite(27,HIGH);
Serial.begin(112560);
initEthernet();
initWiegand();

SerialPort.begin(115200, SERIAL_8N1, 16, 17); //<<RS485
pinMode(Enable, OUTPUT);// RS485 
digitalWrite(Enable, LOW);// rs485
initpcf8574();
init74HC595();
Serial.println(F("Starting connection to server..."));
delay(100);
Serial.println(F("Start..."));
}


void loop()
{ 

    printoutData(); // afficher la reponse de serveur "data"
  if (disconnected_1) {
    Num_lecteur=1;    
    httpRequest_dis(Num_lecteur);
    disconnected_1 = false;
  }

   if (disconnected_2) {
    Num_lecteur=2;    
    httpRequest_dis(Num_lecteur);
    disconnected_2 = false;
  }
  if (disconnected_3) {
    Num_lecteur=3;    
    httpRequest_dis(Num_lecteur);
    disconnected_3 = false;
  }
    if (disconnected_4) {
    Num_lecteur=4;    
    httpRequest_dis(Num_lecteur);
    disconnected_4 = false;
  }

  

  if (porte_1) { //presence carte porte 1
    verifier (4,1); // 4 pour relais 1 P4 ------------ 1 pour led 1 Q1
    porte_1 = false;
  }

  if (porte_2) {
    verifier (5,2); // 5 pour relais 2 P5 ------------ 2 pour led 2 Q2
    porte_2 = false;
  }
  if (porte_3) {
    verifier (6,3);
    porte_3 = false;
  }
  if (porte_4) {
    verifier (7,4);
    porte_4 = false;
  }
  updateShiftRegister() ; 
  check_button (0,4,1) ; // 0 pour P0 (pcf) = Bouton *** 4 pour P4 (pcf) realis 1 *** 1 shifter led_ num 1 Q1 
  check_button (1,5,2) ;
  check_button (2,6,3) ;
  check_button (3,7,4) ;
  noInterrupts(); //desactiver les interrputions pour lire les donnees des lecteurs 
  wiegandReader1.flush();
  wiegandReader2.flush();
  wiegandReader3.flush();
  wiegandReader4.flush();
  interrupts(); //Réactiver les interrpution 
  digitalWrite(Enable, HIGH);
  SerialPort.print(slave_1_id);
  SerialPort.print(slave_2_id);
  SerialPort.print(slave_3_id);
  SerialPort.print(slave_4_id);
  SerialPort.print("ON");
  SerialPort.flush();
 digitalWrite(Enable, LOW);
 if(SerialPort.available()){
    Serial.println(SerialPort.readString());
}
  delay(100);
  if (cardRead1) { //lire du 1er lecteur
    porte="1";    //parametre requette 
    httpRequest(valueStr_1,porte);
    cardRead1 = false;
    porte_1 = true;
     valueStr_1 = "" ;
  }

  if (cardRead2) {
    porte="2";
    httpRequest(valueStr_2,porte);
    cardRead2 = false;
    porte_2 = true;
     valueStr_2 = "" ;
  }
    if (cardRead3) {
    porte="3";
    httpRequest(valueStr_3,porte);
    cardRead3 = false;
    porte_3 = true;
     valueStr_3 = "" ;
  }
    if (cardRead4) {
    porte="4";
    httpRequest(valueStr_4,porte);
    cardRead4 = false;
    porte_4 = true;
     valueStr_4 = "" ;
  }

}

void printoutData() {
  while (client.available()) {
    char c = client.read();
    data += c;
    Serial.write(c);
    Serial.flush();
  }
}


void check_button( int p_boutton, int p_relais, int shifterLedNum)
{
  etatActuel = pcf8574.digitalRead(p_boutton);
  if (etatActuel != etatPrecedent ) 
  {
   active_relais( p_relais, shifterLedNum);
    etatPrecedent = true;  
  }
}



void verifier ( int p_relais ,int shifterLedNum) {
  if (data.indexOf("zaretna el barka") >= 0) {
  active_relais(p_relais, shifterLedNum);
  }
  data = "";
}

void active_relais ( int p_relais, int shifterLedNum)
{
  Shifter_commander(shifterLedNum,HIGH);
  pcf8574.digitalWrite(p_relais,HIGH);
  delay(500);
  pcf8574.digitalWrite(p_relais,LOW);
  delay(250);
  Shifter_commander(shifterLedNum,LOW);
}

 void  Shifter_commander (int pin_of_output , bool etat   ) //to command any outputs exept Q0 of the PCF8574T (the MUX)
{ if (etat==HIGH){
    int led = 1 << pin_of_output;
    digitalWrite(latchPin, LOW); //On s'assure que la gachette est a l'etat LOW
    shiftOut(dataPin, clockPin, MSBFIRST,led ); //On utlise la fonction shiftOut de la librairie
    //arduino pour lui soumettre la valeur a transmettre au registre
    digitalWrite(latchPin, HIGH);
     //On inverse l'etat de la gachette pour mettre a jour le registre
    //delay(time_high);
    digitalWrite(latchPin, LOW);
    delay(5);
    }
}


// interrupt service routine
void buttonISR() {
  buttonPressed = true;
}


void initpcf8574()
{
  pcf8574.pinMode(P0, INPUT_PULLUP);
  pcf8574.pinMode(P1, INPUT_PULLUP);  
  pcf8574.pinMode(P2, INPUT_PULLUP);
  pcf8574.pinMode(P3, INPUT_PULLUP);  
  pcf8574.pinMode(P4, OUTPUT);
  pcf8574.pinMode(P5, OUTPUT);
  pcf8574.pinMode(P6, OUTPUT);
  pcf8574.pinMode(P7, OUTPUT);
    // initialize PCF8574 with an interrupt pin 
  pinMode(intPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(intPin), buttonISR, FALLING);
  pcf8574.begin();
  Serial.print("pcf8574 IS READY");
  if (pcf8574.begin()){
  Serial.println("pcf8574 OK");
  }else{
  Serial.println("pcf8574 NO");
  }
  Serial.println("you can START");
}

void updateShiftRegister() {
  // On s'assure que la gâchette est à l'état LOW
  digitalWrite(latchPin, LOW);
  // On utilise la fonction shiftOut de la librairie Arduino pour soumettre la valeur à transmettre au registre
  shiftOut(dataPin, clockPin, MSBFIRST, 1);
  // On inverse l'état de la gâchette pour mettre à jour le registre
  digitalWrite(latchPin, HIGH);
}

void  init74HC595()
{
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
}



void  httpRequest_dis(  int numero_lecteur) {

  client.stop();

  if (client.connect(server, 8080)) {
    Serial.println(F("Connecting..."));

    client.print("GET /Lecteur/disconnected/");
    client.print(serail_number);
    client.print("/");
    client.print(numero_lecteur);
    client.println(" HTTP/1.1");
    client.println("Host: 192.168.100.8");
    client.println(F("Connection: close"));
    client.println();
  } else {
    Serial.println(F("Connection failed"));
  }
  
}


void httpRequest( String code ,String num_porte ) {
  Serial.println();
 Serial.println(code);
  client.stop();

  if (client.connect(server, 8080)) {
    Serial.println(F("Connecting..."));

    client.print("GET /Controller/des/");
    client.print(serail_number);
    client.print("/");
    client.print(num_porte);
    client.print("/");
    client.print(code);
    client.println(" HTTP/1.1");
    client.println("Host: 192.168.100.8");
    client.println(F("Connection: close"));
    client.println();
  } else {
    Serial.println(F("Connection failed"));
  }
  code="";
  num_porte="";
}



void initEthernet() {
  Serial.println("Begin Ethernet");

  Ethernet.init(5); // Use pin 5 for CS on MKR ETH Shield

  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed to configure Ethernet using DHCP"));

    IPAddress myIP(192, 168, 1, 28);
    IPAddress myDNS(192, 168, 1, 1);
    IPAddress myGateway(192, 168, 1, 1);
    IPAddress mySubnet(255, 255, 255, 0);

    Ethernet.begin(mac, myIP, myDNS, myGateway, mySubnet);
  }

  delay(1000);
  Serial.print("Local IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(Ethernet.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("DNS Server : ");
  Serial.println(Ethernet.dnsServerIP());
  Serial.println("Ethernet Successfully Initialized");
}


void initWiegand()
{
   wiegandReader1.onReceive(receivedData1, "Card reader 1: ");
  wiegandReader1.onStateChange(stateChanged1, "Card reader 1 state: ");
  wiegandReader1.begin(Wiegand::LENGTH_ANY, true);

  wiegandReader2.onReceive(receivedData2, "Card reader 2: ");
  wiegandReader2.onStateChange(stateChanged2, "Card reader 2 state: ");
  wiegandReader2.begin(Wiegand::LENGTH_ANY, true);

  wiegandReader3.onReceive(receivedData3, "Card reader 3: ");
  wiegandReader3.onStateChange(stateChanged3, "Card reader 3 state: ");
  wiegandReader3.begin(Wiegand::LENGTH_ANY, true);

  wiegandReader4.onReceive(receivedData4, "Card reader 4: ");
  wiegandReader4.onStateChange(stateChanged4, "Card reader 4 state: ");
  wiegandReader4.begin(Wiegand::LENGTH_ANY, true);

  pinMode(PIN_D0_READER1, INPUT);
  pinMode(PIN_D1_READER1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_D0_READER1), pinStateChangedReader1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_D1_READER1), pinStateChangedReader1, CHANGE);

  pinMode(PIN_D0_READER2, INPUT);
  pinMode(PIN_D1_READER2, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_D0_READER2), pinStateChangedReader2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_D1_READER2), pinStateChangedReader2, CHANGE);

  pinMode(PIN_D0_READER3, INPUT);
  pinMode(PIN_D1_READER3, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_D0_READER3), pinStateChangedReader3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_D1_READER3), pinStateChangedReader3, CHANGE);
  
  pinMode(PIN_D0_READER4, INPUT);
  pinMode(PIN_D1_READER4, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_D0_READER4), pinStateChangedReader4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_D1_READER4), pinStateChangedReader4, CHANGE);

  pinStateChangedReader1();
  pinStateChangedReader2();
  pinStateChangedReader3();
  pinStateChangedReader4();
}

void pinStateChangedReader1() {
  wiegandReader1.setPin0State(digitalRead(PIN_D0_READER1));
  wiegandReader1.setPin1State(digitalRead(PIN_D1_READER1));
}

void pinStateChangedReader2() {
  wiegandReader2.setPin0State(digitalRead(PIN_D0_READER2));
  wiegandReader2.setPin1State(digitalRead(PIN_D1_READER2));
}

void pinStateChangedReader3() {
  wiegandReader3.setPin0State(digitalRead(PIN_D0_READER2));
  wiegandReader3.setPin1State(digitalRead(PIN_D1_READER2));
}
void pinStateChangedReader4() {
  wiegandReader4.setPin0State(digitalRead(PIN_D0_READER2));
  wiegandReader4.setPin1State(digitalRead(PIN_D1_READER2));
}

void stateChanged1(bool plugged, const char* message) {
   if (!plugged) {
  disconnected_1 =true ;
   }
    Serial.print(message);
    Serial.println(plugged ? "CONNECTED" : "DISCONNECTED");
}

void stateChanged2(bool plugged, const char* message) {
   if (!plugged) {
  disconnected_2 =true ;
   }
    Serial.print(message);
    Serial.println(plugged ? "CONNECTED" : "DISCONNECTED");
}

void stateChanged3(bool plugged, const char* message) {
   if (!plugged) {
  disconnected_3 =true ;
   }
    Serial.print(message);
    Serial.println(plugged ? "CONNECTED" : "DISCONNECTED");
}

void stateChanged4(bool plugged, const char* message) {
   if (!plugged) {
  disconnected_4 =true ;
   }
    Serial.print(message);
    Serial.println(plugged ? "CONNECTED" : "DISCONNECTED");
}

void receivedData1(uint8_t* data, uint8_t bits, const char* message) {
    // Print the original message and bits
    Serial.print(message);
    Serial.print(bits);
    Serial.print(" bits / ");

    uint8_t bytes = (bits + 7) / 8;
    unsigned long decimalValue = 0;

    for (int i = 0; i < bytes; i++) {
        decimalValue = (decimalValue << 8) | data[i];
    }
    // Convert decimalValue to a string
    valueStr_1 = String(decimalValue);

    // Print the decimal value
    Serial.println(valueStr_1);

    // Set cardRead variable to true
    cardRead1 = true;
}



void receivedData2(uint8_t* data, uint8_t bits, const char* message) {
    // Print the original message and bits
    Serial.print(message);
    Serial.print(bits);
    Serial.print(" bits / ");

    uint8_t bytes = (bits + 7) / 8;
    unsigned long decimalValue = 0;

    for (int i = 0; i < bytes; i++) {
        decimalValue = (decimalValue << 8) | data[i];
    }

    // Convert decimalValue to a string
    valueStr_2 = String(decimalValue);

    // Print the decimal value
    Serial.println(valueStr_2);

    cardRead2 = true;
}

void receivedData3(uint8_t* data, uint8_t bits, const char* message) {
    // Print the original message and bits
    Serial.print(message);
    Serial.print(bits);
    Serial.print(" bits / ");

    uint8_t bytes = (bits + 7) / 8;
    unsigned long decimalValue = 0;

    for (int i = 0; i < bytes; i++) {
        decimalValue = (decimalValue << 8) | data[i];
    }
    // Convert decimalValue to a string
    valueStr_3 = String(decimalValue);

    // Print the decimal value
    Serial.println(valueStr_3);

    // Set cardRead variable to true
    cardRead3 = true;
}

void receivedData4(uint8_t* data, uint8_t bits, const char* message) {
    // Print the original message and bits
    Serial.print(message);
    Serial.print(bits);
    Serial.print(" bits / ");

    uint8_t bytes = (bits + 7) / 8;
    unsigned long decimalValue = 0;

    for (int i = 0; i < bytes; i++) {
        decimalValue = (decimalValue << 8) | data[i];
    }
    // Convert decimalValue to a string
    valueStr_4 = String(decimalValue);

    // Print the decimal value
    Serial.println(valueStr_4);

    // Set cardRead variable to true
    cardRead4 = true;
}







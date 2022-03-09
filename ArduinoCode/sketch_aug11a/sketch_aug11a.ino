#include <SPI.h>
#include <SoftwareSerial.h>
#include "ArduinoSort.h"
#include <LiquidCrystal_I2C.h>
SoftwareSerial espSerial(5, 6);
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define MAX_BLE_WAIT 90 // Maximum bluetooth re-connect time in seconds 
#define SLEEP_TIME 300000000 // SleepTime in µs 5 min = 300 s      //
#define MAX_NFC_READTRIES 10 // Amount of tries for every nfc block-scan
#define NFCTIMEOUT 500  // Timeout for NFC response
#define RXBUFSIZE 24
#define NFCMEMSIZE 320  // 40 blocs of 8 bytes
#define NFC15MINPOINTER 26  // 0x1A
#define NFC8HOURPOINTER 27  // 0x1B
#define NFC15MINADDRESS 28  // 0x1C
#define NFC8HOURADDRESS 124 // 0x64
#define NFCSENSORTIMEPOINTER  316 // 0x13C et 0x13D
#define NBEXRAW  5 // Exclus les NBEXRAW valeur dont l'écart type est maximal par rapport à la droite des moindres carré
byte RXBuffer[RXBUFSIZE];
byte NfcMem[NFCMEMSIZE];

byte NFCReady = 0;  // used to track NFC state
////RTC_DATA_ATTR byte FirstRun = 1;
int batteryPcnt;
long batteryMv;
int noDiffCount = 0;
int sensorMinutesElapse;
float trend[16];
float A = 0;
float B = 0;


const int SSPin = 10;  // Slave Select pin
const int IRQPin = 9;  // Sends wake-up pulse for BM019
const int MOSIPin = 11;
const int SCKPin = 13;
//unsigned long boottime;
unsigned long sleeptime;
int bootCount = 0;
byte FirstRun = 1;
float lastGlucose;

bool estConnecte = false;
bool etaitConnecte = false;

void SetProtocol_Command() {

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
  digitalWrite(SSPin, HIGH);
  delay(10);
 
  poll_NFC_UntilResponsIsReady();

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))  // is response code good?
    {
    Serial.println("Protocol Set Command OK");
    NFCReady = 1; // NFC is ready
    }
  else
    {
    Serial.println("Protocol Set Command FAIL");
    NFCReady = 0; // NFC not ready
    }
}

void Inventory_Command() {

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  delay(1);
 
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  delay(1);

  poll_NFC_UntilResponsIsReady();
  receive_NFC_Response();

  if (RXBuffer[0] == 128)  // is response code good?
    {
    Serial.println("Sensor in range ... OK");
    NFCReady = 2;
    }
  else
    {
    Serial.println("Sensor out of range");
    NFCReady = 1;
    }
 }
 
 void poll_NFC_UntilResponsIsReady()
{
  unsigned long ms = millis();
  byte rb;
//  print_state("poll_NFC_UntilResponsIsReady() - ");
  digitalWrite(SSPin , LOW);
  while ( (RXBuffer[0] != 8) && ((millis() - ms) < NFCTIMEOUT) )
  {

    rb = RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until

//    Serial.printf("SPI polling response byte:%x\r\n", RXBuffer[0]);
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
  }
  digitalWrite(SSPin, HIGH);
  delay(1);
  if ( millis() - ms > NFCTIMEOUT ) {
    Serial.print("\r\n *** poll timeout *** -> response ");
    Serial.print(rb);
  }
}

void receive_NFC_Response()
{
//  print_state("receive_NFC_Response()");
  byte datalength;
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);
  RXBuffer[0] = SPI.transfer(0);
  datalength = RXBuffer[1] = SPI.transfer(0);
  for (byte i = 0; i < datalength; i++) RXBuffer[i + 2] = SPI.transfer(0);
  digitalWrite(SSPin, HIGH);
  delay(1);

}

float Read_Memory() {

//Serial.println("Read Memory");
 float currentGlucose;
 float shownGlucose;
 int glucosePointer;
 int histoPointer;
 int raw;
 byte readError = 0;
 int readTry;
 int valididx[16];
  
 for ( int b = 3; b < 40; b++) {
  readTry = 0;
  do {
  readError = 0;   
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows
  SPI.transfer(0x02);  // request Flags byte
  SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  SPI.transfer(b);  // memory block address
  digitalWrite(SSPin, HIGH);
  delay(10);
 
  poll_NFC_UntilResponsIsReady();
  receive_NFC_Response();

if (RXBuffer[0] != 0x80)
       readError = 1;  
    
   for (int i = 0; i < 8; i++) {
     NfcMem[8*b+i]=RXBuffer[i+3];
   }
    readTry++;
  } while( (readError) && (readTry < MAX_NFC_READTRIES) );
  
 }
  if (!readError)
    {
      sensorMinutesElapse = (NfcMem[NFCSENSORTIMEPOINTER+1]<<8) + NfcMem[NFCSENSORTIMEPOINTER];
      glucosePointer = NfcMem[NFC15MINPOINTER];
      histoPointer=NfcMem[NFC8HOURPOINTER];

    float SigmaX=0;
    float SigmaY=0;
    float SigmaX2=0;
    float SigmaXY=0;
    int n = 0;

     for (int j=0; j<16; j++) {     
         raw = (NfcMem[NFC15MINADDRESS + 1 + ((glucosePointer+15-j)%16)*6]<<8) + NfcMem[NFC15MINADDRESS + ((glucosePointer+15-j)%16)*6];
        trend[j] = Glucose_Reading(raw) ;
        valididx[j] = j;
        SigmaX+=j;
        SigmaXY+=j*trend[j];
        SigmaX2+=j*j;
        SigmaY+=trend[j];
        n++;
       
     }

#ifdef PRINTMEM
    for (int j=0; j<32;j++) {
      raw = (NfcMem[NFC8HOURADDRESS + 1 + ((histoPointer+31-j)%32)*6]<<8) + NfcMem[NFC8HOURADDRESS + ((histoPointer+31-j)%32)*6];
      //Serial.println("Tendance " + String((j+1)/4) + "h"+ String((j*15+15)%60) + "min : " + String(Glucose_Reading(raw)) + " Raw : " + String(raw));
    }
#endif        
       
   // Valeur lissée par la droite des moindres carrés en considérant les 15 dernière minutes comme linéaire
   // y = A x + B avec A = (n*SigmaXY - SigmaX*SigmaY)/(n*SigmaX2 - SigmaX*SigmaX) et B =MoyenneY - A MoyenneX = (SigmaY - A*SigmaX)/n
   // Valeur renvoyée correspond à l'estimation pour x = 0, la valeur courante lue est remplacée par son estimation selon la droite des moindres carrés

    A = (n*SigmaXY - SigmaX*SigmaY)/(n*SigmaX2 - SigmaX*SigmaX);
    B = (SigmaY - A*SigmaX)/n;

    sortArray(valididx, 16, firstIsLarger);
    /*
    for (int j=0; j<16; j++) {    
      Serial.println(String(valididx[j]) + " " + String((trend[valididx[j]] - A * valididx[j] -B)*(trend[valididx[j]] - A * valididx[j] -B)));
      delay(20);
    }
  
    Serial.println("Projection moindre carrés 1 : " + String(B));
    Serial.println("Pente 1 : " + String(-A) + " Moyenne : " + String(SigmaY/n));
    Serial.println("Ecart 1 : " + String((B-trend[0])));

    Serial.print("Valeur exclues : ");*/
   // for(int j=16-NBEXRAW; j<16;j++)
      //Serial.print(String(valididx[j]+1) + String(" "));
    //Serial.println();
    
    SigmaX=0;
    SigmaY=0;
    SigmaX2=0;
    SigmaXY=0;
    n = 0;
     for (int j=0; j<(16-NBEXRAW); j++) {     
        SigmaX+=valididx[j];
        SigmaXY+=valididx[j]*trend[valididx[j]];
        SigmaX2+=valididx[j]*valididx[j];
        SigmaY+=trend[valididx[j]];
        n++;   
     }

    A = (n*SigmaXY - SigmaX*SigmaY)/(n*SigmaX2 - SigmaX*SigmaX);
    B = (SigmaY - A*SigmaX)/n;
    shownGlucose = B;

    currentGlucose = trend[0];
/*
    Serial.println("Projection moindre carrés : " + String(shownGlucose));
    Serial.println("Pente 2 : " + String(-A) + " Moyenne : " + String(SigmaY/n));
    Serial.println("Ecart 2 : " + String((shownGlucose-trend[0])));
   */
    if (FirstRun == 1)
       lastGlucose = trend[0];
       
    if ((lastGlucose == currentGlucose) && (sensorMinutesElapse > 21000)) // Expired sensor check
      noDiffCount++;

    if (lastGlucose != currentGlucose) // Reset the counter
      noDiffCount = 0;
    
    
    NFCReady = 2;
    FirstRun = 0;

    if (noDiffCount > 5)
      return 0;
    else  
      return shownGlucose;
    
    }
  else
    {
    Serial.print("Read Memory Block Command FAIL");
    NFCReady = 0;
    readError = 0;
    }
    return 0;
 }
 
float Glucose_Reading(unsigned int val) {
        int bitmask = 0x0FFF;
        return ((val & bitmask) / 13);
}

bool firstIsLarger(int first, int second) {
  if ((trend[first] - A * first -B)*(trend[first] - A * first -B) > (trend[second] - A * second -B)*(trend[second] - A * second -B)) {
    return true;
  } else {
    return false;
  }
}

void setup() {
    
    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH); 
    pinMode(SSPin, OUTPUT);
    digitalWrite(SSPin, HIGH);
    pinMode(MOSIPin, OUTPUT);
    pinMode(SCKPin, OUTPUT);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    SPI.begin();
    delay(10);                      // send a wake up
    digitalWrite(IRQPin, LOW);      // pulse to put the 
    delayMicroseconds(100);         // BM019 into SPI
    digitalWrite(IRQPin, HIGH);     // mode 
    delay(10);
    digitalWrite(IRQPin, LOW);
    Serial.begin(115200);
    espSerial.begin(115200);
    lcd.begin(); //Init with pin default ESP8266 or ARDUINO
    lcd.backlight(); //accende la retroilluminazione
}
void loop() {
  if (NFCReady == 0)
  {
    SetProtocol_Command(); // ISO 15693 settings
    delay(100);
  }
  else if (NFCReady == 1)
  {
    for (int i=0; i<3; i++) {
    Inventory_Command(); // sensor in range?
    if (NFCReady == 2)
      break;
      delay(1000);
    } 
    if (NFCReady == 1) {
      NFCReady = 0;
      delay(100); 
    }
  }
  else
  {
    int glucoseValue = (int)Read_Memory();
    
    Serial.println(glucoseValue);  
    lcd.setCursor(0, 0);
    lcd.print("Valore glucosio:");
    lcd.setCursor(0, 1);
    lcd.print(glucoseValue);
    espSerial.println(glucoseValue);
    delay(10000);
    NFCReady = 0;
    lcd.clear();
    
  }
} 

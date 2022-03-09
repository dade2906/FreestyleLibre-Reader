#include <WiFi.h>
#include <HTTPClient.h>

const char * ssid = "King";
const char * password = "king2906";

String server = "http://maker.ifttt.com";
String eventName = "sensore_letto";
String IFTTT_Key = "n18qfovcC7wW4bf_ywrBlpK0TOMJgbOkYpeH1WCZfIL";
String IFTTTURl = "https://maker.ifttt.com/trigger/sensore_letto/with/key/n18qfovcC7wW4bf_ywrBlpK0TOMJgbOkYpeH1WCZfIL";

int guarda= 0;
int value2=0;
int value3=0;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.write(".");
  }
  Serial.write("Connected");
}

void sendDataToSheet(String server, String eventName, String IFTTT_Key, String value1 ) {
  String url = server +"/trigger/" + eventName + "/with/key/" + IFTTT_Key + "?value1=" + value1;
  
  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();
  http.end();
}

void loop(){
  if(Serial.available()){
    //Serial.write(Serial.readString());
    delay(1000);
    guarda=Serial.parseInt();
    if(guarda != 0){
      sendDataToSheet(server, eventName, IFTTT_Key, String(guarda));
      guarda = 0;
      }  
  }  
}

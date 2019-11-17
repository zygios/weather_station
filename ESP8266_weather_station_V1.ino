#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <BME280I2C.h>
#include <BH1750.h>

#define DEBUG

#ifdef DEBUG
  #define debug(x)     Serial.print(x)
  #define debugln(x)   Serial.println(x)
#else
  #define debug(x)     // define empty, so macro does nothing
  #define debugln(x)
#endif

const char* ssid = "xxx";
const char* password = "xxx";
const char* mqtt_server = "192.168.1.xx";
const int mttq_port = 1883;       
const char* dom_in = "domoticz/in";
const char* Mqtt_clientid = "ESP8266Weather";
#define MQTT_MAX_PACKET_SIZE 128
char msgToPublish[MQTT_MAX_PACKET_SIZE + 1];
//Sensors upload to Domoticz period 
unsigned long SEND_FREQUENCY = 60000*1; // Minimum time between send (in milliseconds). We don't wnat to spam the gateway.
long previousMillis = 0;

WiFiClient espClient;
PubSubClient client(espClient);

//BMP280
const int sclPin = D4;
const int sdaPin = D3;
BME280I2C bme; // I2C
float temp(0), hum(0), pres(0);
float SLpressure_mB;
const int ELEVATION = 139;  //Vilnius elvation from sea level
#define temp_domIdx 159

//BH1750
BH1750 lightMeter;
#define light_domIdx 158

//rain gauge
#define RainGaug D8
#define Rain_domIdx 157
const int Bucket_Size = 279; // bucket size to trigger tip count 
const int Rain_interval = 10;
unsigned long lastSend;               //Last Send millis()
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain
volatile unsigned int rainBucket = 0;


// Anemometer section
#define WindSpPIN D6
#define WinDir A0
int WinVal = 0;
#define NUMDIRS 16
const char *strWinDirVals[NUMDIRS] = { "N", "NNE", "NE", "ENE",  "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" }; //for domoticz for showing value
const int bearWinDirVals[NUMDIRS] = { 0, 22, 45, 67,  90, 112, 135, 157, 180, 202, 225, 247, 270, 292, 315, 337 }; //for domoticz to log win direction
#define windDir_domIdx 160

#define REQUEST_RATE 5000 // in milliseconds - sample rate, 5000 default
const float WindTo_mps = 0.6667;  
unsigned long PulseTimeNow = 0; // Time stamp (in millisecons) for pulse triggering the interrupt
float WindSpeed_mps, WindSpeed_Hz;
byte windspdavg[120]; //120 bytes to keep track of 2 minute average
#define WIND_DIR_AVG_SIZE 120
int winddiravg[WIND_DIR_AVG_SIZE]; //120 ints to keep track of 2 minute average
float windspd_avg2m = 0; // [2 minute average wind speed ]
float windgust_10m = 0; // [past 10 minutes wind gust 
volatile unsigned long PulseTimeLast = 0; // Time stamp of the previous pulse
volatile unsigned long PulsesCumulatedTime = 0; // Time Interval since last wind speed computation 
volatile unsigned long PulsesNbr = 0;           // Number of pulses since last wind speed computation 
long lastWindCheck = 0;
float MaxWind = 0.0; // Max wind speed 
float WindGust = 0.0;
unsigned long lastupdate = 0;  // timer value when last  update was done
uint32_t timer = 0;            // a local timer

volatile unsigned long raintime, rainlast, raininterval, rain;
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  debugln();
  debug("Connecting to ");
  debugln(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    for(int i = 0; i<500; i++){
      delay(1);
    }
    debug(".");
  }
  debugln("WiFi connected");
  debugln("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  debug("Message arrived [");
  Serial.print(topic);
  debug("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  debug(" ");
}

byte windir(){
  WinVal = analogRead(WinDir);
  if(WinVal >= 1024) return 0; //N
  if(WinVal >= 831 && WinVal < 839) return 0; //N
  if(WinVal >= 446 && WinVal < 454) return 1; //NNE
  if(WinVal >= 504 && WinVal < 512) return 2; //NE
  if(WinVal >= 94 && WinVal < 102)  return 3; //ENE
  if(WinVal >= 104 && WinVal < 112) return 4; //E
  if(WinVal >= 74 && WinVal < 82)   return 5; //ESE
  if(WinVal >= 206 && WinVal < 214) return 6; //SE
  if(WinVal >= 144 && WinVal < 152) return 7; //S
  if(WinVal >= 318 && WinVal < 326) return 8; //S
  if(WinVal >= 272 && WinVal < 280) return 9; //SSW
  if(WinVal >= 675 && WinVal < 683) return 10; //SW
  if(WinVal >= 644 && WinVal < 652) return 11; //WSW
  if(WinVal >= 988 && WinVal < 996) return 12; //W
  if(WinVal >= 873 && WinVal < 881) return 13; //WNW
  if(WinVal >= 930 && WinVal < 938) return 14; //NW
  if(WinVal >= 750 && WinVal < 758) return 15; //NNW
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    debug("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(Mqtt_clientid)) {
      debugln("connected");
      // ... and resubscribe
      client.subscribe(dom_in);
    } else {
      debug("failed, rc=");
      debug(client.state());
      debugln(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      for(int i = 0; i<5000; i++){
        delay(1);
      }
    }
  }
}

void sendMQTTPayload(String payload) // Sends MQTT payload to the Mosquitto server running on a Raspberry Pi.
// Mosquitto server deliveres data to Domoticz server running on a same Raspberry Pi
{
  // Convert payload to char array
  payload.toCharArray(msgToPublish, payload.length()+1);

  //Publish payload to MQTT broker
  if (client.publish(dom_in, msgToPublish))
  {
    debug(F("Following data published to MQTT broker: "));
    debug(dom_in);
    debug(F(" "));
    debugln(payload);
  }
  else
    debugln(F("Publishing to MQTT broker failed..."));
}

void setup() {
  
  Serial.begin(115200);
  debugln(F("Starting weather station device setup."));

  Wire.begin(sdaPin, sclPin);
  while(!bme.begin())
  {
    debugln(F("Could not find BME280 sensor!"));
    delay(1000);
  }
  
  lightMeter.begin();
  pinMode(RainGaug, INPUT);
  attachInterrupt(digitalPinToInterrupt(RainGaug), rainIRQ, RISING);
  pinMode(WindSpPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WindSpPIN), AnemometerPulse, RISING);
  PulseTimeLast = micros();
  
  setup_wifi();                   // Connect to wifi
  client.setServer(mqtt_server, mttq_port);
  //client.setCallback(callback);
  PulsesCumulatedTime = 0;
  PulsesNbr = 0;
  seconds = 0;
  lastSecond = millis();

  debugln(F("Device setup ended."));
}

void rainIRQ()
{
  debugln("Rain sensor tipped");
  raintime = millis();
  raininterval = raintime - rainlast; 
  if (raininterval > Rain_interval){
    rainBucket += Bucket_Size;
    //rainHour[minutes] += Bucket_Size;  
    rainlast = raintime;
    debug("rainBucket: ");
    debugln(rainBucket);  
  }  
  
}

void AnemometerPulse() 
{
  noInterrupts(); // disable global interrupts
  PulseTimeNow = micros();
  if (PulseTimeNow - PulseTimeLast > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    PulseTimeLast = PulseTimeNow; //Grab the current time
    PulsesNbr++;
  }
  interrupts();              // Re-enable Interrupts
}

void get_wind_speed()    // START Anemometer section
{
  if(PulsesNbr > 0){
    float deltaTime = millis() - lastWindCheck; //750ms
    deltaTime /= 1000.0; //Covert to seconds
    WindSpeed_Hz = PulsesNbr/deltaTime; 
    WindSpeed_mps = WindSpeed_Hz*WindTo_mps;
    PulsesNbr = 0;
    lastWindCheck = millis();
  }else{
    WindSpeed_mps = 0;
  }
   
  debug("WindSpeed_mps: ");
  debugln(WindSpeed_mps);
}

void loop() {
  unsigned long now = millis();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (millis() - lastSecond >= 1000)
  {
    lastSecond += 1000;

    //Take a speed and direction reading every second for 2 minute average
    if (++seconds_2m > 119) seconds_2m = 0;

    windspdavg[seconds_2m] = (int)WindSpeed_mps;
    
    if (++seconds > 59)
    {
      seconds = 0;

      if (++minutes > 59) minutes = 0;
      if (++minutes_10m > 9) minutes_10m = 0;
    }

    
  }


  //----------------------------------------------------------------------  
  // Sample at REQUEST_RATE, default = 5 seconds
  //----------------------------------------------------------------------
  if ( ( millis()-lastupdate ) > REQUEST_RATE ){
      lastupdate = millis();
      timer = lastupdate;
      get_wind_speed();   // Get Anemometer data 
  }
  
  if (now - lastSend > SEND_FREQUENCY) {
    debug("Rain: ");
    debug(rainBucket);
    debugln("mm/hr");
  
    float lux = lightMeter.readLightLevel();
    debug("Light: ");
    debug(lux);
    debugln(" lx");
    
    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  
    bme.read(pres, temp, hum, tempUnit, presUnit);
    SLpressure_mB = ((pres/pow((1-((float)(ELEVATION))/44330), 5.255))/100.0);
    debug("Temp: ");
    debug(temp);
    debug("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
    debug("\t\tHumidity: ");
    debug(hum);
    debug("% RH");
    debug("\t\tPressure: ");
    debug(SLpressure_mB);
    debugln("hPa");
    

    //sending all data
    //sending rain data
    String payload = "{ \"idx\" : "+ String(Rain_domIdx) +", \"nvalue\" : 0, \"svalue\" : \"" + String(rainBucket/1000) + ";" + String(rainBucket/1000) + "\"}";
    sendMQTTPayload(payload);
    rainBucket=0;
    //sending light data
    payload = "{ \"idx\" : "+ String(light_domIdx) +", \"svalue\" : \"" + String(lux) + "\"}";
    sendMQTTPayload(payload);
    //sending temperature/humidity/bar data
    payload = "{ \"idx\" : "+ String(temp_domIdx) +",\"nvalue\" : 0, \"svalue\": \"" + String(temp) + ";" +  String(hum) + ";0;" + String(SLpressure_mB) +";0\"}";
    sendMQTTPayload(payload);
    //sending wind data
    byte WinDir = windir();
    debugln(WinDir);
    payload = "{ \"idx\" : "+ String(windDir_domIdx) +",\"nvalue\" : 0, \"svalue\": \"" + String(bearWinDirVals[WinDir]) + ";" +  String(strWinDirVals[WinDir]) + ";" + String(WindSpeed_mps*10) + ";" + String(WindGust*10) +";" + String(temp) + ";"+String(temp)+"\"}";
    sendMQTTPayload(payload);
    
    lastSend=now;
  }
}




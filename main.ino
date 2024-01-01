#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6dCJ9EEXH"
#define BLYNK_TEMPLATE_NAME "HydroMonitoringSystem"
#define BLYNK_AUTH_TOKEN "qVkH9wZIAtrMsyEX0Kkvh0tdN8In4NGC"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
char ssid[] = "Project";
char pass[] = "12345678";


//DS18B20 Start
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 27
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//DS18B20 end

//DHT11 Start
#include <DHT.h>
#define DHTPIN 32      // GPIO pin where the DHT11 is connected
#define DHTTYPE DHT11   // DHT 11 sensor type
DHT dht(DHTPIN, DHTTYPE);
//DHT11 End

//TDS stard
#define TdsSensorPin 33
#define VREF 5.0              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 16;       // current temperature for compensation

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

//TDS END

//Light
#define Light 4
// Fan 
#define Fan 22

BLYNK_WRITE(V3)
{
  if(param.asInt()==1){
    digitalWrite(Light, HIGH);
  }
  else{
    digitalWrite(Light, LOW);
  }
}

BLYNK_WRITE(V4)
{
  if(param.asInt()==1){
    digitalWrite(Fan, HIGH);
  }
  else{
    digitalWrite(Fan, LOW);
  }
}

BLYNK_CONNECTED()
{
  Blynk.syncVirtual(V3); 
  Blynk.syncVirtual(V4);  
}

//Air Pump
#define airPump 21

//pump
#define alwPump 19
#define ppmPump 18
#define flotPump 5

//Float
#define floatSensor1 25
#define floatSensor2 26


void setup()
{
  // Debug console
  Serial.begin(115200);
  sensors.begin();
  dht.begin();
  // TDS
  pinMode(TdsSensorPin,INPUT);
  //Relay Device
  pinMode(Light, OUTPUT); // light 4
  pinMode(airPump, OUTPUT); //Air Pump 21
  pinMode(Fan, OUTPUT); // Fan 22
  pinMode(alwPump, OUTPUT); // 19
  pinMode(ppmPump, OUTPUT); // 18
  pinMode(flotPump, OUTPUT); // 5

  pinMode(floatSensor1, INPUT_PULLUP);
  pinMode(floatSensor2, INPUT_PULLUP);



  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop(){
  digitalWrite(airPump, HIGH); 
  delay(100);
  digitalWrite(alwPump, HIGH); 
  delay(100);


//TDS Start
 static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }   
  
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;     
      //convert voltage value to tds value
      tdsValue=((133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5) / 1000;
    }
    Serial.print("TDS Value:");
    Serial.print(tdsValue,0);
    Serial.println("ppm");
    delay(100);
  }
  //TDS END


  //DS18B20 Start
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);

  if (temperatureC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature: ");
    Serial.print(temperatureC);
    Serial.println(" °C");
  } else {
    Serial.println("Error reading temperature");
  }
  delay(100); // Adjust the delay as needed
  //DS18B20END

  //DHT11 Start
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%\t");

  /*Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");*/

  //DHT11 End


  if (tdsValue < 1600){
    digitalWrite(ppmPump, HIGH); 
  }
  else if (tdsValue > 2000){
    digitalWrite(ppmPump, LOW); 
  }

  delay(100);

  int buttonState1 = 1; //reads pushbutton status
  buttonState1 = digitalRead(floatSensor2);

  if (buttonState1 == HIGH) {
    Serial.println("WATER LEVEL - High");  
  }
  else {
    //digitalWrite(led, HIGH);
    Serial.println("WATER LEVEL - LOW");
    digitalWrite(flotPump, HIGH); 
  }

  delay(100);

  int buttonState2 = 1;
  buttonState2 = digitalRead(floatSensor1);
  if (buttonState2 == HIGH) {
    Serial.println("WATER LEVEL - HIGH");
    digitalWrite(flotPump, LOW); 
  }
  
  delay(100);

  Blynk.virtualWrite(V0, temperatureC);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, tdsValue);
  
  Blynk.run();
}


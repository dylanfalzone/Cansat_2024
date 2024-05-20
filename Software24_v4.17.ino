//Payload Flight Software 2024 CanSat by Max Epstein, Yashas Shivaram, and Dylan Falzone, 
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define ACTUATOR_EXTEND_POS_PIN 12
#define ACTUATOR_RETRACT_POS_PIN 11
#define RADIO_DIN_PIN 10
#define RADIO_DOUT_PIN 9
#define GPS_RX_PIN 8
#define GPS_TX_PIN 7 
#define AUDIO_BEACON_POS_PIN 6
#define AUDIO_BEACON_NEG_PIN 5
#define CAMERA_MOSFET_GATE_PIN 4
#define SERVO_PIN 3
#define PX4_ADDRESS 0x28

float seaLevel;

uint16_t PACKET_COUNT = 0;
char MISSION_TIME[7] = "00:00:00";
uint8_t MODE = 70;
uint8_t STATE = 'S';
float ALTITUDE = 0.00;
float AIRSPEED = 0.00;
uint8_t HS_DEPLOYED = 'N';
uint8_t PC_DEPLOYED = 'N';
String GPS_TIME = "00:00:00";
String CMD_ECHO = "";
int n = 0;

char radioSend[110]; // Character array to store the compiled data
char buffer[40]; // Temporary buffer for converting values to strings

float apogee = 600;
boolean camRec = false;
boolean simAct = false;
boolean sendTelemetry = false;

// Create objects for sensors
Adafruit_BMP280 bmp; // I2C
SoftwareSerial RADIO_USART(RADIO_DOUT_PIN,RADIO_DIN_PIN); //USART for radio
//SoftwareSerial GPS_USART(GPS_RX_PIN,GPS_TX_PIN);   //USART for GPS
//Adafruit_GPS GPS(&GPS_USART);
#define GPSSerial Serial
Adafruit_GPS GPS(&GPSSerial); // UART
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);


void setup() {
  //Serial.begin(115200);
  GPS.begin(9600);
  RADIO_USART.begin(19200);
  Wire.begin();
  bmp.begin();
  bno.begin();


  // Initialize Adafruit Ultimate GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  //Servo 
  pinMode(SERVO_PIN, OUTPUT); // Configure SERVO_PIN as an Output
  while(n < 355){ //must be 355
    digitalWrite(SERVO_PIN, HIGH);    // Set pin HIGH
    delayMicroseconds(400);  // Wait for 1ms
    
    digitalWrite(SERVO_PIN, LOW);     // Set pin LOW
    delayMicroseconds(19600); // Wait for 19ms
    n++;
  }
  

  //Sound Beacon 
  pinMode(AUDIO_BEACON_POS_PIN, OUTPUT); // pin is speaker positive
  digitalWrite(AUDIO_BEACON_NEG_PIN, LOW); 
  pinMode(AUDIO_BEACON_NEG_PIN, OUTPUT); //pin is speaker ground
  digitalWrite(AUDIO_BEACON_POS_PIN, LOW); 

  //Camera
  pinMode(CAMERA_MOSFET_GATE_PIN, OUTPUT); // pin is camera mosfet gate. High will activate mosfet. 
  digitalWrite(CAMERA_MOSFET_GATE_PIN, LOW);


}
void loop() {
  //get new incomming
  if (RADIO_USART.available() > 0){
    CMD_ECHO =RADIO_USART.readString();
    //Serial.println(CMD_ECHO);
    CMD_ECHO = CMD_ECHO.substring(9);


        
      if (CMD_ECHO[2] == 'L') {
        seaLevel = bmp.readAltitude(1013.25);
        STATE = 'S';
      }
      else {
        switch (CMD_ECHO[4]) {
          case 'N':
            sendTelemetry = true;
            break;
          case 'F':
            sendTelemetry = false;
            break;
          case 'E':
            //MODE = 'S';
            break;
          case 'A':
            simAct = true;
            break;
          case 'D':
            //MODE = 'F';
            simAct = false;
            break;
          default:
            if (CMD_ECHO[5] == 'N')
              tone(6, 700);
            else if(CMD_ECHO[0] == 'S')
              ALTITUDE = bmp.readAltitude(CMD_ECHO.substring(4).toFloat());
            else
              tone(6,700, 1);
            break;
        }
      }
      
      if(simAct == true){
        MODE = 83;
      }
      else{
        MODE = 70;
      }


  

      if (CMD_ECHO[1] == 'X'){
        CMD_ECHO.remove(2,1);
      }
      else {
        CMD_ECHO.remove(3,1);
      }

  }//end of if avaiable statement
  
  char c = GPS.read();
  
    if (GPS.newNMEAreceived()) {
  
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }

  if (millis()/1000 - PACKET_COUNT > 1 && sendTelemetry && millis()%1000 == 0) {
    PACKET_COUNT += 1; //increase the packet count by 1

    if (simAct == false){
      ALTITUDE = (bmp.readAltitude(1013.25)-seaLevel);
    }
    if (ALTITUDE > apogee){
      apogee = ALTITUDE;
    }

    uint32_t runtime = millis();
    MISSION_TIME[0] = ((runtime/60000)%60)/10 +'0'; //minutes high
    MISSION_TIME[1] = ((runtime/60000)%60)%10 +'0'; //minutes low
    MISSION_TIME[3] = ((runtime/1000)%60)/10 + '0'; //seconds high
    MISSION_TIME[4] = ((runtime/1000)%60)%10 + '0'; //seconds low
    MISSION_TIME[6] = ((runtime/3600000)%100)/10 +'0'; //cs high
    MISSION_TIME[7] = ((runtime/3600000)%100)%10 +'0'; //cs low
    MISSION_TIME[8] = '\0';

    //DO ALL LOGIC HERE
    if (STATE == 'S' && PACKET_COUNT > 1){
      STATE = 'W';
    }
    else if (ALTITUDE > 60 && STATE == 'W'){
      STATE = 'A';
      digitalWrite(CAMERA_MOSFET_GATE_PIN, HIGH); //Turn on camera
    }
    else if(apogee <= ALTITUDE){
      STATE = 'R';
    }
    else if (ALTITUDE < 500 && STATE == 'R'){//payload is released from can, heatshield deployed
      STATE = 'P' ;   
      HS_DEPLOYED = 'P';

    }
    else if (ALTITUDE < 200 && STATE == 'P'){//DESCENT
      STATE = 'D';
      //parachute:
      PC_DEPLOYED = 'C';
      n = 0;
      while(n < 355){ //must be 355
      digitalWrite(SERVO_PIN, HIGH);    // Set pin D5 HIGH
      delayMicroseconds(2500);  // Wait for 1ms
      
      digitalWrite(SERVO_PIN, LOW);     // Set pin D5 LOW
      delayMicroseconds(20000 - 2500); // Wait for 19ms
      n++;
      }

    }
    else if(ALTITUDE < 60 && STATE == 'D'){//LANDED
      STATE = 'L';

      //Audio beacon:
      tone(6, 700);  // Change the frequency as desired

      digitalWrite(CAMERA_MOSFET_GATE_PIN, LOW); //Turn off camera
    }

    // Pitot tube code
    Wire.requestFrom(PX4_ADDRESS, 4);
    if (Wire.available()){
      float diffPressure = (Wire.read() << 8 | Wire.read()) / 10000.0; // Pascal
      float temperature = (Wire.read() << 8 | Wire.read()) / 1000.0; // Celsius
      AIRSPEED = sqrt((2.0 * diffPressure) / (101325.0 / (287.05 * (temperature + 273.15)))); // m/s
      //Serial.print("Airspeed: ");
      //Serial.print(AIRSPEED);
      //Serial.println(" m/s");
    }
    

    
    GPS_TIME[0] = GPS.hour/10 + '0';
    GPS_TIME[1] = GPS.hour%10 + '0';
    GPS_TIME[3] = GPS.minute/10 + '0';
    GPS_TIME[4] = GPS.minute%10 + '0';
    GPS_TIME[6] = GPS.seconds/10 + '0';
    GPS_TIME[7] = GPS.seconds%10 + '0';
    
    // Read and print data from BNO055
    sensors_event_t event;
    bno.getEvent(&event);   

    
    strcpy(radioSend, "2031");
    strcat(radioSend, ",");
    //strcat(radioSend, MISSION_TIME.c_str());
    strcat(radioSend, MISSION_TIME);
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(PACKET_COUNT, 1, 0, buffer));
    strcat(radioSend, ",");
    buffer[0] = MODE; // Convert STATE to a character
    buffer[1] = '\0'; // Null-terminate the buffer
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    buffer[0] = STATE; // Convert STATE to a character
    buffer[1] = '\0'; // Null-terminate the buffer
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(ALTITUDE, 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(AIRSPEED, 0, 2, buffer));
    strcat(radioSend, ",");
    buffer[0] = HS_DEPLOYED; // Convert STATE to a character
    buffer[1] = '\0'; // Null-terminate the buffer
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    buffer[0] = PC_DEPLOYED; // Convert STATE to a character
    buffer[1] = '\0'; // Null-terminate the buffer
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(bmp.readTemperature(), 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(bmp.readPressure() / 100.0F, 0, 2, buffer));
    strcat(radioSend, ",");

    //****************************************************
    //CHANGE A0 to whatever pin is connected to battery
    //****************************************************
    strcat(radioSend, dtostrf((analogRead(A0) * 5.0) / 1024.0, 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, GPS_TIME.c_str());
    //strcat(radioSend, GPS_TIME);
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(GPS.altitude, 0, 0, buffer));
    strcat(radioSend, ",");
    dtostrf(GPS.latitude / 100.0, 7, 4, buffer);
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    dtostrf(GPS.longitude / 100.0, 7, 4, buffer);
    strcat(radioSend, buffer);
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(GPS.satellites, 0, 0, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(event.orientation.x, 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(event.orientation.y, 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, dtostrf(event.orientation.z, 0, 2, buffer));
    strcat(radioSend, ",");
    strcat(radioSend, CMD_ECHO.c_str());

    if (sendTelemetry == true){
      RADIO_USART.println(radioSend); 
      //Serial.println(radioSend);
    }
  }
}

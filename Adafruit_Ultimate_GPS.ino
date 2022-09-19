#include <Adafruit_GPS.h>
#include <vector>
#include <string>

//===========================================================================
static const int ledPin = 13;
bool ledOn = false;

// Define the harware serial port on pins 0/1.
#define SerialGPS Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&SerialGPS);

std::vector<std::string> GpsFixQuality{"invalid", "SPS", "DGPS", "PPS", "RTK", "fRTK", "dead reck", "manual", "sim"};

static const float KNOTS2MPS = 0.514;

static const bool LOG_GGA = true; // Can only use RMC at 10Hz at 9600 baud
static const bool displayLatLonOnly = false;

// This only needs to be called one time if the battery is installed
//uint32_t BAUD_RATE = 9600;
uint32_t BAUD_RATE = 19200;
void changeBaudRate()
{
  SerialGPS.begin(9600); // The GPS module's default baud is 9600
  
  if( BAUD_RATE == 57600 )      GPS.sendCommand("$PMTK251,57600*2C");  //set baud rate to 57600
  else if( BAUD_RATE == 38400 ) GPS.sendCommand("$PMTK251,38400*27");  //set baud rate to 38400
  else if( BAUD_RATE == 19200 ) GPS.sendCommand("$PMTK251,19200*22");  //set baud rate to 19200
  else                          GPS.sendCommand("$PMTK251,9600*17");   //set baud rate to 9600 (default)
  
  SerialGPS.end();  
}



//===========================================================================
void setup() 
{
  pinMode(ledPin, OUTPUT);

  // Initialize the serial monitor port
  Serial.begin(115200); 
  

  //--------------------------------------
  // GPS Serial baud rate  
  //changeBaudRate(); // This only needs to be called one time if the battery is installed
  SerialGPS.begin(BAUD_RATE);
 

  //--------------------------------------
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  if( LOG_GGA ) GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  // NOTE: Does not include altitude data
  if( !LOG_GGA ) GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
 
  // Set the update rate
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);// 1 Hz update rate  
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);// 5 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate - for 9600 baud you'll can only output one message (RMC), if you want more you need to up the baud rate
  
  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  delay(500);

  // Ask for firmware version
  //SerialGPS.println(PMTK_Q_RELEASE);
}



//===========================================================================
uint32_t timer = millis();
void loop() 
{ 
  //-----------------------------------------------
  // Passthrough
  //-----------------------------------------------
  // If GPS data is available
//  if( SerialGPS.available() ) 
//  {
//    Serial.write(SerialGPS.read()); // Send it to the serial monitor
//  }
//
//  // If data is sent to the serial monitor
//  if( Serial.available() ) 
//  {
//    SerialGPS.write(Serial.read()); // send it to the GPS module
//  }

  //-----------------------------------------------
  // Parse
  //-----------------------------------------------
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  
  if( GPS.newNMEAreceived() ) 
  {
    // debug print the message recieved
    //Serial.print( GPS.lastNMEA() ); 
    
    if( !GPS.parse(GPS.lastNMEA()) ) // parse() sets the newNMEAreceived() flag to false
    {
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) 
  {
    // Blink
    ledOn = !ledOn;
    digitalWrite(ledPin, ledOn);
  
    timer = millis(); // reset the timer
    
//    Serial.print("\nTime: ");
//    if (GPS.hour < 10) { Serial.print('0'); }    
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    if (GPS.minute < 10) { Serial.print('0'); }
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    if (GPS.seconds < 10) { Serial.print('0'); }
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    if (GPS.milliseconds < 10) {Serial.print("00");} 
//    else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {Serial.print("0");}
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);

    if( displayLatLonOnly)
    {
      Serial.print(GPS.latitudeDegrees,11); Serial.print(", ");     
      Serial.println(GPS.longitudeDegrees,11);
    }
    else
    {
      Serial.print("Fix: ");
      if( (int)GPS.fix ) Serial.println("True");
      else Serial.println("False");
      
      if(LOG_GGA) {Serial.print("Fix quality: "); Serial.println( GpsFixQuality[(int)GPS.fixquality].c_str() );}
      
      if( GPS.fix ) 
      {
        // decimal
        // places   degrees          distance
        // -------  -------          --------
        // 0        1                111  km
        // 1        0.1              11.1 km
        // 2        0.01             1.11 km
        // 3        0.001            111  m
        // 4        0.0001           11.1 m
        // 5        0.00001          1.11 m
        // 6        0.000001         11.1 cm
        // 7        0.0000001        1.11 cm
        // 8        0.00000001       1.11 mm
        Serial.print("Position: ");
        Serial.print(GPS.latitudeDegrees,11); Serial.print(", ");     
        Serial.println(GPS.longitudeDegrees,11);
        
        if(LOG_GGA)
        {
          Serial.print("Altitude MSL (m): "); Serial.println(GPS.altitude);
          Serial.print("Geoid Height (m): "); Serial.println(GPS.geoidheight);
        }
        
        Serial.print("Speed (m/s): "); Serial.println(GPS.speed*KNOTS2MPS);
        Serial.print("Course (deg): "); Serial.println(GPS.angle);
        //Serial.print("Magnetic Var (deg): "); Serial.println(GPS.magvariation); // should always be zero --> GPS always points true North
        
        if(LOG_GGA) Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        
        Serial.println("");
      }
    }
  }
}

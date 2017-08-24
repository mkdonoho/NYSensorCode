// This #include statement was automatically added by the Particle IDE.
#include "kSeries.h"
#include <SparkIntervalTimer.h>
#include <ParticleSoftSerial.h>
#include <Adafruit_BMP085.h>
#include <adafruit-sht31.h>
#include <SdFat.h>
#include "SdFat.h"
#include <Wire.h>
#include <string.h>

//*****************WITHOUT WIFI PRECAUTIONS*******************************************
SYSTEM_MODE(SEMI_AUTOMATIC)
SYSTEM_THREAD(ENABLED)

const uint32_t msRetryDelay = 5*60000; // retry every 5min
const uint32_t msRetryTime  =   30000; // stop trying after 30sec

bool   retryRunning = false;
Timer retryTimer(msRetryDelay, retryConnect);  // timer to retry connecting
Timer stopTimer(msRetryTime, stopConnect);     // timer to stop a long running try


// Pick an SPI configuration.
// See SPI configuration section below (comments are for photon).
#define SPI_CONFIGURATION 0
//------------------------------------------------------------------------------
// Setup SPI configuration.
#if SPI_CONFIGURATION == 0
// Primary SPI with DMA
// SCK => A3, MISO => A4, MOSI => A5, SS => A2 (default)
SdFat sd;
const uint8_t chipSelect = SS;
#elif SPI_CONFIGURATION == 1
// Secondary SPI with DMA
// SCK => D4, MISO => D3, MOSI => D2, SS => D1
SdFat sd(1);
const uint8_t chipSelect = D1;
#elif SPI_CONFIGURATION == 2
// Primary SPI with Arduino SPI library style byte I/O.
// SCK => A3, MISO => A4, MOSI => A5, SS => A2 (default)
SdFatLibSpi sd;
const uint8_t chipSelect = SS;
#elif SPI_CONFIGURATION == 3
// Software SPI.  Use any digital pins.
// MISO => D5, MOSI => D6, SCK => D7, SS => D0
SdFatSoftSpi<D5, D6, D7> sd;
const uint8_t chipSelect = D0;
#endif  // SPI_CONFIGURATION
//------------------------------------------------------------------------------

File myFile;
String fileName = "SiteSensorDataLog.csv";
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_BMP085 bmp;
kSeries kSensor(0,0);

double co2 =0;
double TRHtemp =0;
double TRHhumid= 0;
double BMP =0 ;
double BMPtemp =0;
double fileSize=0;


void setup() 
{
    Particle.connect();
    Serial1.begin(9600);
    
    
    // Initialize SdFat or print a detailed error message and halt
    // Use half speed like the native library.
    // Change to SPI_FULL_SPEED for more performance.
    if (!sd.begin(chipSelect, SPI_HALF_SPEED)) 
    {
       Particle.publish("Error", "SD Begin Error." );
    }

    if(!sd.exists(fileName))
    {
        // open the file for write at end like the "Native SD library"
        if (!myFile.open(fileName, O_RDWR | O_CREAT | O_AT_END)) 
        {
            Particle.publish("Error","SD Write Error.");
        }
        else 
        {// if the file opened okay, write to it:
            myFile.println("DateTime, TRH Temp(C), TRH Humidity(%), BMP(hPa), BMP Temp (C), CO2(ppm)");
            //Particle.publish("fileSize", String(myFile.fileSize()) );
            fileSize = myFile.fileSize();
        }
    }
    // close the file:
    myFile.close();
    

    //TRH
    sht31.begin(0x44);
    if(!sht31.begin(0x44)) 
        Spark.publish("Error","TRH NOT Found.");
    else
        Spark.publish("Success","TRH Found.");
    
    //BMP
    if(!bmp.begin()) 
        Spark.publish("Error","BMP NOT Found.");
    else
        Spark.publish("Success","BMP Found.");
        
    Particle.variable("CO2", co2);
    Particle.variable("TEMP", TRHtemp);
    Particle.variable("HUMIDITY", TRHhumid);
    Particle.variable("BMP", BMP);
    Particle.variable("BMPtemp", BMPtemp);
  
}//end setup()

void loop() 
{
    if (!retryRunning && !Particle.connected())
    { // if we have not already scheduled a retry and are not connected
        stopTimer.start();         // set timeout for auto-retry by system
        retryRunning = true;
        retryTimer.start();        // schedule a retry
    }//end if not connected.
    
   
    String dataString =""; 
    
    co2 = kSensor.getCO2('P');
    
    TRHtemp = sht31.readTemperature();
    TRHhumid = sht31.readHumidity();
   
    if(bmp.begin())
    {
        BMP = bmp.readPressure();
        BMPtemp = bmp.readTemperature();
    }
    
    BMP = round(BMP*10)/10;
    BMPtemp = round(BMPtemp*10)/10;
    
    dataString += Time.timeStr();
    dataString += ",";
    dataString.concat( TRHtemp );
    dataString += ",";
    dataString.concat( TRHhumid );
    dataString += ",";
    dataString +=  String(BMP);
    dataString += ",";
    dataString.concat( BMPtemp );
    dataString += ",";
    dataString.concat( co2 );
    
    
     //Open the datafile and record the data is possible, else report error
    if (!myFile.open(fileName, O_RDWR | O_CREAT | O_AT_END)) 
    {
        Spark.publish("Error", "SD WRITE ERROR.");
    }
    else
    {
        myFile.println(dataString);
        fileSize = myFile.fileSize();
    }
      // close the file:
    myFile.close();
    publishData();

    delay(30000);
}//end loop()

void publishData()
{
    String data = String::format("{ \"temperature\": \"%f\", \"relativeHumidity\": \"%f\", \"co2\": \"%i\", \"barometricPressure\": \"%f\", \"bmpTemperature\": \"%f\", \"fileSize\": \"%f\", \"GreenhouseId\": \"NYGH_1\", \"IsControl\": \"true\" }", float(TRHtemp), float(TRHhumid), int(co2), float(BMP), float(BMPtemp), float(fileSize));
    Particle.publish("SiteSensor", data, PRIVATE);
    
}

//Attempt to reconnect to WiFi
void retryConnect()
{
  if (!Particle.connected())   // if not connected to cloud
  {
    stopTimer.start();         // set of the timout time
    WiFi.on();
    Particle.connect();        // start a reconnectino attempt
  }
  else                         // if already connected
  {
    retryTimer.stop();         // no further attempts required
    retryRunning = false;
  }
}

//If cannot connect to WiFi, stops attempting, continues program.
void stopConnect()
{

    if (!Particle.connected()) // if after retryTime no connection
      WiFi.off();              // stop trying and swith off WiFi
    stopTimer.stop();
}



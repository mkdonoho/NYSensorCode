// This #include statement was automatically added by the Particle IDE.
#include <EmonLib.h>
#include <Adafruit_ADS1X15.h>
#include <SdFat.h>
#include <string.h>
#include <math.h>
#include <Wire.h>

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
String fileName = "CTSensorDataLog.csv";
double Irms1 = 1.0;
double Irms2 = 0.0;
double Irms3 = 0.0;
double Irms4 = 0.0;
double fileSize = 0;


EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;
EnergyMonitor emon4;

int pin1 = A0;
int pin2 = A1;
int pin3 = A6;
int pin4 = A7;

void setup() 
{
    Particle.connect();
    
    emon1.current(A0, 111.1);             // Current: input pin, calibration.
    emon2.current(A1, 111.1);  
    emon3.current(A6, 111.1);  
    emon4.current(A7, 111.1); 
    
    if (!sd.begin(chipSelect, SPI_HALF_SPEED)) 
    {
        Particle.publish("error", "SD Error.");
        //sd.initErrorHalt();
    }

    if(!sd.exists(fileName))
    {
        // open the file for write at end like the "Native SD library"
        if (!myFile.open(fileName, O_RDWR | O_CREAT | O_AT_END)) 
        {
            Particle.publish("error","SD Write Error.");
            //sd.errorHalt("opening file for write failed");
        }
        else
        {   // if the file opened okay, write to it:
            myFile.println("DateTime, CT1(A), CT2(A), CT3(A), CT4(A)");
            fileSize = myFile.fileSize();
            //Particle.publish("fileSize", String(myFile.fileSize()) );
        }
    }
    // close the file:
    myFile.close();
   
    delay(100);
  

   
    Particle.variable("irms1", Irms1);
    Particle.variable("irms2", Irms2);
    Particle.variable("irms3", Irms3);
    Particle.variable("irms4", Irms4);
  
}//end setup

void loop() 
{
    
    if (!retryRunning && !Particle.connected())
    { // if we have not already scheduled a retry and are not connected
        stopTimer.start();         // set timeout for auto-retry by system
        retryRunning = true;
        retryTimer.start();        // schedule a retry
    }//end if not connected.
    
    Irms1 = emon1.calcIrms(1480);  // Calculate Irms only
    Irms2 = emon2.calcIrms(1480);
    Irms3 = emon3.calcIrms(1480);
    Irms4 = emon4.calcIrms(1480);
  
    
    
    //Data string to record on SD
    String dataString = "";
    dataString += Time.timeStr();
    dataString += ",";
    dataString.concat(Irms1);
    dataString += ",";
    dataString.concat(Irms2);
    dataString += ",";
    dataString.concat(Irms3);
    dataString += ",";
    dataString.concat(Irms4);
   
  
    //Open the datafile and record the data is possible, else report error
    if (!myFile.open(fileName, O_RDWR | O_CREAT | O_AT_END)) 
    {
        Particle.publish("error", "SD WRITE ERROR.");
    }
    else
    {
        myFile.println(dataString);
        fileSize = myFile.fileSize();
    }
  
    publishData();
    // close the file:
    myFile.close();
     delay(30010);

}//end loop

void publishData()
{
    String data = String::format("{ \"current1\": \"%f\", \"current2\": \"%f\", \"current3\": \"%f\", \"current4\": \"%f\", \"fileSize\": \"%i\" , \"greenhouseId\": \"NYGH_1\"  }", float(Irms1), float(Irms2), float(Irms3), float(Irms4), int(fileSize));
    Particle.publish("CurrentSensor", data, PRIVATE);
    
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
  }//end else
}//end retryConnect()

//If cannot connect to WiFi, stops attempting, continues program.
void stopConnect()
{

    if (!Particle.connected()) // if after retryTime no connection
      WiFi.off();              // stop trying and swith off WiFi
    stopTimer.stop();
}//end stopConnect()



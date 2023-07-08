#include <sim808.h>
#include <SoftwareSerial.h> 
#include <DFRobot_SIM808.h>
#include <MySQL_Connection.h>
#include <MySQL_Cursor.h>

#define rxPin 11
#define txPin 10

IPAddress server(192,168,43,56);  // MySQL server IP address
int port = 3306;                     // MySQL server port
char user[] = "rafli";                // MySQL username
char password[] = "Hertzman01";      // MySQL password
char db[] = "gps";                   // MySQL database name

SoftwareSerial mySerial(txPin, rxPin);
DFRobot_SIM808 sim808(&mySerial);
char wlat[12];
char wlon[12];

MySQL_Connection conn((Client *)&mySerial);
MySQL_Cursor cursor(&conn);

void setup() {
  mySerial.begin(9600);
  Serial.begin(115200); 
  Serial.println("Starting...");
  
  while (!sim808.init()) {
    delay(1000);
    Serial.print("Sim808 init error\r\n");
  }
  
if( sim808.attachGPS())
{
  Serial.println("Open the GPS power success");
}
else 
{
    Serial.println("Open the GPS power failure");
}
}

void loop() {
  if (sim808.getGPS()) {
    Serial.println(sim808.GPSdata.lat, 6);

    Serial.println(sim808.GPSdata.lon, 6);
  

    float la = sim808.GPSdata.lat;
    float lo = sim808.GPSdata.lon;

    delay(1000);
    SubmitHttpRequest(la, lo);
  }
}

void SubmitHttpRequest(float la, float lon) {


  mySerial.print("AT+HTTPPARA=\"URL\",\"http://localhost/insert_data.php?lat=");
  mySerial.print(wlat);
  mySerial.print("&lon=");
  mySerial.print(wlon);
  mySerial.println("\"");


  mySerial.println("");
  delay(100);
}

void ShowSerialData() {
  while (mySerial.available() != 0) {
    Serial.write(mySerial.read());
  }
}

#include "RTClib.h"
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "VEDirect.h"

//**********************************************************************************
// EPAMEDIA Solarcontrol V1.0
// Steuerung der Beleuchtung abhängig von Sonnenauf-/Untergang
// Berechnung des Sonnenauf-/Untergang auf Basis des Längengrads
// Einstellung des Längegrads mittels DIPSwitch zwisch 9,5-16 Grad
// Geographische Laenge Wien      16.37
// Geographische Laenge Salzburg: 13.05
// Geographische Laenge Dornbirn:  9.74
// Auslesen des Victron MPPT 75/15 und übertragen via GSM zu jdem Modus Wechsel
//
// Jürgen Krammer 10.2019
//**********************************************************************************

// Werte hier anpassen!!!-----------------------------------------------------------

#define withGSMShield true // Version ohne GSM = false mit = true
#define SWVERSION "V.10"   // SW Version 
#define SWVERSIONDS "10"   // SW Version 
#define wait4Connection 15 // Wartezeit in Sekunden nach gsm on
#define LICHT 2   // Pin für Lichtrelais
#define HEIZUNG 3 // Pin für Heizungsrelais
#define LNG0 4    // Pin für längegrad +1
#define LNG1 5    // Pin für längegrad +2
#define LNG2 6    // Pin für längegrad +4
#define RES1 7
#define RES2 8    // Pin für Testmodus
#define GSMPWR 9  //Pin für power on
#define ONE_WIRE_BUS 11  // Pin für Temp Sensor
#define OLED_ADDR   0x3C // Display Adresse
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define maxVEReads 15    // max. Anzahl von Ausleseversuchen des Victron MPPT
#define maxComChecks 15  // max. Anzahl an versuchen mit dem GSM Modul zu kommunizieren
#define maxDataTrans 3   // max. Anzahl an *bertragungsversuchen
//GSM
#define URL "www.raschbach.at/werbetafel/data2mysql.php?"  // IOT Server Adresse
#define APN "a1.net"                                       // APN des verwendeten Provider z.B. "m2m.tele2.com" "a1.net"           
#define USR "ppp@A1plus.at"                                // User für Datenverbindung
#define PW "ppp"                                           // Passwort für Datenverbindung
//Heizung
#define UNTERESCHWELLE 6.0  // Schwelle Heizung ein
#define OBERESCHWELLE 8.0   // Schwelle Heizung aus
#define HEIZUNGAKTIV true   //akt-/deaktivieren mit true/false
//Lichsteuerung
#define OFFSET 0                    // Ein-/Ausschalten um den Offsetwert verzögert bzw. verlängert
#define ZEITZONE 1                  // 0=Weltzeit, 1=Winterzeit, 2=Sommerzeit
#define NACHTABSCHALTUNGANFANG 60   // Anfang Nachtabschaltung um 0 Uhr
#define NACHTABSCHALTUNGENDE 300    // Ende Nachtabschaltung um 5 Uhr

//-------------------------------------------------------------------------------------

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RTC_DS1307 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
VEDirect myve(Serial3);

// 32 bit ints to collect the data from the device
int32_t   VE_fw, VE_v, VE_i, VE_vpv, VE_ppv, VE_cs, VE_err, VE_il, VE_h19, VE_h20, VE_h21, VE_h22, VE_h23;
// Boolean to collect an ON/OFF value
uint8_t VE_load; 
String VEData = "";
String modus="4"; 

void setup()
{
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();
  Serial.begin(9600);
  Serial1.begin(19200); //GSM Shield 
  //Echtzeituhr
  Serial.println("Initiaisierung der Echzeituhr");
  if (! rtc.begin()) 
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }
 sensors.begin(); 
 pinMode(GSMPWR, OUTPUT); // GSM Shield aktivieren   
 pinMode(LICHT, OUTPUT); //Licht an
 pinMode(HEIZUNG, OUTPUT); //Heizung
 pinMode(RES1, INPUT); //reserve
 pinMode(RES2, INPUT); //reserve
 pinMode(LNG0, INPUT); //Region Ost
 pinMode(LNG1, INPUT); //Region Mitte
 pinMode(LNG2, INPUT); //Region West
 digitalWrite(LICHT, HIGH);
 digitalWrite(HEIZUNG, HIGH);
 if (withGSMShield) while(!myve.begin());  // test VE connection
}

void loop()
{
  display.clearDisplay();
  display.setCursor(1,0);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.print("   EPA Solar "); display.println(SWVERSION);
  Serial.println("EPA Solar control "); Serial.println(SWVERSION);
  
  // ------------------------------------------------
  // Temperatur Sensor abfragen und Heizung schalten
  // ------------------------------------------------
  
  double unterSchwelle = UNTERESCHWELLE; // Schwelle Heizung ein
  double obereSchwelle = OBERESCHWELLE; // Schwelle Heizung aus
  bool heizungAktiv = HEIZUNGAKTIV; //akt-/deaktivieren mit true/false
  float tempC;
 
  Serial.print("Temperatur abfragen...");
  sensors.requestTemperatures();
  Serial.println("erledigt"); 
  tempC = sensors.getTempCByIndex(0);
  if(tempC != DEVICE_DISCONNECTED_C){
    Serial.print("Temperatur: "); Serial.println(tempC);
    display.print("Temperatur: "); display.print(tempC); display.println(" C"); 
    if (digitalRead(HEIZUNG)==HIGH && heizungAktiv) {
      if (tempC<unterSchwelle) digitalWrite(HEIZUNG, LOW);
    }
    else{
       if (tempC>obereSchwelle) digitalWrite(HEIZUNG, HIGH);
    }      
  }
  else
  {
    digitalWrite(HEIZUNG, HIGH);
    display.println("Fehler Temp. Sensor"); 
    Serial.println("Sensor");
  }
  
  // ---------------------------------------------------------
  // Sonnenaufgang und Untergang ermitteln und Licht schalten
  // Victron MPPT auslesen und Daten via GSM Shield übertragen
  // ---------------------------------------------------------
  
  unsigned int dataTrans=1;
  unsigned int offset = OFFSET; // Ein-/Ausschalten um den Offsetwert verzögert bzw. verlängert
  double Zeitzone = ZEITZONE;    // 0=Weltzeit, 1=Winterzeit, 2=Sommerzeit
  unsigned int nachtAbschaltungAnfang = NACHTABSCHALTUNGANFANG; // Anfang Nachtabschaltung um 0 Uhr
  unsigned int nachtAbschaltungEnde = NACHTABSCHALTUNGENDE;   // Ende Nachtabschaltung um 5 Uhr
  uint8_t stunde=0;
  uint8_t minuten=0;
  uint8_t sekunden=0;
  unsigned int jahr=0;
  uint8_t monat=0;
  uint8_t tag=0;
  unsigned int jetzt=0;
  bool result;
        
  DateTime now = rtc.now();
  stunde=now.hour();
  minuten=now.minute(); 
  sekunden=now.second();
  jahr=now.year();
  monat=now.month();
  tag=now.day();
  jetzt= stunde*60+minuten;

  unsigned int GeographischeLaengeOffset;
  double GeographischeLaenge;
   
  GeographischeLaengeOffset = digitalRead(LNG0)+digitalRead(LNG1)*2 + digitalRead(LNG2)*4;
  GeographischeLaenge = 9.5+GeographischeLaengeOffset;
  Serial.print("Geo Länge:"); Serial.println(GeographischeLaenge);    
  display.print("GeoLng: "); display.print(GeographischeLaenge); display.println(" Grad");
 
  unsigned int AufgangStunden; 
  unsigned int AufgangMinuten;
  unsigned int UntergangStunden; 
  unsigned int UntergangMinuten;
  unsigned int AufgangInMinuten; 
  unsigned int UntergangInMinuten;
  
  
  sonnenAufUnterGang(jahr, monat, tag, GeographischeLaenge, Zeitzone, AufgangStunden, AufgangMinuten, UntergangStunden, UntergangMinuten );
  AufgangInMinuten =  AufgangStunden*60+AufgangMinuten;
  UntergangInMinuten = UntergangStunden*60+UntergangMinuten;
  
  
  Serial.print("Datum:    "); Serial.print(jahr); Serial.print("-"); Serial.print(monat); Serial.print("-"); Serial.println(tag);  
  Serial.print("Zeit:            "); Serial.print(stunde); Serial.print(":"); Serial.print(minuten); Serial.print(" - "); Serial.println(jetzt);  
  display.print(jahr); display.print('-'); display.print(monat); display.print('-'); display.print(tag); display.print(" ");
  display.print(stunde); display.print(':'); display.print(minuten); display.print(':'); display.println(sekunden);
  display.print("Auf: "); display.print(AufgangStunden); display.print(":"); display.print(AufgangMinuten);
  display.print(" Unt: "); display.print(UntergangStunden); display.print(":"); display.println(UntergangMinuten);

  if (digitalRead(RES2)==LOW){
    digitalWrite(LICHT, LOW);
    display.setTextSize(2);display.println("Testmodus");display.setTextSize(1);
    if (modus!="3" && withGSMShield){
      modus = "3";
      Serial.print("Data Transmission Trial: ");Serial.println(dataTrans);
      result=sendData(String(tempC*100,0),String(!digitalRead(HEIZUNG)), modus);
      while (result==0 && dataTrans<maxDataTrans){
        if (result==0 && checkCommunication()==1) gsmPWROff(); //Power off
        Serial.print("Data Transmission Trial: ");Serial.println(dataTrans+1);
        result=sendData(String(tempC*100,0),String(!digitalRead(HEIZUNG)), modus);
        dataTrans++;
      }
      if (result==0 && checkCommunication()==1) gsmPWROff(); //Power off
    }
    
  }
  else{
    if ( (jetzt > AufgangInMinuten+offset) && (jetzt+offset < UntergangInMinuten)){
      Serial.println("Tagmodus");
      digitalWrite(LICHT, HIGH); //Licht aus
      display.setTextSize(2);display.println(" Tagmodus");display.setTextSize(1);
      if (modus!="0" && withGSMShield){
        modus = "0";
        Serial.print("Data Transmission Trial: ");Serial.println(dataTrans);
        result=sendData(String(tempC*100,0),String(!digitalRead(HEIZUNG)), modus);
        while (result==0 && dataTrans<maxDataTrans){
          if (result==0 && checkCommunication()==1) gsmPWROff(); //Power off
          Serial.print("Data Transmission Trial: ");Serial.println(dataTrans+1);
          result=sendData(String(tempC*100,0),String(!digitalRead(HEIZUNG)), modus);
          dataTrans++;
        }
        if (result==0 && checkCommunication()==1) gsmPWROff(); //Power off
      }
    }
    else{
      if (jetzt > nachtAbschaltungAnfang && jetzt < nachtAbschaltungEnde) {
        Serial.println("Nachtabschaltung");
        digitalWrite(LICHT, HIGH); //Licht aus
        display.setTextSize(2); display.println("Nacht-Aus"); display.setTextSize(1);
        if (modus!="2" && withGSMShield){
          modus = "2";
          Serial.print("Data Transmission Trial: ");Serial.println(dataTrans);
          result=sendData(String(tempC*100,0),String(!digitalRead(HEIZUNG)), modus);
          while (result==0 && dataTrans<maxDataTrans){
            if (result==0 && checkCommunication()==1) gsmPWROff(); //Power off
            Serial.print("Data Transmission Trial: ");Serial.println(dataTrans+1);
            result=sendData(String(tempC*100,0),String(!digitalRead(HEIZUNG)), modus);
            dataTrans++;
          }
          if (result==0 && checkCommunication()==1) gsmPWROff(); //Power off
        }
      }
      else{
        Serial.println("Nachtnachtmodus");
        digitalWrite(LICHT, LOW); //Licht an
        display.setTextSize(2); display.println("Nachtmode"); display.setTextSize(1); 
        if (modus!="1" && withGSMShield){
          modus = "1";
          Serial.print("Data Transmission Trial: ");Serial.println(dataTrans);
          result=sendData(String(tempC*100,0),String(!digitalRead(HEIZUNG)), modus);
          while (result==0 && dataTrans<maxDataTrans){
            if (result==0 && checkCommunication()==1) gsmPWROff(); //Power off
            Serial.print("Data Transmission Trial: ");Serial.println(dataTrans+1);
            result=sendData(String(tempC*100,0),String(!digitalRead(HEIZUNG)), modus);
            dataTrans++;
          }
          if (result==0 && checkCommunication()==1) gsmPWROff(); //Power off
        }
      }
    }
  }
  
  Serial.println("");
  unsigned int heatState;
  unsigned int lightState;
  if(digitalRead(LICHT)==LOW) heatState=1; else heatState=0;
  if(digitalRead(HEIZUNG)==LOW) lightState=1; else lightState=0;
  display.print("Li:"); display.print(heatState); display.print(" Hz:"); display.print(lightState); 
  display.display();
}

//******************************
// Funktionen für das GSM Shield
//******************************

int sendData(String temp, String h, String state){
  String apn = APN;
  String url = URL;
  String GSMResult="";
  bool error =false;
  unsigned int result=0;
  int x=0;
  
  display.clearDisplay();
  display.setCursor(1,0);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.print("   EPA Solar "); display.println(SWVERSION);
  Serial.println();
  Serial.print("Datenübertragung Modus: ");Serial.print(state);
  display.println("Datenuebertragung"); display.print("Modus: ");display.print(state);
  if (state=="0"){ Serial.println(" Tagmodus"); display.println(" Tagmodus");}
  else if (state=="1"){ Serial.println(" Nachtmodus"); display.println(" Nachtmodus");} 
  else if (state=="3"){ Serial.println(" Testmodus"); display.println(" Testmodus"); }
  else if (state=="2"){ Serial.println(" Nachtabschaltung"); display.println(" Nachtabschaltung"); }
  
  for(int i=0; i<3;i++) if (checkCommunication()==1) break;  
  if (checkCommunication()==0) gsmPWrON(); //Power ON  
  else{
    Serial.println("GSM Shield power was already on");
    display.println("GSM Shield was on"); display.display();
  }
  result=checkCommunication();
  while (result==0 && x<maxComChecks) {
    x++;
    result=checkCommunication();
  }
  if (result == 0) return 0;
  Serial1.flush();
  Serial1.println("ATE0"); delay(1000); Serial.println(getSerialData()); //no echo
  Serial1.println("AT+GSN"); delay(1000); //get Device IMEI
  String IMEI=getSerialData().substring(0, 15);
  Serial.print("IMEI:");Serial.println(IMEI);
  display.print("IMEI:");display.println(IMEI);
  //GSM aktivieren
  Serial.print("waiting for GSM network connection:"); 
  display.print("con.:");
  for (int i=0; i<wait4Connection; i++){
    delay(1000);   
    Serial.print(".");
    display.print(".");
    display.display();
  }
  Serial.println("");
  display.println();
  Serial.println("Reading values from Victron MPPT 75: ");
  for (int x=0; x < 2; x++){
    VE_fw=0;
    int veReads=0;
    while (VE_fw==0 && veReads<maxVEReads){
      veReads++;
      Serial.print(".");
      VE_fw = myve.read(VE_FW);
      VE_v = myve.read(VE_V);
      VE_i = myve.read(VE_I);
      VE_vpv = myve.read(VE_VPV);
      VE_ppv = myve.read(VE_PPV);
      VE_cs = myve.read(VE_CS);
      VE_err = myve.read(VE_ERR);
      VE_load = myve.read(VE_LOAD);
      VE_il = myve.read(VE_IL);
      VE_h19 = myve.read(VE_H19);
      VE_h20 = myve.read(VE_H20);
      VE_h21 = myve.read(VE_H21);
      VE_h22 = myve.read(VE_H22);
      VE_h23 = myve.read(VE_H23);
    } 
  }
  Serial.println(); 
  VEData=url+"IMEI="+IMEI+"&";
  VEData=VEData+"TEMP="+temp+"&";
  VEData=VEData+"H="+h+"&";
  VEData=VEData+"STA="+state+"&";
  VEData=VEData+"FW="+String(VE_fw)+"&";
  VEData=VEData+"V="+String(VE_v)+"&";
  VEData=VEData+"I="+String(VE_i)+"&";
  VEData=VEData+"VPV="+String(VE_vpv)+"&";
  VEData=VEData+"PPV="+String(VE_ppv)+"&";
  VEData=VEData+"CS="+String(VE_cs)+"&";
  VEData=VEData+"ERR="+String(VE_err)+"&";
  VEData=VEData+"LAST="+String(VE_load)+"&";
  VEData=VEData+"IL="+String(VE_il)+"&";
  VEData=VEData+"H19="+String(VE_h19)+"&";
  VEData=VEData+"H20="+String(VE_h20)+"&";
  VEData=VEData+"H21="+String(VE_h21)+"&";
  VEData=VEData+"H22="+String(VE_h22)+"&";
  VEData=VEData+"H23="+String(VE_h23)+"&";
  VEData=VEData+"SWV="+SWVERSIONDS+"&";
  VEData=VEData+"TS=20191028211547";
  //+jahr+"-"+monat+"-"+tag+" "+stunde+":"+minuten+":"+sekunden;
  
  Serial.println("HTTP GET method :");
  Serial1.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""); delay(1000); 
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
  Serial1.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""); delay(1000);
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
  Serial1.print("AT+SAPBR=3,1,\"APN\",\""); Serial1.print(apn); Serial1.println("\""); delay(1000);
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
    
  /*
  Serial1.println("AT+IPR?"); delay(1000); Serial.println(getSerialData()); // Baudrate abfragen
  Serial1.println("AT+ICF?"); delay(1000); Serial.println(getSerialData()); // framing abfragen
  Serial1.println("AT+IPR=19200"); delay(1000); Serial.println(getSerialData()); // Baudrate abfragen
  Serial1.println("AT+IPR?"); delay(1000); Serial.println(getSerialData()); // Baudrate abfragen
  */
  
  Serial1.println("AT+SAPBR=1,1"); delay(5000); // Open GPRS context
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
  Serial1.println("AT+SAPBR=2,1"); delay(1000); // Query the GPRS context
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
  Serial1.println("AT+HTTPINIT"); delay(1000);  // Initialize HTTP service
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
  Serial1.println("AT+HTTPPARA=\"CID\",1"); delay(1000);
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
  Serial.print("AT+HTTPPARA=\"URL\",\""); Serial.print(VEData); Serial.println("\""); 
  Serial1.print("AT+HTTPPARA=\"URL\",\""); Serial1.print(VEData); Serial1.println("\""); delay(5000);
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
  Serial1.println("AT+HTTPACTION=0"); delay(5000); 
  GSMResult=getSerialData();  
  display.println(GSMResult); display.display();
  Serial.println(GSMResult); // Start GETT session - 200 success, 600 error   
  if (!GSMResult.substring(0,2)=="OK") return 0;
  Serial1.println("AT+HTTPTERM"); delay(1000); // Terminate HTTP service
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
  Serial1.println("AT+SAPBR=0,1"); delay(1000); // Close GPRS context 
  GSMResult=getSerialData(); Serial.println(GSMResult); if (!isOK(GSMResult)) return 0;
  //Power off
  gsmPWROff();
  return 1;
}

int checkCommunication(){
  Serial.print("GSM Shield AT Connection Check: ");
  getSerialData();
  Serial1.println("AT"); /* Check Communication */
  delay(1000);
  String test=getSerialData();
  Serial.print(test);
  if (test=="OK" || test=="ATOK" ){
    Serial.println(" connected");
    return 1;
  }
  else{
    Serial.println("not connected");
    return 0;
  }
}

String getSerialData()
{
  char cread="";
  char response[100]="";
  uint8_t idx = 0;
  while(Serial1.available()!=0){
    cread=Serial1.read();
    if (cread!=10 && cread!=13)
     response[idx++]=cread;
  }
  response[idx] = '\0'; 
  return response;
}

bool isOK(String result){
  if (result.substring(result.length()-2)=="OK"){
    return true;
  }
  else{
    return false;
  } 
}

void gsmPWrON() {
  Serial.println("GSM Shield waiting for power on");
  display.println("GSM Shield power on"); display.display();
  digitalWrite(GSMPWR, HIGH);
  delay(1000);
  digitalWrite(GSMPWR, LOW);
  delay(3000);
}

void gsmPWROff(){
  Serial.println("GSM Shield power off");
  digitalWrite(GSMPWR, HIGH);
  delay(1000);
  digitalWrite(GSMPWR, LOW);
  delay(5000);
}

// ***************************************************
// Funktionen zur Berechnung des Sonnen Auf/Untergangs 
// ***************************************************

double pi2=6.283185307179586476925286766559;
double pi=3.1415926535897932384626433832795;
double RAD = 0.017453292519943295769236907684886;
double JulianischesDatum ( int Jahr, int Monat, int Tag, int Stunde, int Minuten, double Sekunden )
{// Gregorianischer Kalender
       int   Gregor;
       if (Monat<=2)
       {
              Monat=Monat +12;
              Jahr = Jahr -1;
       }
       Gregor = (Jahr/400)-(Jahr/100)+(Jahr/4);  // Gregorianischer Kalender
       return 2400000.5+365.0*Jahr - 679004.0 + Gregor + int(30.6001*(Monat+1)) + Tag + Stunde/24.0 + Minuten/1440.0 + Sekunden/86400.0;
}

double InPi(double x)
{
       int n = (int)(x/pi2);
       x = x - n*pi2;
       if (x<0) x +=pi2;
       return x;
}

double eps(double T) // Neigung der Erdachse
{
       return RAD*(23.43929111 + (-46.8150*T - 0.00059*T*T + 0.001813*T*T*T)/3600.0);
}

double BerechneZeitgleichung(double &DK,double T)
{             
       double RA_Mittel = 18.71506921 + 2400.0513369*T +(2.5862e-5 - 1.72e-9*T)*T*T; 
       double M  = InPi(pi2 * (0.993133 + 99.997361*T));
       double L  = InPi(pi2 * (  0.7859453 + M/pi2 + (6893.0*sin(M)+72.0*sin(2.0*M)+6191.2*T) / 1296.0e3));
       double e = eps(T); 
       double RA = atan(tan(L)*cos(e));
       if (RA<0.0) RA+=pi;
       if (L>pi) RA+=pi;
       RA = 24.0*RA/pi2;
       DK = asin(sin(e)*sin(L));
       // Damit 0<=RA_Mittel<24
       RA_Mittel = 24.0*InPi(pi2*RA_Mittel/24.0)/pi2;
       double dRA = RA_Mittel - RA;
       if (dRA < -12.0) dRA+=24.0;
       if (dRA > 12.0) dRA-=24.0;
       dRA = dRA* 1.0027379;
       return dRA ;
}


void sonnenAufUnterGang(uint8_t jahr, uint8_t monat, uint8_t tag, double GeographischeLaenge, double Zeitzone, unsigned int& AufgangStunden, unsigned int& AufgangMinuten, unsigned int& UntergangStunden, unsigned int& UntergangMinuten)
{
  double JD2000 = 2451545.0;
  double JD;
  JD = JulianischesDatum(jahr,monat,tag,12,0,0);
  double T = (JD - JD2000)/36525.0;
  double DK;
  double h = -50.0/60.0*RAD;
  double B = 48.2*RAD; // geographische Breite
         
  double Zeitgleichung = BerechneZeitgleichung(DK,T);
  double Minuten = Zeitgleichung*60.0;
  double Zeitdifferenz = 12.0*acos((sin(h) - sin(B)*sin(DK)) / (cos(B)*cos(DK)))/pi;
  double AufgangOrtszeit = 12.0 - Zeitdifferenz - Zeitgleichung;
  double UntergangOrtszeit = 12.0 + Zeitdifferenz - Zeitgleichung;
  double AufgangWeltzeit = AufgangOrtszeit - GeographischeLaenge /15.0;
  double UntergangWeltzeit = UntergangOrtszeit - GeographischeLaenge /15.0;

  double Aufgang = AufgangWeltzeit + Zeitzone;         // In Stunden
  if (Aufgang<0.0) Aufgang +=24.0;
  else if (Aufgang>=24.0) Aufgang -=24.0;

  double Untergang = UntergangWeltzeit + Zeitzone;    
  if (Untergang<0.0) Untergang +=24.0;
  else if (Untergang>=24.0) Untergang -=24.0;

  AufgangMinuten   = int(60.0*(Aufgang - (int)Aufgang)+0.5);
  AufgangStunden   = (int)Aufgang;
  if (AufgangMinuten>=60.0) { AufgangMinuten-=60.0; AufgangStunden++; }
  else if (AufgangMinuten<0.0) {
    AufgangMinuten+=60.0; AufgangStunden--;
    if (AufgangStunden<0.0) AufgangStunden+=24.0;
  }
       
  UntergangMinuten = int(60.0*(Untergang - (int)Untergang)+0.5);
  UntergangStunden = (int)Untergang;
  if (UntergangMinuten>=60.0) { UntergangMinuten-=60.0; UntergangStunden++; }
  else if (UntergangMinuten<0) {
    UntergangMinuten+=60.0; UntergangStunden--;
    if (UntergangStunden<0.0) UntergangStunden+=24.0;
  }
}

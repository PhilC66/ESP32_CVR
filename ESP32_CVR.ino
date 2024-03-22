/* Ph CORBEL 10/2020
  Gestion Carré VR
  (basé sur ESP32_Signalisation

  todo
  OK a tester Commande D si O ne rien faire, si F -> ouvrir
  Ajouter commande Allumage/extinction Feu Rouge independante
  
  Compilation LOLIN D32,default,80MHz, ESP32 1.0.2 (1.0.4 bugg?)
  Arduino IDE 1.8.19 : x 76%, x 14% sur PC
  Arduino IDE 1.8.19 :  x 76%, x 14% sur raspi

  V2-0 20/01/2024 version 4G
  nouveau magique

  Compilation LOLIN D32,default,80MHz, ESP32 1.0.2 (1.0.4 bugg?)
  Arduino IDE 1.8.10 : 1000014 76%, 47800 14% sur PC
  Arduino IDE 1.8.10 :  999994 76%, 47800 14% sur raspi

  V1-2 10/06/2021 pas encore installé
  nouveau magique
  valeur par defaut tempoouverture et tempofermeture = 20(appliqué 09/06/2021)
  valeur par maxi tempoouverture et tempofermeture = 120

  V1-1 12/12/2020 installé le 29/04/2021
  remplacer <credentials_ftp.h> par <credentials_tpcf.h>
  char ftpUser
  nouveau magic
  Allumage Frouge seulement apres verification position F
  Ajouter info Defaut Position dans message KO

  V1-0 installé 26/11/2020
  
  Etat CVR
             | Cvr | Cde
  Fermé      |  1  |  F
  Ouvert     |  2  |  O
  indefini   |  0  |  Z pas de cde, affichage seulement

  Cde verin  | PinOuvre | PinFerme
  Fermeture  |    0     |     1
  Ouverture  |    1     |     0
  Repos      |    0     |     0

  position   |   S1     |    S2
  Fermé      |    1     |     0
  Ouvert     |    0     |     1
  Indefini   |    0     |     0

*/
#include <Arduino.h>

String ver        = "V2-0";
int    Magique    = 1;

#define TINY_GSM_MODEM_SIM7600

#include <Battpct.h>
#include "defs.h"
// #include <Sim800l.h>              //my SIM800 modifié
#include <TinyGsmClient.h>         // librairie TinyGSM revue PhC 10.11.5
#include <PubSubClient.h>          // modifié define MQTT_MAX_PACKET_SIZE 256
#include <Time.h>
#include <TimeAlarms.h>
// #include <sys/time.h>             //<sys/time.h>
#include <WiFi.h>
// #include <EEPROM.h>               // variable en EEPROM(SPIFFS)
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <FS.h>
#include <SPI.h>
#include <Ticker.h>
#include "passdata.h"
#include <ArduinoJson.h>
#include <credentials_tpcf.h>

#define Serial Serial
#define SerialAT Serial2
#define TINY_GSM_DEBUG Serial
#define TINY_GSM_USE_GPRS true
// #define TINY_GSM_USE_WIFI false
// #define GSM_PIN "1234"

String  webpage = "";
#define ServerVersion "1.0"
bool    SPIFFS_present = false;
#include "CSS.h"               // pageweb

// #define RESET_PIN     18   // declaré par Sim800l.h
#define LED_PIN        5   //
#define PinBattProc   35   // liaison interne carte Lolin32 adc
#define PinBattSol    39   // Batterie générale 12V adc VN
#define PinBattUSB    36   // V USB 5V adc VP 36, 25 ADC2 pas utilisable avec Wifi
#define PinIp1        32   // Entrée Ip1 S1 position CVR
#define PinIp2        33   // Entrée Ip2 S2 position CVR
#define PinFerme      21   // Sortie Commande Fermeture CVR
#define PinFeuxR      19   // Sortie Commande Feux Rouge
#define PinOuvre      15   // Sortie Commande Ouverture CVR
#define RX_PIN        16   // TX Sim800
#define TX_PIN        17   // RX Sim800
#define PinReset      13   // Reset Hard
#define PinTest       27   // Test sans GSM cc a la masse
#define NTPServer "pool.ntp.org"
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define nSample (1<<4)    // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[5][nSample]; // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 24V, 5 Lum
unsigned int adc_mm[5];            // stockage pour la moyenne mobile

uint64_t TIME_TO_SLEEP  = 15;/* Time ESP32 will go to sleep (in seconds) */
unsigned long debut     = 0; // pour decompteur temps wifi
byte calendrier[13][32]; // tableau calendrier ligne 0 et jour 0 non utilisé, 12*31
char fileconfig[12]      = "/config.txt";   // fichier en SPIFFS contenant structure config
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
char filelog[9]          = "/log.txt";      // fichier en SPIFFS contenant le logé

const String soft = "ESP32_CVR.ino.d32"; // nom du soft

const String Mois[13] = {"", "Janvier", "Fevrier", "Mars", "Avril", "Mai", "Juin", "Juillet", "Aout", "Septembre", "Octobre", "Novembre", "Decembre"};
String Sbidon 		= ""; // String texte temporaire
String message;
String bufferrcpt;
String fl = "\n";                   //  saut de ligne SMS
String Id ;                         //  Id du materiel sera lu dans EEPROM
// char   SIM800InBuffer[64];          //  for notifications from the SIM800
char   replybuffer[255];            //  Buffer de reponse SIM800
byte confign = 0;                   // position enregistrement config EEPROM
byte recordn = 200;                 // position enregistrement log EEPROM
byte RgePwmChanel = 0;
bool blinker = false;

RTC_DATA_ATTR bool FlagAlarmeTension       = false; // Alarme tension Batterie
RTC_DATA_ATTR bool FlagLastAlarmeTension   = false;
RTC_DATA_ATTR bool FlagAlarmePosition      = false; // Alarme Position
RTC_DATA_ATTR bool FlagLastAlarmePosition  = false; // Last Alarme Position
RTC_DATA_ATTR bool FirstWakeup             = true;  // envoie premier message vie une seule fois
RTC_DATA_ATTR bool FlagCircule             = false; // circule demandé -> inverse le calendrier, valid 1 seul jour
RTC_DATA_ATTR bool FileLogOnce             = false; // true si log > seuil alerte

int Cvr        = 0; // Etat du Cvr voir tableau au début
int LastCvr    = 0; // memo last etat
bool FlagReset = false;       // Reset demandé
bool jour      = false;				// jour = true, nuit = false
bool gsm       = true;        // carte GSM presente utilisé pour test sans GSM seulement
bool FlagAlarmeGprs          = false; // Alarme confirmée
bool AlarmeGprs              = false; // detection alarme
bool FlagLastAlarmeGprs      = false;
bool FlagAlarmeMQTT          = false;
bool AlarmeMQTT              = false;
bool FlagLastAlarmeMQTT      = false;
String Memo_Demande_CVR[3] = {"","",""}; // 0 num demandeur,1 nom, 2 CVR demandé (O2,F1)
bool FlagDemande_CVR = false;   // si demande encours = true
bool FlagVerif_CVR   = false;   // si fin de la commande, pour verification position
bool FlagFinJournee  = false;   // demande fermeture fin de journée
bool Allume  = false;
int CoeffTension[3];          // Coeff calibration Tension
int CoeffTensionDefaut = 7000;// Coefficient par defaut

int    slot = 0;              //this will be the slot number of the SMS

long   TensionBatterie  = 0; // Tension Batterie solaire
long   VBatterieProc    = 0; // Tension Batterie Processeur
long   VUSB             = 0; // Tension USB

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);
PhonebookEntry Phone;
WebServer server(80);
File UploadFile;

// typedef struct               // declaration structure  pour les log
// {
//   char    dt[10];            // DateTime 0610-1702 9+1
//   char    Act[2];            // Action A/D/S/s 1+1
//   char    Name[15];          // 14 car
// } champ;
// champ record[5];

struct  config_t           // Structure configuration sauvée en EEPROM
{
  int     magic;           // num magique
  int     anticip;         // temps anticipation du reveille au lancement s
  long    DebutJour;       // Heure message Vie, 7h matin en seconde = 7*60*60
  long    FinJour;         // Heure fin jour, 20h matin en seconde = 20*60*60
  long    RepeatWakeUp;    // Periodicité WakeUp Jour non circulé
  int     timeoutWifi;     // tempo coupure Wifi si pas de mise a jour (s)
  int     SlowBlinker;     // ms
  int     FRgePWM;         // Modulation Feux Rouge %
  int     Tempoouverture;  // Tempo ouverture verin
  int     Tempofermeture;  // Tempo fermeture verin
  bool    Pos_Pn_PB[10];   // numero du Phone Book (1-9) à qui envoyer 0/1 0 par defaut
  char    Idchar[11];      // Id
  char    apn[11];         // APN
  char    gprsUser[11];    // user for APN
  char    gprsPass[11];    // pass for APN
  char    ftpServeur[26];  // serveur ftp
  char    ftpUser[9];      // user ftp
  char    ftpPass[16];     // pwd ftp
  int     ftpPort;         // port ftp
  int     TypeBatt;        // Type Batterie 16: Pb 6elts, 24 LiFePO 4elts
  byte    cptAla;          // Compteur alarmes Tracker avant declenchement
  char    mqttServer[26];  // Serveur MQTT
  char    mqttUserName[11];// MQTT User
  char    mqttPass[16];    // MQTT pass
  char    sendTopic[20];   // channel Id output to server commun à tous
  char    receiveTopic[20];// channel Id input from server
  char    permanentTopic[20];  // channel Id
  int     mqttPort;        // Port serveur MQTT
  int     hete;            // decalage Heure été UTC
  int     hhiver;          // decalage Heure hiver UTC
  bool    messageMode;     // false = sms,true=sms+mqtt pour message generé automatiquement
  bool    autoupload;      // Upload automatique du fichier log
} ;
config_t config;
int N_Y, N_M, N_D, N_H, N_m, N_S; // variable Date/Time temporaire
uint32_t lastReconnectMQTTAttempt = 0;
uint32_t lastReconnectGPRSAttempt = 0;

int NbrResetModem = 0;              // Nombre de fois reset modem, remise à 0 signal vie
int Histo_Reseau[5]={0,0,0,0,0};    // Historique Reseau cumul chaque heure

Ticker SlowBlink;          // Clignotant lent
Ticker step;               // pas modulation pwm feux
Ticker ADC;                // Lecture des Adc

AlarmId loopPrincipale;    // boucle principale
AlarmId DebutJour;         // Debut journée
AlarmId FinJour;           // Fin de journée retour deep sleep
AlarmId Aouverture;        // Tempo ouverture verin
AlarmId Afermeture;        // Tempo fermeture verin

// HardwareSerial *SIM800Serial = &Serial2; // liaison serie FONA SIM800
// Sim800l Sim800;                          // to declare the library

// String fieldsToPublish; // Change to allow multiple fields.
String dataToPublish;   // Holds your field data.
//---------------------------------------------------------------------------
void MajHeure(bool force = false);
//---------------------------------------------------------------------------
void setup() {

  message.reserve(300); // texte des SMS

  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialisation failed...");
    SPIFFS_present = false;
  }
  else {
    Serial.println("SPIFFS initialised... file access enabled...");
    SPIFFS_present = true;
  }

  pinMode(PinIp1     , INPUT_PULLUP);
  pinMode(PinIp2     , INPUT_PULLUP);
  pinMode(PinFerme   , OUTPUT);
  pinMode(PinOuvre   , OUTPUT);
  pinMode(PinFeuxR   , OUTPUT);
  pinMode(PinTest    , INPUT_PULLUP);
  adcAttachPin(PinBattProc);
  adcAttachPin(PinBattSol);
  adcAttachPin(PinBattUSB);
  if (digitalRead(PinTest) == 0) { // lire strap test, si = 0 test sans carte gsm
    gsm = false;
    setTime(12, 00, 00, 15, 07, 2019); // il faut initialiser la date et heure, jour circule et midi
    Serial.println("Lancement test sans carte gsm");
    Serial.println("mise à l'heure 14/07/2019 12:00:00");
    Serial.println("retirer le cavalier Pin27 et reset");
    Serial.println("pour redemarrer normalement");
  }

  if (gsm) {
    Serial.println("lancement SIM7600");
    // Set GSM module baud rate valeur par defaut usine = 115200
    // pour nouvelle carte avant premiere utilisation faire AT+IPREX=38400
    // certaine cde comme ATI modeminfo ne fonctionne pas à 115200(val defaut usine),
    // Ok à 57600, on garde une marge un cran en dessous = 38400 
    SerialAT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // fonctionne avec 115200
    Serial.println(F("Initializing modem..."));
    unsigned long debut = millis();
    while(!modem.init()){ // mettre ici (RESET_PIN) pour utiliser powerkey du SIM7600
      modem.setPhoneFunctionality(1);// CFUN=1 full functionality, online mode
      delay(1000);
      if(millis() - debut > 60000) break;
    }
    // modem.restart();
    Serial.print(F("Modem Info: "));
    Serial.println(modem.getModemInfo());
  }
  // parametrage PWM pour les feux
  // https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
  ledcSetup(RgePwmChanel, 1000, 8);
  ledcAttachPin(PinFeuxR, RgePwmChanel);
  ledcWrite(RgePwmChanel, 0); // Feux Rouge 0

  init_adc_mm();// initialisation tableau pour adc Moyenne Mobile
  ADC.attach_ms(100, adc_read); // lecture des adc toute les 100ms
  /* Lecture configuration file config	 */
  readConfig(); // Lecture de la config
  if (config.magic != Magique) {
    /* verification numero magique si different
    		erreur lecture EEPROM ou carte vierge
    		on charge les valeurs par défaut
    */
    Serial.println("Nouvelle Configuration !");
    config.magic         = Magique;
    config.anticip       = 2700;
    config.DebutJour     = 8  * 60 * 60;
    config.FinJour       = 19 * 60 * 60;
    config.RepeatWakeUp  = 60 * 60;
    config.timeoutWifi   = 15 * 60;
    config.SlowBlinker   = 500;
    config.FRgePWM       = 100;
    config.Tempofermeture = 20;
    config.Tempoouverture = 20;
    config.TypeBatt      = 16; // Pb par défaut
    config.mqttPort      = tempmqttPort;
    config.hete          = 2; // heure
    config.hhiver        = 1; // heure
    config.messageMode   = false; // SMS
    config.autoupload    = true;
    config.cptAla        = 10; // 11*Acquisition time
    for (int i = 0; i < 10; i++) {// initialise liste PhoneBook liste restreinte
      config.Pos_Pn_PB[i] = 0;
    }
    // config.Pos_Pn_PB[1]  = 1;	// le premier numero du PB par defaut
    String temp          = "TPCF_CVR1";
    temp.toCharArray(config.Idchar, 11);
    String tempapn       = "free";//"sl2sfr"
    String tempGprsUser  = "";
    String tempGprsPass  = "";
    config.ftpPort       = tempftpPort;
    tempapn.toCharArray(config.apn, (tempapn.length() + 1));
    tempGprsUser.toCharArray(config.gprsUser, (tempGprsUser.length() + 1));
    tempGprsPass.toCharArray(config.gprsPass, (tempGprsPass.length() + 1));
    tempServer.toCharArray(config.ftpServeur, (tempServer.length() + 1));
    tempftpUser.toCharArray(config.ftpUser, (tempftpUser.length() + 1));
    tempftpPass.toCharArray(config.ftpPass, (tempftpPass.length() + 1));
    tempServer.toCharArray(config.mqttServer, (tempServer.length() + 1));
    tempmqttUserName.toCharArray(config.mqttUserName, (tempmqttUserName.length() + 1));
    tempmqttPass.toCharArray(config.mqttPass, (tempmqttPass.length() + 1));

    strncpy(config.sendTopic,("Signalisation/input"),sizeof(config.sendTopic));
    memcpy(config.receiveTopic,&config.Idchar[5],5);
    config.receiveTopic[5] = '\0';
    strcat(config.receiveTopic,"/output");
    memcpy(config.permanentTopic,&config.Idchar[5],5);
    config.permanentTopic[5] = '\0';
    strcat(config.permanentTopic,"/permanent");

    sauvConfig();
  }
  PrintConfig();
  Id  = String(config.Idchar);
  Id += fl;

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(config.Idchar);
  ArduinoOTA.setPasswordHash(OTApwdhash);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.print("Start updating ");
    Serial.println(type);
  })
  .onEnd([]() {
    Serial.println("End");
    delay(100);
    ResetHard();
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
  });


  OuvrirCalendrier();					// ouvre calendrier circulation en SPIFFS
  OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS
  if (gsm) {
    Serial.print(("Waiting for network..."));
    if (!modem.waitForNetwork()) {
      Serial.println(F(" fail"));
      delay(100);
      // return;
    } else {
      Serial.println(F(" success"));
    }
    if (modem.isNetworkConnected()) { Serial.println(F("Network connected")); }
    byte n;
    byte cpt = 0;
    do {												// boucle tant que reseau pas connecté
      Alarm.delay(100);
      n = modem.getRegistrationStatus();
      cpt ++;
      if (cpt > 2) break;				// sortie si 2 tentatives demarrage sans reseau
    } while (!(n == 1 || n == 5));
    Serial.print(F("Network status "));
    Serial.print(n);
    Serial.print(F(": "));
    if (n == 0) Serial.println(F("Not registered"));
    if (n == 1) Serial.println(F("Registered (home)"));
    if (n == 2) Serial.println(F("Not registered (searching)"));
    if (n == 3) Serial.println(F("Denied"));
    if (n == 4) Serial.println(F("Unknown"));
    if (n == 5) Serial.println(F("Registered roaming"));

    // Demande Operateur connecté
    Serial.print(F("Operateur :")), Serial.println(modem.getOperator());

    // Synchro heure réseau du modem
    Serial.println(F("Synchro Heure réseau "));
    if(SyncHeureModem(config.hete*4, true)){ // heure été par defaut, first time
      Serial.println(F("OK"));
    } else {Serial.println(F("KO"));}
    Serial.println(modem.getGSMDateTime(TinyGSMDateTimeFormat(0)));

    message = "";
    read_RSSI();									 // Niveau reseau
    Serial.println(message);
    message = "";

    // GPRS connection parameters are usually set after network registration
    ConnectGPRS();
    if (modem.isGprsConnected()) { Serial.println(F("GPRS connected")); }

    // MQTT Broker setup
    mqtt.setServer(config.mqttServer, config.mqttPort); // Set the MQTT broker details.
    mqtt.setCallback(mqttSubscriptionCallback);   // Set the MQTT message handler function.
    
    if(config.messageMode==1) mqttConnect();
    
    timesstatus();								// Etat synchronisation Heure Sys
    MajHeure();
  }

  loopPrincipale = Alarm.timerRepeat(10, Acquisition); // boucle principale 10s
  Alarm.enable(loopPrincipale);

  DebutJour = Alarm.alarmRepeat(config.DebutJour, SignalVie);
  Alarm.enable(DebutJour);

  FinJour = Alarm.alarmRepeat(config.FinJour, FinJournee); // Fin de journée retour deep sleep
  Alarm.enable(FinJour);

  Aouverture = Alarm.timerRepeat(config.Tempoouverture, ouverture);
  Alarm.disable(Aouverture);
  Afermeture = Alarm.timerRepeat(config.Tempofermeture, fermeture);
  Alarm.disable(Afermeture);

  Serial.print("flag Circule :"), Serial.println(FlagCircule);

  modem.setNetworkMode(2); // force Network Mode Auto
  // reduire consommation eteindre Wifi et BT
  WiFi.mode(WIFI_OFF);
  btStop();
  
  MajLog("Auto", "Lancement");
  Cvr = Position_CVR(); // lire position CVR affectée à Cvr
  LastCvr = Cvr;
}
//---------------------------------------------------------------------------
void loop() {
  recvOneChar(); // lecture port serie
  
  //*************** Verification reception SMS ***************
  // Traitement des messages non sollicté en provenance modem
  // Attente donnée en provenance SIM7600
  // https://github.com/vshymanskyy/TinyGSM/pull/260/files
  
  if(SerialAT.available()) { // reception caracteres depuis modem
    unsigned long timerStart,timerEnd;
    String interrupt = "";
    timerStart = millis();
    while(1) {
      if(SerialAT.available()) {
          char c = SerialAT.read();
          interrupt+=c;   
      }
      timerEnd = millis();
      if(timerEnd - timerStart > 1500){//1500
      Serial.print("unsolicited message:"),Serial.println(interrupt);
      if (interrupt.indexOf(F("CMTI")) >= 0 ) { // reception SMS
        int p = interrupt.indexOf(","); // cherche numero de slot apres la virgule
        slot = interrupt.substring(p+1,interrupt.length()).toInt();
        // Serial.print(F("SMS en reception:")),Serial.println(slot);
        traite_sms(slot);
      }
      if ((interrupt.indexOf(F("RING"))) >= 0) {	// Si appel entrant on raccroche
        // Serial.println(F("Ca sonne!!!!"));
        modem.callHangup();
      }
      if ((interrupt.indexOf(F("+CNTP: 0"))) >= 0 ) { // si reception relance majheure
        // Serial.println(F("Relance mise à l'heure !"));
        MajHeure();
      }
      break;
      }
    }
  }
  if(gsm && config.messageMode == 1){  //config.messageMode = SMS+MQTT
    // make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) { // NETOPEN? Start TCPIP service
      Serial.println(F("GPRS disconnected!"));
      Serial.print(F("Connecting to "));
      Serial.print(config.apn);
      if (!modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {        
        Serial.println(F(" fail"));
        if (millis() - lastReconnectGPRSAttempt > 10000L){
          lastReconnectGPRSAttempt = millis();
          AlarmeGprs = true;
        }
      } else {lastReconnectGPRSAttempt = 0;}
      if (modem.isGprsConnected()) {
        Serial.println(F(" GPRS reconnected"));
        AlarmeGprs = false;
      }
    }
    if (!mqtt.connected()) {      
      // Reconnect every 10 seconds
      if (millis() - lastReconnectMQTTAttempt > 10000L) {
        Serial.println(F("=== MQTT NOT CONNECTED ==="));
        lastReconnectMQTTAttempt = millis();
        AlarmeMQTT = true;
        if (mqttConnect()) {
          lastReconnectMQTTAttempt = millis();
          AlarmeMQTT = false;
          }
      }
    }
    mqtt.loop();
  }
  ArduinoOTA.handle();
  Alarm.delay(0);
}	//fin loop
//---------------------------------------------------------------------------
void Acquisition() {

  Serial.print("position CVR:");
  switch (Position_CVR()) {
    case 0: // Indefini
      Serial.print("Z");
      break;
    case 1: // Fermé
      Serial.print("F");
      break;
    case 2: // Ouvert
      Serial.print("O");
      break;
  }
  Serial.print(";last:");
  switch (LastCvr) {
    case 0: // Indefini
      Serial.println("Z");
      break;
    case 1: // Fermé
      Serial.println("F");
      break;
    case 2: // Ouvert
      Serial.println("O");
      break;
  }
  Serial.print("Demande en attente:"), Serial.println(FlagDemande_CVR);
  if(gsm){
    static int cptRegStatusFault = 0;
    Serial.print(F("Connexion reseau:")),Serial.println(modem.isNetworkConnected());
    Serial.print(F("Reg status      :")),Serial.println(modem.getRegistrationStatus());
    Serial.print(F("Connexion GPRS  :")),Serial.println(modem.isGprsConnected());
    // Patch Blocage modem
    if(modem.getRegistrationStatus() != 1 && modem.getRegistrationStatus() != 5){
      if(cptRegStatusFault ++ > config.cptAla){
        cptRegStatusFault = 0;
        NbrResetModem +=1;
        Serial.println(F("Reset modem suite Reg status fault"));
        modem.send_AT(F("+CRESET"));
        delay(10000);
      }
    }
  }

  static int8_t nsms;
  static int cpt = 0; // compte le nombre de passage boucle
  static bool firstdecision = false;
  static byte nalaPosition = 0;
  static byte nRetourPosition = 0;

  AIntru_HeureActuelle();
  //***************************************************
  // verification position CVR
  if (FlagDemande_CVR == 0 && FlagFinJournee == 0) {
    // pas de demandes en cours
    Cvr = Position_CVR(); // lire position CVR affectée à Cvr
    if ((Cvr != LastCvr) && !FlagAlarmePosition) {
      nalaPosition ++;
      if (nalaPosition == 4) {
        FlagAlarmePosition = true;
        nalaPosition = 0;
        LastCvr = Cvr;
      }
    } else if ((Cvr != LastCvr) && FlagAlarmePosition) {
      nRetourPosition ++;
      if (nRetourPosition == 4) {
        FlagAlarmePosition = false;
        nRetourPosition = 0;
        LastCvr = Cvr;
      }
    } else {
      if (nalaPosition > 0) nalaPosition --; // efface progressivement le compteur
    }
    if (Cvr == 1 && !Allume) {
      ledcWrite(RgePwmChanel, 0);
      SlowBlink.attach_ms(config.SlowBlinker, blink);// Allumage Feux Rouge
      Allume = true;
    }
  } else if (FlagVerif_CVR) {
    // demande en cours et Fin de la tempo de Commande Position
    Cvr = Position_CVR(); // lire position CVR affectée à Cvr
    LastCvr = Cvr;
    // verification position valide
    bool valide = false;
    if (Memo_Demande_CVR[2].indexOf("O") == 0 && Cvr == 2) valide = true;
    if (Memo_Demande_CVR[2].indexOf("F") == 0 && Cvr == 1) valide = true;
    if (!valide)FlagAlarmePosition = true;
    // reponse au demandeur et message au serveur
    if (FlagFinJournee) { // arret auto demandé fin de journée
      generationMessage(0);
      Serial.println("message serveur");
      envoieGroupeSMS(3, 0); // envoie serveur
      FlagFinJournee = false;
      FinJournee();
    }
    GestionMessage_CVR();
    FlagDemande_CVR = false;
    FlagVerif_CVR   = false;
  }
  //***************************************************
  // gestion phase demarrage
  if (cpt > 6 && nsms == 0 && !firstdecision) {
    /* une seule fois au demarrage attendre au moins 70s et plus de sms en attente */
    action_wakeup_reason(get_wakeup_reason());
    firstdecision = true;
  }
  cpt ++;
  //***************************************************
  // patch relecture des coeff perdus
  if (CoeffTension[0] == 0 || CoeffTension[1] == 0 || CoeffTension[2] == 0 ) {
    OuvrirFichierCalibration();
  }
  //***************************************************
  
  // mesures tensions et print regulier
  Serial.println(displayTime(0));
  // Serial.print(F(" Freemem = ")), Serial.println(ESP.getFreeHeap());
  static byte nalaTension = 0;
  static byte nRetourTension = 0;
  TensionBatterie = map(adc_mm[0] / nSample, 0, 4095, 0, CoeffTension[0]);
  VBatterieProc   = map(adc_mm[1] / nSample, 0, 4095, 0, CoeffTension[1]);
  VUSB            = map(adc_mm[2] / nSample, 0, 4095, 0, CoeffTension[2]);

  int etatbatt = 0;
  if (config.TypeBatt == 16) etatbatt = BattPBpct(TensionBatterie, 6);
  if (config.TypeBatt == 24) etatbatt = BattLiFePopct(TensionBatterie, 4);

  if (etatbatt < 25 || VUSB < 4000) { // || VUSB > 6000
    nalaTension ++;
    if (nalaTension == 4) {
      FlagAlarmeTension = true;
      nalaTension = 0;
    }
  }
  else if (etatbatt >= 80 && VUSB >= 4500) { //  && VUSB < 5400	//hysteresis et tempo sur Alarme Batterie
    nRetourTension ++;
    if (nRetourTension == 4) {
      FlagAlarmeTension = false;
      nRetourTension = 0;
      nalaTension = 0;
    }
  }
  else {
    if (nalaTension > 0)nalaTension--; // efface progressivement le compteur
  }

  message = " Batt Solaire = ";
  message += float(TensionBatterie / 100.0);
  message += "V ";
  if (config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
  if (config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
  message += "%";
  message += ", Batt Proc = ";
  message += String(VBatterieProc) + "mV ";
  message += String(BattLipopct(VBatterieProc));
  message += "%, V USB = ";
  message += (float(VUSB / 1000.0));
  message += "V";
  message += fl;
  Serial.print(message);
  //***************************************************
  if (gsm) {
    // verification index new SMS en attente(raté en lecture directe)
    int smsnum = modem.newMessageIndex(0); // verifie index arrivée sms, -1 si pas de sms
    Serial.print(F("Index last SMS = ")), Serial.println (smsnum);

    if (smsnum >= 0) {	// index du SMS en attente
      // il faut les traiter
      traite_sms(smsnum);
    } 
    else if (smsnum < 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
      FlagReset = false;
      ResetHard();					//	reset hard
    }
  }
  else if (FlagReset) {
    FlagReset = false;
    ResetHard();				//	reset hard
  }
  if(gsm && config.messageMode == true && firstdecision){  //config.messageMode = SMS+MQTT
    static byte nalaGprs = 0;
    static byte nalaMQTT = 0;
    if (AlarmeGprs) {
      if (nalaGprs ++ > config.cptAla) {
        FlagAlarmeGprs = true;
        nalaGprs = 0;
      }
    } else {
      if (nalaGprs > 0) {
        nalaGprs --;
      } else {
        FlagAlarmeGprs = false;
      }
    }
    if (AlarmeMQTT) {
      if (nalaMQTT ++ > config.cptAla) {
        FlagAlarmeMQTT = true;
        nalaMQTT = 0;
      }
      Serial.print(F("AlarmeMQTT: ")),Serial.println(nalaMQTT);
    } else {
      FlagAlarmeMQTT = false;
      FlagAlarmeGprs = false;
      nalaMQTT = 0;
    }
  }
  //***************************************************
  envoie_alarme();
  //***************************************************
  digitalWrite(LED_PIN, 0);
  Alarm.delay(20);
  digitalWrite(LED_PIN, 1);
  //***************************************************
  Serial.println();
}
//---------------------------------------------------------------------------
void traite_sms(byte slot) {
  /*
  si slot=255, demande depuis message mqtt
  si slot=99, demande depuis liaison serie en test, traiter sans envoyer de sms
  */
  Sms smsstruct;
  Serial.print(F("slot: ")); Serial.println(slot);
  String nomAppelant;  	//nom expediteur SMS si existe dans Phone Book  
  bool smsserveur = false; // true si le sms provient du serveur index=1
  bool sms = true;

  /* Variables pour mode calibration */
  static int tensionmemo = 0;//	memorisation tension batterie lors de la calibration
  int coef = 0; // coeff temporaire
  static byte P = 0; // Pin entrée a utiliser pour calibration
  static byte M = 0; // Mode calibration 1,2,3,4
  static bool FlagCalibration = false;	// Calibration Tension en cours

  // if (slot == 99) sms = false;
  if (slot == 255){
    smsserveur = true;
    nomAppelant = "serveur_MQTT";
  }
  // /* Retrieve SMS sender address/phone number. */
  if (slot < 99) {
    if (!modem.readSMS(&smsstruct,slot)){
      Serial.print(F("Didn't find SMS message in slot!"));
      Serial.println(slot);
      // continue;	//	Next k
      return;
    }
    if(! Cherche_N_PB(smsstruct.sendernumber)){
      Serial.println(F("Appelant inconnu"));
    } else {
      Serial.print(F("Num :")), Serial.print(Phone.number);
      Serial.print(F(", Nom :")), Serial.println(Phone.text);
    }
    nomAppelant = Phone.text;
    // Serial.print(F("Nom appelant:")), Serial.println(nomAppelant);
    
    // cherche si numero appelant est le serveur position 1
    PhonebookEntry Phone2;
    Phone2 = {"",""};
    modem.readPhonebookEntry(&Phone2, 1); // lecture numero serveur 1
    if (Phone.number == Phone2.number) {
      smsserveur = true; // si demande provient du serveur index=1
      // Serial.print("Num appelant:"),Serial.print(Phone.number);
      // Serial.print("Num serveur :"),Serial.print(Phone2.number);
    }
  
    if(Phone.number.length() < 8){ // numero service free renvoie sms vers Num dans liste restreinte
    Serial.print("Phone.number.length() < 8:"),Serial.println(Phone.number.length());
      for (byte Index = 1; Index < 10; Index++) { // Balayage des Num Tel dans Phone Book
        Phone = {"",""};
        if(modem.readPhonebookEntry(&Phone, Index)){
          if(Phone.number.length() > 0){
            if (config.Pos_Pn_PB[Index] == 1) { // Num dans liste restreinte
              message = smsstruct.message;
              sendSMSReply(Phone.number , slot);
              EffaceSMS(slot);
              return; // sortir de la procedure traite_sms
            }
          } else {Index = 10;}
        }
      }
    }
    if (!(smsstruct.message.indexOf(F("MAJHEURE")) == 0)) { // suppression du SMS sauf si MAJHEURE
      EffaceSMS(slot);
      // Serial.print("efface sms slot dans premier1 slot <99:"),Serial.println(slot);
    }
  } else {
    smsstruct.message = replybuffer;
    if (slot == 99) nomAppelant = "console";
  } // si mesage venant de console
  Serial.print(F("texte du SMS :")), Serial.println(smsstruct.message);
  if (!(smsstruct.message.indexOf(F("TEL")) == 0 || smsstruct.message.indexOf(F("tel")) == 0 || smsstruct.message.indexOf(F("Tel")) == 0
      || smsstruct.message.indexOf(F("Wifi")) == 0 || smsstruct.message.indexOf(F("WIFI")) == 0 || smsstruct.message.indexOf(F("wifi")) == 0
      || smsstruct.message.indexOf(F("MQTTDATA")) > -1 || smsstruct.message.indexOf(F("MQTTSERVEUR")) > -1
      || smsstruct.message.indexOf(F("GPRSDATA")) > -1 || smsstruct.message.indexOf(F("FTPDATA")) > -1 || smsstruct.message.indexOf(F("FTPSERVEUR")) > -1)) {
    smsstruct.message.toUpperCase();	// passe tout en Maj sauf si "TEL" ou "WIFI" parametres pouvant contenir minuscules
    // smsstruct.message.trim();
  }
  smsstruct.message.replace(" ", "");// supp tous les espaces
  Serial.print("smsstruct.message  = "), Serial.println(smsstruct.message);

  if ((sms && nomAppelant.length() > 0) || !sms) {        // si nom appelant existant dans phone book
    // numero.toCharArray(number, numero.length() + 1); // on recupere le numéro
    messageId();
    if (smsstruct.message.indexOf("TIMEOUTWIFI") > -1) { // Parametre Arret Wifi
      if (smsstruct.message.indexOf(char(61)) == 11) {
        int n = smsstruct.message.substring(12, smsstruct.message.length()).toInt();
        if (n > 9 && n < 3601) {
          config.timeoutWifi = n;
          sauvConfig();														// sauvegarde config
        }
      }
      message += "TimeOut Wifi (s) = ";
      message += config.timeoutWifi;
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("WIFIOFF") > -1) { // Arret Wifi
      message += "Wifi off";
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
      WifiOff();
    }
    else if (smsstruct.message.indexOf("Wifi") == 0) { // demande connexion Wifi
      byte pos1 = smsstruct.message.indexOf(char(44));//","
      byte pos2 = smsstruct.message.indexOf(char(44), pos1 + 1);
      String ssids = smsstruct.message.substring(pos1 + 1, pos2);
      String pwds  = smsstruct.message.substring(pos2 + 1, smsstruct.message.length());
      char ssid[25];
      char pwd[30];
      ssids.toCharArray(ssid, ssids.length() + 1);
      ssids.toCharArray(ssid, ssids.length() + 1);
      pwds.toCharArray(pwd, pwds.length() + 1);
      ConnexionWifi(ssid, pwd, smsstruct.sendernumber, sms); // message généré par routine
    }
    else if (gsm && (smsstruct.message.indexOf(F("TEL")) == 0
              || smsstruct.message.indexOf(F("Tel")) == 0
              || smsstruct.message.indexOf(F("tel")) == 0)) { // entrer nouveau num
      String numero;
      String nom;
      byte index = 0;
      bool FlagOK = true;
      byte j = 0;
      bool add = true; // ajouter/modification numero = true, suppression = false
      // String sendAT	= F("AT+CPBW=");	// ecriture dans le phone book
      if (smsstruct.message.indexOf(char(61)) == 4) { // TELn= reserver correction/suppression
        int i = smsstruct.message.substring(3).toInt();// recupere n° de ligne
        i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
        //Serial.println(i);
        if (i < 1) FlagOK = false;
        // sendAT += i;
        index = i;
        j = 5;
        // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
        if ( (i != 1) && ( smsstruct.message.indexOf(F("efface")) == 5
                        || smsstruct.message.indexOf(F("EFFACE")) == 5 )){
          add = false;
          goto fin_tel;
        }
      }
      else if (smsstruct.message.indexOf(char(61)) == 3) { // TEL= nouveau numero
        j = 4;
      }
      else {
        FlagOK = false;
      }
      if (smsstruct.message.indexOf("+") == j) {			// debut du num tel +
        if (smsstruct.message.indexOf(",") == j + 12) {	// verif si longuer ok
          numero = smsstruct.message.substring(j, j + 12);
          nom    = smsstruct.message.substring(j + 13, j + 27);	// pas de verif si long<>0?
          // sendAT += F(",\"");
          // sendAT += numero;
          // sendAT += F("\",145,\"");
          // sendAT += nom;
          // sendAT += F("\"");
        }
        else {
          FlagOK = false;
        }
      }
      else {
        FlagOK = false;
      }
fin_tel:
      if (!FlagOK) { // erreur de format
        //Serial.println(F("false"));
        messageId();
        message += F("Commande non reconnue ?");// non reconnu
        sendSMSReply(smsstruct.sendernumber, slot);						// SMS non reconnu
      }
      else {          
        messageId();
        if(add){
          // ajouter N° à index
          message += F("Nouveau Num Tel: ");
          Serial.println(numero);
          Serial.println(nom);
          Serial.println(index);
          if(modem.addPhonebookEntry(numero,nom,index)){
            message += F("OK");
          } else { message += F("KO");}
          message += fl;
          message += index;
          message += ":";
          message += numero;
          message += ":";
          message += nom;
          message += fl;
        } else {
          // suppression N° à Index
          message += F("Suppression Num index: ");
          message += String(index);
          if(modem.deletePhonebookEntry(index)){
            message += F(" OK");
          } else { message += F(" KO");}
        }
        sendSMSReply(smsstruct.sendernumber, slot);
      }
    }
    else if (gsm && (smsstruct.message == F("LST") || smsstruct.message == F("LST?") || smsstruct.message == F("LST1"))) {	//	Liste des Num Tel
      messageId();
      for (byte i = 1; i < 10; i++) {
        Phone = {"",""};
        // Serial.print("lecture PB:"),Serial.println(i);
        if(modem.readPhonebookEntry(&Phone, i)){
          message += String(i) + ":";
          message += Phone.number;
          message += ",";
          message += Phone.text;
          message += "\n";
        } else {
          i = 10;
        }
      }
      sendSMSReply(smsstruct.sendernumber, slot);// envoi sur plusieurs SMS
    }
    else if (smsstruct.message.indexOf("ETAT") == 0 || smsstruct.message.indexOf("ST") == 0) {// "ETAT? de l'installation"
      generationMessage(0);
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf(F("SYS")) > -1) {
      if (gsm) {
        message += modem.getOperator(); // Operateur
        byte n = modem.getRegistrationStatus();        
        if (n == 5) {
          message += F("rmg, ");// roaming 1.0s
        }
        message += " ";
        message += ConnectedNetwork();
        message += fl;

        read_RSSI();														// info RSSI seront ajoutées à message
        
        message += F("Batt GSM : ");
        message += String(modem.getBattVoltage());
        message += F(" mV, ");
        // message += Batp;
        // message += F(" %");
        message += fl;
      }
      message += F("Ver: ");
      message += ver;
      message += fl;
      message += F("V Batt Sol= ");
      message += String(float(TensionBatterie / 100.0));
      message += F("V, ");
      if (config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
      if (config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
      message += " %";
      message += fl;
      message += F("V USB= ");
      message += (float(VUSB / 1000.0));
      message += "V";
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf(F("ID=")) == 0) {			//	Id= nouvel Id
      String temp = smsstruct.message.substring(3);
      if (temp.length() > 0 && temp.length() < 11) {
        Id = "";
        temp.toCharArray(config.Idchar, 11);
        strncpy(config.sendTopic,("Signalisation/input"),sizeof(config.sendTopic));
        memcpy(config.receiveTopic,&config.Idchar[5],5);
        config.receiveTopic[5] = '\0';
        strcat(config.receiveTopic,"/output");
        memcpy(config.permanentTopic,&config.Idchar[5],5);
        config.permanentTopic[5] = '\0';
        strcat(config.permanentTopic,"/permanent");
        sauvConfig();														// sauvegarde config

        if(config.messageMode == 1){ // changement d'id mqtt
          mqtt.disconnect();
          mqttConnect();
        }

        Id = String(config.Idchar);
        Id += fl;
      }
      messageId();
      message += F("Nouvel Id");
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf(F("LOG")) == 0) {	// demande taille du log
      File f = SPIFFS.open(filelog, "r"); // taille du fichier log en SPIFFS
      message = F("local log size :");
      message += String(f.size()) + fl;
      f.close();
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("ANTICIP") > -1) { // Anticipation du wakeup
      if (smsstruct.message.indexOf(char(61)) == 7) {
        int n = smsstruct.message.substring(8, smsstruct.message.length()).toInt();
        if (n > 9 && n < 3601) {
          config.anticip = n;
          sauvConfig();														// sauvegarde config
        }
      }
      message += "Anticipation WakeUp (s) = ";
      message += config.anticip;
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("DEBUT") == 0) {     //	Heure Message Vie/debutJour
      if (smsstruct.message.indexOf(char(61)) == 5) {
        long i = atol(smsstruct.message.substring(6).c_str()); //	Heure message Vie
        if (i > 0 && i <= 86340) {                    //	ok si entre 0 et 86340(23h59)
          config.DebutJour = i;
          sauvConfig();                               // sauvegarde config
          Alarm.disable(DebutJour);
          Alarm.write(DebutJour, config.DebutJour); // init tempo
          Alarm.enable(DebutJour);
          AIntru_HeureActuelle();
        }
      }
      message += "Debut Journee = ";
      message += Hdectohhmm(config.DebutJour);
      message += "(hh:mm)";
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("TIME") == 0) {
      message += "Heure Sys = ";
      message += displayTime(0);
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure
      if (sms) {
          String mytime = smsstruct.timestamp.substring(0, 20);
          // Serial.print(F("heure du sms:")),Serial.println(mytime);
          // String _temp = F("AT+CCLK=\"");
          String _temp = F("+CCLK=\"");
          _temp += mytime + "\"\r\n";
          // Serial.print(_temp);
          modem.send_AT(_temp);
          Alarm.delay(100);
          MajHeure(true);			// mise a l'heure forcée
        }
        else {
          message += F("pas de mise à l'heure en local");
        }
        sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (gsm && smsstruct.message.indexOf(F("IMEI")) > -1) {
      // char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
      String modemInfo = modem.getModemInfo(); // IMEI: 862195XXXXXX785
      byte pos = modemInfo.indexOf(F("IMEI:"));
      if (pos > 0) {
        // Serial.print(F("Module IMEI: ")), Serial.println(modemInfo.substring(pos+6, pos+6+15));
        message += F("IMEI = ");
        message += modemInfo.substring(pos+6, pos+6+15);
        sendSMSReply(smsstruct.sendernumber, slot);
      }
    }
    else if (smsstruct.message.indexOf("FIN") == 0) {			//	Heure Fin de journée
      if ((smsstruct.message.indexOf(char(61))) == 3) {
        long i = atol(smsstruct.message.substring(4).c_str()); //	Heure
        if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
          config.FinJour = i;
          sauvConfig();															// sauvegarde config
          Alarm.disable(FinJour);
          Alarm.write(FinJour, config.FinJour); // init tempo
          Alarm.enable(FinJour);
        }
      }
      message += "Fin Journee = ";
      message += Hdectohhmm(config.FinJour);
      message += "(hh:mm)";
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("MOIS") > -1) { // Calendrier pour un mois
      /* mise a jour calendrier ;format : MOIS=mm,31 fois 0/1
        demande calendrier pour un mois donné ; format : MOIS=mm? */
      bool flag = true; // validation du format
      bool W = true; // true Write, false Read
      int m = 0;
      if (smsstruct.message.indexOf("{") == 0) { // json
        DynamicJsonDocument doc(540);
        int f = smsstruct.message.lastIndexOf("}");
        DeserializationError err = deserializeJson(doc, smsstruct.message.substring(0, f + 1));
        if (!err) {
          m = doc["MOIS"]; // 12
          JsonArray jour = doc["JOUR"];
          for (int j = 1; j < 32; j++) {
            calendrier[m][j] = jour[j - 1];
          }
          // Serial.print("mois:"),Serial.println(m);
          EnregistreCalendrier(); // Sauvegarde en SPIFFS
        }
        else {
          message += " erreur json ";
          flag = false;
        }
      }
      else { // message normal mois=12,31*0/1
        byte p1 = smsstruct.message.indexOf(char(61)); // =
        byte p2 = smsstruct.message.indexOf(char(44)); // ,
        if (p2 == 255) {                      // pas de ,
          p2 = smsstruct.message.indexOf(char(63));    // ?
          W = false;
        }

        m = smsstruct.message.substring(p1 + 1, p2).toInt(); // mois
        if (!(m > 0 && m < 13)) flag = false;
        if (W && flag) { // Write
          if (!(smsstruct.message.substring(p2 + 1, smsstruct.message.length()).length() == 31)) flag = false; // si longueur = 31(jours)

          for (int i = 1; i < 32; i++) { // verification 0/1
            if (!(smsstruct.message.substring(p2 + i, p2 + i + 1) == "0" || smsstruct.message.substring(p2 + i, p2 + i + 1) == "1")) {
              flag = false;
            }
          }
          if (flag) {
            // Serial.println(F("mise a jour calendrier"));
            for (int i = 1; i < 32; i++) {
              calendrier[m][i] = smsstruct.message.substring(p2 + i, p2 + i + 1).toInt();
              // Serial.print(smsstruct.message.substring(p2+i,p2+i+1));
            }
            EnregistreCalendrier(); // Sauvegarde en SPIFFS
          }
        }
        if (!flag) {
          // printf("flag=%d,W=%d\n",flag,W);
          message += " erreur format ";
        }
      }
      if (flag) { // demande calendrier pour un mois donné
        if (smsserveur || !sms) {
          // si serveur reponse json  {"mois":12,"jour":[1,2,4,5,6 .. 31]}
          DynamicJsonDocument doc(540);
          doc["mois"] = m;
          JsonArray jour = doc.createNestedArray("jour");
          for (int i = 1; i < 32; i++) {
            jour.add(calendrier[m][i]);
          }
          String jsonbidon;
          serializeJson(doc, jsonbidon);
          message += jsonbidon;
        }
        else {
          message += "mois = ";
          message += m;
          message += fl;
          for (int i = 1; i < 32 ; i++) {
            message += calendrier[m][i];
            if ((i % 5)  == 0) message += " ";
            if ((i % 10) == 0) message += fl;
          }
        }
      }
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message == "CIRCULE") {
      bool ok = false;
      /* demande passer en mode Circulé pour le jour courant,
        sans modification calendrier enregistré en SPIFFS */
      if (!(calendrier[month()][day()] ^ FlagCircule)) {
        // calendrier[month()][day()] = 1;
        message += "OK, Circule";
        FlagCircule = !FlagCircule;
        ok = true;
      }
      else {
        message += "Jour deja Circule";
      }
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
      if (ok) {
        // if (sms)EffaceSMS(slot);
        SignalVie();
        // action_wakeup_reason(4);
      }
    }
    else if (smsstruct.message == "NONCIRCULE") {
      bool ok = false;
      /* demande passer en mode nonCirculé pour le jour courant,
        sans modification calendrier enregistré en SPIFFS
        extinction Feux*/
      if (calendrier[month()][day()] ^ FlagCircule) {
        // calendrier[month()][day()] = 0;
        message += "OK, NonCircule";
        FlagCircule = !FlagCircule;
        ok = true;
      }
      else {
        message += "Jour deja NonCircule";
      }
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
      if (ok) {
        // if (sms) {
        //   EffaceSMS(slot);
        // }
        // Extinction();
        action_wakeup_reason(4);
      }
    }
    else if (smsstruct.message.indexOf("TEMPOWAKEUP") == 0) { // Tempo wake up
      if ((smsstruct.message.indexOf(char(61))) == 11) {
        int i = smsstruct.message.substring(12).toInt(); //	durée
        if (i > 59 && i <= 36000) { // 1mn à 10H
          config.RepeatWakeUp = i;
          sauvConfig();															// sauvegarde config
        }
      }
      message += "Tempo repetition Wake up (s)=";
      message += config.RepeatWakeUp;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("LST2") > -1) { //	Liste restreinte	//  =LST2=0,0,0,0,0,0,0,0,0
      bool flag = true; // validation du format
      if (smsstruct.message.indexOf(char(61)) == 4) { // "="
        byte Num[10];
        Sbidon = smsstruct.message.substring(5, 22);
        // Serial.print("bidon="),Serial.print(Sbidon),Serial.print("="),Serial.println(Sbidon.length());
        if (Sbidon.length() == 17) {
          int j = 1;
          for (int i = 0; i < 17; i += 2) {
            if (i == 16 && (Sbidon.substring(i, i + 1) == "0"	|| Sbidon.substring(i, i + 1) == "1")) {
              Num[j] = Sbidon.substring(i, i + 1).toInt();
            }
            else if ((Sbidon.substring(i + 1, i + 2) == ",") && (Sbidon.substring(i, i + 1) == "0"	|| Sbidon.substring(i, i + 1) == "1")) {
              //Serial.print(",="),Serial.println(bidon.substring(i+1,i+2));
              //Serial.print("X="),Serial.println(bidon.substring(i,i+1));
              Num[j] = Sbidon.substring(i, i + 1).toInt();
              //Serial.print(i),Serial.print(","),Serial.print(j),Serial.print(","),Serial.println(Num[j]);
              j++;
            }
            else {
              Serial.println("Format pas reconnu");
              flag = false;
            }
          }
          if (flag) {
            //Serial.println("copie des num");
            for (int i = 1; i < 10; i++) {
              config.Pos_Pn_PB[i] = Num[i];
            }
            sauvConfig();															// sauvegarde config
          }
        }
      }
      message += "Liste restreinte";
      message += fl;
      for (int i = 1; i < 10; i++) {
        message += config.Pos_Pn_PB[i];
        if ( i < 9) message += char(44); // ,
      }
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message == "RST" || smsstruct.message == "RESET") {               // demande RESET
      message += "Le systeme va etre relance";  // apres envoie du SMS!
      message += fl;
      FlagReset = true;                            // reset prochaine boucle
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message == "FAULTRESET") { // reset Erreur Position
      FlagAlarmePosition = false;
      FlagLastAlarmePosition = false;
      Cvr = Position_CVR(); // lire position CVR affectée à Cvr
      LastCvr = Cvr;
      ledcWrite(RgePwmChanel, 0); // Extinction Feux Rouge
      Allume = false;
      SlowBlink.detach();
      envoieGroupeSMS(0, 0);		// envoie groupé
    }
    else if (smsstruct.message.indexOf("CALIBRATION=") == 0) {
      /* 	Mode calibration mesure tension
          recoit message "CALIBRATION=.X"
          entrer mode calibration
          Selection de la tenssion à calibrer X
          X = 1 TensionBatterie : PinBattSol : CoeffTension1
          X = 2 VBatterieProc : PinBattProc : CoeffTension2
          X = 3 VUSB : PinBattUSB : CoeffTension3
          // X = 4 Tension24 : Pin24V : CoeffTension4
          effectue mesure tension avec CoeffTensionDefaut retourne et stock resultat
          recoit message "CALIBRATION=1250" mesure réelle en V*100
          calcul nouveau coeff = mesure reelle/resultat stocké * CoeffTensionDefaut
          applique nouveau coeff
          stock en SPIFFS
          sort du mode calibration

          variables
          FlagCalibration true cal en cours, false par defaut
          Static P pin d'entrée
          static int tensionmemo memorisation de la premiere tension mesurée en calibration
          int CoeffTension = CoeffTensionDefaut 7000 par défaut
      */
      Sbidon = smsstruct.message.substring(12, 16); // texte apres =
      //Serial.print(F("Sbidon=")),Serial.print(Sbidon),Serial.print(char(44)),Serial.println(Sbidon.length());
      long tension = 0;
      if (Sbidon.substring(0, 1) == "." && Sbidon.length() > 1) { // debut mode cal
        if (Sbidon.substring(1, 2) == "1" ) {
          M = 1;
          P = PinBattSol;
          coef = CoeffTension[0];
        }
        if (Sbidon.substring(1, 2) == "2" ) {
          M = 2;
          P = PinBattProc;
          coef = CoeffTension[1];
        }
        if (Sbidon.substring(1, 2) == "3" ) {
          M = 3;
          P = PinBattUSB;
          coef = CoeffTension[2];
        }
        Serial.print("mode = "), Serial.print(M), Serial.println(Sbidon.substring(1, 2));
        FlagCalibration = true;

        coef = CoeffTensionDefaut;
        tension = map(adc_mm[M-1] / nSample, 0, 4095, 0, coef);
        // tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
        // Serial.print("TensionBatterie = "),Serial.println(TensionBatterie);
        tensionmemo = tension;
      }
      else if (FlagCalibration && Sbidon.substring(0, 4).toInt() > 0 && Sbidon.substring(0, 4).toInt() <= 8000) {
        // si Calibration en cours et valeur entre 0 et 5000
        Serial.println(Sbidon.substring(0, 4));
        /* calcul nouveau coeff */
        coef = Sbidon.substring(0, 4).toFloat() / float(tensionmemo) * CoeffTensionDefaut;
        // Serial.print("Coeff Tension = "),Serial.println(coef);
        tension = map(adc_mm[M-1] / nSample, 0, 4095, 0, coef);
        // tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
        CoeffTension[M - 1] = coef;
        FlagCalibration = false;
        Recordcalib();														// sauvegarde en SPIFFS
      }
      else {
        message += "message non reconnu";
        message += fl;
        FlagCalibration = false;
      }
      message += "Mode Calib Tension ";
      message += String(M) + fl;
      message += "TensionMesuree = ";
      message += tension;
      message += fl;
      message += "Coeff Tension = ";
      message += coef;
      if (M == 1) {
        message += fl;
        message += "Batterie = ";
        if(config.TypeBatt == 16) message += String(BattPBpct(tension, 6));
        if(config.TypeBatt == 24) message += String(BattLiFePopct(tension, 4));
        message += "%";
      }
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf(Id.substring(5, 9)) == 1) { // cherche CVR1
      if (smsstruct.message.indexOf("F") == 0) {
        // demande Fermeture
        Memo_Demande_CVR[0] = nomAppelant;            // nom demandeur
        Memo_Demande_CVR[1] = smsstruct.sendernumber; // num demandeur
        Memo_Demande_CVR[2] = smsstruct.message;      // demande d'origine
        FlagDemande_CVR = true;
        MajLog(nomAppelant,  "demande : " + smsstruct.message);
        Fermer_CVR();
      }
      else if (smsstruct.message.indexOf("O") == 0) {
        // demande Ouverture
        Memo_Demande_CVR[0] = nomAppelant;            // nom demandeur
        Memo_Demande_CVR[1] = smsstruct.sendernumber; // num demandeur
        Memo_Demande_CVR[2] = smsstruct.message;      // demande d'origine
        FlagDemande_CVR = true;
        MajLog(nomAppelant, "demande : " + smsstruct.message);
        Ouvrir_CVR();
      }
      else if (smsstruct.message.indexOf("D") == 0) {
        // si O rien faire
        // si F Ouvrir_CVR
        if(Cvr == 1){ // F
          Memo_Demande_CVR[0] = nomAppelant;            // nom demandeur
          Memo_Demande_CVR[1] = smsstruct.sendernumber; // num demandeur
          Memo_Demande_CVR[2] = smsstruct.message;      // demande d'origine
          FlagDemande_CVR = true;
          MajLog(nomAppelant, "demande : " + smsstruct.message);
          Ouvrir_CVR();
        }
      }
      else {
        message += "non reconnu" + fl;
        sendSMSReply(smsstruct.sendernumber, slot);
      }
    }
    else if (smsstruct.message.indexOf("TEMPOOUVERTURE") >= 0) {
      if (smsstruct.message.indexOf(char(61)) == 14) {
        int n = smsstruct.message.substring(15, smsstruct.message.length()).toInt();
        if (n > 4 && n < 121) {
          config.Tempoouverture = n;
          sauvConfig();														// sauvegarde config
        }
      }
      message += "Tempo Ouverture (s)= ";
      message += config.Tempoouverture;
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("TEMPOFERMETURE") >= 0) {
      if (smsstruct.message.indexOf(char(61)) == 14) {
        int n = smsstruct.message.substring(15, smsstruct.message.length()).toInt();
        if (n > 4 && n < 121) {
          config.Tempofermeture = n;
          sauvConfig();														// sauvegarde config
        }
      }
      message += "Tempo Fermeture (s)= ";
      message += config.Tempofermeture;
      message += fl;
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("PARAM") >= 0) {
      // conserve meme format que Cv, remplissage pour variables inexistante dummy
      // message param divisé en 2 trop long depasse long 1sms 160c
      int dummy = 0; // valeur bidon non utilisée ici, utilise meme sms que signalisation
      bool erreur = false;
      // Serial.print("position X:"),Serial.println(smsstruct.message.substring(7, 8));
      if (smsstruct.message.substring(7, 8) == "1") { // PARAM1
        // Serial.print("position ::"),Serial.println(smsstruct.message.substring(9, 10));
        if (smsstruct.message.substring(9, 10) == ":") {
          // json en reception sans lumlut
          DynamicJsonDocument doc(200);
          DeserializationError err = deserializeJson(doc, smsstruct.message);
          if (err) {
            erreur = true;
          }
          else {
            // Serial.print(F("Deserialization succeeded"));
            JsonObject param = doc["PARAM1"];
            config.SlowBlinker = param["SLOWBLINKER"];
            dummy = param["FASTBLINKER"];
            dummy = param["FASTRATER"];
            config.DebutJour = Hhmmtohdec(param["DEBUT"]);
            config.FinJour = Hhmmtohdec(param["FIN"]);
            sauvConfig();
            Alarm.disable(FinJour);
            Alarm.write(FinJour, config.FinJour);
            Alarm.enable(FinJour);
            Alarm.disable(DebutJour);
            Alarm.write(DebutJour, config.DebutJour);
            Alarm.enable(DebutJour);
          }
        }
        else {
          erreur = true;
        }
      }
      else if (smsstruct.message.substring(7, 8) == "2") { // PARAM2
        if (smsstruct.message.substring(9, 10) == ":") {
          // json en reception sans lumlut
          DynamicJsonDocument doc(200);
          DeserializationError err = deserializeJson(doc, smsstruct.message);
          if (err) {
            erreur = true;
          }
          else {
            // Serial.print(F("Deserialization succeeded"));
            JsonObject param = doc["PARAM2"];
            dummy = param["LUMAUTO"];
            config.FRgePWM = param["FVLTPWM"];
            dummy = param["FBLCPWM"];
            dummy = param["AUTOF"];
            dummy = param["TEMPOAUTOF"];
            sauvConfig();
          }
        }
      }
      if (!erreur) {
        // ne fonctionne pas
        // const size_t capacity = JSON_ARRAY_SIZE(11) + JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(11);
        // calculer taille https://arduinojson.org/v6/assistant/
        DynamicJsonDocument doc(500);
        JsonObject param = doc.createNestedObject("param");
        param["slowblinker"] = config.SlowBlinker;
        param["fastblinker"] = 5;
        param["fastrater"] = 5;
        param["debut"] = Hdectohhmm(config.DebutJour);
        param["fin"] = Hdectohhmm(config.FinJour);
        param["autof"] = 0;
        param["tempoautof"] = 100;
        param["fvltpwm"] = config.FRgePWM;
        param["fblcpwm"] = 50;
        param["lumauto"] = 0;

        JsonArray param_lumlut = param.createNestedArray("lumlut");
        for (int i = 0; i < 11; i++) {
          param_lumlut.add(0);
        }
        String jsonbidon;
        serializeJson(doc, jsonbidon);
        // serializeJson(doc, Serial);
        message += jsonbidon;
      }
      else {
        message += "erreur json";
      }
      message += fl;
      if (gsm) {
        sendSMSReply(smsstruct.sendernumber, slot);
      } else {
        Serial.println(message);
      }
    }
    else if (gsm && smsstruct.message.indexOf(F("UPLOADLOG")) == 0) {//upload log sur demande
      message += F("lancement upload log");
      message += fl;
      MajLog(nomAppelant, "upload log");// renseigne log
      Serial.println(F("Starting..."));
      byte reply = FTP_upload_function(filelog); // Upload fichier
      Serial.println("The end... Response: " + String(reply));

      if(reply == true){
        message += F("upload OK");
        SPIFFS.remove(filelog);  // efface fichier log
        MajLog(nomAppelant, "");         // nouveau log
        MajLog(nomAppelant, F("upload OK"));// renseigne nouveau log
      } else {
        message += F("upload fail");
        MajLog(nomAppelant, F("upload fail"));// renseigne log
      }
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (gsm && smsstruct.message.indexOf(F("COEFF")) == 0) {//Lecture/ecriture des coeff
      // COEFF=xxxx,xxxx,xxxx
      if(smsstruct.message.indexOf(char(61)) == 5){ // =
        Sbidon = smsstruct.message.substring(6, smsstruct.message.length());
        Serial.println(Sbidon);
        int tempo[3] = {0,0,0};
        byte p1 = 0;
        byte p2 = 0;
        bool flag = true;
        for(int i = 0; i < 3; i++){          
          // printf("i=%d,p1=%d,p2=%d\n",i,p1,p2);
          p2 = Sbidon.indexOf(char(44), p1 + 1); // ,
          tempo[i] = Sbidon.substring(p1,p2).toInt();
          if(tempo[i] < 0) flag = false;
          if(i!=3 && p2 == 255) flag = false;
          p1 = p2 + 1;          
          // printf("i=%d,p1=%d,p2=%d\n",i,p1,p2);
        }
        if (flag){ // format OK
          for(int i = 0; i < 3; i++){
            CoeffTension[i] = tempo[i];
          }
          Recordcalib(); // enregistre en SPIFFS
        }
      }
      message += "Coeff calibration:" + fl;
      for(int i = 0; i < 3; i++){
        message += String(CoeffTension[i]);
        if(i < 3 ) message += ",";
      }
      Serial.println(message);
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (gsm && smsstruct.message.indexOf(F("UPLOADCOEFF")) == 0) {//upload des coeff
      message += F("lancement upload Coeff");
      message += fl;
      MajLog(nomAppelant, "upload coeff");// renseigne log
      Serial.println(F("Starting..."));
      byte reply = FTP_upload_function(filecalibration); // Upload fichier
      Serial.println("The end... Response: " + String(reply));

      if(reply == true){
        message += F("upload OK");
        MajLog(nomAppelant, F("upload Coeff OK"));// renseigne nouveau log
      } else {
        message += F("upload fail");
        MajLog(nomAppelant, F("upload Coeff fail"));// renseigne log
      }
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("FTPDATA") > -1) {
      // Parametres FTPDATA=Serveur:User:Pass:port
      // {"FTPDATA":{"serveur":"dd.org","user":"user","pass":"pass","port":00}}
      bool erreur = false;
      bool formatsms = false;
      if (smsstruct.message.indexOf(":") == 10) { // format json
        DynamicJsonDocument doc(210); //https://arduinojson.org/v6/assistant/
        DeserializationError err = deserializeJson(doc, smsstruct.message);
        if (err) {
          erreur = true;
        }
        else {
          JsonObject ftpdata = doc["FTPDATA"];
          strncpy(config.ftpServeur,  ftpdata["serveur"], 26);
          strncpy(config.ftpUser,     ftpdata["user"],    9);
          strncpy(config.ftpPass,     ftpdata["pass"],    16);
          config.ftpPort         =    ftpdata["port"];
          sauvConfig();													// sauvegarde config
        }
      }
      else if ((smsstruct.message.indexOf(char(61))) == 7) { // format sms
        formatsms = true;
        byte w = smsstruct.message.indexOf(":");
        byte x = smsstruct.message.indexOf(":", w + 1);
        byte y = smsstruct.message.indexOf(":", x + 1);
        byte zz = smsstruct.message.length();
        if (smsstruct.message.substring(y + 1, zz).toInt() > 0) { // Port > 0
          if ((w - 7) < 25 && (x - w - 1) < 11 && (y - x - 1) < 16) {
            Sbidon = smsstruct.message.substring(7, w);
            Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
            Sbidon = smsstruct.message.substring(w + 1, x);
            Sbidon.toCharArray(config.ftpUser, (Sbidon.length() + 1));
            Sbidon = smsstruct.message.substring(x + 1, y);
            Sbidon.toCharArray(config.ftpPass, (Sbidon.length() + 1));
            config.ftpPort = smsstruct.message.substring(y + 1, zz).toInt();
            sauvConfig();													// sauvegarde config
          }
          else {
            erreur = true;
          }
        } else {
          erreur = true;
        }
      }
      if (!erreur) {
        if (formatsms) {
          message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant";
          message += fl;
          message += "Parametres FTP :";
          message += fl;
          message += "Serveur:" + String(config.ftpServeur) + fl;
          message += "User:"    + String(config.ftpUser) + fl;
          message += "Pass:"    + String(config.ftpPass) + fl;
          message += "Port:"    + String(config.ftpPort) + fl;
        }
        else {
          DynamicJsonDocument doc(210);
          JsonObject FTPDATA = doc.createNestedObject("FTPDATA");
          FTPDATA["serveur"] = config.ftpServeur;
          FTPDATA["user"]    = config.ftpUser;
          FTPDATA["pass"]    = config.ftpPass;
          FTPDATA["port"]    = config.ftpPort;
          Sbidon = "";
          serializeJson(doc, Sbidon);
          message += Sbidon;
          message += fl;
        }
      }
      else {
        message += "Erreur format";
        message += fl;
      }
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("FTPSERVEUR") == 0) { // Serveur FTP
      // case sensitive
      // FTPSERVEUR=xyz.org
      if (smsstruct.message.indexOf(char(61)) == 10) {
        Sbidon = smsstruct.message.substring(11);
        Serial.print("ftpserveur:"), Serial.print(Sbidon);
        Serial.print(" ,"), Serial.println(Sbidon.length());
        Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
        sauvConfig();
      }
      message += "FTPserveur =";
      message += String(config.ftpServeur);
      message += "\n au prochain demarrage";
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("MQTTDATA") > -1) {
      // Parametres MQTTDATA=serveur:user:pass:port:permanent_topic:send_topic:receive_topic
      // {"MQTTDATA":{"serveur":"xxxx.org","user":"uuu","pass":"passpass","port":9999,"permanent_topic":"CV65/permanent","send_topic":"Signalisation/input","receive_topic":"CV65/output"}}

      bool erreur = false;
      // bool formatsms = false;
      if (smsstruct.message.indexOf(":") == 11) { // format json
        DynamicJsonDocument doc(384); //https://arduinojson.org/v6/assistant/
        DeserializationError err = deserializeJson(doc, smsstruct.message);
        if (err) {
          erreur = true;
        }
        else {
          JsonObject mqttdata = doc["MQTTDATA"];
          strncpy(config.mqttServer,     mqttdata["serveur"],         26);
          strncpy(config.mqttUserName,   mqttdata["user"],            11);
          strncpy(config.mqttPass,       mqttdata["pass"],            16);
          config.mqttPort            =   mqttdata["port"];
          strncpy(config.permanentTopic, mqttdata["permanent_topic"], 20);
          strncpy(config.sendTopic,      mqttdata["send_topic"],      20);
          strncpy(config.receiveTopic,   mqttdata["receive_topic"],   17);
          sauvConfig();
        }
      }
      
      if (!erreur) {
        DynamicJsonDocument doc(384);
        JsonObject MQTTDATA = doc.createNestedObject("MQTTDATA");
        MQTTDATA["serveur"] = config.mqttServer;
        MQTTDATA["user"]    = config.mqttUserName;
        MQTTDATA["pass"]    = config.mqttPass;
        MQTTDATA["port"]    = config.mqttPort;
        MQTTDATA["permanent_topic"]   = config.permanentTopic;
        MQTTDATA["send_topic"]   = config.sendTopic;
        MQTTDATA["receive_topic"]   = config.receiveTopic;
        
        Sbidon = "";
        serializeJson(doc, Sbidon);
        message += Sbidon;
        message += fl;
      }
      else {
        message += "Erreur format";
        message += fl;
      }
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("MQTTSERVEUR") == 0) { // Serveur MQTT
      // case sensitive
      // MQTTSERVEUR=abcd.org
      if (smsstruct.message.indexOf(char(61)) == 11) {
        Sbidon = smsstruct.message.substring(12);
        Serial.print("mqttserveur:"),Serial.print(Sbidon);
        Serial.print(" ,"), Serial.println(Sbidon.length());
        Sbidon.toCharArray(config.mqttServer, (Sbidon.length() + 1));
        sauvConfig();
      }
      message += F("MQTTserveur =");
      message += String(config.mqttServer);
      message += F("\n au prochain demarrage");
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("GPRSDATA") > -1) {
      // Parametres GPRSDATA = "APN":"user":"pass"
      // GPRSDATA="sl2sfr":"":""
      // {"GPRSDATA":{"apn":"sl2sfr","user":"","pass":""}}
      bool erreur = false;
      bool formatsms = false;
      if (smsstruct.message.indexOf(":") == 11) { // format json
        DynamicJsonDocument doc(120);
        DeserializationError err = deserializeJson(doc, smsstruct.message);
        if (err) {
          erreur = true;
        }
        else {
          JsonObject gprsdata = doc["GPRSDATA"];
          strncpy(config.apn, gprsdata["apn"], 11);
          strncpy(config.gprsUser, gprsdata["user"], 11);
          strncpy(config.gprsPass, gprsdata["pass"], 11);
          // Serial.print("apn length:"),Serial.println(strlen(gprsdata["apn"]));
          // Serial.print("apn:"),Serial.println(config.apn);
          // Serial.print("user:"),Serial.println(config.gprsUser);
          // Serial.print("pass:"),Serial.println(config.gprsPass);
          sauvConfig();													// sauvegarde config
        }
      }
      else if ((smsstruct.message.indexOf(char(61))) == 8) { // format sms
        formatsms = true;
        byte cpt = 0;
        byte i = 9;
        do { // compte nombre de " doit etre =6
          i = smsstruct.message.indexOf('"', i + 1);
          cpt ++;
        } while (i <= smsstruct.message.length());
        Serial.print("nombre de \" :"), Serial.println(cpt);
        if (cpt == 6) {
          byte x = smsstruct.message.indexOf(':');
          byte y = smsstruct.message.indexOf(':', x + 1);
          byte z = smsstruct.message.lastIndexOf('"');
          // Serial.printf("%d:%d:%d\n",x,y,z);
          // Serial.printf("%d:%d:%d\n", x -1 - 10, y-1 - x-1-1, z - y-1-1);
          if ((x - 11) < 11 && (y - x - 3) < 11 && (z - y - 2) < 11) { // verification longueur des variables
            Sbidon = smsstruct.message.substring(10, x - 1);
            Sbidon.toCharArray(config.apn, (Sbidon.length() + 1));
            Sbidon = smsstruct.message.substring(x + 1 + 1 , y - 1);
            Sbidon.toCharArray(config.gprsUser, (Sbidon.length() + 1));
            Sbidon = smsstruct.message.substring(y + 1 + 1, z);
            Sbidon.toCharArray(config.gprsPass, (Sbidon.length() + 1));

            // Serial.print("apn:"),Serial.println(config.apn);
            // Serial.print("user:"),Serial.println(config.gprsUser);
            // Serial.print("pass:"),Serial.println(config.gprsPass);

            sauvConfig();													// sauvegarde config
          }
          else {
            erreur = true;
          }
        }
        else {
          erreur = true;
        }
      }
      if (!erreur) {
        if (formatsms) {
          message += "Sera pris en compte au prochain demarrage\nOu envoyer RST maintenant" + fl;
          message += "Parametres GPRS \"apn\":\"user\":\"pass\"";
          message += fl + "\"";
          message += String(config.apn);
          message += "\":\"";
          message += String(config.gprsUser);
          message += "\":\"";
          message += String(config.gprsPass);
          message += "\"" + fl;
        }
        else {
          DynamicJsonDocument doc(120);
          JsonObject gprsdata = doc.createNestedObject("GPRSDATA");
          gprsdata["apn"]  = config.apn;
          gprsdata["user"] = config.gprsUser;
          gprsdata["pass"] = config.gprsPass;
          Sbidon = "";
          serializeJson(doc, Sbidon);
          message += Sbidon;
          message += fl;
        }
      }
      else {
        message += "Erreur format";
        message += fl;
      }
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message == "VIDELOG"){
      SPIFFS.remove(filelog);
      FileLogOnce = false;
      message += "Effacement fichier log";
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message == "AUTOUPLOAD"){ // Auto upload log vers serveur FTP
      if (smsstruct.message.indexOf(char(61)) == 10) {
        byte c = smsstruct.message.substring(11).toInt();
        if(c==0 || c==1){
          config.autoupload = c;
          sauvConfig();
        }
        message += "Autoupload:";
        message += String(config.autoupload);
        sendSMSReply(smsstruct.sendernumber, slot);
      }
    }
    else if (smsstruct.message.indexOf(F("CPTALATRCK")) == 0 || smsstruct.message.indexOf(F("CPTALA")) == 0) { // Compteur Ala avant Flag
        if (smsstruct.message.indexOf(char(61)) == 10) {
          int c = smsstruct.message.substring(11).toInt();
          if (c > 1 && c < 501) {
            config.cptAla = c;
            sauvConfig();
          }
        }
        message += F("Cpt Ala Tracker (x15s)=");
        message += String(config.cptAla);
        message += fl;
        sendSMSReply(smsstruct.sendernumber, slot);
      }
    else if (smsstruct.message.indexOf(F("SETNETWORKMODE")) >= 0) {// Set Prefered network Mode
        if(smsstruct.message.indexOf(char(61)) == 14){
          int mode = smsstruct.message.substring(15).toInt();
          if(mode == 2 || mode == 13 || mode == 14 || mode == 19 || mode == 38
          || mode == 39 || mode == 51 || mode == 54){
            modem.setNetworkMode(mode);
            delay(1000);
          }
        }
        message += String(modem.send_AT(F("+CNMP?")));
        sendSMSReply(smsstruct.sendernumber, slot);
      }
    else if (smsstruct.message.indexOf(F("SENDAT")) == 0){
      // envoie commande AT au SIM7600
      // ex: SENDAT=AT+CCLK="23/07/19,10:00:20+04" mise à l'heure
      // attention DANGEREUX pas de verification!
      if (smsstruct.message.indexOf(char(61)) == 6) {
        String CdeAT = smsstruct.message.substring(7, smsstruct.message.length());
        String reply = sendAT(CdeAT,"OK","ERROR",1000);
        // Serial.print("reponse: "),Serial.println(reply);
        message += String(reply);
        sendSMSReply(smsstruct.sendernumber, slot);
      }
    }
    else if (smsstruct.message.indexOf(F("MODEMINFO")) == 0){
      // Get Modem Info
      message += modem.getModemInfo();
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf(F("CPTRESETMODEM")) == 0){
      // Demande nombre de reset modem
      message += F("Compteur reset Modem : ");
      message += String(NbrResetModem);
      sendSMSReply(smsstruct.sendernumber, slot);
    }else if (smsstruct.message.indexOf(F("NETWORKHISTO")) == 0){
      // Demande Changement etat reseau
      message_Monitoring_Reseau();
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf(F("TYPEBATT")) == 0){ // Type Batterie
      if (smsstruct.message.indexOf(char(61)) == 8) {
        int type = smsstruct.message.substring(9, smsstruct.message.length()).toInt();
        if(type == 16 || type == 24){
          config.TypeBatt = type;
          sauvConfig();													// sauvegarde config
        }
      }
      message += "Type Batterie:" + fl;
      if(config.TypeBatt == 16) message += "Pb 12V";
      if(config.TypeBatt == 24) message += "LiFePO 12.8V";
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    else if (smsstruct.message.indexOf("MESSAGEMODE") == 0) {
      // mode communcication 0 SMS only, 1 SMS+MQTT
      if (smsstruct.message.indexOf(char(61)) == 11) {
        int i = atoi(smsstruct.message.substring(12).c_str());
        if (i == 0  || i == 1){
          if(i == 1 && config.messageMode == 1){ // MQTT activer GPRS
            ConnectGPRS();
          } else if(i == 0 && config.messageMode == 0) { // SMS desactiver GPRS
            mqtt.disconnect();
            modem.gprsDisconnect();
          }
          config.messageMode = i;
          sauvConfig();													// sauvegarde config
          Sbidon = F("messagemode=");
          Sbidon += String(config.messageMode);
          MajLog(nomAppelant, Sbidon);// renseigne log
        }
      }
      message += "Message mode : ";
      if(config.messageMode == 0){
        message += "SMS";
      } else { message += "SMS + MQTT";}
      sendSMSReply(smsstruct.sendernumber, slot);
    }
    //**************************************
    else {
      message += F("Commande non reconnue ?");		//"Commande non reconnue ?"
      sendSMSReply(smsstruct.sendernumber, slot);
    }
  }
  else {
    Sbidon = F("Appelant non reconnu ! ");
    Sbidon += String(smsstruct.sendernumber);
    Serial.println(Sbidon);
    MajLog("Auto", Sbidon);// renseigne log
  }

}
//---------------------------------------------------------------------------
void envoie_alarme() {
  /* determine si un SMS appartition/disparition Alarme doit etre envoyé */
  bool SendEtat = false;

  if (FlagAlarmePosition != FlagLastAlarmePosition) {
    SendEtat = true;
    MajLog("Auto", "AlarmePosition");
    FlagLastAlarmePosition = FlagAlarmePosition;
  }
  if (FlagAlarmeTension != FlagLastAlarmeTension) {
    SendEtat = true;
    MajLog("Auto", "AlarmeTension");
    FlagLastAlarmeTension = FlagAlarmeTension;
  }
  if (SendEtat) { 						// si envoie Etat demandé
    envoieGroupeSMS(0, 0);		// envoie groupé
    SendEtat = false;					// efface demande
  }
}
//---------------------------------------------------------------------------
void envoieGroupeSMS(byte grp, bool vie) {
  if (gsm) {
    /* 
      si grp = 0,
      envoie un SMS à tous les numero existant (9 max) du Phone Book
      si grp = 1,
      envoie un SMS à tous les numero existant (9 max) du Phone Book
      de la liste restreinte config.Pos_Pn_PB[x]=1
      si grp = 3,
      Message au Serveur seulement N°1 de la liste			
      vie = true, ajoute au message nombre de reset modem
      et envoie sms y compris si messageMode = 1 */
    generationMessage(0);
    if(vie){
      message += F("Reset modem : ");
      message += String(NbrResetModem);
      message += fl;
      message_Monitoring_Reseau();
    }
    for (byte Index = 1; Index < 10; Index++) {		// Balayage du PB
      Phone = {"",""};      
      if(modem.readPhonebookEntry(&Phone, Index)){
        // Serial.print("groupe:"),Serial.println(grp);
        // Serial.print("Index:"),Serial.println(Index);
        // Serial.print("PB name:"),Serial.println(Phone.text);
        if(Phone.number.length() > 0){
          if (grp == 3){ // Serveur
            if(Index == 1 && config.messageMode == 1){ // Serveur et MQTT
              Envoyer_MQTT();
            } else { // SMS
              sendSMSReply(Phone.number, true);
            }
            break; // Sortir, Serveur seulement
          } else if (grp == 0){ // tous SMS
            sendSMSReply(Phone.number, true);
          } else if (grp == 1) { // liste restreinte seulement
            for (byte Index2 = 1; Index2 <10;Index2 ++){		// Balayage des Num Tel Autorisés=1 dans Phone Book
              // Serial.print(Index),Serial.print(","),Serial.println(Phone.number);            
              if (config.Pos_Pn_PB[Index2] == 1){          
                sendSMSReply(Phone.number, true);
              }
            }
          }
        } else {
          break;
        }
      }
    }    
  }
}
//---------------------------------------------------------------------------
void generationMessage(bool n) {
  messageId();
    if (FlagAlarmeTension || FlagLastAlarmeTension || FlagAlarmePosition
      || FlagAlarmeGprs || FlagAlarmeMQTT) {
    message += "--KO--------KO--";
  }
  else {
    message += "-------OK-------";
  }
  message += fl;

  switch (Cvr) {
    case 0: // Indefini
      message += "Z";
      break;
    case 1: // Fermé
      message += "F";
      break;
    case 2: // Ouvert
      message += "O";
      break;
  }
  message += String(Id.substring(5, 9));
  message += fl;
  message += "Batterie : ";
  if (!FlagAlarmeTension) {
    message += "OK, ";
    if(config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
    if(config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
    message += "%" + fl;
  }
  else {
    message += "Alarme, ";
    if(config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
    if(config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
    message += "%";
    message += fl;
    message += "V USB =";
    message += String(float(VUSB / 1000.0)) + fl;
  }
  if ((calendrier[month()][day()] ^ FlagCircule)) {
    message += "Jour Circule" + fl;
  }
  else {
    message += "Jour Non Circule" + fl;
  }
  message += fl;
  if(FlagAlarmePosition){
    message += "Defaut Position" + fl;
  }

  message += F("Gprs ");
  if (FlagAlarmeGprs) {
    message += F("KO");
  } else {
    message += F("OK");
  }
  message += fl;
  message += F("Mqtt ");
  if (FlagAlarmeMQTT) {
    message += F("KO");
  } else {
    message += F("OK");
  }
  message += fl;
}
//---------------------------------------------------------------------------
void sendSMSReply(String num , int sms) {
  // sms = 0-98 sms, 99 reponse local, 255 MQTT
  int pseq = 0;
  if (gsm) {
    
    if (sms == 255 && config.messageMode){ // MQTT
      // reponse MQTT
      Envoyer_MQTT();
    }
    if (sms < 99) { // SMS
      Serial.print(F("SMS Sent :")),Serial.println(num);
      if(message.length() > 150){ // decoupage sms en nseq parties
        int nseq = message.length()/150;
        if(nseq * 150 < message.length()){
          nseq += 1;
        }
        int fin = 0;
        for (byte i = 0; i < nseq;i++){
          if((i+1)*150 > message.length()){
            fin = message.length();
          } else {
            fin = (i+1)*150;
          }
          // Serial.print("seq"),Serial.print(i+1),Serial.print(":"),Serial.print(i*150),Serial.print(":"),Serial.print(fin),Serial.println(":"),Serial.println(message.substring((i*150),fin));
        
          Serial.print(F("sms part :")),Serial.print(i+1),Serial.println(message.substring(0,pseq));
          if (!modem.sendSMS_Multi(num,message.substring((i*150),fin),i+1,nseq)) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK"));
          }
          delay(10);
        }
      } else { // 1 seul sms
        if (!modem.sendSMS(num,message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK"));
        }
      }
    }
  }
  Serial.println(F("****************************"));
  Serial.println(message);
  Serial.println(F("****************************"));
}
//---------------------------------------------------------------------------
void Envoyer_MQTT(){
  Serial.println("message a envoyer MQTT:"),Serial.println(message);
  Sbidon = message;
  Sbidon.toCharArray(replybuffer, Sbidon.length() + 1);
  byte cpt =0;

  while( mqtt.publish(config.sendTopic, replybuffer) != 1){
    Alarm.delay(500);
    if(cpt ++ > 2){
      Serial.print(F("send mqtt KO:")),Serial.println(cpt);
      return;
    }
  }
  Serial.println(F("send mqtt OK:"));
}
//---------------------------------------------------------------------------
void read_RSSI() {	// lire valeur RSSI et remplir message
  if (gsm) {
    int r;
    byte n = modem.getSignalQuality();
    // Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
    if (n == 0) r = -115;
    if (n == 1) r = -111;
    if (n == 31) r = -52;
    if ((n >= 2) && (n <= 30)) {
      r = map(n, 2, 30, -110, -54);
    }
    message += "RSSI=";
    message += String(n);
    message += ", ";
    message += String(r);
    message += "dBm";
    message += fl;
  }
}
//---------------------------------------------------------------------------
void timesstatus() {	// etat synchronisation time/heure systeme
  Serial.print(F("Synchro Time  : "));
  switch (timeStatus()) {
    case 0:
      Serial.println(F(" pas synchro"));
      break;
    case 1:
      Serial.println(F(" defaut synchro"));
      break;
    case 2:
      Serial.println(F(" OK"));
      break;
  }
}
//---------------------------------------------------------------------------
void MajHeure(bool force) {
  // force = true, force mise à l'heure systeme sur heure modem, meme si defaut NTP
  Monitoring_Reseau();
  static bool First = true;
  if (gsm) {
    Serial.print(F("Mise a l'heure reguliere !, "));
    Serial.println(First);
    if (First) {															  // premiere fois apres le lancement
      SyncHeureModem(config.hete*4, true);
      readmodemtime();	// lire l'heure du modem
      setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
      if(!HeureEte()){
        // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hhiver*4));
        SyncHeureModem(config.hhiver*4, true);
        readmodemtime();	// lire l'heure du modem
        setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino
      }
      First = false;
    }
    else {
      //  calcul décalage entre H sys et H reseau en s
      // resynchroniser H modem avec reseau
      if(HeureEte()){
        // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hete*4));
        if(!SyncHeureModem(config.hete*4, false)){
          if(!force)return; // sortie sans mise à l'heure, continue si forcé
        }
      } else {
        // Serial.print("NTP:"),Serial.println(modem.NTPServerSync(NTPServer, config.hhiver*4));
        if(!SyncHeureModem(config.hhiver*4, false)){
          if(!force)return; // sortie sans mise à l'heure, continue si forcé
        }
      }
      readmodemtime();	// lire l'heure du modem
      //  calcul décalage entre H sys et H reseau en s
      int ecart = (N_H - hour()) * 3600;
      ecart += (N_m - minute()) * 60;
      ecart += N_S - second();
      // ecart += 10;
      Serial.print(F("Ecart s= ")), Serial.println(ecart);
      if (abs(ecart) > 5) {
        Alarm.disable(loopPrincipale);
        Alarm.disable(DebutJour);
        Alarm.disable(FinJour);
        // Alarm.disable(Auto_F);

        readmodemtime();	// mise à l'heure de l'Arduino
        setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure de l'Arduino

        Alarm.enable(loopPrincipale);
        Alarm.enable(DebutJour);
        Alarm.enable(FinJour);
        // if (config.AutoF)Alarm.enable(Auto_F); // armement TempoAutoF
      }
    }
  }
  displayTime(0);
  timesstatus();
  AIntru_HeureActuelle();
}
//---------------------------------------------------------------------------
long DureeSleep(long Htarget) { // Htarget Heure de reveil visée
  /* calcul durée entre maintenant et Htarget */
  long SleepTime = 0;
  long Heureactuelle = HActuelledec();
  if (Heureactuelle < Htarget) {
    SleepTime = Htarget - Heureactuelle;
  }
  else {
    if (Heureactuelle < 86400) { // < 24h00
      SleepTime = (86400 - Heureactuelle) + Htarget;
    }
  }
  return SleepTime;
}
//---------------------------------------------------------------------------
long HActuelledec() {
  long Heureactuelle = hour() * 60; // calcul en 4 lignes sinon bug!
  Heureactuelle += minute();
  Heureactuelle  = Heureactuelle * 60;
  Heureactuelle += second(); // en secondes
  return Heureactuelle;
}
//---------------------------------------------------------------------------
void SignalVie() {
  Serial.println("Signal vie");
  if (gsm) {
    MajHeure("");
    modem.deleteSmsMessage(0,4);// au cas ou, efface tous les SMS envoyé/reçu
  }

  if ((calendrier[month()][day()] ^ FlagCircule) && jour) { // jour circulé
    // 11 jour pour cas lancement de nuit pas d'allumage
    Sbidon = "Jour circule ou demande circulation";
    Serial.println(Sbidon);
    MajLog("Auto", Sbidon);
  }
  envoieGroupeSMS(0, 0);
  action_wakeup_reason(4);
}
//---------------------------------------------------------------------------
String displayTime(byte n) {
  // n = 0 ; dd/mm/yyyy hh:mm:ss
  // n = 1 ; yyyy-mm-dd hh:mm:ss
  char bid[20];
  if (n == 0) {
    sprintf(bid, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  }
  else {
    sprintf(bid, "%4d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  }
  return String(bid);
}
//---------------------------------------------------------------------------
void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
  file.close();
}
//---------------------------------------------------------------------------
void readFileCalendrier(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println(F("- failed to open file for reading"));
    return;
  }
  String buf = "";
  int i = 0;
  // Serial.println("- read from file:");
  while (file.available()) {
    int inchar = file.read();
    if (isDigit(inchar)) {
      buf += char(inchar);
      i ++;
    }
  }
  int m = 0;
  int j = 0;
  for (int i = 0; i < 372; i++) { // 12mois de 31 j =372
    j = 1 + (i % 31);
    if (j == 1) m ++;
    calendrier[m][j] = buf.substring(i, i + 1).toInt();
  }
}
//---------------------------------------------------------------------------
void appendFile(fs::FS &fs, const char * path, const char * message) {
  // Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    // Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    // Serial.println("- message appended");
  } else {
    // Serial.println("- append failed");
  }
}
//---------------------------------------------------------------------------
// Read Config
void readConfig(){
  Serial.printf("Reading file: %s\r\n", fileconfig);

  File file = SPIFFS.open(fileconfig);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return;
  }

  Serial.println("- read from file:");
  file.read((byte *)&config, sizeof(config));
  file.close();
}
//---------------------------------------------------------------------------
// Sauvegarde Config
void sauvConfig(){
  Serial.printf("Writing file: %s\r\n", fileconfig);

  File file = SPIFFS.open(fileconfig, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.write((byte *)&config, sizeof(config))){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}
//---------------------------------------------------------------------------
// Read File
void readFile(fs::FS &fs, const char * path){ // pas utilisé ici
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return;
  }

  Serial.println("- read from file:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}
//---------------------------------------------------------------------------
// Write File
void writeFile(fs::FS &fs, const char * path, const char * message){ // pas utilisé ici
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}
//---------------------------------------------------------------------------
void MajLog(String Id, String Raison) { // mise à jour fichier log en SPIFFS
  if(SPIFFS.exists(filelog)){
    /* verification de la taille du fichier */
    File f = SPIFFS.open(filelog, "r");
    Serial.print(F("Taille fichier log = ")), Serial.println(f.size());
    // Serial.print(Id),Serial.print(","),Serial.println(Raison);
    if (f.size() > 150000 && !FileLogOnce) {
      /* si trop grand on efface */
      FileLogOnce = true;
      messageId();
      message += F("Fichier log presque plein\n");
      message += String(f.size());
      message += F("\nFichier sera efface a 300000");
      if (gsm) {
        Phone = {"",""};
        if(modem.readPhonebookEntry(&Phone, 1)){ // envoyé au premier num seulement
          sendSMSReply(Phone.number, true);          
        }
      }
    }
    else if (f.size() > 300000 && FileLogOnce) { // 292Ko 75000 lignes
      messageId();
      message += F("Fichier log plein\n");
      message += String(f.size());
      if(config.autoupload){
        message += F("\nFichier upload vers serveur ");
        if(FTP_upload_function(filelog)){
          message += ("OK");
        } else {
          message += ("KO");
        }
      } else {
        message += F("\nFichier efface");
      }
      if (gsm) {
        Phone = {"",""};
        if(modem.readPhonebookEntry(&Phone, 1)){ // envoyé au premier num seulement
          sendSMSReply(Phone.number, true);          
        }
      }
      f.close();
      SPIFFS.remove(filelog);
      FileLogOnce = false;
    }
    f.close();
    /* preparation de la ligne */
    char Cbidon[101]; // 100 char maxi
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
    Id = ";" + Id + ";";
    Raison += "\n";
    strcat(Cbidon, Id.c_str());
    strcat(Cbidon, Raison.c_str());
    Serial.println(Cbidon);
    appendFile(SPIFFS, filelog, Cbidon);
  }
  else{ // fichier n'existe pas, création fichier avec première ligne date et Id
    char Cbidon[101]; // 100 char maxi
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d;", day(), month(), year(), hour(), minute(), second());
    strcat(Cbidon,config.Idchar);
    strcat(Cbidon,fl.c_str());
    appendFile(SPIFFS, filelog, Cbidon);
    Serial.print("nouveau fichier log:"),Serial.println(Cbidon);
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
    Id = ";" + Id + ";";
    Raison += "\n";
    strcat(Cbidon, Id.c_str());
    strcat(Cbidon, Raison.c_str());
    appendFile(SPIFFS, filelog, Cbidon);
  }
}
//---------------------------------------------------------------------------
void EnregistreCalendrier() { // remplace le calendrier

  SPIFFS.remove(filecalendrier);
  Sbidon = "";
  char bid[63];
  for (int m = 1; m < 13; m++) {
    for (int j = 1; j < 32; j++) {
      Sbidon += calendrier[m][j];
      if (j < 31)Sbidon += char(59); // ;
    }
    Serial.println(Sbidon);
    Sbidon += fl;
    Sbidon.toCharArray(bid, 63);
    appendFile(SPIFFS, filecalendrier, bid);
    Sbidon = "";
  }
}
//---------------------------------------------------------------------------
void OuvrirCalendrier() {

  listDir(SPIFFS, "/", 0);
  bool f = SPIFFS.exists(filecalendrier);
  // Serial.println(f);
  File f0 = SPIFFS.open(filecalendrier, "r");

  if (!f || f0.size() == 0) {
    Serial.println("File doesn't exist yet. Creating it"); // creation calendrier defaut
    char bid[63];
    Sbidon = "";
    for (int m = 1; m < 13; m++) {
      for (int j = 1; j < 32; j++) {
        if (m == 1 || m == 2 || m == 3 || m == 11 || m == 12) {
          Sbidon += "0;";
        }
        else {
          Sbidon += "1;";
        }
      }
      Serial.println(Sbidon);
      Sbidon += fl;
      Sbidon.toCharArray(bid, 63);
      appendFile(SPIFFS, filecalendrier, bid);
      Sbidon = "";
    }
  }
  readFileCalendrier(SPIFFS, filecalendrier);

  for (int m = 1; m < 13; m++) {
    for (int j = 1; j < 32; j++) {
      Serial.print(calendrier[m][j]), Serial.print(char(44));
    }
    Serial.println();
  }
  listDir(SPIFFS, "/", 0);

}
//---------------------------------------------------------------------------
void FinJournee() {
  // fin de journée retour deep sleep
  // si CVR fermé, demande ouverture avant fin de journée
  static bool first = true; // autorise une seule tentative ouverture
  if (Cvr == 1 && first) {
    first = false;
    FlagFinJournee = true;
    MajLog("Auto", "Ouverture Auto");
    Memo_Demande_CVR[2]="OXXXX";
    Ouvrir_CVR();
    return;
  }
  jour = false;
  FlagCircule = false;
  FirstWakeup = true;
  Serial.println("Fin de journee retour sleep");
  TIME_TO_SLEEP = DureeSleep(config.DebutJour - config.anticip);// xx mn avant
  // calculTimeSleep();
  Sbidon  = "FinJour, sleep for ";
  Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog("Auto", Sbidon);
  DebutSleep();
}
//---------------------------------------------------------------------------
void PrintConfig() {
  Serial.print("Version = ")                 , Serial.println(ver);
  Serial.print("ID = ")                      , Serial.println(config.Idchar);
  Serial.print("magic = ")                   , Serial.println(config.magic);
  Serial.print("Debut Jour = ")              , Serial.println(config.DebutJour);
  Serial.print("Fin jour = ")                , Serial.println(config.FinJour);
  Serial.print("T anticipation Wakeup = ")   , Serial.println(config.anticip);
  Serial.print("Tempo repetition Wake up (s)= "), Serial.println(config.RepeatWakeUp);
  Serial.print("Time Out Wifi (s)= ")        , Serial.println(config.timeoutWifi);
  Serial.print("Tempo ouverture (s) = ")     , Serial.println(config.Tempoouverture);
  Serial.print("Tempo fermeture (s) = ")     , Serial.println(config.Tempofermeture);
  Serial.print("Vitesse SlowBlinker = ")     , Serial.println(config.SlowBlinker);
  Serial.print("PWM Feux Rouge = ")          , Serial.println(config.FRgePWM);
  Serial.print("Type Batterie = ");
  if(config.TypeBatt == 16) Serial.println(F("Pb 12V 6elts"));
  if(config.TypeBatt == 24) Serial.println(F("LiFePO 12.8V 4elts"));
  Serial.print("Liste Restreinte = ");
  for (int i = 1; i < 10; i++) {
    Serial.print(config.Pos_Pn_PB[i]);
    if (i == 9) {
      Serial.println();
    }
    else {
      Serial.print(",");
    }
  }
  Serial.print("GPRS APN = ")                , Serial.println(config.apn);
  Serial.print("GPRS user = ")               , Serial.println(config.gprsUser);
  Serial.print("GPRS pass = ")               , Serial.println(config.gprsPass);
  Serial.print("ftp serveur = ")             , Serial.println(config.ftpServeur);
  Serial.print("ftp port = ")                , Serial.println(config.ftpPort);
  Serial.print("ftp user = ")                , Serial.println(config.ftpUser);
  Serial.print("ftp pass = ")                , Serial.println(config.ftpPass);
  Serial.print("mqtt serveur = ")            , Serial.println(config.mqttServer);
  Serial.print("mqtt port = ")               , Serial.println(config.mqttPort);
  Serial.print("mqtt username = ")           , Serial.println(config.mqttUserName);
  Serial.print("mqtt pass = ")               , Serial.println(config.mqttPass);
  Serial.print("sendTopic = ")               , Serial.println(config.sendTopic);
  Serial.print("receiveTopic = ")            , Serial.println(config.receiveTopic);
  Serial.print("permanentTopic = ")          , Serial.println(config.permanentTopic);
  Serial.print("msg Mode 0SMS,1SMS+MQTT = ") , Serial.println(config.messageMode);
  Serial.print("declage Heure ete = ")       , Serial.println(config.hete);
  Serial.print("declage Heure hiver = ")     , Serial.println(config.hhiver);
  Serial.print("autoupload = ")              , Serial.println(config.autoupload);
}
//---------------------------------------------------------------------------
void ConnexionWifi(char* ssid, char* pwd, String number, int slot) {

  messageId();
  Serial.print("connexion Wifi:"), Serial.print(ssid), Serial.print(char(44)), Serial.println(pwd);
  String ip;
  WiFi.begin(ssid, pwd);
  // WiFi.mode(WIFI_STA);
  byte timeout = 0;
  bool error = false;

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    timeout ++;
    if (timeout > 60) {
      error = true;
      break;
    }
  }
  if (!error) {
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    ip = WiFi.localIP().toString();
    Serial.println(ip);
    ArduinoOTA.begin();

    server.on("/",         HomePage);
    server.on("/download", File_Download);
    server.on("/upload",   File_Upload);
    server.on("/fupload",  HTTP_POST, []() {
      server.send(200);
    }, handleFileUpload);
    server.on("/delete",   File_Delete);
    server.on("/dir",      SPIFFS_dir);
    server.on("/cal",      CalendarPage);
    server.on("/Tel_list", Tel_listPage);
    server.on("/timeremaining", handleTime); // renvoie temps restant sur demande
    server.on("/datetime", handleDateTime); // renvoie Date et Heure
    server.on("/wifioff",  WifiOff);
    ///////////////////////////// End of Request commands
    server.begin();
    Serial.println("HTTP server started");

    message += "Connexion Wifi : ";
    message += fl;
    message += String(ip);
    message += fl;
    message += String(WiFi.RSSI());
    message += " dBm";
    message += fl;
    message += "TimeOut Wifi ";
    message += config.timeoutWifi;
    message += " s";
  }
  else {
    message += "Connexion Wifi impossible";
  }
  sendSMSReply(number, slot);

  // if (sms) { // suppression du SMS
  //   /* Obligatoire ici si non bouclage au redemarrage apres timeoutwifi
  //     ou OTA sms demande Wifi toujours present */
  //   EffaceSMS(slot);
  // }
  debut = millis();
  if (!error) {
    /* boucle permettant de faire une mise à jour OTA et serveur, avec un timeout en cas de blocage */
    unsigned long timeout = millis();
    while (millis() - timeout < config.timeoutWifi * 1000) {
      // if(WiFi.status() != WL_CONNECTED) break; // wifi a été coupé on sort
      ArduinoOTA.handle();
      server.handleClient(); // Listen for client connections
      delay(1);
    }   
  }
  WifiOff();
}
//---------------------------------------------------------------------------
void WifiOff() {
  Serial.println("Wifi off");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_MODE_NULL);
  btStop();
  Alarm.delay(100);
  ResetHard();
}
//---------------------------------------------------------------------------
void ResetHard() {
  // GPIO13 to RS reset hard
  pinMode(PinReset, OUTPUT);
  digitalWrite(PinReset, LOW);
}
//---------------------------------------------------------------------------
int moyenneAnalogique(int Pin) {	// calcul moyenne 10 mesures consécutives
  int moyenne = 0;
  for (int j = 0; j < 10; j++) {
    // Alarm.delay(1);
    moyenne += analogRead(Pin);
  }
  moyenne /= 10;
  return moyenne;
}
//---------------------------------------------------------------------------
void OuvrirFichierCalibration() { // Lecture fichier calibration

  if (SPIFFS.exists(filecalibration)) {
    File f = SPIFFS.open(filecalibration, "r");
    for (int i = 0; i < 3; i++) { //Read
      String s = f.readStringUntil('\n');
      CoeffTension[i] = s.toFloat();
    }
    f.close();
  }
  else {
    Serial.print("Creating Data File:"), Serial.println(filecalibration); // valeur par defaut
    CoeffTension[0] = CoeffTensionDefaut;
    CoeffTension[1] = CoeffTensionDefaut;
    CoeffTension[2] = CoeffTensionDefaut;
    Recordcalib();
  }
  Serial.print("Coeff T Batterie = "), Serial.print(CoeffTension[0]);
  Serial.print(" Coeff T Proc = ")	 , Serial.print(CoeffTension[1]);
  Serial.print(" Coeff T VUSB = ")   , Serial.print(CoeffTension[2]);

}
//---------------------------------------------------------------------------
void Recordcalib() { // enregistrer fichier calibration en SPIFFS
  // Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
  // Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
  // Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
  File f = SPIFFS.open(filecalibration, "w");
  f.println(CoeffTension[0]);
  f.println(CoeffTension[1]);
  f.println(CoeffTension[2]);
  // f.println(CoeffTension[3]);
  f.close();
}
//---------------------------------------------------------------------------
String Hdectohhmm(long Hdec) {
  // convert heure decimale en hh:mm:ss
  String hhmm;
  if (int(Hdec / 3600) < 10) hhmm = "0";
  hhmm += int(Hdec / 3600);
  hhmm += ":";
  if (int((Hdec % 3600) / 60) < 10) hhmm += "0";
  hhmm += int((Hdec % 3600) / 60);
  hhmm += ":";
  if (int((Hdec % 3600) % 60) < 10) hhmm += "0";
  hhmm += int((Hdec % 3600) % 60);
  return hhmm;
}
//---------------------------------------------------------------------------
long Hhmmtohdec(String h) {
  // convert heure hh:mm:ss en decimale
  int H = h.substring(0, 2).toInt();
  int M = h.substring(3, 5).toInt();
  int S = h.substring(6, 8).toInt();
  long hms = H * 3600 + M * 60 + S;
  return hms;
}
//---------------------------------------------------------------------------
void AIntru_HeureActuelle() {

  long Heureactuelle = HActuelledec();

  if (config.FinJour > config.DebutJour) {
    if ((Heureactuelle > config.FinJour && Heureactuelle > config.DebutJour)
        || (Heureactuelle < config.FinJour && Heureactuelle < config.DebutJour)) {
      // Nuit
      jour = false;
    }
    else {	// Jour
      jour = true;
    }
  }
  else {
    if (Heureactuelle > config.FinJour && Heureactuelle < config.DebutJour) {
      // Nuit
      jour = false;
    }
    else {	// Jour
      jour = true;
    }
  }
}
//---------------------------------------------------------------------------
void DebutSleep() {

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.print("Setup ESP32 to sleep for ");
  print_uint64_t(TIME_TO_SLEEP);
  Serial.print("s ;");
  Serial.println(Hdectohhmm(TIME_TO_SLEEP));
  Serial.flush();

  if (TIME_TO_SLEEP == 1) {
    Serial.println("pas de sleep on continue");
    return;
  }
  //Go to sleep now
  Serial.println("Going to sleep now");

  byte i = 0;
  if (gsm) {
    while (!modem.setPhoneFunctionality(0)) { // CFUN=0 minimum functionality
      Alarm.delay(100);
      if (i++ > 10) break;
    }
  }
  Serial.flush();
  esp_deep_sleep_start();
  delay(100);

  Serial.println("This will never be printed");
  Serial.flush();

}
//---------------------------------------------------------------------------
void action_wakeup_reason(byte wr) { // action en fonction du wake up
  Serial.print("Wakeup :"), Serial.print(wr);
  Serial.print(", jour :"), Serial.print(jour);
  Serial.print(" ,Calendrier :"), Serial.print(calendrier[month()][day()]);
  Serial.print(" ,FlagCircule :"), Serial.println(FlagCircule);
  byte pin = 0;
  Serial.println("***********************************");
  if (wr == 99 || wr == 32 || wr == 33 || wr == 34) {
    pin = wr;
    wr = 3;
  }
  if (wr == 0)wr = 4; // demarrage normal, decision idem timer

  switch (wr) {
    case 2: break; // ne rien faire ESP_SLEEP_WAKEUP_EXT0

    case 3: // ESP_SLEEP_WAKEUP_EXT1

      /* declenchement externe pendant deep sleep
        si nuit ou jour noncirculé
        on reste en fonctionnement pendant TempoAnalyse
        avant retour deep sleep*/
      // WupAlarme = true;
      // LastWupAlarme = true;
      // Alarm.enable(TempoAnalyse); // debut tempo analyse ->fonctionnement normal
      Sbidon = "Externe Debut ";
      Sbidon += String(pin);
      MajLog("Alarme", Sbidon);
      // }
      break;

    case 4: // SP_SLEEP_WAKEUP_TIMER
      if (FirstWakeup) { // premier wake up du jour avant DebutJour
        // ne rien faire, attendre DebutJour
        FirstWakeup = false;
        if (HActuelledec() > config.DebutJour) {
          // premier lancement en journée
          SignalVie();
        }
        break;
      }
      if ((calendrier[month()][day()] ^ FlagCircule) && jour) { // jour circulé & jour
        // Sbidon = F("Jour circule ou demande circulation");
        // Serial.println(Sbidon);
        // MajLog(F("Auto"), Sbidon);
        // Feux = 1;
        // Allumage(); // Violet 1, Blanc 0
        // MajLog("Auto", "FCV");
        // envoieGroupeSMS(0, 0);
      }
      else { // non circulé
        Sbidon = "Jour noncircule ou nuit";
        Serial.println(Sbidon);
        MajLog("Auto", Sbidon);
        calculTimeSleep();
        if (TIME_TO_SLEEP <= config.anticip) { // on continue sans sleep attente finjour
          Sbidon = "on continue sans sleep";
          Serial.println(Sbidon);
          MajLog("Auto", Sbidon);
        }
        else {
          DebutSleep();
        }
      }
      break;

    case 5: break;  // ne rien faire ESP_SLEEP_WAKEUP_TOUCHPAD
    case 6: break;  // ne rien faire ESP_SLEEP_WAKEUP_ULP
      // default: break; // demarrage normal
  }
}
//---------------------------------------------------------------------------
void calculTimeSleep() {

  AIntru_HeureActuelle(); // determine si jour/nuit

  if (jour && (HActuelledec() + config.RepeatWakeUp) > config.FinJour) {
    if (HActuelledec() > (config.FinJour - config.anticip)) {
      /* eviter de reporter 24H si on est à moins de anticip de FinJour */
      TIME_TO_SLEEP = 1; // si 1 pas de sleep
    }
    else {
      TIME_TO_SLEEP = DureeSleep(config.FinJour - config.anticip);
      Serial.print("time sleep calcul 1 : "), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
  }
  else if (!jour) {
    if (HActuelledec() < (config.DebutJour - config.anticip)) {
      TIME_TO_SLEEP = DureeSleep(config.DebutJour - config.anticip);
      Serial.print("time sleep calcul 2 : "), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
    else if (HActuelledec() < 86400) {
      TIME_TO_SLEEP = (86400 - HActuelledec()) + config.DebutJour - config.anticip;
      Serial.print("time sleep calcul 2bis : "), print_uint64_t(TIME_TO_SLEEP);
      Serial.println("");
    }
  }
  else {
    TIME_TO_SLEEP = config.RepeatWakeUp;
    Serial.print("time sleep calcul 3 : "), print_uint64_t(TIME_TO_SLEEP);
    Serial.println("");
  }

  /* Garde fou si TIME_TO_SLEEP > 20H00 c'est une erreur, on impose 1H00 */
  if (TIME_TO_SLEEP > 72000) {
    TIME_TO_SLEEP = 3600;
    Sbidon = "jour ";
    Sbidon += jour;
    Sbidon = ", Calendrier ";
    Sbidon += calendrier[month()][day()];
    Sbidon = ", flagCirc ";
    Sbidon += FlagCircule;
    MajLog("Auto", Sbidon);
    Sbidon = "Attention erreur Sleep>20H00 ";
    Sbidon += Hdectohhmm(TIME_TO_SLEEP);
    MajLog("Auto", Sbidon);
  }

  Sbidon = "lance timer : ";
  Sbidon += Hdectohhmm(TIME_TO_SLEEP);
  MajLog("Auto", Sbidon);
}
//---------------------------------------------------------------------------
int get_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  uint64_t wakeup_pin_mask;
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0  : return ESP_SLEEP_WAKEUP_EXT0; // 2
    case ESP_SLEEP_WAKEUP_EXT1: //{// 3
      wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
      if (wakeup_pin_mask != 0) {
        int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
        Serial.print("Wake up from GPIO "); Serial.println(String(pin));
        return pin; // pin
      } else {
        Serial.println(" Wake up from GPIO ?");
        return 99; // 99
      }
      break;
    // }
    case ESP_SLEEP_WAKEUP_TIMER    : return ESP_SLEEP_WAKEUP_TIMER; // 4
    case ESP_SLEEP_WAKEUP_TOUCHPAD : return ESP_SLEEP_WAKEUP_TOUCHPAD; // 5
    case ESP_SLEEP_WAKEUP_ULP      : return ESP_SLEEP_WAKEUP_ULP; // 6
    default : return 0; // Serial.println("Wakeup was not caused by deep sleep"); break;// demarrage normal
  }
  Serial.flush();
}
//---------------------------------------------------------------------------
void EffaceSMS(int slot) {
  bool err;
  byte n = 0;
  do {
    err = modem.deleteSmsMessage(slot,0);
    n ++;
    Serial.print("resultat del Sms ");	Serial.println(err);
    if (n > 10) { // on efface tous si echec
      err = modem.deleteSmsMessage(slot,4);
      Serial.print("resultat delall Sms ");	Serial.println(err);
      break;
    }
  } while (!err);
}
//---------------------------------------------------------------------------
void print_uint64_t(uint64_t num) {

  char rev[128];
  char *p = rev + 1;

  while (num > 0) {
    *p++ = '0' + ( num % 10);
    num /= 10;
  }
  p--;
  /*Print the number which is now in reverse*/
  while (p > rev) {
    Serial.print(*p--);
  }
}
//---------------------------------------------------------------------------
void init_adc_mm(void) {
  //initialisation des tableaux
  /* valeur par defaut facultative,
  	permet d'avoir une moyenne proche
  	du resulat plus rapidement
  	val defaut = valdefaut*nSample */
  unsigned int ini_adc1 = 0;// val defaut adc 1
  unsigned int ini_adc2 = 0;// val defaut adc 2
  unsigned int ini_adc3 = 0;// val defaut adc 3
  // unsigned int ini_adc4 = 0;// val defaut adc 4
  // unsigned int ini_adc5 = 0;// val defaut adc 5
  for (int plus_ancien = 0; plus_ancien < nSample; plus_ancien++) {
    adc_hist[0][plus_ancien] = ini_adc1;
    adc_hist[1][plus_ancien] = ini_adc2;
    adc_hist[2][plus_ancien] = ini_adc3;
    // adc_hist[3][plus_ancien] = ini_adc4;
    // adc_hist[4][plus_ancien] = ini_adc5;
  }
  //on commencera à stocker à cet offset
  adc_mm[0] = ini_adc1;
  adc_mm[1] = ini_adc2;
  adc_mm[2] = ini_adc3;
  // adc_mm[3] = ini_adc4;
  // adc_mm[4] = ini_adc5;
}
//---------------------------------------------------------------------------
void adc_read() {
  read_adc(PinBattSol, PinBattProc, PinBattUSB); // lecture des adc
}
//---------------------------------------------------------------------------
void read_adc(int pin1, int pin2, int pin3) {
  // http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  static int plus_ancien = 0;
  //acquisition
  int sample[3];
  for (byte i = 0; i < 3; i++) {
    if (i == 0)sample[i] = moyenneAnalogique(pin1);
    if (i == 1)sample[i] = moyenneAnalogique(pin2);
    if (i == 2)sample[i] = moyenneAnalogique(pin3);
    // if (i == 3)sample[i] = moyenneAnalogique(pin4);
    // if (i == 4)sample[i] = moyenneAnalogique(pin5);

    //calcul MoyenneMobile
    adc_mm[i] = adc_mm[i] + sample[i] - adc_hist[i][plus_ancien];

    //cette plus ancienne valeur n'est plus utile, on y stocke la plus récente
    adc_hist[i][plus_ancien] = sample[i];
  }
  plus_ancien ++;
  if (plus_ancien == nSample) { //gestion du buffer circulaire
    plus_ancien = 0;
  }
}
//--------------------------------------------------------------------------------//
void messageId() {
  message  = Id;
  message += displayTime(0);
  message += fl;
}
//---------------------------------------------------------------------------
void HomePage() {
  SendHTML_Header();
  webpage += "<h3 class='rcorners_m'>Parametres</h3><br>";
  webpage += "<table align='center'>";
  webpage += "<tr>";
  webpage += "<td>Version</td>";
  webpage += "<td>";	webpage += ver;	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Id</td>";
  webpage += "<td>";	webpage += String(config.Idchar);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Debut Jour</td>";
  webpage += "<td>";	webpage += Hdectohhmm(config.DebutJour);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Anticipation WakeUp (s)</td>";
  webpage += "<td>";	webpage += String(config.anticip);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Fin Jour</td>";
  webpage += "<td>";	webpage += Hdectohhmm(config.FinJour);	webpage += "</td>";
  webpage += "</tr>";

  webpage += F("<tr>");
  webpage += F("<td>Type Batterie</td>");
  webpage += F("<td>");	
  if(config.TypeBatt == 16) webpage += F("Pb 12V 6elts");
  if(config.TypeBatt == 24) webpage += F("LiFePO 12.8V 4elts");
  webpage += F("</td>");
  webpage += F("</tr>");

  webpage += "<tr>";
  webpage += "<td>Vitesse SlowBlinker (5-2000ms)</td>";
  webpage += "<td>";	webpage += String(config.SlowBlinker);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Tempo Ouverture (5-120s)</td>";
  webpage += "<td>";	webpage += String(config.Tempoouverture);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Tempo Fermeture (5-120s)</td>";
  webpage += "<td>";	webpage += String(config.Tempofermeture);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>PWM Feux Rouge (%)</td>";
  webpage += "<td>";	webpage += String(config.FRgePWM);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Tempo r&eacute;p&eacute;tition Wake up Jour Circul&eacute; (s)</td>";
  webpage += "<td>";	webpage += String(config.RepeatWakeUp);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>TimeOut Wifi (s)</td>";
  webpage += "<td>";	webpage += String(config.timeoutWifi);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Liste Restreinte</td>";
  webpage += "<td>";
  for (int i = 1; i < 10; i++) {
    webpage += String(config.Pos_Pn_PB[i]);
    if (i < 9) {
      webpage += ",";
    }
  }
  webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>GPRS APN</td>";
  webpage += "<td>";	webpage += String(config.apn);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>GPRS user</td>";
  webpage += "<td>";	webpage += String(config.gprsUser);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>GPRS pass</td>";
  webpage += "<td>";	webpage += String(config.gprsPass);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>ftp Serveur</td>";
  webpage += "<td>";	webpage += String(config.ftpServeur);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>ftp Port</td>";
  webpage += "<td>";	webpage += String(config.ftpPort);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>ftp User</td>";
  webpage += "<td>";	webpage += String(config.ftpUser);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>ftp Pass</td>";
  webpage += "<td>";	webpage += String(config.ftpPass);	webpage += "</td>";
  webpage += "</tr>";

  webpage += F("<tr>");
  webpage += F("<td>mqtt serveur</td>");
  webpage += F("<td>");	webpage += String(config.mqttServer);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>mqtt port</td>");
  webpage += F("<td>");	webpage += String(config.mqttPort);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>mqtt username</td>");
  webpage += F("<td>");	webpage += String(config.mqttUserName);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>mqtt pass</td>");
  webpage += F("<td>");	webpage += String(config.mqttPass);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>send topic</td>");
  webpage += F("<td>");	webpage += String(config.sendTopic);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>receive topic</td>");
  webpage += F("<td>");	webpage += String(config.receiveTopic);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>mpermanent topic</td>");
  webpage += F("<td>");	webpage += String(config.permanentTopic);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>msg Mode 0SMS,1SMS+MQTT</td>");
  webpage += F("<td>");	webpage += String(config.messageMode);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>declage Heure ete</td>");
  webpage += F("<td>");	webpage += String(config.hete);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>declage Heure hiver</td>");
  webpage += F("<td>");	webpage += String(config.hhiver);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>autoupload</td>");
  webpage += F("<td>");	webpage += String(config.autoupload);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += "</table><br>";

  webpage += "<a href='/download'><button>Download</button></a>";
  webpage += "<a href='/upload'><button>Upload</button></a>";
  webpage += "<a href='/delete'><button>Delete</button></a>";
  webpage += "<a href='/dir'><button>Directory</button></a>";
  webpage += "<a href='/Tel_list'><button>Tel_list</button></a>";
  webpage += "<a href='/cal'><button>Calendar</button></a>";
  webpage += "<a href='/wifioff'><button>Wifi Off</button></a>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
void Tel_listPage() {
  SendHTML_Header();
  webpage += "<h3 class='rcorners_m'>Liste des num&eacute;ros t&eacute;l&eacute;phone</h3><br>";
  webpage += "<table align='center'>";
  webpage += "<tr>";
  webpage += "<th> Nom </th>";
  webpage += "<th> Num&eacute;ro </th>";
  webpage += "<th> Liste restreinte </th>";
  webpage += "</tr>";
  if (gsm) {
    for (byte i = 1; i < 10; i++) {
      if(modem.readPhonebookEntry(&Phone, i)){
        webpage += F("<tr>");
        webpage += F("<td>"); webpage += Phone.text; webpage += F("</td>");
        webpage += F("<td>"); webpage += Phone.number ; webpage += F("</td>");
        webpage += F("<td>"); webpage += String(config.Pos_Pn_PB[i]); webpage += F("</td>");
        webpage += F("</tr>");
      } else {
        i = 10;
      }
    }
  }
  webpage += "</table><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
void CalendarPage() {
  SendHTML_Header();
  webpage += "<h3 class='rcorners_m'>Calendrier</h3><br>";
  webpage += "<table align='center'>";

  for (int m = 1; m < 13; m ++) {
    webpage += "<tr>";
    webpage += "<td>"; webpage += Mois[m]; webpage += "</td>";
    for (int j = 1; j < 32; j++) {
      webpage += "<td>";	webpage += calendrier[m][j];	webpage += "</td>";
      if (j % 5 == 0)webpage += "<td> </td>";
    }
    webpage += "</tr>";
  }

  webpage += "</table><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Download() { // This gets called twice, the first pass selects the input, the second pass then processes the command line arguments
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("download")) DownloadFile(server.arg(0));
  }
  else SelectInput("Enter filename to download", "download", "download");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void DownloadFile(String filename) {
  if (SPIFFS_present) {
    File download = SPIFFS.open("/" + filename,  "r");
    if (download) {
      server.sendHeader("Content-Type", "text/text");
      server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else ReportFileNotPresent("download");
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Upload() {
  append_page_header();
  webpage += "<h3>Select File to Upload</h3>";
  webpage += "<FORM action='/fupload' method='post' enctype='multipart/form-data'>";
  webpage += "<input class='buttons' style='width:40%' type='file' name='fupload' id = 'fupload' value=''><br>";
  webpage += "<br><button class='buttons' style='width:10%' type='submit'>Upload File</button><br>";
  webpage += "<a href='/'>[Back]</a><br><br>";
  append_page_footer();
  server.send(200, "text/html", webpage);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void handleFileUpload() { // upload a new file to the Filing system
  HTTPUpload& uploadfile = server.upload(); // See https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WebServer/srcv
  // For further information on 'status' structure, there are other reasons such as a failed transfer that could be used
  if (uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.print("Upload File Name: "); Serial.println(filename);
    SPIFFS.remove(filename);                  // Remove a previous version, otherwise data is appended the file again
    UploadFile = SPIFFS.open(filename, "w");  // Open the file for writing in SPIFFS (create it, if doesn't exist)
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if (UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
  }
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if (UploadFile)         // If the file was successfully created
    {
      UploadFile.close();   // Close the file again
      Serial.print("Upload Size: "); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += "<h3>File was successfully uploaded</h3>";
      webpage += "<h2>Uploaded File Name: "; webpage += uploadfile.filename + "</h2>";
      webpage += "<h2>File Size: "; webpage += file_size(uploadfile.totalSize) + "</h2><br>";
      append_page_footer();
      server.send(200, "text/html", webpage);
      OuvrirCalendrier();
    }
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_dir() {
  if (SPIFFS_present) {
    File root = SPIFFS.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();
      webpage += "<h3 class='rcorners_m'>SPIFFS Contents</h3><br>";
      webpage += "<table align='center'>";
      webpage += "<tr><th>Name/Type</th><th style='width:20%'>Type File/Dir</th><th>File Size</th></tr>";
      printDirectory("/", 0);
      webpage += "</table>";
      SendHTML_Content();
      root.close();
    }
    else
    {
      SendHTML_Header();
      webpage += "<h3>No Files Found</h3>";
    }
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();   // Stop is needed because no content length was sent
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void printDirectory(const char * dirname, uint8_t levels) {
  File root = SPIFFS.open(dirname);
  if (!root) {
    return;
  }
  if (!root.isDirectory()) {
    return;
  }
  File file = root.openNextFile();
  while (file) {
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }
    if (file.isDirectory()) {
      webpage += "<tr><td>" + String(file.isDirectory() ? "Dir" : "File") + "</td><td>" + String(file.name()) + "</td><td></td></tr>";
      printDirectory(file.name(), levels - 1);
    }
    else
    {
      webpage += "<tr><td>" + String(file.name()) + "</td>";
      webpage += "<td>" + String(file.isDirectory() ? "Dir" : "File") + "</td>";
      webpage += "<td>" + file_size(file.size()) + "</td></tr>";
    }
    file = root.openNextFile();
  }
  file.close();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void File_Delete() {
  if (server.args() > 0 ) { // Arguments were received
    if (server.hasArg("delete")) SPIFFS_file_delete(server.arg(0));
  }
  else SelectInput("Select a File to Delete", "delete", "delete");
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SPIFFS_file_delete(String filename) { // Delete the file
  if (SPIFFS_present) {
    SendHTML_Header();
    File dataFile = SPIFFS.open("/" + filename, "r"); // Now read data from SPIFFS Card
    if (dataFile)
    {
      if (SPIFFS.remove("/" + filename)) {
        Serial.println("File deleted successfully");
        webpage += "<h3>File '" + filename + "' has been erased</h3>";
        webpage += "<a href='/delete'>[Back]</a><br><br>";
      }
      else
      {
        webpage += "<h3>File was not deleted - error</h3>";
        webpage += "<a href='delete'>[Back]</a><br><br>";
      }
    } else ReportFileNotPresent("delete");
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSPIFFSNotPresent();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Header() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Content() {
  server.sendContent(webpage);
  webpage = "";
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SendHTML_Stop() {
  server.sendContent("");
  server.client().stop(); // Stop is needed because no content length was sent
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SelectInput(String heading1, String command, String arg_calling_name) {
  SendHTML_Header();
  webpage += "<h3>" + heading1 + "</h3>";
  webpage += "<FORM action='/" + command + "' method='post'>"; // Must match the calling argument e.g. '/chart' calls '/chart' after selection but with arguments!
  webpage += "<input type='text' name='" + arg_calling_name + "' value=''><br>";
  webpage += "<type='submit' name='" + arg_calling_name + "' value=''><br><br>";
  webpage += "<a href='/'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportSPIFFSNotPresent() {
  SendHTML_Header();
  webpage += "<h3>No SPIFFS Card present</h3>";
  webpage += "<a href='/'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportFileNotPresent(String target) {
  SendHTML_Header();
  webpage += "<h3>File does not exist</h3>";
  webpage += "<a href='/"; webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ReportCouldNotCreateFile(String target) {
  SendHTML_Header();
  webpage += "<h3>Could Not Create Uploaded File (write-protected?)</h3>";
  webpage += "<a href='/"; webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                      fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                                   fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}
//---------------------------------------------------------------------------
void handleTime() { // getion temps restant page web
  char time_str[9];
  const uint32_t millis_in_day    = 1000 * 60 * 60 * 24;
  const uint32_t millis_in_hour   = 1000 * 60 * 60;
  const uint32_t millis_in_minute = 1000 * 60;

  static unsigned long t0 = 0;
  if (millis() - debut > config.timeoutWifi * 1000) debut = millis(); // securité evite t<0
  t0 = debut + (config.timeoutWifi * 1000) - millis();
  // Serial.print(debut),Serial.print("|"),Serial.println(t0);

  uint8_t days     = t0 / (millis_in_day);
  uint8_t hours    = (t0 - (days * millis_in_day)) / millis_in_hour;
  uint8_t minutes  = (t0 - (days * millis_in_day) - (hours * millis_in_hour)) / millis_in_minute;
  uint8_t secondes = (t0 - (days * millis_in_day) - ((hours * millis_in_hour)) / millis_in_minute) / 1000 % 60;
  sprintf(time_str, "%02d:%02d:%02d", hours, minutes, secondes);
  // Serial.println(time_str);
  server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
void handleDateTime() { // getion Date et heure page web
  char time_str[20];
  sprintf(time_str, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
byte FTP_upload_function (char *file2upload){
  Serial.print("file to upload:"),Serial.println(file2upload);
  String tosend = "";
  // copie ud fichier file2upload vers EFS sur modem SIM7600
  sendAT("AT","OK","ERROR",1000);
  // Selection e:
  Sbidon = sendAT("AT+FSCD=E:","OK","ERROR",1000);
  if(Sbidon.indexOf("+FSCD: E:")>-1){
    Serial.println("Select e: OK");
  } else {
    Serial.println("Select e: KO");
    return false;
  }
  // copie file2upload dans e:
  File f = SPIFFS.open(file2upload, "r");
  tosend = "AT+CFTRANRX=\"E:"+String(file2upload)+"\"," +String(f.size());
  Serial.print("copy file :"), Serial.println(tosend);
  Sbidon = sendAT(tosend,">","ERROR",1000);
  while (f.available()){
    SerialAT.write(f.read());
  }
  f.close();
  delay(100);
  // verification file2upload dans e:
  Sbidon = sendAT("AT+FSLS","OK","ERROR",1000);
  Serial.print("List FS Directory"), Serial.println(Sbidon);
  if(Sbidon.indexOf(String(file2upload).substring(1))>-1){// apres le / de debut
    Serial.println("write file to e: OK");
  } else {
    Serial.println("write file to e: KO");
    return false;
  }
  // Envoyer fichier par FTP
  sendAT("AT","OK","ERROR",1000);
  // Start FTP
  Sbidon = sendAT("AT+CFTPSSTART","OK","ERROR",1000);
  Serial.print("FTP Start :"), Serial.println(Sbidon);
  if(Sbidon.indexOf("+CFTPSSTART: 0")>-1){
    Serial.println("FTP Start OK");
  } else {
    Serial.println("FTP Start KO");
    return false;
  }
  // Login FTP
  tosend = "AT+CFTPSLOGIN=\"" + String(config.ftpServeur) + "\","+ String(config.ftpPort)+ ",\""+ String(config.ftpUser)+ "\",\""+ String(config.ftpPass)+ "\",0";
  Serial.print("FTP login :"), Serial.println(tosend);
  Sbidon = sendAT(tosend,"+CFTPSLOGIN","ERROR",10000);
  Serial.print("FTP login :"), Serial.println(Sbidon);
  if(Sbidon.indexOf("+CFTPSLOGIN: 0")>-1){
    Serial.println("FTP login OK");
  } else {
    Serial.println("FTP login KO");// sortir
    sendAT("AT+CFTPSSTOP","OK","ERROR",1000);
    return false;
  }
  // Upload du fichier
  tosend = "AT+CFTPSPUTFILE=\"" + String(config.Idchar) + String(file2upload) + "\",3"; // 3 pour E:
  Serial.print("FTP Upload :"), Serial.println(tosend);
  Sbidon = sendAT(tosend,"OK","ERROR",10000);
  Serial.print("FTP Upload :"), Serial.println(Sbidon);
  if(Sbidon.indexOf("+CFTPSPUTFILE: 0")>-1){
    Serial.println("Upload OK");
  } else {
    Serial.println("Upload KO");// sortir
    sendAT("AT+CFTPSLOGOUT","OK","ERROR",1000);
    sendAT("AT+CFTPSSTOP","OK","ERROR",1000);
    return false;
  }
  delay(500);
  // relecture du repertoir sur FTP pour verification presence du fichier uploadé
  tosend = "AT+CFTPSLIST=\"" + String(config.Idchar) + "\"";
  Sbidon = sendAT(tosend,"+CFTPSLIST","ERROR",10000);
  Serial.print("FTP Dir :"), Serial.println(Sbidon);
  if(Sbidon.indexOf(String(file2upload).substring(1))>-1){// apres le / de debut
    Serial.println("fichier uploader avec succes");
  } else {
    Serial.println("fichier uploader avec erreur");
    sendAT("AT+CFTPSLOGOUT","OK","ERROR",1000);
    sendAT("AT+CFTPSSTOP","OK","ERROR",1000);
    return false;
  }
  Sbidon = sendAT("AT+CFTPSLOGOUT","OK","ERROR",1000);
  Serial.print("FTP logout :"), Serial.println(Sbidon);
  Sbidon = sendAT("AT+CFTPSSTOP","OK","ERROR",1000);
  Serial.print("FTP Stop :"), Serial.println(Sbidon);
  // Suppression du fichier dans EFS
  tosend = "AT+FSDEL=" + String(file2upload);
  Sbidon = sendAT(tosend,"OK","ERROR",1000);
  Serial.print("Suppression fichier en e: :"), Serial.println(Sbidon);
  return true;
}
//---------------------------------------------------------------------------
String sendAT(String ATcommand, String answer1, String answer2, unsigned int timeout){
  byte reply = 1;
  String content = "";
  char character;

  //Clean the modem input buffer
  while(Serial2.available()>0) Serial2.read();

  //Send the atcommand to the modem
  Serial2.println(ATcommand);
  delay(100);
  unsigned int timeprevious = millis();
  while((reply == 1) && ((millis() - timeprevious) < timeout)){
    while(Serial2.available()>0) {
      character = Serial2.read();
      content.concat(character);
      Serial.print(character);
      delay(10);
    }
  }
  // Serial.print("reponse: "),Serial.println(content);
  // Serial.println("fin reponse");
  return content;
}
//---------------------------------------------------------------------------
void Ouvrir_CVR() {
  digitalWrite(PinOuvre, HIGH);// cde relais Verin
  digitalWrite(PinFerme, LOW); // cde relais Verin
  Alarm.enable(Aouverture);
  ledcWrite(RgePwmChanel, 0); // Extinction Feux Rouge
  Allume = false;
  SlowBlink.detach();
}
//---------------------------------------------------------------------------
void Fermer_CVR() {
  digitalWrite(PinOuvre, LOW); // cde relais Verin
  digitalWrite(PinFerme, HIGH);// cde relais Verin
  Alarm.enable(Afermeture);
}
//---------------------------------------------------------------------------
void ouverture() {
  // fin tempo ouverture verin
  Serial.println("fin tempo ouverture verin");
  digitalWrite(PinOuvre, LOW);// cde relais Verin
  digitalWrite(PinFerme, LOW);// cde relais Verin
  Alarm.disable(Aouverture);
  FlagVerif_CVR = true;// flag pour verification position
  Acquisition();
}
//---------------------------------------------------------------------------
void fermeture() {
  // fin tempo fermeture verin
  Serial.println("fin tempo fermeture verin");
  digitalWrite(PinOuvre, LOW);// cde relais Verin
  digitalWrite(PinFerme, LOW);// cde relais Verin
  Alarm.disable(Afermeture);
  FlagVerif_CVR = true;// flag pour verification position
  // ledcWrite(RgePwmChanel, 0);
  // SlowBlink.attach_ms(config.SlowBlinker, blink);// Allumage Feux Rouge
  // Allume = true;
  Acquisition();
}
//---------------------------------------------------------------------------
int Position_CVR() { // lire position du CVR
  bool s1 = digitalRead(PinIp1);
  bool s2 = digitalRead(PinIp2);
  int pos = 0;
  if ((s1 == 0 && s2 == 0) || (s1 == 1 && s2 == 1)) {
    pos = 0; // Indefini
  } else if (s1 == 1 && s2 == 0) {
    pos = 1; // Fermé
  } else if (s1 == 0 && s2 == 1) {
    pos = 2; // Ouvert
  }
  return pos;
}
//---------------------------------------------------------------------------
void blink() {
  if (blinker) {
    ledcWrite(RgePwmChanel, 0);
    blinker = false;
  } else {
    ledcWrite(RgePwmChanel, 255 * config.FRgePWM / 100);
    blinker = true;
  }
}
//---------------------------------------------------------------------------
void GestionMessage_CVR() { // gestion message CVR
  Serial.println("gestion message CVR");
  MajLog("Auto", "Position : " + String(Cvr));
  generationMessage(0);
  char number[13];
  Memo_Demande_CVR[1].toCharArray(number, Memo_Demande_CVR[1].length() + 1);
  bool smsserveur = false;
  Phone = {"",""};
  modem.readPhonebookEntry(&Phone, 1); // lecture numero serveur 1
  if (Memo_Demande_CVR[1] == Phone.number) {
    smsserveur = true; // si demande provient du serveur index=1
  }
    if (Memo_Demande_CVR[0] != "console") {
      if (!smsserveur) {
        Serial.println("reponse demandeur si pas serveur");
        sendSMSReply(number, true); // reponse demandeur si pas serveur
      }
    } else {
      Serial.println(message);
    }
  // } else {
  //   Serial.println(message);
}
//---------------------------------------------------------------------------
void ConnectGPRS(){
  Serial.print(F("Connecting to "));
  Serial.print(config.apn);
  if (modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {
    Serial.println(F(" success"));
  }
  else {
    Serial.println(" fail");
  }
}
//---------------------------------------------------------------------------
boolean mqttConnect() {
  Serial.print(F("Connecting to "));
  Serial.print(config.mqttServer);

  // Connect to MQTT Broker
  bool status = mqtt.connect(config.Idchar, config.mqttUserName, config.mqttPass);
  if (status == false) {
    Serial.println(F(" fail"));
    return false;
  }
  Serial.println(F(" success"));
  if(mqtt.subscribe(config.permanentTopic,1) == 1){
    Serial.print(config.permanentTopic),Serial.println(" subcribed");
  }
  if(mqtt.subscribe(config.receiveTopic) == 1){
    Serial.print(config.receiveTopic),Serial.println(" subcribed");
  }
  return mqtt.connected();
}
//---------------------------------------------------------------------------
bool Cherche_N_PB(String numero){ // Cherche numero dans PB
  for (byte idx = 1; idx < 10; idx++){
    Phone = {"",""};
    if(modem.readPhonebookEntry(&Phone, idx)){
      if(Phone.number == numero){
        Serial.print(F("Nom :")), Serial.println(Phone.text);
        return true;
      }
    }// else { idx = 10;}    
  }
  return false;
}
//---------------------------------------------------------------------------
bool SyncHeureModem(int Savetime, bool FirstTime){
  int rep = 1;
  int compteur = 0;
  if(modem.isNetworkConnected() && modem.isGprsConnected()){
    do{
      rep = modem.NTPServerSync(NTPServer, Savetime);
      Serial.print("NTP:"),Serial.println(modem.ShowNTPError(rep));
      if (compteur > 10){ // 10 tentatives
        if(FirstTime){
          // echec Synchro, force date 01/08/2022 08:00:00, jour toujours circulé
          modem.send_AT("+CCLK=\"22/08/01,08:00:00+08\"");
          delay(500);
        }
        return false;
      }
      compteur += 1;
      delay(1000);
    } while(rep != 0);
    return true;
  }
  return false;
}
//---------------------------------------------------------------------------
void message_Monitoring_Reseau(){
  message += F("Histo Network Chgt : ");
  message += fl;
  message += F("not connect : ");
  message += String(Histo_Reseau[0]);
  message += fl;
  message += F("inconnu : ");
  message += String(Histo_Reseau[1]);
  message += fl;
  message += F("2G : ");
  message += String(Histo_Reseau[2]);
  message += fl;
  message += F("3G : ");
  message += String(Histo_Reseau[3]);
  message += fl;
  message += F("4G : ");
  message += String(Histo_Reseau[4]);
  message += fl;
}
//---------------------------------------------------------------------------
void readmodemtime(){
  String modemHDtate = modem.getGSMDateTime(TinyGSMDateTimeFormat(0));
  // convertir format date time yy/mm/dd,hh:mm:ss
  byte i 	= modemHDtate.indexOf("/");
  byte j 	= modemHDtate.indexOf("/", i + 1);
  N_Y		  = modemHDtate.substring(i - 2, i).toInt();
  N_M 		= modemHDtate.substring(i + 1, j).toInt();
  N_D 		= modemHDtate.substring(j + 1, j + 3).toInt();
  i 	  	= modemHDtate.indexOf(":", 6);
  j     	= modemHDtate.indexOf(":", i + 1);
  N_H 		= modemHDtate.substring(i - 2, i).toInt();
  N_m 		= modemHDtate.substring(i + 1, j).toInt();
  N_S 		= modemHDtate.substring(j + 1, j + 3).toInt();
}
//---------------------------------------------------------------------------
bool HeureEte() {
  // Serial.print("mois:"),Serial.print(month());
  // Serial.print(" jour:"),Serial.print(day());
  // Serial.print(" jsem:"),Serial.println(weekday());
  // return true en été, false en hiver (1=dimanche)
  bool Hete = false;
  if (month() > 10 || month() < 3
      || (month() == 10 && (day() - weekday()) > 22)
      || (month() == 3  && (day() - weekday()) < 24)) {
    Hete = false;                      								// c'est l'hiver
  }
  else {
    Hete = true;                       								// c'est l'été
  }
  return Hete;
}
//---------------------------------------------------------------------------
String ConnectedNetwork(){  
  String network = "";
  int n = modem.getNetworkCurrentMode();
  if(n==0){ network            = F("not connect");}
  else if(n>0 && n<4){network  = F("2G");}
  else if(n>=4 && n<8){network = F("3G");}
  else if(n==8){network        = F("4G");}
  else if(n==16){network       = F("XLTE:CDMA&LTE");}
  else {network = String(n);}
  return network;
}
//---------------------------------------------------------------------------
/* Enregistre mode cnx Reseau
  Histo-Reseau(0) = not connect, (1)=inconnu, (2)=2G, (3)=3G, (4)=4G 
  cummul à chaque heure */
void Monitoring_Reseau(){
  int n = modem.getNetworkCurrentMode();
  if(n==0){ Histo_Reseau[0]            ++ ;}
  else if(n>0 && n<4){Histo_Reseau[2]  ++;}
  else if(n>=4 && n<8){Histo_Reseau[3] ++;}
  else if(n==8){Histo_Reseau[4]        ++;}
  else {Histo_Reseau[1] ++;}
}
//---------------------------------------------------------------------------
void mqttSubscriptionCallback( char* topic, byte* payload, unsigned int mesLength ) {
  /* 6) Use the mqttSubscriptionCallback function to handle incoming MQTT messages.
    The program runs smoother if the main loop performs the processing steps instead of the callback.
    In this function, use flags to cause changes in the main loop. */
  /**
    Process messages received from subscribed channel via MQTT broker.
      topic - Subscription topic for message.
      payload - Field to subscribe to. Value 0 means subscribe to all fields.
      mesLength - Message length.
  */
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(" len:");
  Serial.print(mesLength);
  Serial.print(". Message: ");
  
  Sbidon = "";
  
  for (int i = 0; i < mesLength; i++) {
    Serial.print((char)payload[i]);
    Sbidon += (char)payload[i];
  }
  Sbidon.toCharArray(replybuffer, Sbidon.length() + 1);
  Serial.println();

  if(strcmp(topic,config.receiveTopic) == 0){
    traite_sms(255);
  } else if (strcmp(topic,config.permanentTopic) == 0){
    if(Sbidon.length() > 0){
      //renvoyer "" sur permanentTopic pour eviter repetition
      char rep[2] = "";
      if(mqtt.publish(config.permanentTopic, rep, true)){
        Serial.println("efface permanent topic OK");
      } else {
        Serial.println("efface permanent topic KO");
      }
      traite_sms(255);
    }
  }
}
/* --------------------  test local serial seulement ----------------------*/
void recvOneChar() {

  char   receivedChar;
  static String serialmessage = "";
  static bool   newData = false;

  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    if (receivedChar != 10 && receivedChar != 13) {
      serialmessage += receivedChar;
    }
    else {
      newData = true;
      return;
    }
  }
  if (newData == true) {
    Serial.println(serialmessage);
    interpretemessage(serialmessage);
    newData = false;
    serialmessage = "";
  }
}

void interpretemessage(String demande) {
  String bidons;
  //demande.toUpperCase();
  if (demande.indexOf(char(61)) == 0) {
    bidons = demande.substring(1);
    bidons.toCharArray(replybuffer, bidons.length() + 1);
    traite_sms(99);//	traitement SMS en mode test local
  }
}
//---------------------------------------------------------------------------

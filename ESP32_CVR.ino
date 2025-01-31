/* Ph CORBEL 10/2020
  Gestion Carré VR
  (basé sur ESP32_Signalisation

  todo
  
  V4-00 30/01/2025 Version LTE-M
  1- passage LTE-M
  2- Ajouter prise en compte cde DCVR1 -> OCVR1
  3- Ajouter commande avec/sans Allumage feu rouge si fermé
  Compilation LOLIN D32,default,80MHz, IMPORTANT ESP32 2.0.17
  Arduino IDE 1.8.19 : 1097185 83%, 56080 17% sur PC IDE VSCODE
  Arduino IDE 1.8.19 : x 77%, x 14% sur raspi (sans ULP)

  V1-3 22/03/2024 installé CVR1
  1- Récupération des message long de free avec numero appelant de 29c et message de 268c
    SIM800l modifié 1.1.31
  2- Batterie LiFePo
  3- mesure batterie pendant calibration
  4- SENDAT
  5- VIDELOG
  6- correction envoieGroupeSMS
  
  Compilation LOLIN D32,default,80MHz, IMPORTANT ESP32 1.0.2 (version > bug avec SPIFFS?)
  Arduino IDE 1.8.19 : 1003278 76%, 47760 14% sur PC IDE Arduino et VSCODE
  Arduino IDE 1.8.19 : x 77%, x 14% sur raspi (sans ULP)


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
String ver        = "V4-00";
int    Magique    = 4;

#define TINY_GSM_MODEM_SIM7000
#define Exploitation           // selection des données serveur dans fichier credentials

#include <Battpct.h>
#include "defs.h"
#include <TinyGsmClient.h>         // librairie TinyGSM revue PhC 10.12.0
#include <PubSubClient.h>
#include <TimeAlarms.h>
#include <WiFi.h>
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

#define LED_PIN        5   // 
#define MODEM_PWRKEY  18   // Powerkey SIM7000
#define PinBattProc   35   // liaison interne carte Lolin32 adc
#define PinBattSol    39   // Batterie générale 12V adc VN
#define PinBattUSB    36   // V USB 5V adc VP 36, 25 ADC2 pas utilisable avec Wifi
#define PinIp1        32   // Entrée Ip1 S1 position CVR
#define PinIp2        33   // Entrée Ip2 S2 position CVR
#define PinFerme      21   // Sortie Commande Fermeture CVR
#define PinFeuxR      19   // Sortie Commande Feux Rouge
#define PinOuvre      15   // Sortie Commande Ouverture CVR
#define RX_PIN        17   // TX Sim7000
#define TX_PIN        16   // RX Sim7000
#define PinReset      13   // Reset Hard
#define PinTest       27   // Test sans GSM cc a la masse

#define NTPServer "pool.ntp.org"

#define uS_TO_S_FACTOR 1000000          //* Conversion factor for micro seconds to seconds */
#define FORMAT_SPIFFS_IF_FAILED true    // format LittelFS si lecture impossible
#define nSample (1<<4)                  // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[5][nSample];      // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 24V, 5 Lum
unsigned int adc_mm[5];                 // stockage pour la moyenne mobile

uint64_t TIME_TO_SLEEP  = 15;/* Time ESP32 will go to sleep (in seconds) */
unsigned long debut     = 0; // pour decompteur temps wifi
byte calendrier[13][32]; // tableau calendrier ligne 0 et jour 0 non utilisé, 12*31
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char fileconfig[12]      = "/config.txt";   // fichier en SPIFFS contenant structure config
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
char filelog[9]          = "/log.txt";      // fichier en SPIFFS contenant le log
char filePhoneBook[8]    = "/pb.txt";       // fichier contenant liste N°tel autorisé
char PB_list[10][30];                       // PB liste en ram

const String soft = "ESP32_CVR.ino.d32"; // nom du soft
const String Mois[13] = {"", "Janvier", "Fevrier", "Mars", "Avril", "Mai", "Juin", "Juillet", "Aout", "Septembre", "Octobre", "Novembre", "Decembre"};
String Sbidon 		= ""; // String texte temporaire
String message;                     //  Message envoyé
String Rmessage;                    //  Message reçu
String fl = "\n";                   //  saut de ligne SMS
String Id ;                         //  Id du materiel sera lu dans config

byte RgePwmChanel = 0;
bool blinker = false;

RTC_DATA_ATTR bool FlagAlarmeTension       = false; // Alarme tension Batterie
RTC_DATA_ATTR bool FlagLastAlarmeTension   = false;
RTC_DATA_ATTR bool FlagAlarmePosition      = false; // Alarme Position
RTC_DATA_ATTR bool FlagLastAlarmePosition  = false; // Last Alarme Position
RTC_DATA_ATTR bool FirstWakeup             = true;  // envoie premier message vie une seule fois
RTC_DATA_ATTR bool FlagCircule             = false; // circule demandé -> inverse le calendrier, valid 1 seul jour
RTC_DATA_ATTR bool FileLogOnce             = false; // true si log > seuil alerte

int Cvr                    = 0;          // Etat du Cvr voir tableau au début
int LastCvr                = 0;          // memo last etat
bool FlagReset             = false;      // Reset demandé
bool jour                  = false;	     // jour = true, nuit = false
bool gsm                   = true;       // carte GSM presente utilisé pour test sans GSM seulement
bool FlagAlarmeGprs        = false;      // Alarme confirmée
bool AlarmeGprs            = false;      // detection alarme
bool FlagLastAlarmeGprs    = false;
bool FlagAlarmeMQTT        = false;
bool AlarmeMQTT            = false;
bool FlagLastAlarmeMQTT    = false;
String Memo_Demande_CVR[3] = {"","",""}; // 0 num demandeur,1 nom, 2 CVR demandé (O2,F1)
bool FlagDemande_CVR       = false;      // si demande encours = true
bool FlagVerif_CVR         = false;      // si fin de la commande, pour verification position
bool FlagFinJournee        = false;      // demande fermeture fin de journée
bool Allume                = false;      // Allumage feu rouge
int CoeffTension[3];                     // Coeff calibration Tension
int CoeffTensionDefaut     = 7000;       // Coefficient par defaut
bool firstdecision         = false;      // true si premiere décision apres lancement
bool flagRcvMQTT           = false;      // true si reception MQTT longueur>0, false longueur = 0
bool Online                = false;      // true si network && Gprs && mqtt && subscribe
bool FlagSetTime           = false;      // flag set time valide
long TensionBatterie       = 0;          // Tension Batterie solaire
long VBatterieProc         = 0;          // Tension Batterie Processeur
long VUSB                  = 0;          // Tension USB
bool FlagDemandeAllume     = true;       // avec/sans Allumage feu rouge

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
PubSubClient  mqttClient(client);
WebServer server(80);
File UploadFile;

struct  config_t           // Structure configuration sauvée en SPIFFS
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
  char    Idchar[11];      // Id
  char    apn[11];         // APN
  char    gprsUser[11];    // user for APN
  char    gprsPass[11];    // pass for APN
  char    ftpServeur[26];  // serveur ftp
  char    ftpUser[9];      // user ftp
  char    ftpPass[16];     // pwd ftp
  int     ftpPort;         // port ftp
  int     TypeBatt;        // Type Batterie 16: Pb 6elts, 24 LiFePO 4elts
  char    mqttServer[26];  // Serveur MQTT
  char    mqttUserName[11];// MQTT User
  char    mqttPass[16];    // MQTT pass
  char    sendTopic[2][12];// output to server
  char    recvTopic[2][12];// input from server
  int     mqttPort;        // Port serveur MQTT
  int     hete;            // decalage Heure été UTC
  int     hhiver;          // decalage Heure hiver UTC
  bool    sendSMS;         // Autorisation envoyer SMS
  bool    autoupload;      // Upload automatique du fichier log
  uint16_t keepAlive;      // Paramètre keep alive de Pubsubclient
  byte    cptAla;          // Compteur alarmes avant declenchement
} ;
config_t config;

char willTopic[7];

int N_Y, N_M, N_D, N_H, N_m, N_S; // variable Date/Time temporaire
int NbrResetModem = 0;            // Nombre de fois reset modem, remise à 0 signal vie
int Histo_Reseau[5]={0,0,0,0,0};  // Historique Reseau cumul chaque mise à l'heure

Ticker SlowBlink;          // Clignotant lent
Ticker step;               // pas modulation pwm feux
Ticker ADC;                // Lecture des Adc
AlarmId loopPrincipale;    // boucle principale
AlarmId DebutJour;         // Debut journée
AlarmId FinJour;           // Fin de journée retour deep sleep
AlarmId Aouverture;        // Tempo ouverture verin
AlarmId Afermeture;        // Tempo fermeture verin

//---------------------------------------------------------------------------
// newtime = "" mise à l'heure NTP
// newtime = "YY/MM/DD,hh:mm:ss" mise à l'heure reçue
bool MajHeure(String newtime = "");
//---------------------------------------------------------------------------
void setup() {
  message.reserve(300); // texte des reponses

  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Version Soft : ")), Serial.println(ver);

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) { // Format la première fois utilise SPIFFS
    Serial.println(F("SPIFFS initialisation failed..."));
    SPIFFS_present = false;
  }
  else {
    Serial.println(F("SPIFFS initialised... file access enabled..."));
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
    SerialAT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    modem_on();
    Serial.print(F("Reset modem: "));
    Serial.println(modem.setPhoneFunctionality(1,1));// CFUN=1,1 full functionality, reset online mode
    delay(200);
    Serial.print(F("Modem Info: "));
    Serial.println(modem.getModemInfo());
    modem.setNetworkMode(38);  // Network Mode LTE
    modem.setPreferredMode(1); // Mode CAT-M
    modem.disableGPS();        // Arret GPS
  }
  // parametrage PWM pour le feu Rouge
  // https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
  ledcSetup(RgePwmChanel, 1000, 8);
  ledcAttachPin(PinFeuxR, RgePwmChanel);
  ledcWrite(RgePwmChanel, 0); // Feux Rouge 0

  init_adc_mm();// initialisation tableau pour adc Moyenne Mobile
  ADC.attach_ms(100, adc_read); // lecture des adc toute les 100ms
  /* Lecture configuration file config en SPIFFS */
  readConfig(); // Lecture de la config
  if (config.magic != Magique) {
    /* verification numero magique si different
    		erreur lecture SPIFFS ou carte vierge
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
    config.TypeBatt       = 16; // Pb par défaut
    config.mqttPort      = tempmqttPort;
    config.hete          = 2; // heure
    config.hhiver        = 1; // heure
    config.sendSMS       = false; // pas d'envoie de SMS
    config.keepAlive     = 300; // 5mn, IMPERATIF pour réduire conso data
    config.autoupload    = false;
    config.cptAla        = 10; // 11*Acquisition time
    String temp          = "TPCF_CVR1";
    temp.toCharArray(config.Idchar, 11);
    String tempapn       = "eapn1.net";
    String tempGprsUser  = "";
    String tempGprsPass  = "";
    config.ftpPort       = tempftpPort;
    tempapn.toCharArray(config.apn, (tempapn.length() + 1));
    tempGprsUser.toCharArray(config.gprsUser,(tempGprsUser.length() + 1));
    tempGprsPass.toCharArray(config.gprsPass,(tempGprsPass.length() + 1));
    tempServer.toCharArray(config.ftpServeur,(tempServer.length() + 1));
    tempftpUser.toCharArray(config.ftpUser,(tempftpUser.length() + 1));
    tempftpPass.toCharArray(config.ftpPass,(tempftpPass.length() + 1));
    tempServer.toCharArray(config.mqttServer, (tempServer.length() + 1));
    tempmqttUserName.toCharArray(config.mqttUserName, (tempmqttUserName.length() + 1));
    tempmqttPass.toCharArray(config.mqttPass, (tempmqttPass.length() + 1));
    copie_Topic();
    
    sauvConfig();   
  }
  PrintConfig();

  strncpy(willTopic,("S/will"),sizeof(willTopic));// topic commun

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
    delay(1000);
    // ESP.restart();
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
  Ouvrir_PB();                // ouvre le fichier Phone book
  if (gsm) {
    Serial.print(("Waiting for network..."));
    if (!modem.waitForNetwork()) {
      Serial.println(F(" fail"));
      delay(100);
      // return;
    } else {
      Serial.println(F(" success"));
    }
    if (modem.isNetworkConnected()) {
      Serial.println("Network connected");

      // Demande Operateur connecté
      Serial.print(F("Operateur :")), Serial.println(modem.getOperator());

      ConnectGPRS();

      if (modem.isGprsConnected()) { Serial.println(F("GPRS connected")); }
      IPAddress local = modem.localIP();
      Serial.println("Local IP:" + local.toString());

      Serial.print("Signal quality:"), Serial.println(read_RSSI());
    }
    
    mqttClient.setBufferSize(384);
    mqttClient.setKeepAlive(config.keepAlive);                // Set Pubsub keep alive interval
    mqttClient.setServer(config.mqttServer, config.mqttPort); // Set the MQTT broker details.
    mqttClient.setCallback(mqttSubscriptionCallback);         // Set the MQTT message handler function.

    // Synchro heure
    readmodemtime();           // Lecture heure modem
    // set modem Localtime
    Serial.println("set CLTS=1:"),Serial.println(modem.send_AT("+CLTS=1"));
    timesstatus();								// Etat synchronisation Heure Sys
    // sync Heure system avec modem (si Heure modem valide)
    // place FlagSetTime=true si Ok, sinon relancer dans la boucle
    set_system_time();
    timesstatus();								// Etat synchronisation Heure Sys
    Serial.print("Syst Time ="),Serial.println(displayTime(0));
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

  MajLog("Auto", "Lancement");
  Cvr = Position_CVR(); // lire position CVR affectée à Cvr
  LastCvr = Cvr;
  // reduire consommation eteindre Wifi et BT
  WiFi.mode(WIFI_OFF);
  btStop();

  Check_Online();
}
//---------------------------------------------------------------------------
void loop() {
  static unsigned int t0 = millis(); // timer checkOnline
  if(millis() - t0 > 5000){
    t0 = millis();
    Check_Online();
  }

  recvOneChar(); // Capture reception liaison serie locale

  if(Online){
    mqttClient.loop(); // Call the loop to maintain connection to the server.
  }

  ArduinoOTA.handle();
  Alarm.delay(0);
}	//fin loop
//---------------------------------------------------------------------------
void Acquisition() {

  if (!FlagSetTime){ // si mise à l'heure non valide
    StartStopAlarme(false);
    set_system_time();
    StartStopAlarme(true);
  }

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
  static byte lastminute = minute();
  if(minute() == 0 && minute()!=lastminute){
    lastminute = minute();
    Monitoring_Reseau();
    Serial.println("monitoring reseau");
  } else {
    lastminute = minute();
  }
  static int cpt = 0; // compte le nombre de passage boucle
  
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
    if (Cvr == 1 && !Allume && FlagDemandeAllume) {
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
      generationMessage();
      Serial.println("message serveur");
      envoieGroupeMessage(false,true);  // envoie serveur
      envoieGroupeMessage(false,false); // envoie user
      FlagFinJournee = false;
      FinJournee();
    }
    GestionMessage_CVR();
    FlagDemande_CVR = false;
    FlagVerif_CVR   = false;
  }
  //***************************************************
  // gestion phase demarrage
  if (cpt > 5 && !firstdecision) {
    /* une seule fois au demarrage attendre au moins 60s */
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
  int smsnum = -1;
  if (gsm && Online) {
    // verification index new SMS en attente(raté en lecture directe)
    smsnum = modem.newMessageIndex(0); // verifie index arrivée sms, -1 si pas de sms
    Serial.print(F("Index last SMS = ")), Serial.println (smsnum);

    if (smsnum >= 0) {	// index du SMS en attente
      // il faut les traiter
      ReadSMS(smsnum);// Lecture et traitement de tous les SMS en attente
    }
  }
  if (smsnum < 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
    FlagReset = false;
    ResetHard();					//	reset hard
  }
  //***************************************************
  if(gsm && firstdecision){ // apres demarrage
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
      // mqttConnect(); // tentative reconnexion MQTT
      if (nalaMQTT ++ > config.cptAla) {
        FlagAlarmeMQTT = true;
        // A faire action pour reconnecter MQTT
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
// check if we are Oline
void Check_Online(){  
  static int cptRegStatusFault = 0; // compteur defaut network
  bool net_status  = false;
  bool gprs_status = false;
  bool mqtt_status = false;

  net_status = modem.isNetworkConnected();
  if (net_status){
    if (modem.isGprsConnected()){
      gprs_status = true;
    } else {
      if (ConnectGPRS()){
        gprs_status = true;
      }
    }
  }
  if (net_status && gprs_status){
    if (!mqttClient.connected()){ // mqtt pas connecté, mqtt connect et on relance subscribe
      if (mqttConnect()){ // (re)Connect if MQTT client is not connected.
        if (mqttSubscribe(0) == true) {
          Serial.println("Subscribed");
          mqtt_status = true;
        }
      }
    } else { // si mqqt connecté, suppose subscribe ok
      mqtt_status = true;
    }
  }

  if(net_status && gprs_status && mqtt_status){ // we are Online
    Online = true;
  } else {
    Online = false;
  }

  Sbidon = displayTime(0) + fl;
  Sbidon += "Online,res,gprs,mqtt,nRegFault :" + String(Online) + ":" + String(net_status) + ":" + String(gprs_status);
  Sbidon += ":" + String(mqtt_status) + ":" + String(cptRegStatusFault) + fl;
  Serial.print(Sbidon);
  if(net_status){
    Serial.print(modem.getOperator());
    IPAddress local = modem.localIP();
    Serial.println("; IP:" + local.toString());
  }

  // Patch Blocage modem
  if(! net_status){
    if(cptRegStatusFault ++ > 24){ // si hors réseau > 24*5s=120s
      cptRegStatusFault = 0;
      NbrResetModem +=1;
      Sbidon = "Reset modem suite Reg status fault" + String(cptRegStatusFault);
      Serial.println(Sbidon);
      if(modem.setPhoneFunctionality(1,1)){// CFUN=1,1 full functionality, reset online mode
        modem.setNetworkMode(38);  // Network Mode LTE
        modem.setPreferredMode(1); // Mode CAT-M
      }
    }
  } else {
    cptRegStatusFault = 0;// reset compteur
  }
}
//---------------------------------------------------------------------------
// Lecture SMS
void ReadSMS(int index){
  // index du SMS
  // verifier appelant connu si OK copier texte sms dans Rmessage
  // effacer SMS
  // et envoyer traite_sms("SMS")

  Sms smsstruct;
  if (!modem.readSMS(&smsstruct,index)){
    Serial.print(F("Didn't find SMS message in slot! "));
    Serial.println(index);
  }
  if(! Cherche_N_PB(smsstruct.sendernumber)){
    Serial.println(F("Appelant inconnu"));
    EffaceSMS(index);
    return;
  }
  Rmessage = smsstruct.message;
  EffaceSMS(index);
  traite_sms("SMS");
}
//---------------------------------------------------------------------------
// Interpretation des messages
// Origine = Local, BLE, SMS, MQTTS (serveur), MQTTU (user)
void traite_sms(String Origine) {
  bool sms = false;
  if(Origine == "SMS") sms = true;
  
  bool smsserveur = false; // true si le sms provient du serveur
  if (Origine == "MQTTS") smsserveur = true;
  
  /* Variables pour mode calibration */
  static int tensionmemo = 0;           //	memorisation tension batterie lors de la calibration
  int coef = 0;                         // coeff temporaire
  static byte P = 0;                    // Pin entrée a utiliser pour calibration
  static byte M = 0;                    // Mode calibration 1,2,3,4
  static bool FlagCalibration = false;	// Calibration Tension en cours

  if (!(Rmessage.indexOf(F("TEL")) == 0 || Rmessage.indexOf(F("tel")) == 0 || Rmessage.indexOf(F("Tel")) == 0
      || Rmessage.indexOf(F("Wifi")) == 0
      || Rmessage.indexOf(F("MQTTDATA")) > -1 || Rmessage.indexOf(F("MQTTSERVEUR")) > -1
      || Rmessage.indexOf(F("GPRSDATA")) > -1 || Rmessage.indexOf(F("FTPDATA")) > -1 || Rmessage.indexOf(F("FTPSERVEUR")) > -1)) {
    Rmessage.toUpperCase();	// passe tout en Maj sauf si "TEL" ou "WIFI"... parametres pouvant contenir minuscules
    Rmessage.replace(" ", "");// supp tous les espaces
  }

  messageId();
  if (Rmessage.indexOf("TIMEOUTWIFI") > -1) { // Parametre Arret Wifi
    if (Rmessage.indexOf(char(61)) == 11) {
      int n = Rmessage.substring(12, Rmessage.length()).toInt();
      if (n > 9 && n < 3601) {
        config.timeoutWifi = n;
        sauvConfig();														// sauvegarde en SPIFFS
      }
    }
    message += "TimeOut Wifi (s) = ";
    message += config.timeoutWifi;
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("WIFIOFF") > -1) { // Arret Wifi
    message += "Wifi off";
    message += fl;
    sendReply(Origine);
    WifiOff();
  }
  else if (Rmessage.indexOf("Wifi") == 0) { // demande connexion Wifi
    byte pos1 = Rmessage.indexOf(char(44));//","
    byte pos2 = Rmessage.indexOf(char(44), pos1 + 1);
    if(pos1==255 || pos1<4 || pos2==255 || pos2<4){
      // format incomplet
      message += "erreur format";
      sendReply(Origine);
      return;
    }
    String ssids = Rmessage.substring(pos1 + 1, pos2);
    String pwds  = Rmessage.substring(pos2 + 1, Rmessage.length());
    char ssid[25];
    char pwd[30];
    ssids.toCharArray(ssid, ssids.length() + 1);
    ssids.toCharArray(ssid, ssids.length() + 1);
    pwds.toCharArray(pwd, pwds.length() + 1);
    ConnexionWifi(ssid, pwd, Origine);
  }
  else if (Rmessage.indexOf(F("TEL")) == 0
        || Rmessage.indexOf(F("Tel")) == 0
        || Rmessage.indexOf(F("tel")) == 0) { // entrer nouveau num
    byte lastPBline = last_PB(); // recupere le n° de la derniere ligne du PB
    bool newPB = false;
    bool FlagOK = true;
    bool efface = false;
    byte j = 0;
    String newnumero;
    String newnom;
    int indexreplace = 0;
    if (Rmessage.indexOf(char(61)) == 4) {  // TELn= reserver correction/suppression
      int i = Rmessage.substring(3).toInt();// recupere n° de index
      i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
      if (i < 1) FlagOK = false;
      indexreplace = i;// index du PB a remplacer
      j = 5;
      // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
      if ((i != 1) && (i<=lastPBline) &&(Rmessage.indexOf(F("efface")) == 5 || Rmessage.indexOf(F("EFFACE")) == 5 )) {
        efface = true;
        strcpy(PB_list[i] , "");
        if(i < lastPBline){
          // il faut décaler les lignes vers le bas
          for (int ligne = i;ligne<lastPBline;ligne ++){
            strcpy(PB_list[ligne] , PB_list[ligne+1]);
          }
          if(lastPBline < 9){
            strcpy(PB_list[lastPBline] , "");// efface derniere ligne
          }
        }
        Save_PB();
        message += "ligne effacee";
        goto fin_tel;
      }
    }
    else if (Rmessage.indexOf(char(61)) == 3) { // TEL= nouveau numero
      j = 4;
      newPB = true;
    }
    else {
      FlagOK = false;
    }
    if (Rmessage.indexOf("+") == j) {			          // debut du num tel +
      if (Rmessage.indexOf(char(44)) == j + 12) {	  // verif si longuer ok
        newnumero = Rmessage.substring(j, j + 12);
        newnom = Rmessage.substring(j + 13, j + 27);// tronque à 14 car
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
      message += F("Cde non reconnue/erreur ?");// non reconnu
      message += fl;
      sendReply(Origine);
    }
    else {
      if (!efface) {
        String bidon = newnumero + ";" + newnom;
        if(newPB){ // Nouvelle ligne
          strcpy(PB_list[lastPBline + 1] , bidon.c_str());
        } else {   // Remplacement ligne
          strcpy(PB_list[indexreplace] , bidon.c_str());
        }
        message += "Nouvelle entree Phone Book :" + fl;
        message += bidon;

      }
      Save_PB();
      sendReply(Origine);
    }
  }
  else if (gsm && (Rmessage == F("LST") || Rmessage == F("LST?") || Rmessage == F("LST1"))) {	//	Liste des Num Tel
    Read_PB();
    for(byte i = 1;i<10;i++){
      if(strlen(PB_list[i]) > 0){
        // Serial.println(PB_list[i]);
        message += i;
        message += ":";
        message += String(PB_list[i]);
        message += fl;
      }
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("ETAT") == 0 || Rmessage.indexOf("ST") == 0) {// "ETAT? de l'installation"
    generationMessage();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("SYS") > -1) {
    if (gsm) {
      message += modem.getOperator(); // Operateur
      message += fl;
      byte n = modem.getRegistrationStatus();        
      if (n == 5) {
        message += F(("rmg, "));// roaming 1.0s
      }
      message += " ";
      message += ConnectedNetwork();
      message += fl;
      message += read_RSSI();														// info RSSI
      message += fl;
      message += "Batt GSM : ";
      message += String(modem.getBattVoltage());
      message += " mV, ";
      message += fl;
    }
    message += "Ver: ";
    message += ver;
    message += fl;
    message += "V Batt Sol= ";
    message += String(float(TensionBatterie / 100.0));
    message += "V, ";
    if (config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
    if (config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
    message += " %";
    message += fl;
    message += "V USB= ";
    message += (float(VUSB / 1000.0));
    message += "V";
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("ID=") == 0) {			//	Id= nouvel Id
    String temp = Rmessage.substring(3);
    if (temp.length() > 0 && temp.length() < 11) {
      Id = "";
      temp.toCharArray(config.Idchar, 11);
      mqttSubscribe(1); // unsubscribe
      mqttClient.disconnect();

      copie_Topic(); // Nouvel Id dans Topic
      sauvConfig();														// sauvegarde config

      mqttConnect();
      mqttSubscribe(0); // subscribe

      Id = String(config.Idchar);
      Id += fl;
    }
    messageId();
    message += "Nouvel Id";
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("LOG") == 0) { // demande taille du log
    File f = SPIFFS.open(filelog, "r");    // taille du fichier log en SPIFFS
    message = F("local log size :");
    message += String(f.size()) + fl;
    f.close();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("ANTICIP") > -1) { // Anticipation du wakeup
    if (Rmessage.indexOf(char(61)) == 7) {
      int n = Rmessage.substring(8, Rmessage.length()).toInt();
      if (n > 9 && n < 3601) {
        config.anticip = n;
        sauvConfig();														// sauvegarde en SPIFFS
      }
    }
    message += "Anticipation WakeUp (s) = ";
    message += config.anticip;
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("DEBUT") == 0) {     //	Heure Message Vie/debutJour
    if (Rmessage.indexOf(char(61)) == 5) {
      long i = atol(Rmessage.substring(6).c_str()); //	Heure message Vie
      if (i > 0 && i <= 86340) {                    //	ok si entre 0 et 86340(23h59)
        config.DebutJour = i;
        sauvConfig();                               // sauvegarde en SPIFFS
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
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("TIME") == 0) {
    message += "Heure Sys = ";
    message += displayTime(0);
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("MAJHEURE")) == 0) {	//	forcer mise a l'heure
    messageId();
    if (Rmessage.indexOf(char(61)) == 8) {
      Sbidon = Rmessage.substring(9,Rmessage.length());      
      if (Sbidon.length()== 17){
        if (MajHeure(Sbidon)){	// mise a l'heure demandée
          messageId();
          message += "Mise à l'heure OK";
        } else {
          message += "Erreur mise à l'heure";
        }
      }
    } else {
      MajHeure("");			// mise a l'heure NTP
      messageId();
      message += "Mise à l'heure NTP";
    }  
    sendReply(Origine);
  }
  else if (gsm && Rmessage.indexOf(F("IMEI")) > -1) {
    message += F("IMEI = ");
    message += modem.getIMEI();
    sendReply(Origine);
  }
  else if (gsm && Rmessage.indexOf(F("CCID")) > -1) {
    message += F("CCID = ");
    message += modem.getSimCCID();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("FIN") == 0) {			//	Heure Fin de journée
    if ((Rmessage.indexOf(char(61))) == 3) {
      long i = atol(Rmessage.substring(4).c_str()); //	Heure
      if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
        config.FinJour = i;
        sauvConfig();															// sauvegarde en SPIFFS
        Alarm.disable(FinJour);
        Alarm.write(FinJour, config.FinJour); // init tempo
        Alarm.enable(FinJour);
      }
    }
    message += "Fin Journee = ";
    message += Hdectohhmm(config.FinJour);
    message += "(hh:mm)";
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("MOIS") > -1) { // Calendrier pour un mois
    /* mise a jour calendrier ;format : MOIS=mm,31 fois 0/1
      demande calendrier pour un mois donné ; format : MOIS=mm? */
    bool flag = true; // validation du format
    bool W = true;    // true Write, false Read
    int m = 0;
    if (Rmessage.indexOf("{") == 0) { // json
      JsonDocument doc;
      int f = Rmessage.lastIndexOf("}");
      DeserializationError err = deserializeJson(doc, Rmessage.substring(0, f + 1));
      if(!err){
        m = doc["MOIS"]; // 12
        JsonArray jour = doc["JOUR"];
        for (int j = 1; j < 32; j++) {
          calendrier[m][j] = jour[j - 1];
        }
        // Serial.print("mois:"),Serial.println(m);
        EnregistreCalendrier(); // Sauvegarde en SPIFFS
      }
      else{
        message += " erreur json ";
        flag = false;
      }
    }
    else { // message normal mois=12,31*0/1
      byte p1 = Rmessage.indexOf(char(61)); // =
      byte p2 = Rmessage.indexOf(char(44)); // ,
      if (p2 == 255) {                      // pas de ,
        p2 = Rmessage.indexOf(char(63));    // ?
        W = false;
      }

      m = Rmessage.substring(p1 + 1, p2).toInt(); // mois
      if (!(m > 0 && m < 13)) flag = false;
      if (W && flag) { // Write
        if (!(Rmessage.substring(p2 + 1, Rmessage.length()).length() == 31)) flag = false; // si longueur = 31(jours)

        for (int i = 1; i < 32; i++) { // verification 0/1
          if (!(Rmessage.substring(p2 + i, p2 + i + 1) == "0" || Rmessage.substring(p2 + i, p2 + i + 1) == "1")) {
            flag = false;
          }
        }
        if (flag) {
          // Serial.println(F("mise a jour calendrier"));
          for (int i = 1; i < 32; i++) {
            calendrier[m][i] = Rmessage.substring(p2 + i, p2 + i + 1).toInt();
            // Serial.print(Rmessage.substring(p2+i,p2+i+1));
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
        JsonDocument doc;
        doc["mois"] = m;
        JsonArray jour = doc["jour"].to<JsonArray>();
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
    sendReply(Origine);
  }
  else if (Rmessage == "CIRCULE") {
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
    sendReply(Origine);
    // necessaire pour jour non circulé sur reception circule on lance
    // si reception CIRCULE on ne lance pas avant firstdecision 
    if (ok && firstdecision) {
      SignalVie();
    }
  }
  else if (Rmessage == F("NONCIRCULE")) {
    bool ok = false;
    /* demande passer en mode nonCirculé pour le jour courant,
      sans modification calendrier enregistré en SPIFFS 
      extinction Feux*/
    if (calendrier[month()][day()] ^ FlagCircule) {
      // calendrier[month()][day()] = 0;
      message += F("OK, NonCircule");
      FlagCircule = !FlagCircule;
      ok = true;
    }
    else {
      message += F("Jour deja NonCircule");
    }
    message += fl;
    sendReply(Origine);
    if (ok && firstdecision) {
      // Seulement si déjà lancé apres première décision
      // sinon au lancement, on attend première décision
      // action_wakeup_reason(4);
      FinJournee(); // si fermé, ouverture avant sleep
    }
  }
  else if (Rmessage.indexOf("TEMPOWAKEUP") == 0) { // Tempo wake up
    if ((Rmessage.indexOf(char(61))) == 11) {
      int i = Rmessage.substring(12).toInt(); //	durée
      if (i > 59 && i <= 36000) { // 1mn à 10H
        config.RepeatWakeUp = i;
        sauvConfig();															// sauvegarde en SPIFFS
      }
    }
    message += "Tempo repetition Wake up (s)=";
    message += config.RepeatWakeUp;
    sendReply(Origine);
  }
  else if (Rmessage == "RST" || Rmessage == "RESET") {               // demande RESET
    message += "Le systeme va etre relance";  // apres envoie du SMS!
    message += fl;
    FlagReset = true;                            // reset prochaine boucle
    sendReply(Origine);
  }
  else if (Rmessage == "FAULTRESET") { // reset Erreur Position
    FlagAlarmePosition = false;
    FlagLastAlarmePosition = false;
    Cvr = Position_CVR(); // lire position CVR affectée à Cvr
    LastCvr = Cvr;
    ledcWrite(RgePwmChanel, 0); // Extinction Feux Rouge
    Allume = false;
    SlowBlink.detach();
    envoieGroupeMessage(false, false); // pasVie, User
  }
  else if (Rmessage.indexOf("CALIBRATION=") == 0) {
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
    Sbidon = Rmessage.substring(12, 16); // texte apres =
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
      if (config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
      if (config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
      message += "%";
    }
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(Id.substring(5, 9)) == 1) { // cherche CVR1
    if (Rmessage.indexOf("F") == 0) {
      // demande Fermeture
      Memo_Demande_CVR[0] = Origine;  // nom demandeur
      Memo_Demande_CVR[1] = Origine;  // num demandeur
      Memo_Demande_CVR[2] = Rmessage; // demande d'origine
      FlagDemande_CVR = true;
      MajLog(Origine,  "demande : " + Rmessage);
      Fermer_CVR();
    }
    else if (Rmessage.indexOf("O") == 0 || Rmessage.indexOf("D") == 0) {// D compatible avec DCvxx
      // demande Ouverture
      Memo_Demande_CVR[0] = Origine;  // nom demandeur
      Memo_Demande_CVR[1] = Origine;  // num demandeur
      Memo_Demande_CVR[2] = Rmessage; // demande d'origine
      FlagDemande_CVR = true;
      MajLog(Origine, "demande : " + Rmessage);
      Ouvrir_CVR();
    }
    else {
      message += "non reconnu" + fl;
      sendReply(Origine);
    }
  }
  else if (Rmessage.indexOf("TEMPOOUVERTURE") >= 0) {
    if (Rmessage.indexOf(char(61)) == 14) {
      int n = Rmessage.substring(15, Rmessage.length()).toInt();
      if (n > 4 && n < 121) {
        config.Tempoouverture = n;
        sauvConfig();														// sauvegarde en SPIFFS
      }
    }
    message += "Tempo Ouverture (s)= ";
    message += config.Tempoouverture;
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("TEMPOFERMETURE") >= 0) {
    if (Rmessage.indexOf(char(61)) == 14) {
      int n = Rmessage.substring(15, Rmessage.length()).toInt();
      if (n > 4 && n < 121) {
        config.Tempofermeture = n;
        sauvConfig();														// sauvegarde en SPIFFS
      }
    }
    message += "Tempo Fermeture (s)= ";
    message += config.Tempofermeture;
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("PARAM") >= 0) {
    // conserve meme format que Cv, remplissage pour variables inexistante dummy
    // message param divisé en 2 trop long depasse long 1sms 160c
    int dummy = 0;
    bool erreur = false;
    // Serial.print("position X:"),Serial.println(Rmessage.substring(7, 8));
    if (Rmessage.substring(7, 8) == "1") { // PARAM1
      // Serial.print("position ::"),Serial.println(Rmessage.substring(9, 10));
      if (Rmessage.substring(9, 10) == ":") {
        // json en reception sans lumlut
        DynamicJsonDocument doc(200);
        DeserializationError err = deserializeJson(doc, Rmessage);
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
    else if (Rmessage.substring(7, 8) == "2") { // PARAM2
      if (Rmessage.substring(9, 10) == ":") {
        // json en reception sans lumlut
        DynamicJsonDocument doc(200);
        DeserializationError err = deserializeJson(doc, Rmessage);
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
      sendReply(Origine);
    } else {
      Serial.println(message);
    }
  }
  else if (gsm && Rmessage.indexOf(F("UPLOADLOG")) == 0) {//upload log sur demande
    message += "Fonction non active";
    // message += F("lancement upload log");
    // message += fl;
    // MajLog(Origine, "upload log");// renseigne log
    // Serial.println(F("Starting..."));
    // bool reply = FTP_upload_function(filelog); // Upload fichier
    // Serial.println("The end... Response: " + String(reply));

    // if(reply == true){
    //   message += F("upload OK");
    //   SPIFFS.remove(filelog);          // efface fichier log
    //   MajLog(Origine, "");             // nouveau log
    //   MajLog(Origine, F("upload OK")); // renseigne nouveau log
    // } else {
    //   message += F("upload fail");
    //   MajLog(Origine, F("upload fail"));// renseigne log
    // }
    sendReply(Origine);
  }
  else if (gsm && Rmessage.indexOf(F("COEFF")) == 0) {//Lecture/ecriture des coeff
    // COEFF=xxxx,xxxx,xxxx,xxxx
    if(Rmessage.indexOf(char(61)) == 5){ // =
      Sbidon = Rmessage.substring(6, Rmessage.length());
      Serial.println(Sbidon);
      int tempo[4] = {0,0,0,0};
      byte p1 = 0;
      byte p2 = 0;
      bool flag = true;
      for(int i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++){          
        // printf("i=%d,p1=%d,p2=%d\n",i,p1,p2);
        p2 = Sbidon.indexOf(char(44), p1 + 1); // ,
        tempo[i] = Sbidon.substring(p1,p2).toInt();
        if(tempo[i] < 0) flag = false;
        if(i!=3 && p2 == 255) flag = false;
        p1 = p2 + 1;          
        // printf("i=%d,p1=%d,p2=%d\n",i,p1,p2);
      }
      if (flag){ // format OK
        for(int i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++){
          CoeffTension[i] = tempo[i];
        }
        Recordcalib(); // enregistre en SPIFFS
      }
    }
    message += "Coeff calibration:" + fl;
    for(int i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++){
      message += String(CoeffTension[i]);
      if(i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])- 1) ) message += ",";
    }
    Serial.println(message);
    sendReply(Origine);
  }
  else if (gsm && Rmessage.indexOf(F("UPLOADCOEFF")) == 0) {//upload des coeff
    message += "Fonction non active";
    // message += F("lancement upload Coeff");
    // message += fl;
    // MajLog(Origine, "upload coeff");// renseigne log
    // Serial.println(F("Starting..."));
    // bool reply = FTP_upload_function(filecalibration); // Upload fichier
    // Serial.println("The end... Response: " + String(reply));

    // if(reply == true){
    //   message += F("upload OK");
    //   MajLog(Origine, F("upload Coeff OK"));// renseigne nouveau log
    // } else {
    //   message += F("upload fail");
    //   MajLog(Origine, F("upload Coeff fail"));// renseigne log
    // }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("FTPDATA") > -1) {
    // Parametres FTPDATA=Serveur:User:Pass:port
    // {"FTPDATA":{"serveur":"dd.org","user":"user","pass":"pass",,"port":00}}
    bool erreur = false;
    bool formatsms = false;
    if (Rmessage.indexOf(":") == 10) { // format json
      DynamicJsonDocument doc(210); //https://arduinojson.org/v6/assistant/
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject ftpdata = doc["FTPDATA"];
        strncpy(config.ftpServeur,  ftpdata["serveur"], 26);
        strncpy(config.ftpUser,     ftpdata["user"],    11);
        strncpy(config.ftpPass,     ftpdata["pass"],    16);
        config.ftpPort         =    ftpdata["port"];
        sauvConfig();													// sauvegarde en SPIFFS
      }
    }
    else if ((Rmessage.indexOf(char(61))) == 7) { // format sms
      formatsms = true;
      byte w = Rmessage.indexOf(":");
      byte x = Rmessage.indexOf(":", w + 1);
      byte y = Rmessage.indexOf(":", x + 1);
      byte zz = Rmessage.length();
      if (Rmessage.substring(y + 1, zz).toInt() > 0) { // Port > 0
        if ((w - 7) < 25 && (x - w - 1) < 11 && (y - x - 1) < 16) {
          Sbidon = Rmessage.substring(7, w);
          Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(w + 1, x);
          Sbidon.toCharArray(config.ftpUser, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(x + 1, y);
          Sbidon.toCharArray(config.ftpPass, (Sbidon.length() + 1));
          config.ftpPort = Rmessage.substring(y + 1, zz).toInt();
          sauvConfig();													// sauvegarde en SPIFFS
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
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("FTPSERVEUR") == 0) { // Serveur FTP
    // case sensitive
    // FTPSERVEUR=xyz.org
    if (Rmessage.indexOf(char(61)) == 10) {
      Sbidon = Rmessage.substring(11);
      Serial.print("ftpserveur:"), Serial.print(Sbidon);
      Serial.print(" ,"), Serial.println(Sbidon.length());
      Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
      sauvConfig();
    }
    message += "FTPserveur =";
    message += String(config.ftpServeur);
    message += "\n au prochain demarrage";
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("MQTTDATA") > -1) {
    // Parametres MQTTDATA=serveur:user:pass:port
    // {"MQTTDATA":{"serveur":"xxxx.org","user":"uuu","pass":"passpass","port":9999}}

    bool erreur = false;
    // bool formatsms = false;
    if (Rmessage.indexOf(":") == 11) { // format json
      JsonDocument doc; //https://arduinojson.org/v7/assistant/#/step1
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject mqttdata = doc["MQTTDATA"];
        strncpy(config.mqttServer,     mqttdata["serveur"], 26);
        strncpy(config.mqttUserName,   mqttdata["user"]   , 11);
        strncpy(config.mqttPass,       mqttdata["pass"]   , 16);
        config.mqttPort            =   mqttdata["port"];
        sauvConfig();
      }
    }
    
    if (!erreur) {
      JsonDocument doc;
      JsonObject MQTTDATA = doc["MQTTDATA"].to<JsonObject>();
      MQTTDATA["serveur"] = config.mqttServer;
      MQTTDATA["user"]    = config.mqttUserName;
      MQTTDATA["pass"]    = config.mqttPass;
      MQTTDATA["port"]    = config.mqttPort;
      
      Sbidon = "";
      serializeJson(doc, Sbidon);
      message += Sbidon;
      message += fl;
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("TOPIC") > -1) { // MQTT Topic
    // Parametres TOPIC="sendTopic":"sendtopic0,sendtopic1","recvTopic":"recvtopic0,recvtopic1"
    //"{"TOPIC":{"sendTopic0": "sendtopic0", "sendTopic1": "sendtopic1","recvTopic0":"recvtopic0","recvTopic1":"recvtopic1"}}"
    
    bool erreur = false;
    if (Rmessage.indexOf(":") == 8) { // format json
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, Rmessage);
      if (err) {
        erreur = true;
      }
      else {
        JsonObject TOPIC = doc["TOPIC"];
        strncpy(config.sendTopic[0],TOPIC["sendTopic0"],12);
        strncpy(config.sendTopic[1],TOPIC["sendTopic1"],12);
        strncpy(config.recvTopic[0],TOPIC["recvTopic0"],12);
        strncpy(config.recvTopic[1],TOPIC["recvTopic1"],12);
        sauvConfig();
        copie_Topic();
      }
    }
    if (!erreur){
      JsonDocument doc;
      JsonObject TOPIC = doc["TOPIC"].to<JsonObject>();
      
      TOPIC["sendTopic0"] = config.sendTopic[0];
      TOPIC["sendTopic1"] = config.sendTopic[1];
      TOPIC["recvTopic0"] = config.recvTopic[0];
      TOPIC["recvTopic1"] = config.recvTopic[1];

      Sbidon = "";
      serializeJson(doc, Sbidon);
      message += Sbidon;
      message += fl;
    }
    else {
      message += "Erreur format";
      message += fl;
    }
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("MQTTSERVEUR") == 0) { // Serveur MQTT
    // case sensitive
    // MQTTSERVEUR=abcd.org
    if (Rmessage.indexOf(char(61)) == 11) {
      Sbidon = Rmessage.substring(12);
      Serial.print("mqttserveur:"),Serial.print(Sbidon);
      Serial.print(" ,"), Serial.println(Sbidon.length());
      Sbidon.toCharArray(config.mqttServer, (Sbidon.length() + 1));
      sauvConfig();
    }
    message += F("MQTTserveur =");
    message += String(config.mqttServer);
    message += F("\n au prochain demarrage");
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("GPRSDATA") > -1) {
    // Parametres GPRSDATA = "APN":"user":"pass"
    // GPRSDATA="sl2sfr":"":""
    // {"GPRSDATA":{"apn":"sl2sfr","user":"","pass":""}}
    bool erreur = false;
    bool formatsms = false;
    if (Rmessage.indexOf(":") == 11) { // format json
      DynamicJsonDocument doc(120);
      DeserializationError err = deserializeJson(doc, Rmessage);
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
        sauvConfig();													// sauvegarde en SPIFFS
      }
    }
    else if ((Rmessage.indexOf(char(61))) == 8) { // format sms
      formatsms = true;
      byte cpt = 0;
      byte i = 9;
      do { // compte nombre de " doit etre =6
        i = Rmessage.indexOf('"', i + 1);
        cpt ++;
      } while (i <= Rmessage.length());
      Serial.print("nombre de \" :"), Serial.println(cpt);
      if (cpt == 6) {
        byte x = Rmessage.indexOf(':');
        byte y = Rmessage.indexOf(':', x + 1);
        byte z = Rmessage.lastIndexOf('"');
        // Serial.printf("%d:%d:%d\n",x,y,z);
        // Serial.printf("%d:%d:%d\n", x -1 - 10, y-1 - x-1-1, z - y-1-1);
        if ((x - 11) < 11 && (y - x - 3) < 11 && (z - y - 2) < 11) { // verification longueur des variables
          Sbidon = Rmessage.substring(10, x - 1);
          Sbidon.toCharArray(config.apn, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(x + 1 + 1 , y - 1);
          Sbidon.toCharArray(config.gprsUser, (Sbidon.length() + 1));
          Sbidon = Rmessage.substring(y + 1 + 1, z);
          Sbidon.toCharArray(config.gprsPass, (Sbidon.length() + 1));

          // Serial.print("apn:"),Serial.println(config.apn);
          // Serial.print("user:"),Serial.println(config.gprsUser);
          // Serial.print("pass:"),Serial.println(config.gprsPass);

          sauvConfig();													// sauvegarde en SPIFFS
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
    sendReply(Origine);
  }
  else if (Rmessage == "VIDELOG"){
    SPIFFS.remove(filelog);
    FileLogOnce = false;
    message += "Effacement fichier log";
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("AUTOUPLOAD") == 0){ // Auto upload log vers serveur FTP
    message += "Fonction non active";
    // if (Rmessage.indexOf(char(61)) == 10) {
    //   byte c = Rmessage.substring(11).toInt();
    //   if(c==0 || c==1){
    //     config.autoupload = c;
    //     sauvConfig();
    //   }
    // }
    // message += "Autoupload:";
    // message += String(config.autoupload);
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("CPTALATRCK")) == 0 || Rmessage.indexOf(F("CPTALA")) == 0) { // Compteur Ala avant Flag
    if (Rmessage.indexOf(char(61)) == 10) {
      int c = Rmessage.substring(11).toInt();
      if (c > 1 && c < 501) {
        config.cptAla = c;
        sauvConfig();
      }
    }
    message += F("Cpt Ala Tracker (x10s)=");
    message += String(config.cptAla);
    message += fl;
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("SETNETWORKMODE")) >= 0) {// Set Prefered network Mode
    if(Rmessage.indexOf(char(61)) == 14){
      int mode = Rmessage.substring(15).toInt();
      if(mode == 2 || mode == 13 || mode == 38 || mode == 51){
        modem.setNetworkMode(mode);
        delay(1000);
      }
    }
    message += String(modem.send_AT(F("+CNMP?")));
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("SENDAT")) == 0){
    // envoie commande AT au SIM7000
    // ex: SENDAT=AT+CCLK="23/07/19,10:00:20+04" mise à l'heure
    // attention DANGEREUX pas de verification!
    if (Rmessage.indexOf(char(61)) == 6) {
      String CdeAT = Rmessage.substring(7, Rmessage.length());
      String reply = sendAT(CdeAT,"OK","ERROR",1000);
      // Serial.print("reponse: "),Serial.println(reply);
      message += String(reply);
      sendReply(Origine);
    }
  }
  else if (Rmessage.indexOf(F("MODEMINFO")) == 0){
    // Get Modem Info
    message += modem.getModemInfo();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("CPTRESETMODEM")) == 0){
    // Demande nombre de reset modem
    message += F("Compteur reset Modem : ");
    message += String(NbrResetModem);
    sendReply(Origine);
  }else if (Rmessage.indexOf(F("NETWORKHISTO")) == 0){
    // Demande historique Changement etat reseau
    message_Monitoring_Reseau();
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("TYPEBATT")) == 0){ // Type Batterie
    if (Rmessage.indexOf(char(61)) == 8) {
      int type = Rmessage.substring(9, Rmessage.length()).toInt();
      if(type == 16 || type == 24){
        config.TypeBatt = type;
        sauvConfig();													// sauvegarde en SPIFFS
      }
    }
    message += "Type Batterie:" + fl;
    if(config.TypeBatt == 16) message += "Pb 12V";
    if(config.TypeBatt == 24) message += "LiFePO 12.8V";
    sendReply(Origine);
  }
  else if (Rmessage.indexOf("AUTORISATIONSMS") == 0) {
    // Autorisation envoie SMS
    if (Rmessage.indexOf(char(61)) == 15) {
      int i = Rmessage.substring(16).toInt();
      if (i == 0){
        config.sendSMS = 0;
      } else if (i == 1){
        config.sendSMS = 1;
      }
      sauvConfig();													// sauvegarde config
      Sbidon = F("Autorisation SMS=");
      Sbidon += String(config.sendSMS);
      MajLog(Origine, Sbidon);// renseigne log
    }
    message += "Autorisation SMS : ";
    message += String(config.sendSMS);
    sendReply(Origine);
  }
  else if (Rmessage.indexOf(F("FEUROUGE")) == 0){ // Avec/sans allumage du feu rouge si fermé
    if (Rmessage.indexOf(char(61)) == 8) {
      if(Rmessage.substring(9, Rmessage.length()).toInt() == 1){
        FlagDemandeAllume = true;
      } else {
        FlagDemandeAllume = false;        
        ledcWrite(RgePwmChanel, 0); // Extinction Feux Rouge
        Allume = false;
        SlowBlink.detach();
      }
    }
    if(FlagDemandeAllume){
      message += "Avec Feu Rouge Allume";
    } else {
      message += "Sans Feu Rouge Allume";
    }
    sendReply(Origine);
    if (FlagDemandeAllume) Acquisition(); // Allumage au passage dans Acquisition
  }
  //**************************************
  else {
    message += "Commande non reconnu !";		//"Commande non reconnue ?"
    sendReply(Origine);
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
    envoieGroupeMessage(false, true);	 // pasVie, Serveur
    envoieGroupeMessage(false, false); // pasVie, User
    SendEtat = false;					// efface demande
  }
}
//---------------------------------------------------------------------------
void envoieGroupeMessage(bool vie, bool Serveur) {
  generationMessage();
  if (vie) { 						// si envoie Etat demandé
  }
  if(config.sendSMS){
  // A Finir 
  }
  Envoyer_MQTT(Serveur);
}
//---------------------------------------------------------------------------
void generationMessage() {
  messageId();
  if (FlagAlarmeTension || FlagLastAlarmeTension || FlagAlarmePosition ) {
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
    if (config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
    if (config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
    message += "%" + fl;
  }
  else {
    message += "Alarme, ";
    if (config.TypeBatt == 16) message += String(BattPBpct(TensionBatterie, 6));
    if (config.TypeBatt == 24) message += String(BattLiFePopct(TensionBatterie, 4));
    message += "%";
    message += fl;
    message += "V USB =";
    message += String(float(VUSB / 1000.0)) + fl;
  }
  if ((calendrier[month()][day()] ^ FlagCircule)) {
    message += "Jour Circule";
  }
  else {
    message += "Jour Non Circule";
  }
  message += fl;
  if(FlagAlarmePosition){
    message += "Defaut Position";
    message += fl;
  }
  if(config.sendSMS){
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
}
//---------------------------------------------------------------------------
// Envoyer une réponse
// Origine = Local, BLE, SMS, MQTTS (serveur), MQTTU (user)
void sendReply(String Origine) {
  if (gsm) {
    if (Origine == "MQTTS"){ // reponse MQTT      
      Envoyer_MQTT(true); // Serveur
    }
    else if (Origine == "MQTTU"){ // reponse MQTT
      Envoyer_MQTT(false); // User
    }
    else if (Origine == "SMS"){
      if (config.sendSMS){
        // A finir
      }
    }
  }
  Serial.println(F("****************************"));
  Serial.println(message);
  Serial.println(F("****************************"));
}
//---------------------------------------------------------------------------
// Envoyer message en MQTT
// destinataire = true Serveur, false à User
void Envoyer_MQTT(bool dest){
  Serial.println("Sending MQTT len:");
  Serial.println(message.length());
  Serial.print(F("to:"));
  Serial.print(dest ? config.sendTopic[0] : config.sendTopic[1]);
  Serial.print(F(":"));
  if(Online){
    if(dest){ // message Serveur
      if (mqttClient.publish(config.sendTopic[0], message.c_str()), true){ // Serveur
        AlarmeMQTT = false;
        Serial.println(F("OK"));
      } else {
        Serial.println(F("KO"));
        AlarmeMQTT = true;
      }
    } else { // message user
      if(mqttClient.publish(config.sendTopic[1], message.c_str()), true){ // User
        AlarmeMQTT = false;
        Serial.println(F("OK"));
      } else {
        Serial.println(F("KO"));
        AlarmeMQTT = true;
      }
    }
  } else {Serial.println("Off Line");}
}
//---------------------------------------------------------------------------
// newtime = "" mise à l'heure NTP
// newtime = "YY/MM/DD,hh:mm:ss" mise à l'heure reçue
bool MajHeure(String newtime){
  if(newtime.length() == 0){ // mise à l'heure NTP
    if(HeureEte()){
      SyncHeureModem(config.hete*4, true);
    } else {
      SyncHeureModem(config.hhiver*4, true);
    }
    StartStopAlarme(false);
    set_system_time(); // Set systeme time to modem time
    StartStopAlarme(true);
    return true;
  } else if(newtime.length() == 17){ // mise à l'heure reçue, long OK
    newtime.replace(","," ");
    Sbidon = "AT+CCLK=\"" + newtime + "\"";
    modem.send_AT(Sbidon);
    StartStopAlarme(false);
    set_system_time();  // Set systeme time to modem time
    StartStopAlarme(true);
    return true;
  } else { // format non reconnu
    return false;
  }
}
//---------------------------------------------------------------------------
long DureeSleep(long Htarget) { // Htarget Heure de reveil visée
  /* calcul durée entre maintenant et Htarget*/
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
    modem.deleteSmsMessage(0,4);// au cas ou, efface tous les SMS envoyé/reçu
    NbrResetModem = 0; // reset compteur reset modem
  }
  AIntru_HeureActuelle();
  if ((calendrier[month()][day()] ^ FlagCircule) && jour) { // jour circulé
    // 11 jour pour cas lancement de nuit pas d'allumage
    Sbidon = "Jour circule ou demande circulation";
    Serial.println(Sbidon);
    MajLog("Auto", Sbidon);
  }
  envoieGroupeMessage(true,true);  // pasVie,Serveur
  envoieGroupeMessage(true,false); // pasVie,User
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
void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
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
// mise à jour fichier log en SPIFFS
void MajLog(String Id, String Raison) {
  if(SPIFFS.exists(filelog)){
    /* verification de la taille du fichier */
    File f = SPIFFS.open(filelog, "r");
    Serial.print(F("Taille fichier log = ")), Serial.println(f.size());
    // Serial.print(Id),Serial.print(","),Serial.println(Raison);
    if (f.size() > 150000 && !FileLogOnce) {
      /* si trop grand on efface */
      FileLogOnce = true;
      messageId();
      message += F("KO Fichier log presque plein\n");
      message += String(f.size());
      message += F("\nFichier sera efface a 300000");
      if (gsm) {
        sendReply("MQTTU"); // message U
        sendReply("MQTTS"); // message S
      }
    }
    else if (f.size() > 300000 && FileLogOnce) { // 292Ko 75000 lignes
      messageId();
      message += F("KO Fichier log plein\n");
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
        sendReply("MQTTU"); // message U
        sendReply("MQTTS"); // message S
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
  else { // fichier n'existe pas, création fichier avec première ligne date et Id
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
// remplace le calendrier
void EnregistreCalendrier() {

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
// Ouvrir le fichier de calendrier
void OuvrirCalendrier() {

  // this opens the file "f.txt" in read-mode
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
  readFile(SPIFFS, filecalendrier);

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
// Print fichier de config
void PrintConfig() {
  Serial.print("Version = ")                    , Serial.println(ver);
  Serial.print("ID = ")                         , Serial.println(config.Idchar);
  Serial.print("magic = ")                      , Serial.println(config.magic);
  Serial.print("Debut Jour = ")                 , Serial.println(config.DebutJour);
  Serial.print("Fin jour = ")                   , Serial.println(config.FinJour);
  Serial.print("T anticipation Wakeup = ")      , Serial.println(config.anticip);
  Serial.print("Tempo repetition Wake up (s)= "), Serial.println(config.RepeatWakeUp);
  Serial.print("Time Out Wifi (s)= ")           , Serial.println(config.timeoutWifi);
  Serial.print("Tempo ouverture (s) = ")        , Serial.println(config.Tempoouverture);
  Serial.print("Tempo fermeture (s) = ")        , Serial.println(config.Tempofermeture);
  Serial.print("Vitesse SlowBlinker = ")        , Serial.println(config.SlowBlinker);
  Serial.print("PWM Feux Rouge = ")             , Serial.println(config.FRgePWM);
  Serial.print("Type Batterie = ");
  if(config.TypeBatt == 16) Serial.println(F("Pb 12V 6elts"));
  if(config.TypeBatt == 24) Serial.println(F("LiFePO 12.8V 4elts"));
  Serial.print(F("GPRS APN = "))                , Serial.println(config.apn);
  Serial.print(F("GPRS user = "))               , Serial.println(config.gprsUser);
  Serial.print(F("GPRS pass = "))               , Serial.println(config.gprsPass);
  Serial.print(F("ftp serveur = "))             , Serial.println(config.ftpServeur);
  Serial.print(F("ftp port = "))                , Serial.println(config.ftpPort);
  Serial.print(F("ftp user = "))                , Serial.println(config.ftpUser);
  Serial.print(F("ftp pass = "))                , Serial.println(config.ftpPass);
  Serial.print(F("mqtt serveur = "))            , Serial.println(config.mqttServer);
  Serial.print(F("mqtt port = "))               , Serial.println(config.mqttPort);
  Serial.print(F("mqtt username = "))           , Serial.println(config.mqttUserName);
  Serial.print(F("mqtt pass = "))               , Serial.println(config.mqttPass);
  Serial.print(F("sendTopic = "))               , Serial.println(config.sendTopic[0]);
  Serial.print(F("sendTopic = "))               , Serial.println(config.sendTopic[1]);
  Serial.print(F("recvTopic = "))               , Serial.println(config.recvTopic[0]);
  Serial.print(F("recvTopic = "))               , Serial.println(config.recvTopic[1]);
  Serial.print(F("Send SMS autorisation = "))   , Serial.println(config.sendSMS);
  Serial.print(F("declage Heure ete = "))       , Serial.println(config.hete);
  Serial.print(F("declage Heure hiver = "))     , Serial.println(config.hhiver);
  Serial.print(F("autoupload = "))              , Serial.println(config.autoupload);
}
//---------------------------------------------------------------------------
// Connexion Wifi
void ConnexionWifi(char* ssid, char* pwd, String origine) {

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
  sendReply(origine);

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
// Arret du Wifi
void WifiOff() {
  Serial.println("Wifi off");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_MODE_NULL);
  btStop();
  delay(1000);// imperatif
  ResetHard();
}
//---------------------------------------------------------------------------
// Reset hard de ESP32
void ResetHard() {
  // GPIO13 to RS reset hard
  Serial.println("Reset Hard");
  delay(100);// imperatif
  ESP.restart();

  // Reset par PinReset, le reset se fait IO retombent
  // mais ne redemarre plus depuis "derniere" version ESP32 >2?

  pinMode(PinReset, OUTPUT);
  digitalWrite(PinReset, LOW);
  // normalement on n'arrive jamais là
  delay(100);
  ESP.restart();
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
    for (int i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++) { //Read
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
// Enregistrement fichier calibration en SPIFFS
void Recordcalib() {
  // Serial.print(F("Coeff T Batterie = ")),Serial.println(CoeffTension1);
  // Serial.print(F("Coeff T Proc = "))	  ,Serial.println(CoeffTension2);
  // Serial.print(F("Coeff T VUSB = "))		,Serial.println(CoeffTension3);
  File f = SPIFFS.open(filecalibration, "w");
  f.println(CoeffTension[0]);
  f.println(CoeffTension[1]);
  f.println(CoeffTension[2]);
  f.close();
}
//---------------------------------------------------------------------------
// convert heure decimale en hh:mm:ss
String Hdectohhmm(long Hdec) {
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
// convert heure hh:mm:ss en decimale
long Hhmmtohdec(String h) {
  // convert heure hh:mm:ss en decimale
  int H = h.substring(0, 2).toInt();
  int M = h.substring(3, 5).toInt();
  int S = h.substring(6, 8).toInt();
  long hms = H * 3600 + M * 60 + S;
  return hms;
}
//---------------------------------------------------------------------------
// Heure actuelle jour/nuit
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
// lance le mode sleep
void DebutSleep() {

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.print(F("Setup ESP32 to sleep for "));
  print_uint64_t(TIME_TO_SLEEP);
  Serial.print(F("s ;"));
  Serial.println(Hdectohhmm(TIME_TO_SLEEP));
  Serial.flush();

  if (TIME_TO_SLEEP == 1) {
    Serial.println(F("pas de sleep on continue"));
    return;
  }
  //Go to sleep now
  Serial.println(F("Going to sleep now"));

  byte i = 0;
  if (gsm) {
    // mqttClient.disconnect(); // ne pas faire disconnect pour garder session active
    delay(1000);
    while (!modem.poweroff()) { // Power off
      Alarm.delay(100);
      if (i++ > 10) break;
    }
    Serial.print("power off:"),Serial.println(i);
  }
  Serial.flush();
  esp_deep_sleep_start();
  delay(100);

  Serial.println("This will never be printed");
  Serial.flush();

}
//---------------------------------------------------------------------------
// action en fonction du wake up
void action_wakeup_reason(byte wr) {
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
// calcul durée de sleep
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
// Récupere raison du wake up
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
// Efface SMS
void EffaceSMS(int index) {
  bool err;
  byte n = 0;
  do {
    err = modem.deleteSmsMessage(index,0);
    n ++;
    Serial.print(F("resultat del Sms "));	Serial.println(err);
    if (n > 10) { // on efface tous si echec
      err = modem.deleteSmsMessage(index,4);
      Serial.print(F("resultat delall Sms "));	Serial.println(err);
      break;
    }
  } while (!err);
}
//---------------------------------------------------------------------------
// Print variable 64bits
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
//initialisation des tableaux
void init_adc_mm(void) {
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
// lecture des adc
void adc_read() {
  read_adc(PinBattSol, PinBattProc, PinBattUSB); // lecture des adc
}
//---------------------------------------------------------------------------
// lecture adc
void read_adc(int pin1, int pin2, int pin3) {
  // http://www.f4grx.net/algo-comment-calculer-une-moyenne-glissante-sur-un-microcontroleur-a-faibles-ressources/
  static int plus_ancien = 0;
  //acquisition
  int sample[3];
  for (byte i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++) {
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
//---------------------------------------------------------------------------
// preparation message de réponse
void messageId() {
  message  = Id;
  message += displayTime(0);
  message += fl;
}
//---------------------------------------------------------------------------
// page html
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

  webpage += F("<tr>");
  webpage += F("<td>Type Batterie</td>");
  webpage += F("<td>");	
  if(config.TypeBatt == 16) webpage += F("Pb 12V 6elts");
  if(config.TypeBatt == 24) webpage += F("LiFePO 12.8V 4elts");
  webpage += F("</td>");
  webpage += F("</tr>");

  webpage += "<tr>";
  webpage += "<td>Anticipation WakeUp (s)</td>";
  webpage += "<td>";	webpage += String(config.anticip);	webpage += "</td>";
  webpage += "</tr>";

  webpage += "<tr>";
  webpage += "<td>Fin Jour</td>";
  webpage += "<td>";	webpage += Hdectohhmm(config.FinJour);	webpage += "</td>";
  webpage += "</tr>";

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
  webpage += F("<td>");	webpage += String(config.sendTopic[0]);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>send topic</td>");
  webpage += F("<td>");	webpage += String(config.sendTopic[1]);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>recv topic</td>");
  webpage += F("<td>");	webpage += String(config.recvTopic[0]);	webpage += F("</td>");
  webpage += F("</tr>");

  webpage += F("<tr>");
  webpage += F("<td>recv topic</td>");
  webpage += F("<td>");	webpage += String(config.recvTopic[1]);	webpage += F("</td>");
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
// liste PB en html
void Tel_listPage() {
  SendHTML_Header();
  webpage += "<h3 class='rcorners_m'>Liste des num&eacute;ros t&eacute;l&eacute;phone</h3><br>";
  webpage += "<table align='center'>";
  webpage += "<tr>";
  webpage += "<th> Nom </th>";
  webpage += "<th> Num&eacute;ro </th>";
  webpage += "<th> Liste restreinte </th>";
  webpage += "</tr>";

    File file = SPIFFS.open(filePhoneBook, "r");
    while (file.available()) {
      String ligne = file.readStringUntil('\n');
      byte pos1 = ligne.indexOf(";");
      String number   = ligne.substring(0,pos1);
      String name = ligne.substring(pos1+1,ligne.length()-1);
      webpage += F("<tr>");
      webpage += F("<td>"); webpage += String(name); webpage += F("</td>");
      webpage += F("<td>"); webpage += String(number); webpage += F("</td>");
      webpage += F("</tr>");
    }
    file.close();

  webpage += "</table><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop(); // Stop is needed because no content length was sent
}
//---------------------------------------------------------------------------
// Calendrier en html
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
// SPIFS file size
String file_size(int bytes) {
  String fsize = "";
  if (bytes < 1024)                      fsize = String(bytes) + " B";
  else if (bytes < (1024 * 1024))        fsize = String(bytes / 1024.0, 3) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) fsize = String(bytes / 1024.0 / 1024.0, 3) + " MB";
  else                                   fsize = String(bytes / 1024.0 / 1024.0 / 1024.0, 3) + " GB";
  return fsize;
}
//---------------------------------------------------------------------------
// getion temps restant page web
void handleTime() {
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
// getion Date et heure page web
void handleDateTime() {
  char time_str[20];
  sprintf(time_str, "%02d/%02d/%4d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
  server.send(200, "text/plane", String(time_str)); //Send Time value only to client ajax request
}
//---------------------------------------------------------------------------
bool FTP_Connect(){
  char charbidon[100];
  strncpy(charbidon, "+FTPCID=1",12);
  // Sbidon = sendAT(String(charbidon),"OK","ERROR",1000);
  Sbidon = modem.send_AT(charbidon);
  // Lecture FTPSERVEUR si OK on saute parametrage suivant
  Sbidon = modem.send_AT("+FTPSERV?");
  // Serial.print("ftpserveur:"),Serial.println(Sbidon);
  Sbidon = Sbidon.substring(Sbidon.indexOf("\"") + 1,Sbidon.lastIndexOf("\""));
  Serial.print("Serveur deja parametre sur SIM7000:"),Serial.println(Sbidon);

  if(Sbidon != String(config.ftpServeur)){ // ftpserveur not configure
    sprintf(charbidon,"+FTPSERV=\"%s\"", config.ftpServeur);
    // Serial.println(charbidon);
    // Sbidon = sendAT(String(charbidon),"OK","ERROR",1000);
    Sbidon = modem.send_AT(charbidon);
    Serial.print("FTP serveur :"), Serial.println(Sbidon);

    sprintf(charbidon, "+FTPPORT=%i", config.ftpPort);
    // Sbidon = sendAT(String(charbidon),"OK","ERROR",1000);
    Sbidon = modem.send_AT(charbidon);
    Serial.print("FTP port :"), Serial.println(Sbidon);

    sprintf(charbidon, "+FTPUN=\"%s\"", config.ftpUser);
    // Sbidon = sendAT(String(charbidon),"OK","ERROR",1000);
    Sbidon = modem.send_AT(charbidon);
    Serial.print("FTP user :"), Serial.println(Sbidon);

    sprintf(charbidon, "+FTPPW=\"%s\"", config.ftpPass);
    // modem.sendAT(String(charbidon));
    // Sbidon = sendAT(String(charbidon),"OK","ERROR",10000);
    Serial.print("FTP pass :"), Serial.println(modem.send_AT(String(charbidon)));
  }
  Serial.println("A finir gestion erreur");
  return true;
}
//---------------------------------------------------------------------------
  bool FTP_Quit() {
    Serial.println(modem.send_AT(F("+FTPQUIT")));
    // Serial.println("A finir gestion erreur");
  return true;
}
//---------------------------------------------------------------------------
// FTP upload file
bool FTP_upload_function (char *file2upload){
  // https://github.com/OscarVanL/SIM7000-LTE-Shield/blob/master/Code/Adafruit_FONA.cpp#L1944
  // FTP ne marche pas si MQTT actif?

  Serial.print("file to upload:"),Serial.println(file2upload);
  if(strcmp(file2upload,filecalibration) == 0){ // seulement pour filecalibration pour le moment
    if(!FTP_Connect()){
      return false;
    }
    delay(1000);

    // Upload du fichier
    char charbidon[100];
    char path[50];
    // destination chemin et filename
    sprintf(path,"/%s/",Id);
    sprintf(charbidon, "+FTPPUTPATH=\"%s\"", path);
    Serial.println(charbidon);
    Serial.println(modem.send_AT(String(charbidon)));
    // Serial.print("FTP put path fichier :"), Serial.println(modem.waitResponse("OK","ERROR"));

    sprintf(charbidon, "+FTPPUTNAME=\"%s\"", "coeff.txt");
    Serial.println(charbidon);
    Serial.println(modem.send_AT(String(charbidon)));
    // Serial.print("FTP put name fichier :"), Serial.println(modem.waitResponse("OK","ERROR"));

    // Ouvrir FTP
    int maxlength = modem.setFTPUpload();
    if(maxlength == -1){
      Serial.println("FTP erreur");
      return false;
    }
    Serial.print("maxlength:"),Serial.println(maxlength);

    // envoyer data
    delay(500);
    // pour test
    // int CoeffTension[4];          // Coeff calibration Tension
    // char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
    // if (SPIFFS.exists(filecalibration)) {
    //   File f = SPIFFS.open(filecalibration, "r");
    //   for (int i = 0; i < (sizeof(CoeffTension) / sizeof(CoeffTension[0])); i++) { //Read
    //     String s = f.readStringUntil('\n');
    //     CoeffTension[i] = s.toFloat();
    //   }
    //   f.close();
    // }
    Serial.print(F("Coeff T Batterie = ")), Serial.print(CoeffTension[0]);
    Serial.print(F(" Coeff T Proc = "))	  , Serial.print(CoeffTension[1]);
    Serial.print(F(" Coeff T VUSB = "))		, Serial.print(CoeffTension[2]);
    Serial.print(F(" Coeff T 24V = "))		, Serial.println(CoeffTension[3]);

    char data[1000];
    for(int i=0;i<4;i++){
      strcat(data,String(CoeffTension[i]).c_str());
      strcat(data,"\n");
    }
    Serial.print("data:"),Serial.println(data);
    int length=strlen(data);
    Serial.print("len:"),Serial.println(length);
    if(length<=maxlength){
      sprintf(charbidon, "+FTPPUT=2,%d",length);
      Serial.print("ATcde:"),Serial.println(charbidon);
      Serial.println(modem.send_AT(charbidon));
      SerialAT.println(data);
    }
    // Fermer FTP
    Serial.println(modem.send_AT("+FTPPUT=2,0"));

    FTP_Quit();
  } else {
    Serial.println("pas supporte pour ce fichier");
    return false;
  }
  return true;
}
//---------------------------------------------------------------------------
// envoyer commande AT au modem
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
  generationMessage();
  sendReply("MQTTS"); // envoie serveur
  if(Memo_Demande_CVR[1] == "MQTTU"){ // reponse User
    sendReply("MQTTU");
  } else if (Memo_Demande_CVR[1] == "Local"){ // reponse Local
    sendReply("Local");
  }
}
//---------------------------------------------------------------------------
// etat synchronisation time/heure systeme
void timesstatus() {
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
// Connexion Gprs
bool ConnectGPRS(){
  Serial.print(F("Connecting to "));
  Serial.print(config.apn);
  if (modem.gprsConnect(config.apn, config.gprsUser, config.gprsPass)) {
    Serial.println(F(" success"));
    return true;
  }
  else {
    Serial.println(" fail");
    return false;
  }
}
//---------------------------------------------------------------------------
// Connexion MQTT, Clean session false
bool mqttConnect() {
  // if (modem.isGprsConnected()) {
    // Connect to the MQTT broker.
    Serial.print("Attempting MQTT connection...");
    if ( mqttClient.connect(config.Idchar, config.mqttUserName, config.mqttPass,willTopic,1,true,config.Idchar,false)) {
      Serial.println( "Connected with Client ID:  " + String(config.Idchar) + " User " + String(config.mqttUserName) + " Pwd " + String(config.mqttPass));
      AlarmeMQTT = false;
      return true;
    } else {
      Serial.print( "failed, rc = " );
      // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
      Serial.print( mqttClient.state() );
      Serial.println( " Will try again in 5 seconds" );
      AlarmeMQTT = true;
      return false;
    }
  // }
}
//---------------------------------------------------------------------------
// Subscription MQTT, qos=1
bool mqttSubscribe(bool unsubSub) {
  byte rep = 1;
  // unsubSub = 0 subscribe, = 1 unsubscribe
  if (unsubSub == 0) {
    for(byte i = 0; i<2;i++){
      if (mqttClient.subscribe( config.recvTopic[i] , 1 )){ // Subscribe
        rep *= rep;
      } else {
        rep *= 0;
      }      
      delay(200);
    }
    return rep;
  } else {
    for(byte i = 0; i<2;i++){
      if (mqttClient.unsubscribe(config.recvTopic[i])){ // Unsubscribe
        rep *= rep;
      } else {
        rep *= 0;
      }
      delay(200);
    }
    return rep;
  }
}
//---------------------------------------------------------------------------
// Cherche number existe dans fichier PhoneBook
bool Cherche_N_PB(String number){
  if (SPIFFS.exists(filePhoneBook)) {
    File file = SPIFFS.open(filePhoneBook, "r");
    while (file.available()) {
      String s = file.readStringUntil('\n');
      if(s.indexOf(number)>-1){
        Serial.print("N° trouve:"),Serial.println(s);
        file.close();
        return true;
      }
    }
    file.close();
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
    } while(rep != 1);
    return true;
  }
  return false;
}
//---------------------------------------------------------------------------
void message_Monitoring_Reseau(){
  message += F("Histo Network Chgt : ");
  message += fl;
  message += F("no service : ");
  message += String(Histo_Reseau[0]);
  message += fl;
  message += F("GSM : ");
  message += String(Histo_Reseau[1]);
  message += fl;
  message += F("EGPRS : ");
  message += String(Histo_Reseau[2]);
  message += fl;
  message += F("LTE-M1 : ");
  message += String(Histo_Reseau[3]);
  message += fl;
  message += F("LTE-NB : ");
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
  Serial.println(modemHDtate);
}
//---------------------------------------------------------------------------
// Set systeme time to modem time
void set_system_time(){
  readmodemtime();
  if(N_Y > 20 && N_Y!= 80){
    // année > 2020 et n'est pas égale à 2080(date defaut modem si pas à l'heure)
    setTime(N_H,N_m, N_S, N_D, N_M, N_Y);	    // mise à l'heure du systeme
    Serial.println("MajHeure du systeme OK");
    FlagSetTime = true;
  } else {
    // echec Synchro, force date 01/08/2022 08:00:00, jour toujours circulé
    setTime(8,0, 0, 1, 8, 22);	    // mise à l'heure du systeme
    // modem.send_AT("+CCLK=\"22/08/01,08:00:00+08\"");
    Serial.println("MajHeure du systeme KO");
    FlagSetTime = false;
  }
}
//---------------------------------------------------------------------------
// start true -> Start Alarmes
// start false-> Stop Alarmes
void StartStopAlarme(bool start){
  if (start){
    Alarm.enable(loopPrincipale);
    Alarm.enable(DebutJour);
    Alarm.enable(FinJour);
  } else {
    Alarm.disable(loopPrincipale);
    Alarm.disable(DebutJour);
    Alarm.disable(FinJour);
  }
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
  if(n==0){     network = F("no service");}
  else if(n==1){network = F("GSM");}
  else if(n==3){network = F("EGPRS");}
  else if(n==7){network = F("LTE-M1");}
  else if(n==9){network = F("LTE-NB");}
  return network;
}
//---------------------------------------------------------------------------
/* Enregistre mode cnx Reseau
  Histo-Reseau(0) = no service, (1)=GSM, (3)=EGPRS, (7)= LTE-M1, (9)= LTE-NB
  cummul à chaque heure */
void Monitoring_Reseau(){
  int n = modem.getNetworkCurrentMode();
  if(n==0){     Histo_Reseau[0]  ++ ;}
  else if(n==1){Histo_Reseau[1]  ++;}
  else if(n==3){Histo_Reseau[2]  ++;}
  else if(n==7){Histo_Reseau[3]  ++;}
  else if(n==9){Histo_Reseau[4]  ++;}
}
//---------------------------------------------------------------------------
// Reception message subscription
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
  // variable stockage temporaire
  static char temptopic[12];
  static String tempmessage = "";

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
  Serial.println();

  /* si len>0, flagRcvMQTT = true, le traitement des commandes bloquantes
    NONCIRCULE, "Wifi,SSID,PW"
    ne seront pas executées,
    ne le seront qu'au retour de flagRcvMQTT = false
    garantie que le message à bien été effacé du serveur
    cela évitera un bouclage sur ce message
    ATTENTION
    entre temps les variables Rmessage et Origine ne doivent pas etres altérées!
    */
  if(Sbidon.length() > 0){
    flagRcvMQTT = true;
    // sauvegarde topic et message
    tempmessage = Sbidon;
    // temptopic   = String(topic);
    strcpy(temptopic,topic);
    Rmessage    = Sbidon;
    // on efface le topic sur le serveur
    if(strcmp(topic,config.recvTopic[0]) == 0){ // Serveur
      Serial.println(mqttClient.publish(config.recvTopic[0],"")); // efface topic sur serveur
    } else if(strcmp(topic,config.recvTopic[1]) == 0){ // User
      Serial.println(mqttClient.publish(config.recvTopic[1],"")); // efface topic sur serveur    
    }
    Serial.println(Rmessage);
    if(flagRcvMQTT){
      // message bloquant sera traité apres retour message len=0.
      Serial.println("message sera traite apres reception message len=0");
      return;
    } else {
      flagRcvMQTT = false;
      Serial.println("message traite maintenant");
      // message non bloquant, on traite de suite
      if(strcmp(temptopic,config.recvTopic[0]) == 0){ // Serveur
        Serial.println("message from serveur");
        traite_sms("MQTTS");
      } else if(strcmp(temptopic,config.recvTopic[1]) == 0){ // User
        Serial.println("message from user");
        traite_sms("MQTTU");
      }
    }
  } else if(Sbidon.length() == 0){
    Serial.println("len = 0");
    if (flagRcvMQTT){
      flagRcvMQTT = false;
      Rmessage = tempmessage;
      // on traite maintenant
      Serial.println("on traite maintenant");
      if(strcmp(temptopic,config.recvTopic[0]) == 0){ // Serveur
        traite_sms("MQTTS");
      } else if(strcmp(temptopic,config.recvTopic[1]) == 0){ // User
        traite_sms("MQTTU");
      }
    }
  }
}
//---------------------------------------------------------------------------
// lire valeur RSSI et remplir message
String read_RSSI() {
  if(gsm){
    String rssi = "";
    int r;
    byte n = modem.getSignalQuality();
    // Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(F(": "));
    if (n == 0) r = -115;
    if (n == 1) r = -111;
    if (n == 31) r = -52;
    if ((n >= 2) && (n <= 30)) {
      r = map(n, 2, 30, -110, -54);
    }
    rssi  = F("RSSI= ");
    rssi += String(n);
    rssi += ", ";
    rssi += String(r);
    rssi += F("dBm");
    return rssi;
  }
}
//---------------------------------------------------------------------------
// Allumage modem
int modem_on() {
    /*
    The indicator light of the board can be controlled
    */
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    /*
    MODEM_PWRKEY IO:18 The power-on signal of the modulator must be given to it,
    otherwise the modulator will not reply when the command is sent
    */
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(300);
    digitalWrite(MODEM_PWRKEY, HIGH);

    /*
    MODEM_FLIGHT IO:25 Modulator flight mode control,
    need to enable modulator, this pin must be set to high
    */
    // pinMode(MODEM_FLIGHT, OUTPUT);
    // digitalWrite(MODEM_FLIGHT, HIGH);

  int i = 10;
  Serial.println("Testing Modem Response...");
  Serial.println("****");
  while (i) {
    SerialAT.println("AT");
    delay(500);
    if (SerialAT.available()) {
      String r = SerialAT.readString();
      Serial.println(r);
      if ( r.indexOf("OK") >= 0 ) {
        // reply = true;
        break;
      }
    }
    delay(500);
    i--;
  }
  Serial.println("****");
  return i;
}
//---------------------------------------------------------------------------
// retourne n° derniere ligne PhoneBook
byte last_PB(){
  Read_PB();
  byte dernier = 0;
  for(byte i = 1;i<10;i++){
    if(strlen(PB_list[i]) > 0){
      dernier = i;
    }
  }
  return dernier;
}
//---------------------------------------------------------------------------
// Vérification fichier PhoneBook existe
void Ouvrir_PB() {
  // par ligne : N°tel;Nom\n
  if (!SPIFFS.exists(filePhoneBook)) {
    // fichier n'existe pas
    Serial.print(F("Creating Data File:")), Serial.println(filePhoneBook); // valeur par defaut
    File file = SPIFFS.open(filePhoneBook, "w+");
    file.println(default_PB);
    file.close();
  }
  Read_PB();
}
//---------------------------------------------------------------------------
// Lecture filePhoneBook copie dans PB_list
void Read_PB(){
  // vide PB_list
  for(byte i = 1;i<10;i++){
    strcpy(PB_list[i] , "");
  }
  // lire fichier
  File file = SPIFFS.open(filePhoneBook, "r");
  byte idx = 0;
  while (file.available()) {
    idx ++;
    String ligne = file.readStringUntil('\n');
    strcpy(PB_list[idx] , ligne.c_str());
  }
  file.close();
}
//---------------------------------------------------------------------------
// Sauvegarde filePhoneBook
void Save_PB(){
  File file = SPIFFS.open(filePhoneBook, "w+");
  for (byte i = 1;i<10;i++){
    if(strlen(PB_list[i])>0){
      file.println(String(PB_list[i]));
    }
  }
  file.close();
}
//---------------------------------------------------------------------------
// Copie valeur topic en config
void copie_Topic(){
  strncpy(config.sendTopic[0],("S/in"),sizeof(config.sendTopic[0]));
  strncpy(config.sendTopic[1],("S/Uin/"),sizeof(config.sendTopic[1]));
  strcat( config.sendTopic[1], &config.Idchar[5]); // S/Uin/CVxx

  strncpy(config.recvTopic[0],("S/Sout/"),sizeof(config.recvTopic[0]));
  strcat( config.recvTopic[0], &config.Idchar[5]); // S/Sout/CVxx
  
  strncpy(config.recvTopic[1],("S/Uout/"),sizeof(config.recvTopic[1]));
  strcat( config.recvTopic[1], &config.Idchar[5]); // S/Uout/CVxx
}
//---------------------------------------------------------------------------
byte gprs_upload_function (){
  // https://forum.arduino.cc/index.php?topic=376911.15
  int buffer_space = 1000;
  UploadFile = SPIFFS.open(filelog, "r");
  byte reply = 1;
  int i = 0;
  // ne fonctionne pas dans tous les cas ex roaming
  // while (i < 10 && reply == 1){ //Try 10 times...
    // reply = sendATcommand("AT+CREG?","+CREG: 0,1","ERROR", 1000);
    // i++;
    // delay(1000);
  // }
  reply = 0;
if (reply == 0){ // ouverture GPRS
// reply = sendATcommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"","OK","ERROR", 1000);
if (reply == 0){
// reply = sendATcommand("AT+SAPBR=3,1,\"APN\",\"sl2sfr\"", "OK", "ERROR", 1000);//Replace with your APN
if (reply == 0){
//reply = sendATcommand("AT+SAPBR=3,1,\"USER\",\"entelpcs\"", "OK", "ERROR", 1000);
if (reply == 0){
//reply = sendATcommand("AT+SAPBR=3,1,\"PWD\",\"entelpcs\"", "OK", "ERROR", 1000);
if (reply == 0){
reply = 2;
i = 0;
while (i < 3 && reply == 2){ //Try 3 times...
  reply = sendATcommand("AT+SAPBR=1,1", "OK", "ERROR", 10000);
  if (reply == 2){
    sendATcommand("AT+SAPBR=0,1", "OK", "ERROR", 10000);
  }
  i++;
}
if (reply == 0){
reply = sendATcommand("AT+SAPBR=2,1", "OK", "ERROR", 1000);
if (reply == 0){
reply = sendATcommand("AT+FTPCID=1", "OK", "ERROR", 1000);
if (reply == 0){
reply = sendATcommand("AT+FTPSERV=\"" + String(config.ftpServeur) + "\"", "OK", "ERROR", 1000);//replace ftp.sample.com with your server address
if (reply == 0){
reply = sendATcommand("AT+FTPPORT="+ String(config.ftpPort), "OK", "ERROR", 1000);
if (reply == 0){
reply = sendATcommand("AT+FTPUN=\"" + String(config.ftpUser) + "\"", "OK", "ERROR", 1000);//Replace 1234@sample.com with your username
if (reply == 0){
reply = sendATcommand("AT+FTPPW=\"" + String(config.ftpPass) + "\"", "OK", "ERROR", 1000);//Replace 12345 with your password
if (reply == 0){
reply = sendATcommand("AT+FTPPUTNAME=\"" + String(filelog) + "\"", "OK", "ERROR", 1000);
if (reply == 0){
  reply = sendATcommand("AT+FTPPUTPATH=\"/" + String(config.Idchar) + "/\"", "OK", "ERROR", 1000);// repertoire "/Id/"
if (reply == 0){
  unsigned int ptime = millis();
  reply = sendATcommand("AT+FTPPUT=1", "+FTPPUT: 1,1", "+FTPPUT: 1,6", 60000);
  Serial.println("Time: " + String(millis() - ptime));
  if (reply == 0){
    if (UploadFile) {
      int i = 0;
      unsigned int ptime = millis();
      long archivosize = UploadFile.size();
      while (UploadFile.available()) {
        while(archivosize >= buffer_space){
          reply = sendATcommand("AT+FTPPUT=2," + String(buffer_space), "+FTPPUT: 2,1", "OK", 3000);
            if (reply == 0) { //This loop checks for positive reply to upload bytes and in case or error it retries to upload
              Serial.println("Remaining Characters: " + String(UploadFile.available()));
              for(int d = 0; d < buffer_space; d++){
                Serial2.write(UploadFile.read());
                archivosize -= 1;
              }
            }
            else {
              Serial.println("Error while sending data:");
              reply = 1;
            }
        }
        if (sendATcommand("AT+FTPPUT=2," + String(archivosize), "+FTPPUT: 2," + String(archivosize), "ERROR", 10000) == 0) {
          for(int t = 0; t < archivosize; t++){
            Serial2.write(UploadFile.read());
          }
        }
      }
    UploadFile.close();
    Serial.println("Time: " + String(millis() - ptime));
    }
  }
}
}
}
}
}
}
}
}
}
}
}
}
}
}
  sendATcommand("AT+SAPBR=0,1", "OK", "ERROR", 10000); // fermeture GPRS
return reply;
}
//---------------------------------------------------------------------------
byte sendATcommand(String ATcommand, String answer1, String answer2, unsigned int timeout){
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
    //Stop reading conditions
    if (content.indexOf(answer1) != -1){
      reply = 0;
    }else if(content.indexOf(answer2) != -1){
      reply = 2;
    }else{
      //Nothing to do...
    }
  }
  return reply;
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
    Rmessage = serialmessage;
    Serial.flush();
    traite_sms("Local");//	traitement en mode local
    newData = false;
    serialmessage = "";
  }
}
//---------------------------------------------------------------------------

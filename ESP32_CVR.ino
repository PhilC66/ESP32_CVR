/* Ph CORBEL 10/2020
  Gestion Carré VR
  (basé sur ESP32_Signalisation

  todo
  Ajouter commande Allumage/extinction Feu Rouge independante
  
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

#include <Battpct.h>
#include <Sim800l.h>              //my SIM800 modifié
#include <Time.h>
#include <TimeAlarms.h>
#include <sys/time.h>             //<sys/time.h>
#include <WiFi.h>
#include <EEPROM.h>               // variable en EEPROM(SPIFFS)
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

String  webpage = "";
#define ServerVersion "1.0"
bool    SPIFFS_present = false;
#include "CSS.h"               // pageweb

// #define RESET_PIN     18   // declaré par Sim800l.h
// #define LED_PIN        5   // declaré par Sim800l.h
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

#define SIMPIN        1234 // Code PIN carte SIM

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

#define nSample (1<<4)    // nSample est une puissance de 2, ici 16 (4bits)
unsigned int adc_hist[5][nSample]; // tableau stockage mesure adc, 0 Batt, 1 Proc, 2 USB, 3 24V, 5 Lum
unsigned int adc_mm[5];            // stockage pour la moyenne mobile

uint64_t TIME_TO_SLEEP  = 15;/* Time ESP32 will go to sleep (in seconds) */
unsigned long debut     = 0; // pour decompteur temps wifi
byte calendrier[13][32]; // tableau calendrier ligne 0 et jour 0 non utilisé, 12*31
char filecalendrier[13]  = "/filecal.csv";  // fichier en SPIFFS contenant le calendrier de circulation
char filecalibration[11] = "/coeff.txt";    // fichier en SPIFFS contenant les data de calibration
char filelog[9]          = "/log.txt";      // fichier en SPIFFS contenant le logé

const String soft = "ESP32_CVR.ino.d32"; // nom du soft
String ver        = "V1-3";
int    Magique    = 4;
const String Mois[13] = {"", "Janvier", "Fevrier", "Mars", "Avril", "Mai", "Juin", "Juillet", "Aout", "Septembre", "Octobre", "Novembre", "Decembre"};
String Sbidon 		= ""; // String texte temporaire
String message;
String bufferrcpt;
String fl = "\n";                   //  saut de ligne SMS
String Id ;                         //  Id du materiel sera lu dans EEPROM
char   SIM800InBuffer[64];          //  for notifications from the SIM800
char   replybuffer[270];            //  Buffer de reponse SIM800, historique 255, 270 message long de free
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
String Memo_Demande_CVR[3] = ""; // 0 num demandeur,1 nom, 2 CVR demandé (O2,F1)
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

WebServer server(80);
File UploadFile;

typedef struct               // declaration structure  pour les log
{
  char    dt[10];            // DateTime 0610-1702 9+1
  char    Act[2];            // Action A/D/S/s 1+1
  char    Name[15];          // 14 car
} champ;
champ record[5];

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
} ;
config_t config;

Ticker SlowBlink;          // Clignotant lent
Ticker step;               // pas modulation pwm feux
Ticker ADC;                // Lecture des Adc

AlarmId loopPrincipale;    // boucle principale
AlarmId DebutJour;         // Debut journée
AlarmId FinJour;           // Fin de journée retour deep sleep
AlarmId Aouverture;        // Tempo ouverture verin
AlarmId Afermeture;        // Tempo fermeture verin

HardwareSerial *SIM800Serial = &Serial2; // liaison serie FONA SIM800
Sim800l Sim800;                          // to declare the library

//---------------------------------------------------------------------------

void setup() {

  message.reserve(140);

  Serial.begin(115200);
  Serial.println();

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
    Serial.println("lancement SIM800");
    SIM800Serial->begin(9600); // 4800
    Sim800.begin();
  }
  // parametrage PWM pour les feux
  // https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
  ledcSetup(RgePwmChanel, 1000, 8);
  ledcAttachPin(PinFeuxR, RgePwmChanel);
  ledcWrite(RgePwmChanel, 0); // Feux Rouge 0

  init_adc_mm();// initialisation tableau pour adc Moyenne Mobile
  ADC.attach_ms(100, adc_read); // lecture des adc toute les 100ms
  /* Lecture configuration en EEPROM	 */
  EEPROM.begin(512);

  EEPROM.get(confign, config); // lecture config
  recordn = sizeof(config);
  Serial.print("len config ="), Serial.println(sizeof(config));
  EEPROM.get(recordn, record); // Lecture des log
  Alarm.delay(500);
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
    config.TypeBatt       = 16; // Pb par défaut
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

    EEPROM.put(confign, config);
    EEPROM.commit();
    // valeur par defaut des record (log)
    for (int i = 0; i < 5 ; i++) {
      temp = "";
      temp.toCharArray(record[i].dt, 10);
      temp.toCharArray(record[i].Act, 2);
      temp.toCharArray(record[i].Name, 15);
    }
    EEPROM.put(recordn, record);// ecriture des valeurs par defaut
    EEPROM.commit();
  }
  EEPROM.end();
  PrintEEPROM();
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

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialisation failed...");
    SPIFFS_present = false;
  }
  else {
    Serial.println("SPIFFS initialised... file access enabled...");
    SPIFFS_present = true;
  }

  OuvrirCalendrier();					// ouvre calendrier circulation en SPIFFS
  OuvrirFichierCalibration(); // ouvre fichier calibration en SPIFFS
  if (gsm) {
    Sim800.reset(SIMPIN);					// lancer SIM800
    MajHeure("");
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
}
//---------------------------------------------------------------------------
void loop() {
  recvOneChar(); // lecture port serie

  char* bufPtr = SIM800InBuffer;	//buffer pointer
  if (Serial2.available()) {      	//any data available from the FONA?
    int charCount = 0;
    /* Read the notification into SIM800InBuffer */
    do  {
      *bufPtr = Serial2.read();
      bufferrcpt += *bufPtr;
      Serial.write(*bufPtr);
      delay(1);// Alarm.delay(1);
    } while ((*bufPtr++ != '\n') && (Serial2.available()) && (++charCount < (sizeof(SIM800InBuffer) - 1)));
    /* Add a terminal NULL to the notification string */
    *bufPtr = 0;
    if (charCount > 1) {
      // Serial.print(F("Buffer ="));
      Serial.println(bufferrcpt);
    }
    if ((bufferrcpt.indexOf("RING")) > -1) {	// RING, Ca sonne
      Sim800.hangoffCall();									// on raccroche
    }
    /* Scan the notification string for an SMS received notification.
      If it's an SMS message, we'll get the slot number in 'slot' */
    if (1 == sscanf(SIM800InBuffer, "+CMTI: \"SM\",%d", &slot)) {
      Serial.println("sms recu lance traitement");
      traite_sms(slot);
    }
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
  if (gsm) {
    if (!Sim800.getetatSIM())Sim800.reset(SIMPIN); // verification SIM
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
    /* verification nombre SMS en attente(raté en lecture directe)
       traitement des sms en memoire un par un,
       pas de traitement en serie par commande 51, traitement beaucoup trop long */
    nsms = Sim800.getNumSms(); // nombre de SMS en attente (1s)
    Serial.print("Sms en attente = "), Serial.println(nsms);

    if (nsms > 0) {	// nombre de SMS en attente
      // il faut les traiter
      int numsms = Sim800.getIndexSms(); // cherche l'index des sms en mémoire
      Serial.print("Numero Sms en attente = "), Serial.println(numsms);
      if (numsms > 10) {
        // grand nombre sms en memoire, trop long a traiter
        Serial.print("num sms > 10, efface tout :"), Serial.println(numsms);
        MajLog("Auto", "numsms > 10 efface tous sms");// renseigne log
        Sim800.delAllSms();
      }
      traite_sms(numsms);// traitement des SMS en attente
    }
    else if (nsms == 0 && FlagReset) { // on verifie que tous les SMS sont traités avant Reset
      FlagReset = false;
      ResetHard();				//	reset hard
    }
  }
  else if (FlagReset) {
    FlagReset = false;
    ResetHard();				//	reset hard
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
  /* il y a 50 slots dispo
  	si slot=51, demande de balayer tous les slots pour verification (pas utilisé trop long)
  	si slot=99, demande depuis liaison serie en test, traiter sans envoyer de sms
  */

  char number[13];													// numero expediteur SMS
  String textesms;													// texte du SMS reçu
  textesms.reserve(270); // historique 140, 270 pour message long de free
  String numero;
  String nom;
  bool smsserveur = false; // true si le sms provient du serveur index=1

  byte i;
  byte j;
  bool sms = true;

  /* Variables pour mode calibration */
  static int tensionmemo = 0;//	memorisation tension batterie lors de la calibration
  int coef = 0; // coeff temporaire
  static byte P = 0; // Pin entrée a utiliser pour calibration
  static byte M = 0; // Mode calibration 1,2,3,4
  static bool FlagCalibration = false;	// Calibration Tension en cours

  if (slot == 99) sms = false;
  if (slot == 51) { // demande de traitement des SMS en attente
    i = 1;
    j = 50;
  }
  else {
    i = slot;
    j = slot;
  }
  for (byte k = i; k <= j; k++) {
    slot = k;
    // /* Retrieve SMS sender address/phone number. */
    if (sms) {
      numero = Sim800.getNumberSms(slot); // recupere le Numero appelant
      nom = Sim800.getNameSms(slot);      // recupere le nom appelant
      textesms = Sim800.readSms(slot);    // recupere le contenu
      textesms = ExtraireSms(textesms);
      if (Sim800.getNumberSms(slot) == Sim800.getPhoneBookNumber(1)) {
        smsserveur = true; // si sms provient du serveur index=1
      }
      if (nom.length() < 1) { // si nom vide, cherche si numero est num de la SIM
        if (numero == Sim800.getNumTel()) {
          nom = "Moi meme";
        }
      }
      Serial.print("Nom appelant = "), Serial.println(nom);
      Serial.print("Numero = "), Serial.println(numero);
      byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
      if(numero.length() < 8 || numero.length() > 20 
        || numero == "Free Mobile" || numero.indexOf("Free") > -1){ // numero service free numero court et long(29) ou "Free Mobile"
        for (byte Index = 1; Index < n + 1; Index++) { // Balayage des Num Tel dans Phone Book
          if (config.Pos_Pn_PB[Index] == 1) { // Num dans liste restreinte
            String number = Sim800.getPhoneBookNumber(Index);
            char num[13];
            number.toCharArray(num, 13);
            message = textesms;
            EnvoyerSms(num, true);
            EffaceSMS(slot);
            return; // sortir de la procedure traite_sms
          }
        }
      }
    }
    else {
      textesms = String(replybuffer);
      nom = "console";
    }

    if (!(textesms.indexOf("TEL") == 0 || textesms.indexOf("tel") == 0 || textesms.indexOf("Tel") == 0
          || textesms.indexOf("Wifi") == 0 || textesms.indexOf("WIFI") == 0 || textesms.indexOf("wifi") == 0
          || textesms.indexOf("GPRSDATA") > -1 || textesms.indexOf("FTPDATA") > -1 || textesms.indexOf("FTPSERVEUR") > -1)) {
      textesms.toUpperCase();	// passe tout en Maj sauf si "TEL" ou "WIFI" parametres pouvant contenir minuscules
      // textesms.trim();
    }
    textesms.replace(" ", "");// supp tous les espaces
    Serial.print("textesms  = "), Serial.println(textesms);

    if ((sms && nom.length() > 0) || !sms) {        // si nom appelant existant dans phone book
      numero.toCharArray(number, numero.length() + 1); // on recupere le numéro
      messageId();
      if (textesms.indexOf("TIMEOUTWIFI") > -1) { // Parametre Arret Wifi
        if (textesms.indexOf(char(61)) == 11) {
          int n = textesms.substring(12, textesms.length()).toInt();
          if (n > 9 && n < 3601) {
            config.timeoutWifi = n;
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        message += "TimeOut Wifi (s) = ";
        message += config.timeoutWifi;
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("WIFIOFF") > -1) { // Arret Wifi
        message += "Wifi off";
        message += fl;
        EnvoyerSms(number, sms);
        WifiOff();
      }
      else if (textesms.indexOf("Wifi") == 0) { // demande connexion Wifi
        byte pos1 = textesms.indexOf(char(44));//","
        byte pos2 = textesms.indexOf(char(44), pos1 + 1);
        String ssids = textesms.substring(pos1 + 1, pos2);
        String pwds  = textesms.substring(pos2 + 1, textesms.length());
        char ssid[20];
        char pwd[20];
        ssids.toCharArray(ssid, ssids.length() + 1);
        ssids.toCharArray(ssid, ssids.length() + 1);
        pwds.toCharArray(pwd, pwds.length() + 1);
        ConnexionWifi(ssid, pwd, number, sms); // message généré par routine
      }
      else if (gsm && (textesms.indexOf("TEL") == 0
                       || textesms.indexOf("Tel") == 0
                       || textesms.indexOf("tel") == 0)) { // entrer nouveau num
        bool FlagOK = true;
        byte j = 0;
        String Send	= "AT+CPBW=";// message ecriture dans le phone book
        if (textesms.indexOf(char(61)) == 4) { // TELn= reserver correction/suppression
          int i = textesms.substring(3).toInt();// recupere n° de ligne
          i = i / 1; // important sinon i ne prend pas sa valeur dans les comparaison?
          //Serial.println(i);
          if (i < 1) FlagOK = false;
          Send += i;
          j = 5;
          // on efface la ligne sauf la 1 pour toujours garder au moins un numéro
          if ( (i != 1) && (textesms.indexOf("efface") == 5
                            || textesms.indexOf("EFFACE") == 5 )) goto fin_tel;
        }
        else if (textesms.indexOf(char(61)) == 3) { // TEL= nouveau numero
          j = 4;
        }
        else {
          FlagOK = false;
        }
        if (textesms.indexOf("+") == j) {			// debut du num tel +
          if (textesms.indexOf(char(44)) == j + 12) {	// verif si longuer ok
            String numero = textesms.substring(j, j + 12);
            String nom = textesms.substring(j + 13, j + 27);// pas de verif si long<>0?
            Send += ",\"";
            Send += numero;
            Send += "\",145,\"";
            Send += nom;
            Send += "\"";
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
          message += "Commande non reconnue ?";// non reconnu
          message += fl;
          EnvoyerSms(number, sms);					// SMS non reconnu
        }
        else {
          Serial.println(Send);
          if (gsm) {
            Sim800.WritePhoneBook(Send);					//ecriture dans PhoneBook
            Alarm.delay(500);
            Sim800.ModeText(); //pour purger buffer fona
            Alarm.delay(500);
          }
          messageId();
          message += "Nouveau Num Tel: ";
          message += "OK";
          message += fl;
          EnvoyerSms(number, sms);
        }
      }
      else if (gsm && (textesms == "LST?" || textesms == "LST1")) {	//	Liste des Num Tel
        byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
        for (byte i = 1; i < n + 1; i++) {
          String num = Sim800.getPhoneBookNumber(i);
          // Serial.print(num.length()), Serial.print(" "), Serial.println(num);
          if (num.indexOf("+CPBR:") == 0) { // si existe pas sortir
            Serial.println("Failed!");// next i
            goto fin_i;
          }
          String name = Sim800.getPhoneBookName(i);
          // Serial.println(name);
          message += String(i) + ":";
          message += num;
          message += "," + fl;
          message += name;
          message += fl;
          Serial.println(message);
          if ((i % 3) == 0) {
            EnvoyerSms(number, sms);
            messageId();
          }
        }
fin_i:
        if (message.length() > Id.length() + 20) EnvoyerSms(number, sms);; // SMS final
      }
      else if (textesms.indexOf("ETAT") == 0 || textesms.indexOf("ST") == 0) {// "ETAT? de l'installation"
        generationMessage(0);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("SYS") > -1) {
        if (gsm) {
          Sim800.getetatSIM();// 1s
          byte n = Sim800.getNetworkStatus();// 1.1s
          String Op = Sim800.getNetworkName();// 1.05s
          if (n == 5) {
            message += ("rmg, ");// roaming 1.0s
          }
          message += Op + fl;
          read_RSSI();
          int Vbat = Sim800.BattVoltage();
          byte Batp = Sim800.BattPct();
          message += "Batt GSM : ";
          message += Vbat;
          message += " mV, ";
          message += Batp;
          message += " %";
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("ID=") == 0) {			//	Id= nouvel Id
        String temp = textesms.substring(3);
        if (temp.length() > 0 && temp.length() < 11) {
          Id = "";
          temp.toCharArray(config.Idchar, 11);
          sauvConfig();														// sauvegarde en EEPROM
          Id = String(config.Idchar);
          Id += fl;
        }
        messageId();
        message += "Nouvel Id";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("LOG") == 0) {	// demande log des 5 derniers commandes
        File f = SPIFFS.open(filelog, "r"); // taille du fichier log en SPIFFS
        message = "local log size :";
        message += String(f.size()) + fl;
        f.close();
        for (int i = 0; i < 5; i++) {
          message += String(record[i].dt) + "," + String(record[i].Act) + "," + String(record[i].Name) + fl;
        }
        //Serial.println( message);
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("ANTICIP") > -1) { // Anticipation du wakeup
        if (textesms.indexOf(char(61)) == 7) {
          int n = textesms.substring(8, textesms.length()).toInt();
          if (n > 9 && n < 3601) {
            config.anticip = n;
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        message += "Anticipation WakeUp (s) = ";
        message += config.anticip;
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("DEBUT") == 0) {     //	Heure Message Vie/debutJour
        if (textesms.indexOf(char(61)) == 5) {
          long i = atol(textesms.substring(6).c_str()); //	Heure message Vie
          if (i > 0 && i <= 86340) {                    //	ok si entre 0 et 86340(23h59)
            config.DebutJour = i;
            sauvConfig();                               // sauvegarde en EEPROM
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("TIME") == 0) {
        message += "Heure Sys = ";
        message += displayTime(0);
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("MAJHEURE") == 0) {	//	forcer mise a l'heure
        message += "Mise a l'heure";
        // Sim800.reset(SIMPIN);// lancer SIM800
        if (gsm)MajHeure(Sim800.getTimeSms(slot)); // mise a l'heure du sms
        if (nom != "Moi meme") EnvoyerSms(number, sms);
      }
      else if (gsm && textesms.indexOf("IMEI") > -1) {
        message += "IMEI = ";
        String m = Sim800.getIMEI();
        message += m + fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("FIN") == 0) {			//	Heure Fin de journée
        if ((textesms.indexOf(char(61))) == 3) {
          long i = atol(textesms.substring(4).c_str()); //	Heure
          if (i > 0 && i <= 86340) {										//	ok si entre 0 et 86340(23h59)
            config.FinJour = i;
            sauvConfig();															// sauvegarde en EEPROM
            Alarm.disable(FinJour);
            Alarm.write(FinJour, config.FinJour); // init tempo
            Alarm.enable(FinJour);
          }
        }
        message += "Fin Journee = ";
        message += Hdectohhmm(config.FinJour);
        message += "(hh:mm)";
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("MOIS") > -1) { // Calendrier pour un mois
        /* mise a jour calendrier ;format : MOIS=mm,31 fois 0/1
          demande calendrier pour un mois donné ; format : MOIS=mm? */
        bool flag = true; // validation du format
        bool W = true; // true Write, false Read
        int m = 0;
        if (textesms.indexOf("{") == 0) { // json
          DynamicJsonDocument doc(540);
          int f = textesms.lastIndexOf("}");
          // Serial.print("pos }:"),Serial.println(f);
          // Serial.print("json:"),Serial.print(textesms.substring(0,f+1)),Serial.println(".");,1,1,1,1,1,0,0,0,0,0,0]}";
          DeserializationError err = deserializeJson(doc, textesms.substring(0, f + 1));
          if (!err) {
            m = doc["MOIS"]; // 12
            JsonArray jour = doc["JOUR"];
            for (int j = 1; j < 32; j++) {
              calendrier[m][j] = jour[j - 1];
            }
            // Serial.print("mois:"),Serial.println(m);
            EnregistreCalendrier(); // Sauvegarde en SPIFFS
            // message += F("Mise a jour calendrier \nmois:");
            // message += m;
            // message += " OK (json)";
          }
          else {
            message += " erreur json ";
            flag = false;
          }
        }
        else { // message normal mois=12,31*0/1
          byte p1 = textesms.indexOf(char(61)); // =
          byte p2 = textesms.indexOf(char(44)); // ,
          if (p2 == 255) {                      // pas de ,
            p2 = textesms.indexOf(char(63));    // ?
            W = false;
          }

          m = textesms.substring(p1 + 1, p2).toInt(); // mois

          // printf("p1=%d,p2=%d\n",p1,p2);
          // Serial.println(textesms.substring(p1+1,p2).toInt());
          // Serial.println(textesms.substring(p2+1,textesms.length()).length());
          if (!(m > 0 && m < 13)) flag = false;
          if (W && flag) { // Write
            if (!(textesms.substring(p2 + 1, textesms.length()).length() == 31)) flag = false; // si longueur = 31(jours)

            for (int i = 1; i < 32; i++) { // verification 0/1
              if (!(textesms.substring(p2 + i, p2 + i + 1) == "0" || textesms.substring(p2 + i, p2 + i + 1) == "1")) {
                flag = false;
              }
            }
            if (flag) {
              // Serial.println(F("mise a jour calendrier"));
              for (int i = 1; i < 32; i++) {
                calendrier[m][i] = textesms.substring(p2 + i, p2 + i + 1).toInt();
                // Serial.print(textesms.substring(p2+i,p2+i+1));
              }
              EnregistreCalendrier(); // Sauvegarde en SPIFFS
              // message += F("Mise a jour calendrier mois:");
              // message += m;
              // message += " OK";
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
            // message +="{\"mois\":" + String(m) + "," +fl;
            // message += "\"jour\":[";
            // for (int i = 1; i < 32 ; i++){
            // message += String(calendrier[m][i]);
            // if (i < 31) message += ",";
            // }
            // message += "]}";
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
        EnvoyerSms(number, sms);
      }
      else if (textesms == "CIRCULE") {
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
        EnvoyerSms(number, sms);
        if (ok) {
          if (sms)EffaceSMS(slot);
          SignalVie();
          // action_wakeup_reason(4);
        }
      }
      else if (textesms == "NONCIRCULE") {
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
        EnvoyerSms(number, sms);
        if (ok) {
          if (sms) {
            EffaceSMS(slot);
          }
          // Extinction();
          action_wakeup_reason(4);
        }
      }
      else if (textesms.indexOf("TEMPOWAKEUP") == 0) { // Tempo wake up
        if ((textesms.indexOf(char(61))) == 11) {
          int i = textesms.substring(12).toInt(); //	durée
          if (i > 59 && i <= 36000) { // 1mn à 10H
            config.RepeatWakeUp = i;
            sauvConfig();															// sauvegarde en EEPROM
          }
        }
        message += "Tempo repetition Wake up (s)=";
        message += config.RepeatWakeUp;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("LST2") > -1) { //	Liste restreinte	//  =LST2=0,0,0,0,0,0,0,0,0
        bool flag = true; // validation du format
        if (textesms.indexOf(char(61)) == 4) { // "="
          byte Num[10];
          Sbidon = textesms.substring(5, 22);
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
              sauvConfig();															// sauvegarde en EEPROM
            }
          }
        }
        message += "Liste restreinte";
        message += fl;
        for (int i = 1; i < 10; i++) {
          message += config.Pos_Pn_PB[i];
          if ( i < 9) message += char(44); // ,
        }
        EnvoyerSms(number, sms);
      }
      else if (textesms == "RST" || textesms == "RESET") {               // demande RESET
        message += "Le systeme va etre relance";  // apres envoie du SMS!
        message += fl;
        FlagReset = true;                            // reset prochaine boucle
        EnvoyerSms(number, sms);
      }
      else if (textesms == "FAULTRESET") { // reset Erreur Position
        FlagAlarmePosition = false;
        FlagLastAlarmePosition = false;
        Cvr = Position_CVR(); // lire position CVR affectée à Cvr
        LastCvr = Cvr;
        ledcWrite(RgePwmChanel, 0); // Extinction Feux Rouge
        Allume = false;
        SlowBlink.detach();
        envoieGroupeSMS(0, 0);		// envoie groupé
      }
      else if (textesms.indexOf("CALIBRATION=") == 0) {
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
        		stock en EEPROM
        		sort du mode calibration

        		variables
        		FlagCalibration true cal en cours, false par defaut
        		Static P pin d'entrée
        		static int tensionmemo memorisation de la premiere tension mesurée en calibration
        		int CoeffTension = CoeffTensionDefaut 7000 par défaut
        */
        Sbidon = textesms.substring(12, 16); // texte apres =
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
          // tension = map(moyenneAnalogique(P), 0, 4095, 0, coef);
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(Id.substring(5, 9)) == 1) { // cherche CVR1
        if (textesms.indexOf("F") == 0) {
          // demande Fermeture
          Memo_Demande_CVR[0] = nom;      // nom demandeur
          Memo_Demande_CVR[1] = number;   // num demandeur
          Memo_Demande_CVR[2] = textesms; // demande d'origine
          FlagDemande_CVR = true;
          MajLog(nom,  "demande : " + textesms);
          Fermer_CVR();
        }
        else if (textesms.indexOf("O") == 0) {
          // demande Ouverture
          Memo_Demande_CVR[0] = nom;      // nom demandeur
          Memo_Demande_CVR[1] = number;   // num demandeur
          Memo_Demande_CVR[2] = textesms; // demande d'origine
          FlagDemande_CVR = true;
          MajLog(nom, "demande : " + textesms);
          Ouvrir_CVR();
        }
        else {
          message += "non reconnu" + fl;
          EnvoyerSms(number, sms);
        }
      }
      else if (textesms.indexOf("TEMPOOUVERTURE") >= 0) {
        if (textesms.indexOf(char(61)) == 14) {
          int n = textesms.substring(15, textesms.length()).toInt();
          if (n > 4 && n < 121) {
            config.Tempoouverture = n;
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        message += "Tempo Ouverture (s)= ";
        message += config.Tempoouverture;
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("TEMPOFERMETURE") >= 0) {
        if (textesms.indexOf(char(61)) == 14) {
          int n = textesms.substring(15, textesms.length()).toInt();
          if (n > 4 && n < 121) {
            config.Tempofermeture = n;
            sauvConfig();														// sauvegarde en EEPROM
          }
        }
        message += "Tempo Fermeture (s)= ";
        message += config.Tempofermeture;
        message += fl;
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("PARAM") >= 0) {
        // conserve meme format que Cv, remplissage pour variables inexistante dummy
        // message param divisé en 2 trop long depasse long 1sms 160c
        int dummy = 0;
        bool erreur = false;
        // Serial.print("position X:"),Serial.println(textesms.substring(7, 8));
        if (textesms.substring(7, 8) == "1") { // PARAM1
          // Serial.print("position ::"),Serial.println(textesms.substring(9, 10));
          if (textesms.substring(9, 10) == ":") {
            // json en reception sans lumlut
            DynamicJsonDocument doc(200);
            DeserializationError err = deserializeJson(doc, textesms);
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
        else if (textesms.substring(7, 8) == "2") { // PARAM2
          if (textesms.substring(9, 10) == ":") {
            // json en reception sans lumlut
            DynamicJsonDocument doc(200);
            DeserializationError err = deserializeJson(doc, textesms);
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
          EnvoyerSms(number, sms);
        } else {
          Serial.println(message);
        }
      }
      else if (gsm && textesms.indexOf("UPLOADLOG") == 0) {//upload log
        message += "lancement upload log";
        message += fl;
        MajLog(nom, "upload log");// renseigne log
        Sbidon = String(config.apn);
        Sim800.activateBearerProfile(config.apn); // ouverture GPRS

        Serial.println("Starting...");
        int reply = gprs_upload_function (); // Upload fichier
        Serial.println("The end... Response: " + String(reply));

        if (reply == 0) {
          message += "upload OK";
          SPIFFS.remove(filelog);  // efface fichier log
          MajLog(nom, "");         // nouveau log
          MajLog(nom, "upload OK");// renseigne nouveau log
        } else {
          message += "upload fail";
          MajLog(nom, "upload fail");// renseigne log
        }
        Sim800.deactivateBearerProfile(); // fermeture GPRS
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("FTPDATA") > -1) {
        // Parametres FTPDATA=Serveur:User:Pass:port
        // {"FTPDATA":{"serveur":"dd.org","user":"user","pass":"pass",,"port":00}}
        bool erreur = false;
        bool formatsms = false;
        if (textesms.indexOf(":") == 10) { // format json
          DynamicJsonDocument doc(210); //https://arduinojson.org/v6/assistant/
          DeserializationError err = deserializeJson(doc, textesms);
          if (err) {
            erreur = true;
          }
          else {
            JsonObject ftpdata = doc["FTPDATA"];
            strncpy(config.ftpServeur,  ftpdata["serveur"], 26);
            strncpy(config.ftpUser,     ftpdata["user"],    11);
            strncpy(config.ftpPass,     ftpdata["pass"],    16);
            config.ftpPort         =    ftpdata["port"];
            sauvConfig();													// sauvegarde en EEPROM
          }
        }
        else if ((textesms.indexOf(char(61))) == 7) { // format sms
          formatsms = true;
          byte w = textesms.indexOf(":");
          byte x = textesms.indexOf(":", w + 1);
          byte y = textesms.indexOf(":", x + 1);
          byte zz = textesms.length();
          if (textesms.substring(y + 1, zz).toInt() > 0) { // Port > 0
            if ((w - 7) < 25 && (x - w - 1) < 11 && (y - x - 1) < 16) {
              Sbidon = textesms.substring(7, w);
              Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
              Sbidon = textesms.substring(w + 1, x);
              Sbidon.toCharArray(config.ftpUser, (Sbidon.length() + 1));
              Sbidon = textesms.substring(x + 1, y);
              Sbidon.toCharArray(config.ftpPass, (Sbidon.length() + 1));
              config.ftpPort = textesms.substring(y + 1, zz).toInt();
              sauvConfig();													// sauvegarde en EEPROM
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
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("FTPSERVEUR") == 0) { // Serveur FTP
        // case sensitive
        // FTPSERVEUR=xyz.org
        if (textesms.indexOf(char(61)) == 10) {
          Sbidon = textesms.substring(11);
          Serial.print("ftpserveur:"), Serial.print(Sbidon);
          Serial.print(" ,"), Serial.println(Sbidon.length());
          Sbidon.toCharArray(config.ftpServeur, (Sbidon.length() + 1));
          sauvConfig();
        }
        message += "FTPserveur =";
        message += String(config.ftpServeur);
        message += "\n au prochain demarrage";
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf("GPRSDATA") > -1) {
        // Parametres GPRSDATA = "APN":"user":"pass"
        // GPRSDATA="sl2sfr":"":""
        // {"GPRSDATA":{"apn":"sl2sfr","user":"","pass":""}}
        bool erreur = false;
        bool formatsms = false;
        if (textesms.indexOf(":") == 11) { // format json
          DynamicJsonDocument doc(120);
          DeserializationError err = deserializeJson(doc, textesms);
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
            sauvConfig();													// sauvegarde en EEPROM
          }
        }
        else if ((textesms.indexOf(char(61))) == 8) { // format sms
          formatsms = true;
          byte cpt = 0;
          byte i = 9;
          do { // compte nombre de " doit etre =6
            i = textesms.indexOf('"', i + 1);
            cpt ++;
          } while (i <= textesms.length());
          Serial.print("nombre de \" :"), Serial.println(cpt);
          if (cpt == 6) {
            byte x = textesms.indexOf(':');
            byte y = textesms.indexOf(':', x + 1);
            byte z = textesms.lastIndexOf('"');
            // Serial.printf("%d:%d:%d\n",x,y,z);
            // Serial.printf("%d:%d:%d\n", x -1 - 10, y-1 - x-1-1, z - y-1-1);
            if ((x - 11) < 11 && (y - x - 3) < 11 && (z - y - 2) < 11) { // verification longueur des variables
              Sbidon = textesms.substring(10, x - 1);
              Sbidon.toCharArray(config.apn, (Sbidon.length() + 1));
              Sbidon = textesms.substring(x + 1 + 1 , y - 1);
              Sbidon.toCharArray(config.gprsUser, (Sbidon.length() + 1));
              Sbidon = textesms.substring(y + 1 + 1, z);
              Sbidon.toCharArray(config.gprsPass, (Sbidon.length() + 1));

              // Serial.print("apn:"),Serial.println(config.apn);
              // Serial.print("user:"),Serial.println(config.gprsUser);
              // Serial.print("pass:"),Serial.println(config.gprsPass);

              sauvConfig();													// sauvegarde en EEPROM
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
        EnvoyerSms(number, sms);
      }
      else if (textesms == "VIDELOG"){
        SPIFFS.remove(filelog);
        FileLogOnce = false;
        message += "Effacement fichier log";
        EnvoyerSms(number, sms);
      }
      else if (textesms.indexOf(F("SENDAT")) == 0){
        // envoie commande AT au SIM800
        // ex: SENDAT=AT+CCLK="23/07/19,10:00:20+04" mise à l'heure
        // attention DANGEREUX pas de verification!
        if (textesms.indexOf(char(61)) == 6) {
          String CdeAT = textesms.substring(7, textesms.length());
          String reply = sendAT(CdeAT,"OK","ERROR",1000);
          // Serial.print("reponse: "),Serial.println(reply);
          message += String(reply);
          EnvoyerSms(number, sms);
        }
      }
      else if (textesms.indexOf(F("TYPEBATT")) == 0){ // Type Batterie
        if (textesms.indexOf(char(61)) == 8) {
          int type = textesms.substring(9, textesms.length()).toInt();
          if(type == 16 || type == 24){
            config.TypeBatt = type;
            sauvConfig();													// sauvegarde en EEPROM
          }
        }
        message += "Type Batterie:" + fl;
        if(config.TypeBatt == 16) message += "Pb 12V";
        if(config.TypeBatt == 24) message += "LiFePO 12.8V";
        EnvoyerSms(number, sms);
      }
      //**************************************
      else {
        message += "message non reconnu !";
        message += fl;
        if (nom != "Moi meme") EnvoyerSms(number, sms);
      }
    }
    else {
      Serial.print("Appelant non reconnu ! ");
    }
    if (sms) { // suppression du SMS
      EffaceSMS(slot);
    }
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
void envoieGroupeSMS(byte grp, bool m) {
  if (gsm) {
    /* m=0 message normal/finanalyse
    	si grp = 0,
      envoie un SMS à tous les numero existant (9 max) du Phone Book
    	SAUF ceux de la liste restreinte
      si grp = 1,
      envoie un SMS à tous les numero existant (9 max) du Phone Book
      de la liste restreinte config.Pos_Pn_PB[x]=1
      si grp = 3,
      Message au Serveur seulement N°1 de la liste			*/

    byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
    // Serial.print(F("Nombre de ligne PB=")),Serial.println(n);
    if (grp == 3) n = 1; // limite la liste à ligne 1
    for (byte Index = 1; Index < n + 1; Index++) { // Balayage des Num Tel dans Phone Book
      // if ((grp == 3) || (grp == 0 && config.Pos_Pn_PB[Index] == 0) || (grp == 1 && config.Pos_Pn_PB[Index] == 1)) {
      if ((grp == 3) || (grp == 0) || (grp == 1 && config.Pos_Pn_PB[Index] == 1)) {
        String number = Sim800.getPhoneBookNumber(Index);
        generationMessage(m);
        char num[13];
        number.toCharArray(num, 13);
        EnvoyerSms(num, true);
      }
    }
  }
}
//---------------------------------------------------------------------------
void generationMessage(bool n) {
  // n = 0 message normal
  // n = 1 message fin analyse
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
}
//---------------------------------------------------------------------------
void EnvoyerSms(char *num, bool sms) {
  Serial.print("destinataire:"), Serial.println(num);
  if (sms && gsm) { // envoie sms
    message.toCharArray(replybuffer, message.length() + 1);
    bool OK = Sim800.sendSms(num, replybuffer);
    if (OK) {
      Serial.print("send sms OK:");
      Serial.println(num);
    }
  }
  Serial.print ("Message (long) = "), Serial.println(message.length());
  Serial.println(message);
}
//---------------------------------------------------------------------------
void read_RSSI() {	// lire valeur RSSI et remplir message
  if (gsm) {
    int r;
    byte n = Sim800.getRSSI();
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
void MajHeure(String smsdate) {
  if (gsm) {
    /*parametrage du SIM800 a faire une fois
      AT+CLTS? si retourne 0
      AT+CLTS=1
      AT+CENG=3
      AT&W pour sauvegarder ce parametre
      si AT+CCLK? pas OK
      avec Fonatest passer en GPRS 'G', envoyer 'Y' la sync doit se faire, couper GPRS 'g'
      't' ou AT+CCLK? doit donner la date et heure réseau
      format date retourné par Fona "yy/MM/dd,hh:mm:ss±zz",
      +CCLK: "14/08/08,02:25:43-16" -16= décalage GMT en n*1/4heures(-4) */
    if (smsdate.length() > 1) { // si smsdate present mise a l'heure forcé
      Sim800.SetTime(smsdate);
    }
    static bool First = true;
    int ecart;
    Serial.print("Mise a l'heure reguliere !, ");
    // setTime(10,10,0,1,1,18);
    int Nday, Nmonth, Nyear, Nminute, Nsecond, Nhour;
    Sim800.RTCtime(&Nday, &Nmonth, &Nyear, &Nhour, &Nminute, &Nsecond);


    printf("%s %02d/%02d/%d %02d:%02d:%02d\n", "MajH1", Nday, Nmonth, Nyear, Nhour, Nminute, Nsecond);
    long debut = millis();
    if (First || Nyear < 17) {
      while (Nyear < 17) {
        Sim800.RTCtime(&Nday, &Nmonth, &Nyear, &Nhour, &Nminute, &Nsecond);
        printf("%s %02d/%02d/%d %02d:%02d:%02d\n", "MajH2", Nday, Nmonth, Nyear, Nhour, Nminute, Nsecond);
        Alarm.delay(1000);
        // if (millis() - debut > 10000) {// supprimé risque de deconnexion reseau plus de redemarage
        // Sim800.setPhoneFunctionality(0);
        // Alarm.delay(1000);
        // Sim800.setPhoneFunctionality(1);
        // Alarm.delay(1000);
        // }
        if (millis() - debut > 15000) {
          Serial.println("Impossible de mettre à l'heure !");
          //on s'envoie à soi même un SMS "MAJHEURE"
          message = "MAJHEURE";
          char numchar[13];
          String numstring = Sim800.getNumTel();
          numstring.toCharArray(numchar, 13);
          EnvoyerSms(numchar, true);
          break;
        }
      }
      setTime(Nhour, Nminute, Nsecond, Nday, Nmonth, Nyear);
      First = false;
    }
    else {
      //  calcul décalage entre H sys et H reseau en s
      ecart = (Nhour - hour()) * 3600;
      ecart += (Nminute - minute()) * 60;
      ecart += Nsecond - second();
      // ecart += 10;
      Serial.print("Ecart s= "), Serial.println(ecart);
      if (abs(ecart) > 5) {
        // ArretSonnerie();	// Arret Sonnerie propre
        Alarm.disable(loopPrincipale);
        Alarm.disable(DebutJour);
        Alarm.disable(FinJour);

        setTime(Nhour, Nminute, Nsecond, Nday, Nmonth, Nyear);

        Alarm.enable(loopPrincipale);
        Alarm.enable(DebutJour);
        Alarm.enable(FinJour);
      }
    }
  }
  displayTime(0);
  AIntru_HeureActuelle();
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
    MajHeure("");
    Sim800.delAllSms();// au cas ou, efface tous les SMS envoyé/reçu
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
void sauvConfig() { // sauve configuration en EEPROM
  EEPROM.begin(512);
  EEPROM.put(confign, config);
  EEPROM.commit();
  EEPROM.end();
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
void logRecord(String nom, String action) { // renseigne log et enregistre EEPROM
  static int index = 0;
  String temp;
  if (month() < 10) {
    temp =  "0" + String(month());
  }
  else {
    temp = String(month());
  }
  if (day() < 10 ) {
    temp += "0" + String(day());
  }
  else {
    temp += String(day());
  }
  if (hour() < 10) {
    temp += "-0" + String(hour());
  }
  else {
    temp += "-" + String(hour());
  }
  if (minute() < 10) {
    temp += "0" + String(minute());
  }
  else {
    temp += String(minute());
  }
  temp  .toCharArray(record[index].dt, 10);
  nom   .toCharArray(record[index].Name, 15);
  action.toCharArray(record[index].Act, 2);

  EEPROM.begin(512);
  EEPROM.put(recordn, record);// ecriture des valeurs par defaut
  EEPROM.commit();
  EEPROM.end();
  if (index < 4) {
    index ++;
  }
  else {
    index = 0;
  }
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
void MajLog(String Id, String Raison) { // mise à jour fichier log en SPIFFS
  if (SPIFFS.exists(filelog)) {
    /* verification de la taille du fichier */
    File f = SPIFFS.open(filelog, "r");
    Serial.print("Taille fichier log = "), Serial.println(f.size());
    // Serial.print(Id),Serial.print(","),Serial.println(Raison);
    if (f.size() > 150000 && !FileLogOnce) {
      /* si trop grand on efface */
      FileLogOnce = true;
      messageId();
      message += "Fichier log presque plein\n";
      message += String(f.size());
      message += "\nFichier sera efface a 300000";
      if (gsm) {
        String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
        char num[13];
        number.toCharArray(num, 13);
        EnvoyerSms(num, true);
      }
    }
    else if (f.size() > 300000 && FileLogOnce) { // 292Ko 75000 lignes
      messageId();
      message += "Fichier log plein\n";
      message += String(f.size());
      message += "\nFichier efface";
      if (gsm) {
        String number = Sim800.getPhoneBookNumber(1); // envoyé au premier num seulement
        char num[13];
        number.toCharArray(num, 13);
        EnvoyerSms(num, true);
      }
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
  else {
    char Cbidon[101]; // 100 char maxi
    sprintf(Cbidon, "%02d/%02d/%4d %02d:%02d:%02d;", day(), month(), year(), hour(), minute(), second());
    strcat(Cbidon, config.Idchar);
    strcat(Cbidon, fl.c_str());
    appendFile(SPIFFS, filelog, Cbidon);
    Serial.print("nouveau fichier log:"), Serial.println(Cbidon);
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
void PrintEEPROM() {
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
  Serial.print("GPRS APN = "), Serial.println(config.apn);
  Serial.print("GPRS user = "), Serial.println(config.gprsUser);
  Serial.print("GPRS pass = "), Serial.println(config.gprsPass);
  Serial.print("ftp serveur = "), Serial.println(config.ftpServeur);
  Serial.print("ftp port = "), Serial.println(config.ftpPort);
  Serial.print("ftp user = "), Serial.println(config.ftpUser);
  Serial.print("ftp pass = "), Serial.println(config.ftpPass);
}
//---------------------------------------------------------------------------
void ConnexionWifi(char* ssid, char* pwd, char* number, bool sms) {

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
  EnvoyerSms(number, sms);

  if (sms) { // suppression du SMS
    /* Obligatoire ici si non bouclage au redemarrage apres timeoutwifi
      ou OTA sms demande Wifi toujours present */
    EffaceSMS(slot);
  }
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
    WifiOff();
  }
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
String ExtraireSms(String msgbrut) { //Extraction du contenu du SMS

  int pos[10];									// SMS jusqu'a 5 lignes
  int i = 0;
  for (i = 0; i < 10; i++) {
    if (i == 0) {
      pos[i] = msgbrut.indexOf("\n");
    }
    else {
      pos[i] = msgbrut.indexOf("\n", pos[i - 1] + 1);
    }
    // Serial.print(i),Serial.print(" pos = "),Serial.println(pos[i]);
    if (pos[i] == -1) {
      i --;
      break;
    }
  }

  String message = msgbrut.substring(pos[1] + 1, pos[i - 1] - 1);
  // Serial.print("message extrait = "),Serial.println(message);
  message.replace("\n", "|");				// remplacement des sauts de lignes par |
  message = message.substring(0, message.length() - 2);
  // Serial.print("message extrait sans \n= "),Serial.println(message);

  return message;
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
    while (!Sim800.sleep()) {
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
void EffaceSMS(int s) {
  bool err;
  byte n = 0;
  do {
    err = Sim800.delSms(s);
    n ++;
    Serial.print("resultat del Sms ");	Serial.println(err);
    if (n > 10) { // on efface tous si echec
      err = Sim800.delAllSms();
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
    byte n = Sim800.ListPhoneBook(); // nombre de ligne PhoneBook
    for (byte i = 1; i < n + 1; i++) {
      String num = Sim800.getPhoneBookNumber(i);
      // Serial.print(num.length()), Serial.print(" "), Serial.println(num);
      if (num.indexOf("+CPBR:") == 0) { // si existe pas sortir
        Serial.println("Failed!");// next i
        goto fin_liste;
      }
      String name = Sim800.getPhoneBookName(i);
      // Serial.println(name);
      webpage += "<tr>";
      webpage += "<td>"; webpage += name; webpage += "</td>";
      webpage += "<td>"; webpage += num ; webpage += "</td>";
      webpage += "<td>"; webpage += String(config.Pos_Pn_PB[i]); webpage += "</td>";
      webpage += "</tr>";
    }
  }
fin_liste:

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
byte gprs_upload_function () {
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
  if (reply == 0) { // ouverture GPRS
    // reply = sendATcommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"","OK","ERROR", 1000);
    if (reply == 0) {
      // reply = sendATcommand("AT+SAPBR=3,1,\"APN\",\"sl2sfr\"", "OK", "ERROR", 1000);//Replace with your APN
      if (reply == 0) {
        //reply = sendATcommand("AT+SAPBR=3,1,\"USER\",\"entelpcs\"", "OK", "ERROR", 1000);
        if (reply == 0) {
          //reply = sendATcommand("AT+SAPBR=3,1,\"PWD\",\"entelpcs\"", "OK", "ERROR", 1000);
          if (reply == 0) {
            reply = 2;
            i = 0;
            while (i < 3 && reply == 2) { //Try 3 times...
              reply = sendATcommand("AT+SAPBR=1,1", "OK", "ERROR", 10000);
              if (reply == 2) {
                sendATcommand("AT+SAPBR=0,1", "OK", "ERROR", 10000);
              }
              i++;
            }
            if (reply == 0) {
              reply = sendATcommand("AT+SAPBR=2,1", "OK", "ERROR", 1000);
              if (reply == 0) {
                reply = sendATcommand("AT+FTPCID=1", "OK", "ERROR", 1000);
                if (reply == 0) {
                  reply = sendATcommand("AT+FTPSERV=\"" + String(config.ftpServeur) + "\"", "OK", "ERROR", 1000);//replace ftp.sample.com with your server address
                  if (reply == 0) {
                    reply = sendATcommand("AT+FTPPORT=" + String(config.ftpPort), "OK", "ERROR", 1000);
                    if (reply == 0) {
                      reply = sendATcommand("AT+FTPUN=\"" + String(config.ftpUser) + "\"", "OK", "ERROR", 1000);//Replace 1234@sample.com with your username
                      if (reply == 0) {
                        reply = sendATcommand("AT+FTPPW=\"" + String(config.ftpPass) + "\"", "OK", "ERROR", 1000);//Replace 12345 with your password
                        if (reply == 0) {
                          reply = sendATcommand("AT+FTPPUTNAME=\"" + String(filelog) + "\"", "OK", "ERROR", 1000);
                          if (reply == 0) {
                            reply = sendATcommand("AT+FTPPUTPATH=\"/" + String(config.Idchar) + "/\"", "OK", "ERROR", 1000);// repertoire "/Id/"
                            if (reply == 0) {
                              unsigned int ptime = millis();
                              reply = sendATcommand("AT+FTPPUT=1", "+FTPPUT: 1,1", "+FTPPUT: 1,6", 60000);
                              Serial.println("Time: " + String(millis() - ptime));
                              if (reply == 0) {
                                if (UploadFile) {
                                  //int i = 0;
                                  unsigned int ptime = millis();
                                  long archivosize = UploadFile.size();
                                  while (UploadFile.available()) {
                                    while (archivosize >= buffer_space) {
                                      reply = sendATcommand("AT+FTPPUT=2," + String(buffer_space), "+FTPPUT: 2,1", "OK", 3000);
                                      if (reply == 0) { //This loop checks for positive reply to upload bytes and in case or error it retries to upload
                                        Serial.println("Remaining Characters: " + String(UploadFile.available()));
                                        for (int d = 0; d < buffer_space; d++) {
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
                                      for (int t = 0; t < archivosize; t++) {
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
byte sendATcommand(String ATcommand, String answer1, String answer2, unsigned int timeout) {
  byte reply = 1;
  String content = "";
  char character;

  //Clean the modem input buffer
  while (Serial2.available() > 0) Serial2.read();

  //Send the atcommand to the modem
  Serial2.println(ATcommand);
  delay(100);
  unsigned int timeprevious = millis();
  while ((reply == 1) && ((millis() - timeprevious) < timeout)) {
    while (Serial2.available() > 0) {
      character = Serial2.read();
      content.concat(character);
      Serial.print(character);
      delay(10);
    }
    //Stop reading conditions
    if (content.indexOf(answer1) != -1) {
      reply = 0;
    } else if (content.indexOf(answer2) != -1) {
      reply = 2;
    } else {
      //Nothing to do...
    }
  }
  return reply;
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
  if (gsm) {
    Serial.println("message serveur");
    envoieGroupeSMS(3, 0); // envoie serveur
    if (Memo_Demande_CVR[1] == Sim800.getPhoneBookNumber(1)) {
      smsserveur = true; // si demande provient du serveur index=1
    }
    if (Memo_Demande_CVR[0] != "console") {
      if (!smsserveur) {
        Serial.println("reponse demandeur si pas serveur");
        EnvoyerSms(number, true); // reponse demandeur si pas serveur
      }
    } else {
      Serial.println(message);
    }
  } else {
    Serial.println(message);
  }
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

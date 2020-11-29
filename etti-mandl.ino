/* Ettimandl


  Copyright (C) 2020 by Martin Zahn
  2020-10-12 Martin Zahn    | initial version V0.1
                            Den Code kann jeder frei verwenden, ändern und hochladen wo er will,
                            solange er nicht seinen eigenen Namen drüber setzt, oder diesen kommerziell verwertet, beispielsweise
                            indem Etikettiermaschinen mit diesem Code versehen und verkauft werden.
  2020-10-29 Marc Junker    Ansteuerung des Stempels auf ServoEasing umgestellt.
  2020-11-14 Marc Junker    StartSensorPin von 23 auf 25 umgestellt. Sensor-Abfrage von ttl auf analog geändert. OTA eingebaut.
                            Auslagerung der Konfiguration in etti-mandl.h



*/


/*
  Pinbelegung:
  Rotary small   CLK  34
                 DT   35
                 SW   13

  DRV8025        DIR  32
                 STEP 33
                 EN   14

  POTI                12
  ROTARY BIG     A    16
                 B    17

  Buttons        1    15
                 2    2
                 3    4
                 4    5
                 5    18

  Start-Sensor        25     (zuvor 23)  // MarcN
  Servo               19

*/


const char versionTag[] = "ver0.2";


//
// Hier den Code auf die verwendete Hardware einstellen
//

#define USE_STEMPEL           // Datumsstempeleinheit vorhanden


// Ab hier nur verstellen wenn Du genau weisst, was Du tust!
//
//#define isDebug             // serielle debug-Ausgabe aktivieren.

// OTA Support. Experimenteller Beta-Betrieb
//#define useOTA



// Strom messen und einstellen: http://sturm.selfhost.eu/wordpress/pololu-schrittmotortreiber-einstellen/

// Blau DRV8825 0,1 Ohm mit mittl. SM max 1,7 A
// Max Strom: 1,7A x 0,7 = 1,19A
// Vref = Imax x (5 x Rs)
// Vref = 1,19A x 5 x 0,1Ohm=0,595 Volt

// Rot A4988: mit mittl. Motor max 1,7 A
// Vref = Imax x (8 x Rs) mit Rs zu 0,05Ohm.
// Vref = 1,19 x 8 x 0,05 = 0,476 Volt
// Einstellen zwischen Poti-Mitte und Masse





// #include <Arduino.h>
#include <AccelStepper.h> // für Schrittmotor. https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include <Preferences.h>  // für EEPROM
//#include <ESP32_Servo.h>  // https://github.com/jkb-git/ESP32Servo
#include "etti-mandl.h"


// OTA
#ifdef useOTA
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

// Encoder
#include <ESP32Encoder.h>
ESP32Encoder encoder;
ESP32Encoder encoderSmall;


//Display
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

// fuer Heltec WiFi Kit 32 (ESP32 onboard OLED)
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);








// Initialisierung des Schrittmotors
AccelStepper stepper(2, step_pin, dir_pin); // Motortyp, STEP_PIN, DIR_PIN

// Daten in Eeprom lesen/speichern
Preferences preferences;

// Initialisierung des Servos für den Datumsstempel
#ifdef USE_STEMPEL
#include <ServoEasing.h>
ServoEasing servo;
#endif



//################################################# SETUP #############################################


void setup()
{

  Serial.begin(115200);
  // while (!Serial) {
  //   }

  // Boot Screen
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  print_logo();
  delay(2000);
  u8g2.clearBuffer();          // clear the internal memory


  ////////////////////////////// OTA

#ifdef useOTA

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("EttiMandl");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    //Serial.println("Start updating " + type);
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font

    u8g2.setCursor(0, 10); // spalte zeile
    u8g2.print("Receiving");
    u8g2.setCursor(0, 30); // spalte zeile
    u8g2.print("OTA Update");


    u8g2.sendBuffer();
  })
  .onEnd([]() {
    //Serial.println("\nEnd");
    u8g2.setCursor(0, 70); // spalte zeile
    u8g2.printf("Done !");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    u8g2.setCursor(0, 50); // spalte zeile
    u8g2.printf("Progress: %u%%\r", (progress / (total / 100)));
    u8g2.sendBuffer();
  })
  .onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

#endif

  //Serial.println("Ready");
  //Serial.print("IP address: ");
  //Serial.println(WiFi.localIP());

  ////////////////////////////////////




#ifdef isDebug
  Serial.println("Ettimandl Start");
#endif


  // Eingänge und Ausgänge
  //------------------------

  // 3 Buttons - TODO ersetzen durch Rotary Encoder klein
  //----------------
  pinMode(button1, INPUT_PULLUP); // internal pullup
  pinMode(button2, INPUT_PULLUP); // internal pullup buttonsave
  pinMode(buttonsave, INPUT_PULLUP); // internal pullup


  // Rotary Encoder groß
  //----------------
  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoder.attachHalfQuad(16, 17);
  encoder.clearCount();

  // Rotary Encoder klein
  //----------------
  encoderSmall.attachHalfQuad(34, 35);
  //encoderSmall.attachfullQuad(34, 35);
  encoderSmall.clearCount();



  //pinMode(RotaryA, INPUT_PULLUP); // internal pullup
  //pinMode(RotaryB, INPUT_PULLUP); // internal pullup

  // Schrittmotor Enable
  pinMode(enable_pin, OUTPUT); // DRV8025 ENABLE // TODO: Prüfen ob erforderlich, da Accelstepper verwendet wird


  // SCHRITTMOTORGRUNDEINSTELLUNGEN
  //--------------

  stepper.setEnablePin(enable_pin);
  stepper.setPinsInverted(false, false, true); // directioninvert, stepinvert, enableinvert

  stepper.setMaxSpeed(MaxSpeed); // Maximale Geschwindigkeit
  stepper.setAcceleration(Acceleration); // Beschleunigung
  stepper.setCurrentPosition(0);


#ifdef USE_STEMPEL
  servo.attach(servo_pin);
  servo.setEasingType(EASE_CUBIC_IN_OUT);
  servo.setSpeed(ServoSpeed);
  servo.write(WinkelRuhe);
#endif




  // Preferences aus dem EEPROM lesen
  getPreferences(); // Eeprom auslesen


  // Displayausgabe
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font

  u8g2.setCursor(0, 15); // spalte zeile
  u8g2.print("Glas einlegen");

  u8g2.sendBuffer();


  if (Length == 0)
  {
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font

    u8g2.setCursor(0, 15); // spalte zeile
    u8g2.print("Etikettenlänge");
    u8g2.setCursor(0, 30);
    u8g2.print("einstellen!");

    u8g2.sendBuffer();          // transfer internal memory to the display
#ifdef isDebug
    Serial.println("keine Etikettenlänge gespeichert");
#endif
    u8g2.clearBuffer();          // clear the internal memory
  }



  /*
    servo.easeTo(WinkelRuhe);
    delay(2000);


    while (true) {
      //servo.easeTo( (int)encoderSmallgetCounthalf());
         u8g2.clearBuffer();
          u8g2.setCursor(0, 30);
      u8g2.print((int)encoderSmallgetCounthalf());
       u8g2.sendBuffer();
      delay(200);
      if (digitalRead(13) == LOW) {
        servo.easeTo(WinkelAktiv);
        servo.easeTo(WinkelAktiv-10);
        delay(StempelPause);
        servo.easeTo(WinkelAktiv);
        servo.easeTo(WinkelAktiv-10);
        delay(StempelPause);
        servo.easeTo(WinkelAktiv);
        servo.easeTo(WinkelAktiv-10);
        delay(StempelPause);
        servo.easeTo(WinkelAktiv);
        servo.easeTo(WinkelAktiv-10);
        delay(StempelPause);
        servo.easeTo(WinkelAktiv);
        servo.easeTo(WinkelAktiv-10);
        delay(StempelPause);
        servo.easeTo(WinkelAktiv);
        servo.easeTo(WinkelRuhe);
      }
      };
  */


} // Ende Setup


//################################################# LOOP  #############################################


void loop()
{
  // OTA
#ifdef useOTA
  ArduinoOTA.handle();
#endif

  Position = encoder.getCount();



  // großen Rotary-Encoder erfassen (Position des Etiketts)
  // if ( Position != temp ) {
  //   temp = Position;
  //  Serial.print ("Rotary-Pos: ");
  // Serial.println (Position);
  // }

  /*
    #ifdef isDebug
    Serial.print (" Modus: ");
    Serial.print (modus);
    Serial.print (" Pos: ");
    Serial.print (stepper.currentPosition());
    Serial.print (" Speed: ");
    Serial.print (stepper.speed());
    Serial.print (" Rotary: ");
    Serial.println (Position);
    #endif
  */


  // ------ RotarySmall Testing ----------------------------------------------------
  if (digitalRead(13) == LOW && (modus == RUHE || modus == BOOT)) {
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font

    u8g2.setCursor(0, 15); // x y
    u8g2.print("RotS:");
    u8g2.setCursor(60, 15);
    u8g2.print(((int)encoderSmallgetCounthalf()));
    /*
        u8g2.setCursor(0, 30); // x y
        u8g2.print("Step:");
        u8g2.setCursor(50, 30);
        u8g2.print(stepper.currentPosition());
    */
    u8g2.sendBuffer();

  }



  // ------------------------------------------------------------------------------------------------------------------
  // ----------------------------- ETIKETTENLÄNGE FESTLEGEN BZW. VOR- UND ZURÜCKSPULEN --------------------------------
  // ------------------------------------------------------------------------------------------------------------------



  // Buttons 1 und 2 bewegen jeweils den Schrittmotor um 4 Steps = 1 Schritt. (TODO: Ersetzen durch Rotary-Steuerung)


  // --------------------------------------------- ETIKETT VORSPULEN -------------------------------------------


  if (digitalRead(button1) == LOW && modus == BOOT) // Etikett vorspulen
  {
    stepper.enableOutputs(); //enable pins
    target = stepper.currentPosition() + manualStep;
    stepper.moveTo(target);

    while (stepper.distanceToGo() != 0)
    {
      stepper.setSpeed(CreepSpeed);
      stepper.runSpeedToPosition();
    }

    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font

    u8g2.setCursor(0, 15); // x y
    u8g2.print("Rot:");
    u8g2.setCursor(50, 15);
    u8g2.print(Position);

    u8g2.setCursor(0, 30); // x y
    u8g2.print("Step:");
    u8g2.setCursor(50, 30);
    u8g2.print(stepper.currentPosition());

    u8g2.sendBuffer();          // transfer internal memory to the display

    Length = Position;

#ifdef isDebug
    Serial.print("Button1+ ");
    Serial.println(Position);
#endif
  }

  // --------------------------------------------- ETIKETT ZURÜCKSPULEN -------------------------------------------

  if (digitalRead(button2) == LOW && modus == BOOT) // Etikett zurückspulen
  {
    stepper.enableOutputs(); //enable pins
    target = stepper.currentPosition() - manualStep;
    stepper.moveTo(target);

    while (stepper.distanceToGo() != 0)
    {
      stepper.setSpeed(CreepSpeed);
      stepper.runSpeedToPosition();
    }
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font

    u8g2.setCursor(0, 15); // x y
    u8g2.print("Rot:");
    u8g2.setCursor(50, 15);
    u8g2.print(Position);

    u8g2.setCursor(0, 30); // x y
    u8g2.print("Step:");
    u8g2.setCursor(50, 30);
    u8g2.print(stepper.currentPosition());

    u8g2.sendBuffer();         // transfer internal memory to the display
    Length = Position;
#ifdef isDebug
    Serial.print("Button2- Schleife ");
    Serial.println(Position);
#endif
  }


  // ----------------------------------------------- Länge verkürzen ---------------------------
  if (digitalRead(button2) == LOW && modus == RUHE) // Etikettlänge anpassen
  {

    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
    Length--;
    Position = Length;
    delay(100);
    u8g2.setCursor(0, 15); // x y
    u8g2.print("Länge:");
    u8g2.setCursor(50, 15);
    u8g2.print(Length);
    u8g2.sendBuffer();         // transfer internal memory to the display

#ifdef isDebug
    Serial.print("Button2- Schleife ");
    Serial.println(Position);
#endif
  }
  // ----------------------------------------------- Länge verlängern ---------------------------
  if (digitalRead(button1) == LOW && modus == RUHE) // Etikettlänge anpassen
  {

    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
    Length++;
    Position = Length;
    delay(100);
    u8g2.setCursor(0, 15); // x y
    u8g2.print("Länge:");
    u8g2.setCursor(50, 15);
    u8g2.print(Length);
    u8g2.sendBuffer();         // transfer internal memory to the display

#ifdef isDebug
    Serial.print("Button2- Schleife ");
    Serial.println(Position);
#endif
  }



  // ------------------------------------------------ LÄNGE SPEICHERN ----------------------------------------------

  // aktuelle Position als Etikettenlänge speichern
  if (digitalRead(buttonsave) == LOW && (modus == RUHE || modus == BOOT)) // Länge speichern // ggf ersetzen durch RotarySW
  {
#ifdef isDebug
    Serial.println("Speichern");
#endif
    stepper.stop();
    stepper.disableOutputs();
    setPreferences();
    delay(2000);
    modus = RUHE;
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
    u8g2.setCursor(0, 15); // spalte zeile
    u8g2.print("Glas einlegen");
    u8g2.sendBuffer();
  }

  // ------------------------------------------------------------------------------------------------------------------
  // ---------------------------------------------- Etikettiervorgang -------------------------------------------------
  // ------------------------------------------------------------------------------------------------------------------

  if (readStartSensor() == LOW && (modus == RUHE || modus == BOOT))  //Sensor erkannt -> Start
  {

    Position = 0; // Position des Rotary Encoders auf 0 setzen
    encoder.clearCount();
#ifdef isDebug
    Serial.println("Startsensor erkannt");
    Serial.print("Length: ");
    Serial.println(Length);
    Serial.print("LastSteps: ");
    Serial.println(LastSteps);
#endif

    if (Length == 0) { // Abbruch, falls noch keine Etikettenlänge im Eeprom gespeichert wurde
      //Displayausgabe
      u8g2.clearBuffer();          // clear the internal memory
      u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
      u8g2.setCursor(0, 15); // spalte zeile
      u8g2.print("Keine Länge definiert");
      u8g2.sendBuffer();

      delay(2000);
      modus = RUHE;
    }

    else {
      /*
           //Displayausgabe
           u8g2.clearBuffer();          // clear the internal memory
           u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
           u8g2.setCursor(0, 15); // spalte zeile
           u8g2.print("START");
           u8g2.sendBuffer();
      */

      modus = START;

      for (int i = 2; i > 0; i--) {
        u8g2.clearBuffer();          // clear the internal memory
        u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
        u8g2.setCursor(0, 15); // spalte zeile
        u8g2.print("START in ");
        u8g2.print(i);
        u8g2.sendBuffer();
        delay(1000);
      }

      //delay(1500);


      // ----------------------------------------------  Vorbereitungen ------------------------------------

      stepper.enableOutputs();
      stepper.setCurrentPosition(0);
      stepper.setAcceleration(Acceleration); // ggf brauch man das hier nicht nochmal.
      stepper.setMaxSpeed(MaxSpeed); // Maximale Geschwindigkeit
      stepper.moveTo(LastSteps * 0.98); //  Fahre mit ca 95% des Etiketts mit Speed (etwas weniger, da nur bis Creepspeed)
      //stepper.moveTo(LastSteps); //  Fahre bis zu Schluss..

#ifdef isDebug
      Serial.println("Vorbereitungen");
      Serial.print("Acc: ");
      Serial.println(Acceleration);
      Serial.print("Speed: ");
      Serial.println(MaxSpeed);
      Serial.print("Length: ");
      Serial.println(Length);
      Serial.print("LastSteps: ");
      Serial.println(LastSteps);
#endif

      // Displayausgabe
      u8g2.clearBuffer();          // clear the internal memory
      u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
      u8g2.setCursor(0, 15); // spalte zeile
      u8g2.print("Etikettieren");
      u8g2.sendBuffer();

    }
  }


  // ---------------------------------------------- Volle Fahrt voraus (mit Beschleunigung) ------------------------------------

  // Beim ersten Etikettiervorgang nur Schleichfahrt!
  if (modus == START && LastSteps == 0)
  {
    modus = SCHLEICHEN;
#ifdef isDebug
    Serial.println("LastSteps=0, daher Schleichfahrt");
#endif
  }


  // Bei allen nachfolgenden Etikettiervorgängen mit Speed.
  else if (modus == START && LastSteps != 0) // Ab dem zweiten Etikettiervorgang steht fest, wieviele Schritte der Nema zuvor gebraucht hat.
  {
    stepper.run();
#ifdef isDebug
    Serial.print("NemaPosition: ");
    Serial.println(stepper.currentPosition());
    Serial.print("NemaSpeed: ");
    Serial.println(stepper.speed());
    Serial.print("Rotary: ");
    Serial.println(Position);
#endif

    if (Position >= Length)  // Stoppe, wenn Rotary Encoder Ziellänge erreicht. Sollte eigentlich nicht vorkommen,
      // ist aber theoretisch möglich wenn die Etikettenlänge deutlich verkürzt wird.
    {
      modus = ENDE;
    }

    // Wenn mehr als das halbe Etikett abgespult ist und die Geschwindigkeit während der Decelleration die Schleichfahrtgeschwindigkeit erreicht,
    // bleibe bei Schleichfahrt-geschwindigkeit bis zum Schluss.
    if (stepper.currentPosition() >= LastSteps / 2 && stepper.speed() <= CreepSpeed)
    {
      modus = SCHLEICHEN;

    }
  }


  // -------------------------------------------------  Schleichfahrt -----------------------------------------------

  else if (modus == SCHLEICHEN)
  {
    stepper.setSpeed(CreepSpeed);
    stepper.runSpeed(); // Schleichfahrt mit fester Geschwindigkeit
#ifdef isDebug
    Serial.println("SCHLEICHEN");
    Serial.print("Position: ");
    Serial.println(Position);
    Serial.print("Length: ");
    Serial.println(Length);
#endif


    // ----------------------------------------------  Etikettenlänge erreicht -> Stop -------------------------------------------


    if (Position >= Length)//  Stoppe, wenn Rotary Encoder Ziellänge erreicht
    {
      modus = ENDE;
    }
  }

  else if (modus == ENDE)
  {

#ifdef isDebug
    Serial.print (" STOP ");
    Serial.print (" Nema-Schritte: ");
    Serial.print (stepper.currentPosition());
    Serial.print (" Rotary: ");
    Serial.println (Position);
#endif

    LastSteps = stepper.currentPosition(); // Neuer Wert für gefahrene Schritte des Schrittmotors
    stepper.stop();
    stepper.disableOutputs();


    // ---------------------------------------------------------- Stempeln ----------------------------------------------------


#ifdef USE_STEMPEL
    //Displayausgabe
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
    u8g2.setCursor(0, 15); // x y
    u8g2.print("Stempeln");
    u8g2.sendBuffer();

    servo.easeTo(WinkelAktiv);
    delay(StempelPause);

    for (int i = 0; i <= StempelTrockenTupfen; i++) {
      servo.easeTo(WinkelAktiv - 10);
      delay(StempelPause);
      servo.easeTo(WinkelAktiv);
      delay(StempelPause);
    }
    
    servo.easeTo(WinkelRuhe);
           
#endif



           // Endeaktionen
           //--------------

           //Displayausgabe
           u8g2.clearBuffer();          // clear the internal memory
           u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
           u8g2.setCursor(0, 15); // spalte zeile
           u8g2.print("Glas entnehmen");
           u8g2.setCursor(0, 30); // x y
           u8g2.print("Mot:");
           u8g2.setCursor(50, 30);
           u8g2.print(stepper.currentPosition());
           u8g2.setCursor(0, 55); // x y
           u8g2.print("Enc:");
           u8g2.setCursor(50, 55);
           u8g2.print(Position);
           u8g2.sendBuffer();
           //delay(3000);
           // Mein Startsensor prellt. Deshalb 500ms zwischen zwei LOWs abwarten
           timeIdle = millis();
    while ( (millis() - timeIdle) < 1000) {
      if (readStartSensor() == LOW) {
          timeIdle = millis();
          //Serial.println("wait..pressed");
        }
        delay(1);
      }

    // Displayausgabe
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
    u8g2.setCursor(0, 15); // spalte zeile
    u8g2.print("Glas einlegen");
    u8g2.sendBuffer();
    delay(1000);
    modus = RUHE;
            //delay(1500);  // Zeit um Glas zu entnehmen

  } // Etikettieren ende

} // ende loop





//################################################# FUNKTIONEN #############################################

// MarcN
boolean readStartSensor()
{
  if (analogRead(start_pin) < highLowSchwelle) {
    return LOW;
  }
  else {
    return HIGH;
  }

}


void getPreferences(void) // Daten aus Eeprom lesen
{

  preferences.begin("EEPROM", false);       //Parameter aus eeprom lesen
  Length = preferences.getUInt("Length", 0);
  preferences_chksum = Length;

  preferences.end();

#ifdef isDebug
  Serial.println("Get Preferences:");
  Serial.print("Length = ");          Serial.println(Length);
#endif

  //Displayausgabe
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
  u8g2.setCursor(0, 15); // spalte zeile
  u8g2.print("Geladene");
  u8g2.setCursor(0, 30); // x y
  u8g2.print("Etikettenlänge:");
  u8g2.setCursor(0, 50);
  u8g2.print(Length);
  u8g2.sendBuffer();
  delay(3000);

}

int encoderSmallgetCounthalf() {
  return encoderSmall.getCount() / 2;
}


void setPreferences() // Daten in Eeprom schreiben
{

  long preferences_newchksum;
  //Length = Position;

  preferences_newchksum = Length;

  /* // TODO: Nur speichern, wenn sich Werte geändert haben.
    if ( preferences_newchksum == preferences_chksum ) {
    #ifdef isDebug
      Serial.println("Preferences unverändert");
    #endif
      getPreferences();


    #ifdef isDebug
    Serial.println("Get Preferences:");
    Serial.print("Length = ");          Serial.println(Length);

    #endif

      return;
    }

  */


  //Displayausgabe
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
  u8g2.setCursor(0, 15); // spalte zeile
  u8g2.print("speichere:");
  u8g2.setCursor(0, 30); // x y
  u8g2.print("Länge:");
  u8g2.setCursor(50, 30);
  u8g2.print(Length);

  u8g2.sendBuffer();
  delay(3000);


  preferences_chksum = preferences_newchksum;

  preferences.begin("EEPROM", false);
  preferences.putUInt("Length", Length);
  preferences.end();

#ifdef isDebug
  Serial.println("Set Preferences:");
  Serial.print("Length = ");          Serial.println(Length);
#endif


}


void print_logo() {
  const unsigned char logo_biene1[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xC0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0,
    0x00, 0xC0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x01, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x01, 0x60, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xF8, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x3F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x70, 0x00, 0xF0, 0xFF, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x80, 0xFF, 0xFF, 0x0F, 0x00, 0xFF, 0xFF, 0x80, 0xF1, 0x47, 0xF0, 0x07, 0x00, 0x3E, 0xE0, 0xFF, 0xFF, 0x07,
    0xF9, 0x07, 0x7E, 0x00, 0x00, 0x78, 0xF0, 0x03, 0xE0, 0x1F, 0xF8, 0x07, 0x1F, 0x00, 0x00, 0x70, 0x3C, 0x00, 0x00, 0xFE, 0x38, 0xC0, 0x03, 0x00,
    0x00, 0xF0, 0x0E, 0x00, 0x00, 0xF8, 0x03, 0xF8, 0x00, 0x00, 0x00, 0xE0, 0x06, 0x00, 0x00, 0xC0, 0x0F, 0x7C, 0x00, 0x00, 0x00, 0xE0, 0x06, 0x00,
    0x00, 0x00, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x70, 0x03, 0x00, 0x00, 0x00, 0xFC, 0x07, 0x00, 0x00, 0x00, 0x70, 0x03, 0x00, 0x00, 0x00, 0xF0, 0x03,
    0x00, 0x00, 0x00, 0x38, 0x03, 0x00, 0x00, 0x00, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x1C, 0x07, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0x07, 0x00, 0x00, 0x0F,
    0x0F, 0x00, 0x00, 0x78, 0x78, 0xE0, 0x3F, 0x00, 0xC0, 0x07, 0x3E, 0x00, 0x80, 0xFF, 0x3C, 0xC0, 0x7F, 0x00, 0xF0, 0x01, 0xFC, 0x00, 0xE0, 0xFF,
    0x1C, 0x80, 0xFF, 0x01, 0x7E, 0x00, 0xF0, 0xFF, 0xFF, 0x3F, 0x0E, 0x00, 0xFE, 0xFF, 0x0F, 0x00, 0xC0, 0xFF, 0xFF, 0x07, 0x0F, 0x00, 0xC0, 0x1F,
    0x00, 0x00, 0x00, 0xFC, 0x3F, 0x00, 0x07, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x80, 0x03, 0x80, 0x03, 0xE0, 0x00, 0x70, 0x00, 0x00, 0x00, 0xC0,
    0x01, 0xC0, 0x03, 0xC0, 0x01, 0xE0, 0x00, 0x00, 0x00, 0xE0, 0x00, 0xE0, 0x81, 0xC3, 0x01, 0xC0, 0x01, 0x00, 0x00, 0x70, 0x00, 0xE0, 0xF1, 0x8F,
    0x03, 0x80, 0x03, 0x00, 0x00, 0x38, 0x00, 0xF0, 0xFC, 0x9F, 0x07, 0x00, 0x07, 0x00, 0x00, 0x1C, 0x00, 0xF8, 0x1C, 0x1C, 0x0F, 0x00, 0x06, 0x00,
    0x00, 0x1C, 0x00, 0xFE, 0x00, 0x00, 0x1F, 0x00, 0x0C, 0x00, 0x00, 0x0E, 0x00, 0xF7, 0x00, 0x00, 0x7F, 0x00, 0x0C, 0x00, 0x00, 0x06, 0x80, 0x73,
    0x00, 0x00, 0xE6, 0x00, 0x0C, 0x00, 0x00, 0x07, 0xE0, 0x71, 0x00, 0x00, 0xC6, 0x03, 0x0C, 0x00, 0x00, 0x07, 0x70, 0x70, 0xF0, 0x0F, 0x86, 0x07,
    0x0C, 0x00, 0x00, 0x03, 0x3C, 0x70, 0xFC, 0x3F, 0x06, 0x1F, 0x0E, 0x00, 0x00, 0x03, 0x1E, 0x70, 0xFE, 0x3F, 0x06, 0xFC, 0x07, 0x00, 0x00, 0x87,
    0x0F, 0x70, 0x1E, 0x38, 0x06, 0xF0, 0x03, 0x00, 0x00, 0xFE, 0x03, 0xF0, 0x00, 0x00, 0x06, 0xC0, 0x00, 0x00, 0x00, 0xFC, 0x00, 0xF0, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x80, 0x03, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0x0F, 0x07, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xE0, 0xF1, 0x9F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3B, 0x9C, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0,
    0x07, 0xC0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7C, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x0D,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  u8g2.clearBuffer();
  u8g2.drawXBM(0, 0, 80, 64, logo_biene1);
  u8g2.setFont(u8g2_font_courB10_tf);
  u8g2.setCursor(85, 27);    u8g2.print("ETTI");
  u8g2.setCursor(75, 43);    u8g2.print("MANDL");
  u8g2.setFont(u8g2_font_courB08_tf);
  u8g2.setCursor(85, 64);    u8g2.print(versionTag);
  u8g2.sendBuffer();
  delay(2000);
  u8g2.clearBuffer();
}

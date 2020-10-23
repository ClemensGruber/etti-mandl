/* Ettimandl


  Copyright (C) 2020 by Martin Zahn
  2020-10-12 Martin Zahn    | initial version V0.1
                            Den Code kann jeder frei verwenden, ändern und hochladen wo er will,
                            solange er nicht seinen eigenen Namen drüber setzt, oder diesen kommerziell verwertet, beispielsweise
                            indem Etikettiermaschinen mit diesem Code versehen und verkauft werden.

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

  Start-Sensor        23
  Servo               19

*/



// Strom messen und einstellen: http://sturm.selfhost.eu/wordpress/pololu-schrittmotortreiber-einstellen/

// Blau DRV8825 0,1 Ohm mit mittl. SM max 1,7 A
// Max Strom: 1,7A x 0,7 = 1,19A
// Vref = Imax x (5 x Rs)
// Vref = 1,19A x 5 x 0,1Ohm=0,595 Volt

// Rot A4988: mit mittl. Motor max 1,7 A
//Vref = Imax x (8 x Rs) mit Rs zu 0,05Ohm.
// Vref = 1,19 x 8 x 0,05 = 0,476 Volt
// Einstellen zwischen Poti-Mitte und Masse



// #include <Arduino.h>
#include <AccelStepper.h> // für Schrittmotor. https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include <Preferences.h>  // für EEPROM
//#include <ESP32_Servo.h>  // https://github.com/jkb-git/ESP32Servo
#include <ServoEasing.h>

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




//
// Hier den Code auf die verwendete Hardware einstellen
//

#define USE_STEMPEL           // Datumsstempeleinheit vorhanden


// Ab hier nur verstellen wenn Du genau weisst, was Du tust!
//
#define isDebug             // serielle debug-Ausgabe aktivieren. 



// ** Definition der pins
// ----------------------

const byte enable_pin = 14;    // Schrittmotor Enable-Pin
const byte step_pin = 33;
const byte dir_pin = 32;

// Rotary Encoder (Inkrementaler Drehgeber für Etikettenposition)
// Grün = A-Phase, Weiß = B-Phase, Rot = Vcc-Leistung +, Schwarz = V0
const byte RotaryA  = 16;      // A-Phase
const byte RotaryB  = 17;      // B-Phase


// Rotary Encoder klein für Steuerung
//const int outputA  = 34; // CLK
//const int outputB  = 35; // DT
//const int outputSW = 13;


// Servo
const byte servo_pin = 19;     // Servo für Stempel


// Start-Sensor
const byte start_pin = 23;

// 2 Buttons - ggf. ersetzen durch kleinen Rotary
const byte button1 = 15;
const byte button2 = 2;
const byte buttonsave = 4;

// Initialisierung des Schrittmotors
AccelStepper stepper(2, step_pin, dir_pin); // Motortyp, STEP_PIN, DIR_PIN

// Daten in Eeprom lesen/speichern
Preferences preferences;

// Initialisierung des Servos für den Datumsstempel
#ifdef USE_STEMPEL
//Servo servo;
ServoEasing servo;
#endif




// Allgemeine Variablen

//int i;                             // allgemeine Zählvariable
volatile long temp, target, Position = 0;  // This variable will increase or decrease depending on the rotation of encoder
int Length;                        // Etikettenlänge in Schritten (Rotary) = Zielgröße
int LastSteps;                   // Benötigte Schritte des Schrittmotors für das letzte Etikett
int MaxSpeed = 400;                  // Schrittmotor Maximalgeschwindigkeit
int Acceleration = 100;              // Schrittmotor Beschleunigung (höher = schneller)
int CreepSpeed = 50;                 // Schrittmotor Langsamfahrt am Etikettende
int WinkelRuhe = 10;                   // Stempelposition in Ruhestellung
int WinkelAktiv = 150;                   // Stempelposition beim Stempeln
int EasingSpeed = 80;                   // Geschwindigkeit der Servo-Bewegung
long preferences_chksum;        // Checksumme, damit wir nicht sinnlos Prefs schreiben
enum MODUS {RUHE, START, SCHLEICHEN, ENDE};
byte modus = RUHE;



//################################################# SETUP #############################################


void setup()
{

  Serial.begin(115200);
  // while (!Serial) {
  //   }


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
  pinMode(RotaryA, INPUT_PULLUP); // internal pullup
  pinMode(RotaryB, INPUT_PULLUP); // internal pullup

  // Schrittmotor Enable
  pinMode(enable_pin, OUTPUT); // DRV8025 ENABLE // TODO: Prüfen ob erforderlich, da Accelstepper verwendet wird


  // Interrupts festlegen für großen Rotary Encoder
  //A rising pulse from encodenren activated ai0().
  attachInterrupt(digitalPinToInterrupt(RotaryA), ai0, RISING);
  //B rising pulse from encodenren activated ai1().
  attachInterrupt(digitalPinToInterrupt(RotaryB), ai1, RISING);


  // SCHRITTMOTORGRUNDEINSTELLUNGEN
  //--------------

  stepper.setEnablePin(enable_pin);
  stepper.setPinsInverted(false, false, true); // directioninvert, stepinvert, enableinvert

  stepper.setMaxSpeed(MaxSpeed); // Maximale Geschwindigkeit
  stepper.setAcceleration(Acceleration); // Beschleunigung
  stepper.setCurrentPosition(0);


#ifdef USE_STEMPEL

  servo.attach(servo_pin);
  servo.setSpeed(EasingSpeed);
  servo.setEasingType(EASE_CUBIC_IN_OUT);
  servo.write(WinkelRuhe);
#endif


  // Boot Screen
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  print_logo();
  delay(2000);
  u8g2.clearBuffer();          // clear the internal memory



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
    u8g2.setCursor(50, 15);
    u8g2.print("einstellen!");

    u8g2.sendBuffer();          // transfer internal memory to the display
#ifdef isDebug
    Serial.println("keine Etikettenlänge gespeichert");
#endif
    u8g2.clearBuffer();          // clear the internal memory
  }

  delay(2000);


} // Ende Setup


//################################################# LOOP  #############################################


void loop()
{

  // großen Rotary-Encoder erfassen (Position des Etiketts)
  if ( Position != temp ) {
    temp = Position;
    // Serial.print ("Rotary-Pos: ");
    // Serial.println (Position);
  }

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


  // ------------------------------------------------------------------------------------------------------------------
  // ----------------------------- ETIKETTENLÄNGE FESTLEGEN BZW. VOR- UND ZURÜCKSPULEN --------------------------------
  // ------------------------------------------------------------------------------------------------------------------



  // Buttons 1 und 2 bewegen jeweils den Schrittmotor um 4 Steps = 1 Schritt. (TODO: Ersetzen durch Rotary-Steuerung)


  // --------------------------------------------- ETIKETT VORSPULEN -------------------------------------------


  if (digitalRead(button1) == HIGH && modus == RUHE) // Etikett vorspulen
  {
    stepper.enableOutputs(); //enable pins
    target = stepper.currentPosition() + 4;
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

#ifdef isDebug
    Serial.print("Button1+ ");
    Serial.println(Position);
#endif
  }

  // --------------------------------------------- ETIKETT ZURÜCKSPULEN -------------------------------------------

  if (digitalRead(button2) == HIGH && modus == RUHE) // Etikett zurückspulen
  {
    stepper.enableOutputs(); //enable pins
    target = stepper.currentPosition() - 4;
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

#ifdef isDebug
    Serial.print("Button2- Schleife ");
    Serial.println(Position);
#endif
  }


  // ------------------------------------------------ LÄNGE SPEICHERN ----------------------------------------------

  // aktuelle Position als Etikettenlänge speichern
  if (digitalRead(buttonsave) == HIGH && modus == RUHE) // Länge speichern // ggf ersetzen durch RotarySW
  {
#ifdef isDebug
    Serial.println("Speichern");
#endif
    setPreferences();
  }

  // ------------------------------------------------------------------------------------------------------------------
  // ---------------------------------------------- Etikettiervorgang -------------------------------------------------
  // ------------------------------------------------------------------------------------------------------------------

  if (digitalRead(start_pin) == LOW && modus == RUHE)  //Sensor erkannt -> Start
  {

    Position = 0; // Position des Rotary Encoders auf 0 setzen
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
      //Displayausgabe
      u8g2.clearBuffer();          // clear the internal memory
      u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
      u8g2.setCursor(0, 15); // spalte zeile
      u8g2.print("START");
      u8g2.sendBuffer();


      modus = START;
      delay(1500);

      // ----------------------------------------------  Vorbereitungen ------------------------------------

      stepper.enableOutputs();
      stepper.setCurrentPosition(0);
      stepper.setAcceleration(Acceleration); // ggf brauch man das hier nicht nochmal.
      stepper.setMaxSpeed(MaxSpeed); // Maximale Geschwindigkeit
      stepper.moveTo(LastSteps * 0.9); //  Fahre mit ca 90% des Etiketts mit Speed (etwas weniger, da nur bis Creepspeed)

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


    // ---------------------------------------------------------- Stempeln ----------------------------------------------------


#ifdef USE_STEMPEL
    //Displayausgabe
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
    u8g2.setCursor(0, 15); // x y
    u8g2.print("Stempeln");
    u8g2.sendBuffer();

    //servo.write(WinkelAktiv);
    servo.startEaseTo(WinkelAktiv);
    delay(1500);
    //servo.write(WinkelRuhe);
    servo.startEaseTo(WinkelRuhe);
    delay(500);
#endif



    // Endeaktionen
    //--------------


    stepper.disableOutputs();

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
    delay(3000);


    while  (digitalRead(start_pin) == LOW) // Warten bis Glas entnommen wird.
    {
      delay(1);
    }


    // Displayausgabe
    u8g2.clearBuffer();          // clear the internal memory
    u8g2.setFont(u8g2_font_courB10_tf); // choose a suitable font
    u8g2.setCursor(0, 15); // spalte zeile
    u8g2.print("Glas einlegen");
    u8g2.sendBuffer();

    modus = RUHE;
    delay(1500);  // Zeit um Glas zu entnehmen

  } // Etikettieren ende

} // ende loop





//################################################# FUNKTIONEN #############################################



// Rotary Encoder Interrupt1
void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(RotaryB) == LOW) {
    Position++;
  } else {
    Position--;
  }
}

// Rotary Encoder Interrupt2
void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(RotaryA) == LOW) {
    Position--;
  } else {
    Position++;
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
  u8g2.print("geladen:");
  u8g2.setCursor(0, 30); // x y
  u8g2.print("Länge:");
  u8g2.setCursor(50, 30);
  u8g2.print(Length);
  u8g2.sendBuffer();
  delay(3000);

}



void setPreferences() // Daten in Eeprom schreiben
{

  long preferences_newchksum;
  Length = Position;

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
  u8g2.setCursor(85, 64);    u8g2.print("v.0.1");
  u8g2.sendBuffer();
  delay(2000);
  u8g2.clearBuffer();
}

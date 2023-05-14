



const char* ssid = "";
const char* password = "";

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
const int outputA  = 34; // CLK
const int outputB  = 35; // DT
const int rotaryButton = 13;


// Servo
const byte servo_pin = 19;     // Servo für Stempel


// Start-Sensor
//const byte start_pin = 23;
const byte start_pin = 25;     // MarcN

// 2 Buttons - ggf. ersetzen durch kleinen Rotary
const byte button1 = 15;
const byte button2 = 2;
const byte buttonsave = 4;

// Allgemeine Variablen

//int i;                                   // allgemeine Zählvariable
volatile long temp, target, Position = 0;  // This variable will increase or decrease depending on the rotation of encoder
int Length;                                // Etikettenlänge in Schritten (Rotary) = Zielgröße
int LengthOld;                             // alter Längenwert, um Rotary-Änderungen festestellen  zu können
int LastSteps;                             // Benötigte Schritte des Schrittmotors für das letzte Etikett
int manualStep = 50;                       // Anzahl Schritte je manuellem spulen 
int MaxSpeed = 12000;                        // Schrittmotor Maximalgeschwindigkeit (bis jetzt immer 5000, // 70000, 9000 auch ok)
int Acceleration = 3000;                    // Schrittmotor Beschleunigung (höher = schneller)
//int MaxSpeed = 9000;                        // Schrittmotor Maximalgeschwindigkeit
//int Acceleration = 7000;                    // Schrittmotor Beschleunigung (höher = schneller)
int CreepSpeed = 4000;                      // Schrittmotor Langsamfahrt am Etikettende
int WinkelRuhe = 3;                       // Stempelposition in Ruhestellung
int WinkelAktiv = 71;                     // Stempelposition beim Stempeln
int StempelPause = 50;                     // Zeit für das Aufdrücken des Stempels
int StempelTrockenTupfen = 0;             // Wiederholungen zum trockentupfen 
int ServoSpeed = 70;                      // Geschwindigkeit des Servo-Arms 
int StartLatenz = 1;                      // CountDown ins Sekunden, bevor gestartet wird

long preferences_chksum;                   // Checksumme, damit wir nicht sinnlos Prefs schreiben
enum MODUS {RUHE, START, SCHLEICHEN, ENDE, BOOT, CONFIG, NEWDATE };
byte modus = BOOT;
unsigned long timeIdle;
unsigned long timePressed; 
boolean glasEntfernt;

//const int highLowSchwelle = 1200;           // Grenze des StartSensors zwischen HIGH / LOW 
const int highLowSchwelle = 100;           // Grenze des StartSensors zwischen HIGH / LOW 

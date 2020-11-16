



const char* ssid = "xxx";
const char* password = "yyy";

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
int LastSteps;                             // Benötigte Schritte des Schrittmotors für das letzte Etikett
int MaxSpeed = 4500;                        // Schrittmotor Maximalgeschwindigkeit
int Acceleration = 2300;                    // Schrittmotor Beschleunigung (höher = schneller)
int CreepSpeed = 2000;                      // Schrittmotor Langsamfahrt am Etikettende
int WinkelRuhe = 10;                       // Stempelposition in Ruhestellung
int WinkelAktiv = 110;                     // Stempelposition beim Stempeln
int ServoSpeed = 100;                      // Geschwindigkeit des Servo-Arms 
long preferences_chksum;                   // Checksumme, damit wir nicht sinnlos Prefs schreiben
enum MODUS {RUHE, START, SCHLEICHEN, ENDE};
byte modus = RUHE;
unsigned long timeIdle;

const int highLowSchwelle = 1200;           // Grenze des StartSensors zwischen HIGH / LOW 

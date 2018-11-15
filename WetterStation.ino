/*
 Name:		WetterStation.ino
 Created:	02.11.2018 18:37:17
 Author:	HN und GS
*/

//Debug Modus
#define DEBUG 1 //auskommentieren f�r reale Nutzung

//EEPROM initalisieren
#include <EEPROM.h>

//Display - Bibliotheken
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Display auf Port initalisieren
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Deklaration der PINS
#define FeuchtLuftPD  PD3 //PIN fuer den Feuchtigkeits- und Temp- Sensor
#define dht22Sensor   PD4 //PIN fuer DHT22-Sensor (Feuchtigkeit+Temp)
#define ledPD     PB5 //PIN fuer LED
#define tasterSensor  PB0 //PIN fuer Taster zum Umschalten der Sensoren (PIN 8)
#define tasterVerlauf PB1 //PIN fuer Taster zum Abfragen der historischen Daten (PIN 9)

// Globale Variablen festlegen
double wetterSensor[3]; //Sensor fuer Temp und Feuchtigkeit (0. Wert: Zeitstempel (ab Start vom Arduino in �s), 1.Wert: Feucht, 2.Wert: Temp
unsigned int eepromMaximum = 0;
int32_t abrufIntervallSekunden = 5;
int32_t letzteMesszeitpunkt = 0;

bool tasterSens = 1, tasterVerl = 1;

void setup() {
	//LCD Dispay initialisieren
	lcd.begin(16, 2);
	lcd.clear();

  //maximalen Rom ermitteln
  eepromMaximum = E2END + 1;
  
  #ifdef DEBUG
    Serial.begin(9600);

    Serial.println("\n\nAusgabe am Monitor");
    Serial.println("------------------");

    Serial.print("eepromMaximum = ");
    Serial.println(eepromMaximum);
    Serial.println("------------------");
  #endif

	DDRD |= (1 << FeuchtLuftPD);  // als Ausgang setzen
	DDRD |= (1 << dht22Sensor); // als Ausgang setzen
	DDRB |= (1 << ledPD);     // als Ausgang
	DDRB &= ~(1 << tasterSensor); // als Eingang setzen - LOW-aktiv 
	DDRB &= ~(1 << tasterVerlauf);  // als Eingang setzen - LOW-aktiv

	DDRB |= (1 << ledPD);			// als Ausgang
	PORTD &= ~(1 << ledPD);	// LED auf LOW setzen

}

void loop() {
	int8_t fehler = 0;
	// abrufen der Daten
	fehler = feuchtLuftAbfrage(FeuchtLuftPD);

	if (fehler == 0) {
		#ifdef DEBUG
		// Anzeige auf Seriellen Monitor
		Serial.print("Sensor-Daten:\nZeit: ");
		Serial.print(wetterSensor[0] / 1000);
		Serial.print("ms, Temperatur: ");
		Serial.print(wetterSensor[1]);
		Serial.print(", Feuchtigkeit: ");
		Serial.print(wetterSensor[2]);
		Serial.println("\n\n");
		#endif

		// Anzeige auf Display
		lcdAnzeige();
	}
	tasterAbfrage();
}

int feuchtLuftAbfrage(int pin) {
	// Zeitintervall der Abfrage ueberpruefen
	if ((millis() - letzteMesszeitpunkt) < abrufIntervallSekunden*1000) {
		if (letzteMesszeitpunkt != 0) {
			return -99;  // nach Einschalten soll gemessen werden
		}
	}
	letzteMesszeitpunkt = millis();

	//Variablen

	uint8_t wert[5];		//Bytes des Sensorwertes
	uint8_t bitwert = 7;	//Bit der einzelnen Bytes der Sensorwertes

	// Hilfsvariablen
	uint16_t timercounter = 100;
	float t = 0;			//Zeitabfrage
	int8_t i = 0, j = 0;	//Schleifenvariablen
	int16_t sum;			//Summe fuer Paritaetsbit
	uint8_t sensorBitWert;	//Variable zum Festhalten des aktuellen Bits am Sensor
	int16_t fehler[6];    // fuer Fehlerkontrolle

#ifdef DEBUG
	uint16_t zeitBit[40];
	uint8_t zwischenzeit[41];

	// entsprechende Werte vorbelegen
	for (i = 0; i < 40; i++) {
		zeitBit[i] = 255;
		zwischenzeit[i] = 0;
	}
	zwischenzeit[40] = 0;

	Serial.print("Sensor-Abfrage ");
	Serial.println(pin);
#endif

	//Puffer leeren
	for (i = 0; i < 5; i++) {
		wert[i] = 0;
		fehler[i] = 0;
	}
	fehler[5] = 0;


	// Sensor starten - PIN als Ausgang verwenden (laut Datenblatt)
	DDRD |= (1 << FeuchtLuftPD);	// als Ausgang setzen
	PORTD &= ~(1 << FeuchtLuftPD);	// auf LOW setzen (invertiere den Port und setze mit Register PORTD zusammen)
	delay(18);
	PORTD |= (1 << FeuchtLuftPD);	// auf HIGH setzen
	delayMicroseconds(40);
	DDRD &= ~(1 << FeuchtLuftPD);	// PIN als Eingang setzen
	//PORTD &= ~(1 << FeuchtLuftPD);	// Pull-Up-Widerstand am Arduino nicht setzen, da am Sensor vorhanden (invertiere den Port und setze mit Register PORTD zusammen)


	  //Sensorantwort auswerten
	for (i = 0; i <= 1; i++) {	//Antwort vom Sensor: 1. Bit ist LOW, 2. Bit ist HIGH
		timercounter = 100;
		t = micros();
		delayMicroseconds(5);	//Umschaltung zwischen Aus- und Eingang; LOW-Setzen vom Sensor
		sensorBitWert = ((PIND & (1 << pin)) >> pin);
		while (sensorBitWert == i) { //ist Sensor auf LOW-Signal?
			sensorBitWert = ((PIND & (1 << pin)) >> pin);
			delayMicroseconds(3);
			if (timercounter-- == 0) {
				fehler[0] = -1;
				break;
			}
		}
		zwischenzeit[0] = micros() - t;
	}

	// Datenbits auswerten
	for (i = 0; i < 5; i++) {
		for (j = 7; j >= 0; j--) {
			//Pausenzeit zwischen den Datenbits ist 50 Microsekunden
			timercounter == 10000;
			sensorBitWert = ((PIND & (1 << pin)) >> pin);
			while (sensorBitWert == 0) { //ist Sensor auf LOW-Signal?
				sensorBitWert = (PIND & (1 << pin));
				if (timercounter-- == 0) {
					fehler[2] = -4;
					break;
				}
			}
			timercounter = 10000;
			t = micros();
			sensorBitWert = ((PIND & (1 << pin)) >> pin);
			while (sensorBitWert != 0) {		//ist Sensor auf HIGH-Signal?
				sensorBitWert = ((PIND & (1 << pin)) >> pin);
				if (timercounter-- == 0) {
					fehler[3] = -4;
					break;
				}
			}

			// gesendetes BIT HIGH?
			if ((micros() - t) >= 40) {
				wert[i] |= (1 << j);
			}
		}
	}

	//Paritaetspruefung
	sum = wert[0] + wert[1] + wert[2] + wert[3];
	if (sum != wert[4]) {
		fehler[4] = -5;
	}

	fehler[5] = fehler[0] + fehler[1] + fehler[2] + fehler[4];

	DDRD |= (1 << FeuchtLuftPD);  // als Ausgang setzen

#ifdef DEBUG
	if (sum != wert[4]) {
		Serial.println("Paritaetspruefung fehlgeschlagen");
	}
	// Anzeige zur Fehlereingrenzung)
	for (i = 0; i < 6; i++) {
		Serial.print("Fehler");
		Serial.print(i);
		Serial.print(" ");
		Serial.print(fehler[i]);
		Serial.println(", ");
	}
	Serial.print("SensorBitWert: ");
	Serial.println(sensorBitWert);
	for (i = 0; i < 5; i++) {
		Serial.print("Bit");
		Serial.print(i);
		Serial.print(" ");
		Serial.print(wert[i]);
		Serial.println(", ");
	}
	Serial.print("\nParitaetssumme: ");
	Serial.println(sum);
	Serial.println("\n\n");
#endif


	if (fehler[5] != 0) {
		return fehler[5];
	}

	wetterSensor[0] = micros();
	wetterSensor[1] = wert[2];
	wetterSensor[2] = wert[0];

	return fehler[5];
}

void lcdAnzeige() {
	lcd.clear();

	//Werte auf LCD drucken
	lcd.setCursor(0, 0);
	lcd.print("Luftf.: ");
	lcd.setCursor(10, 0);
	lcd.print(wetterSensor[2]);
	lcd.setCursor(0, 1);
	lcd.print("Temp.: ");
	lcd.setCursor(10, 1);
	lcd.print(wetterSensor[1]);
}
void tasterAbfrage() {
  uint8_t i = 0;
  for (i = 0; i <= 1; i++) {
    tasterSens = PINB & (1 << tasterSensor);
    delay(1);
  }
  for (i = 0; i <= 1; i++) {
	  tasterVerl = PINB & (1 << tasterVerlauf);
	  delay(1);
  }

  if ((tasterSens == 1) | (tasterVerl == 1)) {  //taster wurde gedrueckt
    PORTB ^= (1 << ledPD);  // LED toggelnd
  }
}
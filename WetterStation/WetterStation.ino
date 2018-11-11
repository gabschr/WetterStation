/*
 Name:		WetterStation.ino
 Created:	02.11.2018 18:37:17
 Author:	HN und GS
*/

//Debug Modus
#define DEBUG 1 //auskommentieren für reale Nutzung


//Display - Bibliotheken
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Display auf Port initalisieren
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Deklaration der PINS
#define FeuchtLuftPD	PD3	//PIN fuer den Feuchtigkeits- und Temp- Sensor
#define ledPD			PB5 //PIN fuer LED

// Globale Variablen festlegen
double wetterSensor[3]; //Sensor fuer Temp und Feuchtigkeit (0. Wert: Zeitstempel (ab Start vom Arduino in µs), 1.Wert: Feucht, 2.Wert: Temp

int32_t abrufIntervallSekunden = 5;
int32_t letzteMesszeitpunkt = 0;

void setup() {
	//LCD Dispay initialisieren
	lcd.begin(16, 2);
	lcd.clear();

  #ifdef DEBUG
    Serial.begin(9600);
  	Serial.println("\n\nAusgabe am Monitor");
  	Serial.println("------------------");
  #endif

	DDRD |= (1 << FeuchtLuftPD);	// als Ausgang setzen (ODER-Verknuepfung mit PIN)
	DDRB |= (1 << ledPD);			// als Ausgang
}

void loop() {
	int8_t fehler = 0;
	// LED
	DDRB |= (1 << ledPD);			// als Ausgang
	PORTD &= ~(1 << ledPD);	// LED auf LOW setzen

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
	uint16_t counter = 0;	// nur DEBUG
	float t = 0;			//Zeitabfrage
	int8_t i = 0, j = 0;	//Schleifenvariablen
	uint8_t sensorBitWert;	//Variable zum Festhalten des aktuellen Bits am Sensor

	// Fehlersuche
	int fehler = 0, counter1 = 0;
	int pausenzeit = 0;

	// leere Puffer
	for (i = 0; i < 5; i++) {
		wert[i] = 0;
	}

  #ifdef DEBUG
	  Serial.print("Sensor-Abfrage ");
	  Serial.println(pin);
  #endif
	
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
		counter = 0;
		t = micros();
		delayMicroseconds(5);	//Umschaltung zwischen Aus- und Eingang; LOW-Setzen vom Sensor
		sensorBitWert = ((PIND & (1 << pin)) >> pin);
		while (sensorBitWert == i) { //ist Sensor auf LOW-Signal?
			sensorBitWert = ((PIND & (1 << pin)) >> pin);
			counter++;
			delayMicroseconds(3);
			if (timercounter-- == 0) {
				fehler = -1;
				break;
			}
		}
	}

	// Datenbits auswerten
	for (i = 0; i < 5; i++) {
		for (j = 7; j >= 0; j--) {
			//Pausenzeit zwischen den Datenbits ist 50µs
			timercounter == 10000;
			sensorBitWert = ((PIND & (1 << pin)) >> pin);
			while (sensorBitWert == 0) { //ist Sensor auf LOW-Signal?
				sensorBitWert = (PIND & (1 << pin));
				if (timercounter-- == 0) {
					fehler = -3;
					break;
				}
			}

			timercounter = 10000;
			t = micros();
			sensorBitWert = ((PIND & (1 << pin)) >> pin);
			while (sensorBitWert != 0) {		//ist Sensor auf HIGH-Signal?
				sensorBitWert = ((PIND & (1 << pin)) >> pin);
				if (timercounter-- == 0) {
					fehler = -4;
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
	int sum = wert[0] + wert[1] + wert[2] + wert[3];
  
  	if (sum != wert[4]) {
		#ifdef DEBUG
  		Serial.println("Paritaetspruefung fehlgeschlagen");
		#endif
		fehler = -5;
  	}

#ifdef DEBUG
	// Anzeige zur Fehlereingrenzung
	Serial.print("counter: ");
	Serial.print(counter);
	Serial.print(", fehler: ");
	Serial.print(fehler);
	Serial.print(", SensorBitWert: ");
	Serial.println(sensorBitWert);
	for (i = 0; i < 5; i++) {
		Serial.print(", Bit ");
		Serial.print(i);
		Serial.print(" ");
		Serial.print(wert[i]);
		Serial.print(", ");
	}
	Serial.print("\nParitaetssumme: ");
	Serial.println(sum);
	Serial.print("\nZeitverbrauch: ");
	Serial.print(millis() - letzteMesszeitpunkt);
	Serial.println("\n\n");
#endif

	if (fehler != 0) {
		return fehler;
	}

	wetterSensor[0] = micros();
	wetterSensor[1] = wert[2];
	wetterSensor[2] = wert[0];

	return fehler;
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

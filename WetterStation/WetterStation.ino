/*
 Name:		WetterStation.ino
 Created:	02.11.2018 18:37:17
 Author:	HN und GS
*/

//Display - Bibliotheken
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//Display auf Port initalisieren
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Deklaration der PINS
#define FeuchtLuftPD	PD3	//PIN f�r den Feuchtigkeits- und Temp- Sensor

// f�r die Daten Feuchtigkeit und Temperatur
float wetterSensor[3]; //Sensor fuer Temp und Feuchtigkeit (0. Wert: Zeitstempel (ab Start vom Arduino in �s), 1.Wert: Feucht, 2.Wert: Temp 

void setup() {
	//LCD Dispay initialisieren
	lcd.begin(16, 2);
	lcd.clear();

	//Temperatur && Feuchtigkeit
	Serial.begin(9600);
	Serial.println("\n\nAusgabe am Monitor");
	Serial.println("------------------");

	DDRD |= (1 << FeuchtLuftPD);   // als Ausgang setzen (ODER-Verknuepfung mit PIN)
}

void loop() {
	// abrufen der Daten
	feuchtLuftAbfrage(FeuchtLuftPD);

	// Anzeige auf Seriellen Monitor
	Serial.print("Sensor-Daten:\nZeit: ");
	Serial.print(wetterSensor[0] / 1000);
	Serial.print("ms, Temperatur: ");
	Serial.print(wetterSensor[1]);
	Serial.print(", Feuchtigkeit: ");
	Serial.print(wetterSensor[2]);
	Serial.println("\n\n");

	// Anzeige auf Display
	lcdAnzeige();



}

int feuchtLuftAbfrage(int pin) {
	//Variablen
	int wert[5];		//Bytes des Sensorwertes
	int bitwert = 7;	//Bit der einzelnen Bytes der Sensorwertes

	// Hilfsvariablen
	int timercounter = 10000;
	int counter = 0;
	float t = 0, t1 = 0;	//Zeitabfrage
	int i = 0, j = 0;		//Schleifenvariablen
	int sensorBitWert;		//Sensorwert

	// Fehlersuche
	int fehler = 0, counter1 = 0;
	int pausenzeit = 0;

	// leere Puffer
	for (i = 0; i < 5; i++) {
		wert[i] = 0;
	}

	Serial.print("Sensor-Abfrage ");
	Serial.println(pin);

	// Sensor starten - PIN als Ausgang verwenden (laut Datenblatt)
	DDRD |= (1 << FeuchtLuftPD);	// als Ausgang setzen
	PORTD &= ~(1 << FeuchtLuftPD);	// auf LOW setzen (invertiere den Port und setze mit Register PORTD zusammen)
	delay(18);
	PORTD |= (1 << FeuchtLuftPD);	// auf HIGH setzen
	delayMicroseconds(40);
	DDRD &= ~(1 << FeuchtLuftPD);	// PIN als Eingang setzen
	//PORTD &= ~(1 << FeuchtLuftPD);	// Pull-Up-Widerstand am Arduino nicht setzen, da am Sensor vorhanden (invertiere den Port und setze mit Register PORTD zusammen)

	//pinMode(pin, INPUT);

	//Sensor auf Antwort abfragen
	t = micros();

	sensorBitWert = (pin & (1 << pin));

	// while ((pin & (1 << pin)) == 0) { //ist Sensor auf LOW-Signal?
	while ((digitalRead(pin)) == LOW) {	//ist Sensor auf LOW-Signal?
		counter1++;
		delayMicroseconds(1);
		if (timercounter-- == 0) {
			fehler = -1;
			break;
			//return fehler;
		}
	}
	counter1 = micros() - t;

	timercounter = 10000;
	counter = 0;

	t1 = micros();
	t = micros();
	while ((digitalRead(pin)) == HIGH) {		//ist Sensor auf HIGH-Signal?
		counter++;
		if (timercounter-- == 0) {
			fehler = -2;
			break;
			//return fehler;
		}
	}
	counter += micros() - t;

	for (i = 0; i < 5; i++) {
		for (j = 7; j >= 0; j--) {
			timercounter == 10000;
			t = micros();
			//Pausenzeit ist 50�s
			while ((digitalRead(pin)) == LOW) {		//ist Sensor auf LOW-Signal?
				if (timercounter-- == 0) {
					fehler = -3;
					break;
					//return fehler;
				}
			}
			pausenzeit = micros() - t;

			t = micros();
			while ((digitalRead(pin)) == HIGH) {		//ist Sensor auf LOW-Signal?
				if (timercounter-- == 0) {
					fehler = -3;
					break;
					//return fehler;
				}
			}

			// gesendetes BIT HIGH?
			if ((micros() - t) >= 60) {
				wert[i] |= (1 << j);
			}
		}
	}

	//Paritaetspruefung
	int sum = wert[0] + wert[1] + wert[2] + wert[3];
	if (sum != wert[4]) {
		Serial.println("Paritaetspruefung fehlgeschlagen");
		//return -5;
	}
	wetterSensor[0] = micros();
	wetterSensor[1] = wert[2];
	wetterSensor[2] = wert[0];

	Serial.print("counter: ");
	Serial.print(counter);
	Serial.print("counter1: ");
	Serial.print(counter1);
	Serial.print(", fehler: ");
	Serial.print(fehler);
	Serial.print(", Pausenzeit: ");
	Serial.print(pausenzeit);
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
	Serial.print((micros() - t1) / 1000);

	Serial.println("\n\n");
	delay(2000);		//zwischen den Abfragen etwas Zeit lassen

	return 0;
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
	lcd.print(wetterSensor[0]);
}
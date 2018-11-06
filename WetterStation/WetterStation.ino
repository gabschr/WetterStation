/*
 Name:		WetterStation.ino
 Created:	02.11.2018 18:37:17
 Author:	HN und GS
*/

//zum Testen des Sensors
#include <dht11.h>
dht11 DHT11;

#define ledDDR   DDRB  // Port Group B DDR - als Ausgang oder Eingang definieren
#define ledPORT  PORTB // Portgrupppe B PORTB
#define ledPIN   PD5   // LED-PIN 5 an PortGroup B (Port 13)

#define tasterDDR   DDRD   // Port Group D
#define tasterPIN   PIND   // PIN GROUP D

#define FeuchtLuftPD	PD3	//PIN für den Feuchtigkeits- und Temp- Sensor

// Taster
bool buttonState = 0;  //Tasterstatus feststellen
int counter = 0;
bool buttonStateBevor = LOW; //Tastenstatus vorher feststellen

// für die Daten Feuchtigkeit und Temperatur
float Feucht = 0, Luft = 0;

void setup() {
	ledDDR = (1 << PB5);    // LED als Ausgang definiert
	ledPORT = (0 << PB5);   // LED auf LOW setzen

	tasterDDR = (0 << PB2);    // taster als Eingang definieren
   
	Serial.begin(9600);
	Serial.println("/n/n Ausgabe am Monitor");
	Serial.println("------------------");
}

void loop() {

	buttonState = (tasterPIN & (1 << PD2));  // Taster abfragen

	if (buttonState == LOW) {       // Taster wurde gedrückt - Masse wurde angelegt (0)
		ledPORT = (1 << ledPIN);          // LED auf High setzen

		if (buttonStateBevor == HIGH) {
			counter += 1;
			buttonStateBevor = LOW;
			Serial.print("COUNTER ");
			Serial.println(counter);
		}
	}

	if (buttonState != 0) {      // Taster wurde nicht gedrückt
		ledPORT = (0 << ledPIN);   // LED auf LOW setzen
		if (buttonStateBevor == LOW) {	// Prellen des Schalters zaehlen
			buttonStateBevor = HIGH;
		}
	}

	delay(1000);
	
	Serial.println("\n Mit Bibliothek:");
	
	DHT11Abfrage(FeuchtLuftPD);
	Serial.println();

	TempFeuchtAbfr(FeuchtLuftPD);
	Serial.print("Feuchtigkeit: ");
	Serial.print(Feucht);
	Serial.print("%, Lufttemperatur: ");
	Serial.print(Luft);
	Serial.println("°C");


	// fröhlich blinken lassen :)
	int counter = 0;

	for (counter = 1; counter <= 10; counter++) {
		// LED blinken lassen
		ledPORT = (1 << ledPIN);   // LED auf HIGH setzen
		delay(100);
		ledPORT = (0 << ledPIN);   // LED auf LOW setzen
		delay(200);
	}


}

void TempFeuchtAbfr(int pin) {
	#define FeuchtLuftDDR   DDRD   // Port Group D, wo auch Feuchtigkeitssensor dran hängt
	#define FeuchtLuftPORT  PORTD  // Portgruppe D PORTD 
	#define FeuchtLuftPIN   PIND   // PIN GROUP D

	uint32_t timeout = 0;
	uint8_t counter = 0;
	uint8_t abfragewert = 0;
	uint8_t i = 0;	//Laufzeitvariable
	uint8_t wertSensor[5];	//Vektor für die einzelnen Byte-Werte des Sensores
	uint8_t wertBit = 7;

	pin = PD3;

	//wertSensor auf 0 setzen:
	for (i = 0; i < 5; i++) {
		wertSensor[i] = 0;
	}

	// Sensorabfrage aktivieren, gemäß Datenblatt: Arduino muss 18ms LOW setzen, danach 40µs HIGH
	Serial.println("Ich bin bei der Abfrage");
	FeuchtLuftDDR = (1 << pin);   // als Ausgang setzen
	FeuchtLuftPORT = (0 << pin);  // auf LOW setzen
	delay(18);
	FeuchtLuftPORT = (1 << pin);  // auf HIGH setzen
	delayMicroseconds(40);
	FeuchtLuftDDR = (0 << pin);   // als Eingang setzen  FeuchtLuftPD

	// Antwort des Sensors abfragen (80µs LOW, dann 80µs HIGH)
	for (i = 0; i <= 1; i++) {
		timeout = 100000;
		counter = 0;

		/*
		while (timeout > 0){
			abfragewert = ((FeuchtLuftPIN & (1 << pin)) >> pin);	//High oder Low am Eingang des Sensors; BitStelle des Sensors zurück, damit 0 oder 1

			if (abfragewert == i) {  //Low am Eingang des Sensors?
				counter--;
				if (counter == 0) {
					break;
				}
			}
			delayMicroseconds (1);
			timeout--;
		}
		*/

		while ((FeuchtLuftPIN & (1 << pin)) == i) {	//((FeuchtLuftPIN & (1 << pin)) >> pin)
			delayMicroseconds(1);
			counter++;
			timeout--;
			// abfragewert = ((FeuchtLuftPIN & (1 << pin)) >> pin);
			/*
			if (timeout == 0) {
				Serial.println("Fehler beim Abfragen");
				Serial.print("abfragewert: ");
				Serial.print(((FeuchtLuftPIN & (1 << pin)));
				Serial.print(", counter: ");
				Serial.print(counter);
				Serial.print(", i= ");
				Serial.print(i);
				Serial.print(", timeout= ");
				Serial.println(timeout);
				return;
			}
			*/
		}
		if (counter > 90) {
			Serial.println("Fehler beim Abfragen nach Auswertung des Counters");

				Serial.print("abfragewert: ");
				Serial.print(abfragewert);
				Serial.print(", counter: ");
				Serial.print(counter);
				Serial.print(", i= ");
				Serial.println(i);

				Feucht = 99;
				Luft = 99;
			return;
		}
	}
	
	/*
	ab jetzt kommen die Daten
	1. Byte: Relative Luftfeuchtigkeit in %
	2. Byte: Nachkommastelle Relative Luftfeuchtigkeit in % (bei DHT11 immer 0)
	3. Byte: Temperatur in Celsiusgrad
	4. Byte: Nachkommastellen der Temperatur (DHT11 immer 0)
	5. Byte: Summe der ersten 4 Bytes (Kontrolle)
	
	Bit- Aufbau (Länge ist entscheidend):
	- 50µs LOW: Anfang des Bits
	- 26 bis 28 µs HIGH: LOW-Signal
		oder: 70µs HIGH: HIGH-Signal
	*/

	for (i = 0; i < 5; i++){
		timeout = 90;
		counter = 0;

		for (wertBit = 7; wertBit = 0; wertBit--){
			/*
			while (timeout > 0) {
				// abfragewert = (FeuchtLuftPIN & (1 << pin) >> pin);	//High oder Low am Eingang des Sensors; BitStelle des Sensors zurück, damit 0 oder 1
				if (((FeuchtLuftPIN & (1 << pin)) >> pin) == 0) {  //Low am Eingang des Sensors?
					counter--;
					if (counter == 0) {
						break;
					}
				}
				delayMicroseconds(1);
				timeout--;
			}
			if (counter > 0) {
				Serial.println("Fehler beim Auslesen");
				Feucht = 99;
				Luft = 99;
				return;
			}
			*/

			while (((FeuchtLuftPIN & (1 << pin)) >> pin) == 0) {
				counter++;
				timeout--;
				delayMicroseconds(1);
				if (timeout == 0) {
					Serial.println("Fehler beim Auslesen");
					break;
				}
			}

			if (counter < 80) {
				Serial.println("Fehler beim Auslesen counter");
				break;
			}


			counter = 0;
			timeout = 80;
			while (((FeuchtLuftPIN & (1 << pin)) >> pin) == 1) {		//so lange HIGH am Eingang anliegt, zähle die Mikrosekunden
				counter++;
				timeout--;
				delayMicroseconds(1);
				if (timeout == 0) {
					Serial.println("Fehler beim Auslesen");
					break;
				}
			}

			if (counter <= 28) {
				if (counter <= 25) {	//LOW-Signal am aktuellen BIT
					Serial.println("Fehler beim Auslesen");
					break;
				}
				wertSensor[i] |= (0 << wertBit);
				}
			if (counter >= 70) {
				wertSensor[i] |= (1 << wertBit);
			}
		}
		/*
		Serial.print("wertSensor ");
		Serial.println(wertSensor[i]);
		*/
	}

	// Paritätsbit-Prüufung
	if (wertSensor[4] != wertSensor[0] + wertSensor[1] + wertSensor[2] + wertSensor[3]) {
		Serial.println("Paritätsprüfung fehlgeschlagen");
	}

	Serial.println(wertSensor[0]);
	Serial.println(wertSensor[2]);

	Feucht = wertSensor[1];
	Feucht = Feucht / 100 + wertSensor[0];

	Luft = wertSensor[3];
	Luft = Luft / 100 + wertSensor[2];

	Serial.print("Feuchtigkeit: ");
	Serial.print(Feucht);
	Serial.print("%, Lufttemperatur: ");
	Serial.print(Luft);
	Serial.println("°C");

}

void DHT11Abfrage(int pin) {
	int chk = DHT11.read(pin);
	Serial.print("Read sensor: ");
	switch (chk)
	{
	case 0: Serial.println("OK"); break;
	case -1: Serial.println("Checksum error"); break;
	case -2: Serial.println("Time out error"); break;
	default: Serial.println("Unknown error"); break;
	}

	Serial.print("Humidity (%): ");
	Serial.println((float)DHT11.humidity, 2);

	Serial.print("Temperature (oC): ");
	Serial.println((float)DHT11.temperature, 2);
}
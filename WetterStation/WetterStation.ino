/*
 Name:		WetterStation.ino
 Created:	02.11.2018 18:37:17
 Author:	HN und GS
*/

// #include <dht11.h>

#define ledDDR   DDRB  // Port Group B DDR - als Ausgang oder Eingang definieren
#define ledPORT  PORTB // Portgrupppe B PORTB
#define ledPIN   PB5   // LED-PIN 5 an PortGroup B (Port 13)

#define tasterDDR   DDRD   // Port Group D
#define tasterPIN   PIND   // PIN GROUP D

#define FeuchtLuftDDR   DDRD   // Port Group D, wo auch Feuchtigkeitssensor dran hängt
#define FeuchtLuftPORT  PORTD  // Portgruppe D PORTD 
#define FeuchtLuftPD    PD3    // PD für Feuchtigkeitssensor
#define FeuchtLuftPIN   PIND   // PIN GROUP D

// Bibliothek zum Auslesen 
#define DHT11PIN 3
dht11 DHT11;

/*
  taster = (0 << PD2); //als Eingang definieren, Pin ist PortD, PIN 2
  ledPORT = (1 << PB5); //als Ausgang definieren, PIN ist PortB, PIN 5 (Digital 13)
*/

bool buttonState = 0;  //Tasterstatus feststellen
int counter = 0;
bool buttonStateBevor = LOW; //Tastenstatus vorher feststellen

int FeuchtLuft = 1;
int FeuchtLuftVorher = 0;

void setup() {
	// put your setup code here, to run once:

	ledDDR = (1 << PB5);    // LED als Ausgang definiert
	ledPORT = (0 << PB5);   // LED auf LOW setzen

	tasterDDR = (0 << PB2);    // taster als Eingang definieren
   /*
	 pinMode(ledPORT, OUTPUT);
	 pinMode(taster, INPUT_PULLUP);
   */

	Serial.begin(9600);
	Serial.println("Ausgabe am Monitor");
	Serial.println("------------------");
	// digitalWrite(ledPORT, LOW);

}

void loop() {
	// put your main code here, to run repeatedly:

	buttonState = (tasterPIN & (1 << PD2));  // Taster abfragen

				   // buttonState = digitalRead(taster);

	if (buttonState == LOW) {       // Taster wurde gedrückt - Masse wurde angelegt (0)
		ledPORT = (1 << ledPIN);          // LED auf High setzen
		// digitalWrite(ledPORT, HIGH);

		if (buttonStateBevor == HIGH) {
			counter += 1;
			buttonStateBevor = LOW;
			Serial.print("COUNTER ");
			Serial.println(counter);
		}
	}

	if (buttonState != 0) {      // Taster wurde nicht gedrückt
		ledPORT = (0 << ledPIN);   // LED auf LOW setzen
		// digitalWrite(ledPORT, LOW);
		if (buttonStateBevor == LOW) {
			buttonStateBevor = HIGH;
		}
	}

	/*
	  // Temp und Feuchtigkeit abfragen

	  // Sensor aktivieren
	  FeuchtLuftDDR = (1 << FeuchtLuftPD);   // als Ausgang setzen
	  FeuchtLuftPORT = (0 << FeuchtLuftPD);  // auf LOW setzen
	  delay(18);

	  FeuchtLuftPORT = (1 << FeuchtLuftPD);            // auf HIGH setzen
	  delayMicroseconds(40);

	  FeuchtLuftDDR = (0 << FeuchtLuftPIN);   // als Eingang setzen

	  // Response abwarten
	  for (int i=0; i < 2; i++){
		delayMicroseconds(40);
		Serial.print("Feuchtigkeitssensor aktiv, FeuchtLuft = ");
		Serial.print(FeuchtLuft);
		Serial.print(", FeuchtLuftVorher ");
		Serial.println(FeuchtLuftVorher);
		FeuchtLuft = (FeuchtLuftPIN & (1 << FeuchtLuftPD));
		if (FeuchtLuftVorher == FeuchtLuft) {
		   FeuchtLuft = 99;
		   break;
		}
		if (FeuchtLuftVorher != FeuchtLuft){
		  if (FeuchtLuft == 1){
			FeuchtLuftVorher == 1;
		  }
		  if (FeuchtLuft == 0){
			FeuchtLuftVorher == 0;
		  }
		}
		delayMicroseconds(40);
	  }
	  */

	  //Feuchtigkeitssensor auslesen mit Bibliothek
	int chk = DHT11.read(DHT11PIN);
	Serial.print("Temperatur: ");
	Serial.print((float)DHT11.temperature, 2);
	Serial.print(", Feuchtigkeit: ");
	Serial.println(DHT11.humidity, 2);

	delay(3600);


	FeuchtLuft = 0;

	for (FeuchtLuft = 1; FeuchtLuft <= 10; FeuchtLuft++) {
		// LED blinken lassen
		ledPORT = (1 << ledPIN);   // LED auf HIGH setzen
		delay(100);
		ledPORT = (0 << ledPIN);   // LED auf LOW setzen
		delay(200);
	}


}

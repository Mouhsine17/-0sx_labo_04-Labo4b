#include <LCD_I2C.h>
#include <HCSR04.h>
#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 4
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37

#define DEG_MIN 10
#define DEG_MAX 170

LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

// Gestion du temps
unsigned long lastDistanceCheck = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastLcdUpdate = 0;

// État actuel
enum Etat { TROP_PRES,
            TROP_LOIN,
            DANS_ZONE };
Etat etatActuel = TROP_LOIN;

float distance = 0;
int angle = 0;
int lastAngle = -1;

void allumage() {
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("2349185");
  lcd.setCursor(0, 1);
  lcd.print("Labo 4b");
  delay(2000);
  lcd.clear();
}

void setup() {
  Serial.begin(115200);
  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(100);
  allumage();
}

void loop() {
  unsigned long now = millis();

  // Mesure de distance toutes les 50ms
  if (now - lastDistanceCheck >= 50) {
    distance = hc.dist();
    updateEtatEtAngle();
    lastDistanceCheck = now;
  }

  // Affichage max 10 fois/sec
  if (now - lastLcdUpdate >= 100) {
    afficherLCD();
    lastLcdUpdate = now;
  }

  // Transmission série toutes les 100ms
  if (now - lastSerialPrint >= 100) {
    Serial.print("etd:");
    Serial.print("2349185");
    Serial.print(",dist:");
    Serial.print((int)distance);
    Serial.print(",deg:");
    Serial.println(angle);
    lastSerialPrint = now;
  }

  // Déplacement moteur si besoin
  if (etatActuel == DANS_ZONE) {
    if (angle != lastAngle) {
      int steps = map(angle, DEG_MIN, DEG_MAX, 57, 967);
      myStepper.enableOutputs();  
      myStepper.moveTo(steps);
      lastAngle = angle;
    }
    myStepper.run();
  } else {
    myStepper.disableOutputs();  
  }
}

void updateEtatEtAngle() {
  if (distance < 30) {
    etatActuel = TROP_PRES;
    angle = 10;
  } else if (distance > 60) {
    etatActuel = TROP_LOIN;
    angle = 170;
  } else {
    etatActuel = DANS_ZONE;
    angle = map(distance, 30, 60, DEG_MIN, DEG_MAX);
  }
}

void afficherLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist:");
  lcd.print((int)distance);
  lcd.print("cm");
  lcd.setCursor(0, 1);
  if (etatActuel == TROP_PRES) {
    lcd.print("obj :Trop pres");
  } else if (etatActuel == TROP_LOIN) {
    lcd.print("obj :Trop loin");
  } else {
    lcd.print("obj:");
    lcd.print(angle);
    lcd.print("deg");
  }
}

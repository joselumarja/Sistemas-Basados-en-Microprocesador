#include <rgb_lcd.h>

char Buffer;
rgb_lcd lcd;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd=rgb_lcd();
  lcd.begin(16,1,4);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0)
  {
    Buffer=Serial.read();

    switch(Buffer)
    {
      case 'R':
        lcd.clear();
        lcd.print("Pase");
        lcd.setColor(RED);
        break;
       case 'Y':
        lcd.clear();
        lcd.print("Espere verde");
        lcd.setColor(BLUE);
        break;
       case 'G':
        lcd.clear();
        lcd.print("Pulse Boton");
        lcd.setColor(GREEN);
        break;
       default:
        break;
    }
  }
}

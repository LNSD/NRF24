#include "BSP.h"

#define PB 3
#define LD 4

LED led(LD);
PushButton button(PB);

void setup() 
{
  // Start serial connection
  Serial.begin(115200);
}

void loop() 
{  
  // Read the pushbutton value into a variable
  bool pushed = button.isPushed();
 
  // Print out the value of the pushbutton
  Serial.print(" - Button pushed: ");
  Serial.print((pushed)? "TRUE":"FALSE");
  Serial.print(" (");
  Serial.print(digitalRead(PB));
  Serial.println(")");
  
  // Turn on the led when the button's pressed,
  // and off when it's not:
  if (pushed) {
    led.turnOn();
  } else {
    led.turnOff();
  }

  delay(100); // Wait 100ms
}

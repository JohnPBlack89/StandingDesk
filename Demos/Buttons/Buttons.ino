#include <OneButton.h>

/**** Button ****
  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.
*/

// constants won't change. They're used here to set pin numbers:
const int buttonPin1 = 12;     // the number of the pushbutton pin
const int buttonPin2 = 13;     // the number of the pushbutton pin

// variables will change:
int buttonState1 = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin1, INPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin2, INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState1 == HIGH && buttonState2 == LOW) {
    // turn LED on:
    digitalWrite(LED_BUILTIN, LOW);
  } else if (buttonState1 == LOW && buttonState2 == HIGH) {
    // turn LED on:
    digitalWrite(LED_BUILTIN, LOW);
  } else if (buttonState1 == HIGH && buttonState2 == HIGH) {
    // turn LED off:
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    // turn LED off:
    digitalWrite(LED_BUILTIN, LOW);
  }
}

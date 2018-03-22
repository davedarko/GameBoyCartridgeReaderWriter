// PORTC lower OUT
// PORTD higher OUT
// PORTF DATA
// GB_RD E1
// GB_WR E0

int led_blue = 24; // B5
int led_red = 25; // B6
int led_green = 26; // B7

void setup() {
  // put your setup code here, to run once:
  pinMode(led_blue, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  digitalWrite(led_blue, HIGH);
  digitalWrite(led_red, HIGH);
  digitalWrite(led_green, HIGH);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(led_green, 255);
  delay(1000);
  analogWrite(led_green, 255-4);
  delay(1000);
  analogWrite(led_green, 255-16);
  delay(1000);
  analogWrite(led_green, 255-64);
  delay(1000);
  Serial.println("Hello World!");
}

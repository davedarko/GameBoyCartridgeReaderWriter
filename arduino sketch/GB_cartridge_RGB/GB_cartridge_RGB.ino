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
  digitalWrite(led_red, LOW);
  delay(1000);
  digitalWrite(led_blue, LOW);
  delay(1000);
  digitalWrite(led_green, LOW);
  delay(1000);
  Serial.println("Hello World!");

  digitalWrite(led_blue, HIGH);
  digitalWrite(led_red, HIGH);
  digitalWrite(led_green, HIGH);
}

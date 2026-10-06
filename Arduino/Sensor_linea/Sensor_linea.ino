bool dato = 0;
void setup() {
  Serial.begin(9600);
  pinMode(8, INPUT);

}

void loop() {
  dato = digitalRead(8);
  Serial.println(dato);
  delay(100);

}

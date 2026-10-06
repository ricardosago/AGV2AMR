const int encoderPinA = 7;  // Pin del canal A
const int encoderPinB = 11; // Pin del canal B

volatile long pulseCount = 0;
volatile int direction = 0;  // 1 = Horario, -1 = Antihorario
volatile long lastTime = 0;
volatile float velocity = 0;
float pulsesPerRevolution = 20.0;  // Ajusta esto según tu encoder
float timePerRevolution = 0.0;
float rpm = 0.0;

void setup() {
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Configura interrupciones para detectar cambios en el canal A
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);  
  Serial.begin(9600);
}

void loop() {
  // Calcula la velocidad y el sentido de giro cada segundo
  long currentTime = millis();
  
  if (currentTime - lastTime > 1000) {  // Cada 1 segundo
    velocity = pulseCount;  // Pulsos por segundo
    rpm = (velocity / pulsesPerRevolution) * 60.0;  // Velocidad en RPM
    
    // Muestra la velocidad y el sentido de giro
    Serial.print("Velocidad (pulsos/segundo): ");
    Serial.println(velocity);
    Serial.print("Velocidad (RPM): ");
    Serial.println(rpm);

    if (direction == 1) {
      Serial.println("Sentido de giro: Horario");
    } else if (direction == -1) {
      Serial.println("Sentido de giro: Antihorario");
    } else {
      Serial.println("Sentido de giro: No se detecta movimiento");
    }

    pulseCount = 0;  // Reinicia el contador para el siguiente ciclo
    lastTime = currentTime;  // Actualiza el tiempo
  }
}

// Esta función se llama cada vez que hay un cambio en el canal A
void updateEncoder() {
  int stateA = digitalRead(encoderPinA);  // Estado del canal A
  int stateB = digitalRead(encoderPinB);  // Estado del canal B

  // Determina la dirección de giro
  if (stateA == stateB) {
    direction = 1;  // Giro en sentido horario
  } else {
    direction = -1; // Giro en sentido antihorario
  }

  // Actualiza el contador de pulsos
  pulseCount++;
}

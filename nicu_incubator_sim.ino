// Define LED pins
const int GREEN_LED = 12;
const int YELLOW_LED = 11;
const int RED_LED = 10;
const int BLUE_LED = 9;

// Define button pins
const int ABNORMAL_RESPIRATION_RATE = 13;
const int ABNORMAL_HEART_RATE = 8;
const int ADJUST_TEMPERATURE = A0;
const int VITAL_CHECK_DONE = 6;
const int RESET_BUTTON = 5;
const int CORRECTION_BUTTON = 3;


// Define timer variables
unsigned long previousTime = 0;
unsigned long r_previousMillis = 0;
unsigned long h_previousMillis = 0;
unsigned long t_previousMillis = 0;
unsigned long lapsedtime1 = 0;
const unsigned long interval1 = 3000; // 3 seconds
const unsigned long interval2 = 9000; // 9 seconds
const unsigned long interval3 = 5000; // 5 seconds
const unsigned long interval4 = 10000; // 10 seconds


// Define interrupt variables
volatile bool h_interruptTriggered = false;
volatile bool h_stopInterrupt = true;
volatile bool r_interruptTriggered = false;
volatile bool r1_interruptTriggered = false;
volatile bool r_stopInterrupt = true;
volatile bool t_interruptTriggered = false;
volatile bool t_stopInterrupt = true;
volatile bool M_interruptTriggered = false;
volatile bool M_stopInterrupt = true;

//Define Temperature Components & Variables 
int ThermistorPin = A2;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;


void setup() {

  Serial.begin(9600); // to communicate with serial monitor
  
  // Initialize LED pin as an output
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  digitalWrite(GREEN_LED, HIGH);
  
  // Initialize button pin as an input with pull-up resistor
  pinMode(ABNORMAL_RESPIRATION_RATE, INPUT_PULLUP);
  pinMode(ADJUST_TEMPERATURE, INPUT_PULLUP);
  pinMode(ABNORMAL_HEART_RATE, INPUT_PULLUP);
  pinMode(VITAL_CHECK_DONE, INPUT_PULLUP);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(CORRECTION_BUTTON, INPUT_PULLUP);
  
// Initialize timer 1
  TCCR1A = 0; // Reset timer control register A
  TCCR1B = (1<<CS11)|(1<<CS10); // Set prescaler to 64
  
// Set timer 0 prescaler to 256
  TCCR0B |= (1 << CS02); 
  TCCR0B &= ~(1 << CS01);
  TCCR0B &= ~(1 << CS00);

  sei(); // Enable global interrupts

  attachInterrupt(digitalPinToInterrupt(2),samePinISR, FALLING); //Hardware interrupt for manuals vital check button and reset button
  attachInterrupt(digitalPinToInterrupt(CORRECTION_BUTTON),AbnormalityCorrected, FALLING);  //Hardware interrupt for system correction button
}

void loop() {

// Check if only green LED is on for 10 seconds
  if (digitalRead(GREEN_LED) == HIGH && digitalRead(RED_LED) == LOW && digitalRead(BLUE_LED) == LOW && digitalRead(YELLOW_LED) == LOW) {
    unsigned long currentTime = millis()*4;
    lapsedtime1 = currentTime - previousTime; 
    Serial.print ("Amount time that has lapsed in ms since system is in nominal state:" ); 
    Serial.print (lapsedtime1); 
    Serial.println (); 
    if (lapsedtime1 >= interval4) {
      M_interruptTriggered = false;
      // All LEDs should flash
      flashLEDs();
    }
  } else {
    // Reset timer if green LED is not on
    previousTime = millis()*4;
  }
  
   HeartRate();
   AbnormalityCorrected();
   RespirationRate();
   Temperature();
}

void samePinISR() {
  if (digitalRead(RESET_BUTTON) == LOW) {
    ResetSystem();
  } 
  else if (digitalRead(VITAL_CHECK_DONE) == LOW) {
    ManualVitalsCheckDone();
  }
}

void flashLEDs() {
  digitalWrite(BLUE_LED, LOW);
  while (!M_interruptTriggered) {
    digitalWrite(GREEN_LED, digitalRead(GREEN_LED) ^ 1);
    delay(100/4);
  }
}

void HeartRate(){
  // Check if button has been pressed
   if (digitalRead(ABNORMAL_HEART_RATE) == LOW) {
    // Start timer
    TCNT1 = 0; // reset timer 1 counter
    TIMSK1 |= (1 << TOIE1); // enable timer 1 overflow interrupt
    h_interruptTriggered = false;
    h_stopInterrupt = false;
    
    digitalWrite(GREEN_LED, LOW);
  }
  
  // Check if interrupt has been triggered
  if (h_interruptTriggered && !h_stopInterrupt) {// timer 1 has overflowed 11 times thus 3 seconds have passed
    // Flash LED
    digitalWrite(RED_LED, HIGH);
  }
}

void RespirationRate(){
    
  // Check if button has been pressed
   if (digitalRead(ABNORMAL_RESPIRATION_RATE) == LOW) {
    // Start timer
    r_previousMillis = millis()*4;
    r_interruptTriggered = false;
    r_stopInterrupt = false;
    r1_interruptTriggered = false;
    
    digitalWrite(GREEN_LED, LOW);
  }
   
  // Check if timer has reached the interval
  if ( millis()*4 - r_previousMillis >= interval1) {
    
    // Trigger interrupt
    r_interruptTriggered = true;
  }

  if ( millis()*4 - r_previousMillis >= interval2) {
    // Trigger interrupt
    r_interruptTriggered = false;
    r1_interruptTriggered = true;
    
    //Reset previous millis
    r_previousMillis = millis()*4;
   }
  
  // Check if interrupt has been triggered
  if (r_interruptTriggered && !r_stopInterrupt && !r1_interruptTriggered) {
    // Flash LED
    digitalWrite(YELLOW_LED, HIGH);
  }
  // Check if interrupt has been triggered
  if (!r_interruptTriggered && !r_stopInterrupt && r1_interruptTriggered ) {
    // Flash LED
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);
  }
}

void Temperature(){
    Vo = analogRead(ThermistorPin);
    R2 = R1 * (1023.0 / (float)Vo - 1.0); //Voltage Divider
    logR2 = log(R2);
    T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2)); //The Steinhart-Hart equation is used to convert the resistance of the thermistor to a temperature reading in Kelvin.
    T = T - 273.15; //Convert from Kelvin to Celcius
    T = (T * 9.0)/ 5.0 + 32.0; // Convert from celcius to farenheit

  // Check if start button has been pressed
   if (T <= 50) {
    Serial.print("Temperature: "); 
    Serial.print(T);
    Serial.println(" F"); 
    Serial.println("Temperature is too low. Time to adjust"); 
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
    // Start timer
    t_previousMillis = millis()*4;
    t_interruptTriggered = false;
    t_stopInterrupt = false;
  }
  else if (T >= 80){
    Serial.print("Temperature: "); 
    Serial.print(T);
    Serial.println(" F"); 
    Serial.println("Temperature is too high. Time to adjust"); 
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
    // Start timer
    t_previousMillis = millis()*4;
    t_interruptTriggered = false;
    t_stopInterrupt = false;
  }

    if ( millis()*4 - t_previousMillis >= interval3) {
    
    // Trigger interrupt
    t_interruptTriggered = true;
    
    // Reset previous millis
    t_previousMillis = millis()*4;
    }
  
  // Check if interrupt has been triggered
  if (t_interruptTriggered && !t_stopInterrupt) {
    digitalWrite(BLUE_LED, LOW);
    // Flash LED
    digitalWrite(GREEN_LED, digitalRead(GREEN_LED) ^ 1);
    delay(100/4);
  }
}
 
void AbnormalityCorrected(){
  // Check if stop button has been pressed
  if (digitalRead(CORRECTION_BUTTON) == LOW) {
    
     //Stop timer and turn off LED
      if (digitalRead(RED_LED) == LOW){
      h_interruptTriggered = false;
      h_stopInterrupt = true;
      r_interruptTriggered = false;
      r1_interruptTriggered = false;
      r_stopInterrupt = true;
    if (digitalRead(BLUE_LED) == LOW){
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      }
    }
  }
}

void ResetSystem(){
  // Check if stop button has been pressed
  if (digitalRead(RESET_BUTTON) == LOW) {

    if (digitalRead(BLUE_LED) == LOW){
      // Stop timer and turn off LED
      h_interruptTriggered = false;
      h_stopInterrupt = true;
      r_interruptTriggered = false;
      r1_interruptTriggered = false;
      r_stopInterrupt = true;
  
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
    }
  }
}

void ManualVitalsCheckDone(){
  // Check if stop button has been pressed
  if (digitalRead(VITAL_CHECK_DONE) == LOW) {
    if (digitalRead(BLUE_LED) == LOW){
       // Reset timer variables and turn off all LEDs
      M_interruptTriggered = true;
      t_interruptTriggered = true;
      t_stopInterrupt = true;
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
    }
  }
}

// Timer overflow interrupt service routine
ISR(TIMER1_OVF_vect) {

 static uint16_t timerCounter_h = 0;
  
  timerCounter_h++;
Serial.println(timerCounter_h);
  // Check if timer has reached the heart abnormality (alarm state) interval of 3 seconds, hence 11 overflows
  if ( timerCounter_h >= 11) { // 3 second interval with 64 prescaler
    timerCounter_h = 0;
    h_interruptTriggered = true;
    TIMSK1 &= ~(1 << TOIE1); // disable timer 1 overflow interrupt
  }

} 
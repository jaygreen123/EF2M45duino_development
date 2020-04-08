
uint32_t count,i;
uint8_t isFirstScan = 1;
uint8_t str[] = "HELLO WORLD!!!";
long long n;

void startLed(){
    digitalWrite(gpio1_0, LOW);
    delay(500);
    digitalWrite(gpio1_0, HIGH);
    delay(100);
    digitalWrite(gpio1_0, LOW);
    delay(100);
    digitalWrite(gpio1_0, HIGH);
}

void setup() { 
    pinMode(gpio1_0, OUTPUT);
    pinMode(gpio1_1, INPUT);  
    pinMode(gpio1_2, INPUT);  
    pinMode(gpio1_3, OUTPUT); 
    pwmInit();
    Serial.begin(9600);
}

void loop() {
  if(isFirstScan == 1){  
    startLed();
    Serial.println("Hello World!!!");
    isFirstScan = 0;
  }
  else{
    
    if(HIGH == digitalRead(gpio1_1))
      digitalWrite(gpio1_3, HIGH);
    else
      digitalWrite(gpio1_3, LOW);
    
    analogWrite(9,count);
    count = (count<255)?(count+1):0;
    delay(5);
  }
}



#define Leftlight   2
#define Rightlight  12

void setup() {

  pinMode(Leftlight, OUTPUT);
  pinMode(Rightlight, OUTPUT);
}

void loop() {
digitalWrite(Leftlight, HIGH);
digitalWrite(Rightlight, HIGH);

}

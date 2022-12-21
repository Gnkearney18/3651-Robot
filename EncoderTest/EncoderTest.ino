int bLouta = 22;
int bLoutb = 23;
int pwmbL = 8;

int bRouta = 24;
int bRoutb = 25;
int pwmbR = 7;

int fLouta = 28;
int fLoutb = 29;
int pwmfL = 9;


int fRouta = 26;
int fRoutb = 27;
int pwmfR = 6;

void setup() {
  // put your setup code here, to run once:


  pinMode(bLouta, OUTPUT);
  pinMode(bLoutb, OUTPUT);

  pinMode(bRouta, OUTPUT);
  pinMode(bRoutb, OUTPUT);

  pinMode(fLouta, OUTPUT);
  pinMode(fLoutb, OUTPUT);

  pinMode(fRouta, OUTPUT);
  pinMode(fRoutb, OUTPUT);

  pinMode(32, INPUT);
  pinMode(36, INPUT);


  Serial.begin(115200);


}

int i = 0;


void loop() {

  digitalWrite(fRouta, HIGH);
  digitalWrite(fRoutb, LOW);

  analogWrite(pwmfR, 255);
  
  // put your main code here, to run repeatedly:

}

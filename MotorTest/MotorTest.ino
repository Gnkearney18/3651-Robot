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


int button = 40;


void setup() {
  // put your setup code here, to run once:

  pinMode(button, INPUT);
  
  pinMode(bLouta, OUTPUT);
  pinMode(bLoutb, OUTPUT);

  pinMode(bRouta, OUTPUT);
  pinMode(bRoutb, OUTPUT);

  pinMode(fLouta, OUTPUT);
  pinMode(fLoutb, OUTPUT);

  pinMode(fRouta, OUTPUT);
  pinMode(fRoutb, OUTPUT);


  Serial.begin(115200);

  while(digitalRead(button) == 0){}
  
}

void loop() {
  analogWrite(pwmfR, 255);
  analogWrite(pwmfL, 255);
  analogWrite(pwmbR, 255);
  analogWrite(pwmbL, 255);
  
  digitalWrite(bLouta, HIGH);
  digitalWrite(bLoutb, LOW);

  digitalWrite(bRouta, HIGH);
  digitalWrite(bRoutb, LOW);

  digitalWrite(fLouta, HIGH);
  digitalWrite(fLoutb, LOW);

  digitalWrite(fRouta, HIGH);
  digitalWrite(fRoutb, LOW);
}

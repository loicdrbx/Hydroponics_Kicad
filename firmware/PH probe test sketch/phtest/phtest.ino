
#define PHPIN A1
#define VGUARD A2
#define REF_PH 7
#define OFFSET 95

int adcin = 0;
int vin = 0;
int direct_adcin = 0;
int vout = 0;
int vprobe = 0;
int i = 0;
long long int adcsum = 0;
long long int guardsum = 0;
int vph = 0;
float ph = 0;

int vguard = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PHPIN ,INPUT);
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  adcsum = 0;
  guardsum = 0;
  for(i=0;i<=999;i++)
  {
    adcsum = adcsum + analogRead(PHPIN);
    guardsum = guardsum + analogRead(VGUARD);
  }
  
  adcin = adcsum/1000;
  vguard = guardsum/1000;

  vout = map(adcin,0,1023,0,5000);
  vprobe = map(adcin,0,1023,-2500,2500);
  vprobe = vprobe - OFFSET;
  vguard = map(vguard,0,1023,0,5000);

  ph = REF_PH + ((float)(-vprobe)*0.01708389);

  
  Serial.print("ADC input voltage is: ");
  Serial.print(vout);
  Serial.print(", probe input voltage is: ");
  Serial.print(vprobe);
  Serial.print(", guard input voltage is: ");
  Serial.print(vguard);
  Serial.print(", calculated PH is: ");
  Serial.print(ph);
  

  Serial.print('\n');
  delay(100);

}

#include <Arduino.h>
#include <DAC7678.h>

// AnalogIn pins for reading DAC1 outputs (channels A through B)
#define OUT1A 0
#define OUT1B 1
#define OUT1C 2
#define OUT1D 3
// AnalogIn pins for reading DAC1 outputs (channels E through H)
#define OUT2E 9
#define OUT2F 8
#define OUT2G 7
#define OUT2H 6

// DigitalOut pins for controlling LD and CLR pins (active low)
#define LD1 11
#define CLR1 10
#define LD2 4
#define CLR2 3

DAC7678 DAC1 = DAC7678(DAC7678_DEFAULT_ADDRESS, Wire);
DAC7678 DAC2 = DAC7678(0b1001100, Wire);

const uint8_t VALID_ADDRESSES[8] =
{
    0b1001000, 0b1001010, 0b1001100
};

bool testDAC1Output(DAC7678::Channel chan, uint16_t value);
bool testDAC2Output(DAC7678::Channel chan, uint16_t value);

void writeAll(uint16_t val);
void readAll();

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  digitalWrite(CLR1, HIGH);
  digitalWrite(CLR2, HIGH);
  digitalWrite(LD1, LOW);
  digitalWrite(LD2, LOW);
  delay(5);
}

int val = 0;

void loop() {
  writeAll(val);
  readAll();
  val += 5;
  if(val >= 4096)
    val = 0;
}

void writeAll(uint16_t val)
{
  for(int i = 0; i < 3; i++)
  {
    DAC7678 DAC = DAC7678(VALID_ADDRESSES[i], Wire);
    for(int i = 0; i < 8; i++)
      DAC.setAndUpdateAll(i, val);
      delay(1);
  }
}

void readAll()
{
  Serial.println();
  for(int i = 0; i <= 3; i++)
  {
    int val = analogRead(i);
    Serial.print(val);
    Serial.print(" ");
  }
  for(int i = 6; i <= 9; i++)
  {
    int val = analogRead(i);
    Serial.print(val);
    Serial.print(" ");
  }
}

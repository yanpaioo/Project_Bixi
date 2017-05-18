

void setup()
{
  Serial3.begin(9600);
}
int DATA = 10;
void loop()
{
  Serial3.write(DATA);
  delay(500);
}


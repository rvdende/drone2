#include <Receiver_DUE.h>


unsigned long timer;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Receiver library test");
  initializeReceiver();   
}

void loop() {
  
  if((millis() - timer) > 50) // 20Hz
  {
    timer = millis();  

    Serial.print(" CH1: ");
    Serial.print(getRawChannelValue(1));
    Serial.print(" CH2: ");
    Serial.print(getRawChannelValue(2));
    Serial.print(" CH3: ");
    Serial.print(getRawChannelValue(3));
    Serial.print(" CH4: ");
    Serial.print(getRawChannelValue(4));
    Serial.print(" CH5: ");
    Serial.print(getRawChannelValue(5));
    Serial.print(" CH6: ");
    Serial.print(getRawChannelValue(6));
    Serial.print(" CH7: ");
    Serial.print(getRawChannelValue(7));
    Serial.print(" CH8: ");
    Serial.print(getRawChannelValue(8));
    Serial.println();
  }
}

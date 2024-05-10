// Code written by Ben Braham
//MIT License

//Copyright (c) 2024 BenBmakes

//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  // Initialize servos
  servo1.attach(9); // Attach servo to pin 9
  servo2.attach(10); // Attach servo to pin 10
  servo3.attach(11);
  
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming data
    String data = Serial.readStringUntil('\n');
    
    // Split the data into two parts separated by a comma
    int commaIndex = data.indexOf(',');
    String servo1Data = data.substring(0, commaIndex);
    String servo2Data = data.substring(commaIndex + 1);
    
    // Convert the data to integers
    int servo1Value = servo1Data.toInt();
    int servo2Value = servo2Data.toInt();
    int servo3Value = servo2Value;
    // Move the servos based on the received values
    servo1.write(servo1Value);
    servo2.write(180 - servo2Value);
    servo3.write(servo3Value);
  }
}

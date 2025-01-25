#include <Arduino.h>

String receivedMessage = ""; // Buffer to store the incoming message

void setup()
{
    Serial.begin(460800); // Initialize serial communication
    Serial.println("Serial port ready. Send a message:");
}

void loop()
{
    while(Serial.available() > 0)
    {
        char incomingChar = Serial.read();
        
        // Check for end of message (newline character)
        if (incomingChar == '\n') 
        {
            // Convert the received string to a float
            float receivedFloat = receivedMessage.toFloat();
            
            // Print the received float value
            Serial.println("Received float: " + String(receivedFloat));
            
            receivedMessage = ""; // Clear the buffer
        }
        else 
        {
            // Append incoming character to the message
            receivedMessage += incomingChar;
        }
    }
}

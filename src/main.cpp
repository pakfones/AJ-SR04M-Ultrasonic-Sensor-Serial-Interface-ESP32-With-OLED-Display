/*
 * Ultrasonic Sensor Distance Display using SH1106 OLED and UART Communication
 * 
 * This program interfaces an ultrasonic sensor (e.g., AJ-SR04M) with an ESP32 using UART communication.
 * The distance data is retrieved from the sensor and displayed on an SH1106 OLED screen. 
 * If no data is received from the sensor or an invalid response is detected, the program handles the error gracefully.
 * 
 * The program includes:
 * - Initialization of the SH1106 OLED display
 * - UART communication with the sensor using defined pins
 * - Functions to request and parse distance data from the sensor
 * - Error handling and display feedback on the OLED
 * 
 * Libraries required:
 * - Adafruit_SH110X (for OLED display control)
 * - Wire (for I2C communication)
 * 
 * Author: [PAKFONES]
 * Date: [18.10.2014]
 * 
 * Pin Connections:
 * - SH1106 OLED: I2C (SDA, SCL)
 * - Ultrasonic Sensor (AJ-SR04M): TX (GPIO 10), RX (GPIO 9)
 * - Replace (R19) with 47k Resistor To Switch AJ-SR04M to Low Power Serial Port Mode
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Wire.h>

// SH110X OLED Display
#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SH110X_I2C_ADDRESS 0x3C
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define UART pins for the sensor
#define UART_TX_PIN 10  // Pin connected to the sensor's RX (TRIG)
#define UART_RX_PIN 9   // Pin connected to the sensor's TX (ECHO)
HardwareSerial abhSerial(1);  // Using Serial-1 for the ultrasonic sensor

// Function Prototypes
void getDistance();
void displayError(const char* errorMsg);
void displayDistance(float distanceCm);
void parseSensorData();

void setup() {
    Serial.begin(115200);  // Initialize Serial for debugging
    abhSerial.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN); // Sensor setup for UART
    Serial.println("Starting setup...");
    Serial.println("Initialized UART for sensor.");

    // Initialize SH1106G OLED Display
    if (!display.begin(SH110X_I2C_ADDRESS, true)) {
        Serial.println("OLED initialization failed.");
        displayError("OLED Init Fail");
    } else {
        Serial.println("OLED initialization successful.");
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(17, 25);
        display.print("PAKFONES ");
        display.display();
        delay(1000);

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(34, 25);
        display.print("LOADING");
        display.display();

        Serial.println("Displaying loading message...");

        // Display animation dots
        for (int i = 0; i < 3; i++) {
            delay(600);  // 0.6 second delay
            display.print(".");
            display.display();
            Serial.print("."); // Debug for loading animation
        }
        Serial.println("\nLoading complete.");
    }
}

void loop() {
    getDistance(); // Request distance from the sensor
    delay(1000);  // Adjust the loop delay as needed
}


void displayError(const char* errorMsg) {
    Serial.print("Error: ");
    Serial.println(errorMsg);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("Error: ");
    display.println(errorMsg);
    display.display();
    while (true) {}  // Block to indicate error (remove for regular operation)
}

void displayDistance(float distanceCm) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("Distance:");
    display.setCursor(0, 30);
    display.print(distanceCm, 1); // 1 decimal place
    display.print(" cm");
    display.display();
}

// single ready bytes
void getDistance() {
    abhSerial.write(0x01); // Directly send the trigger command to the sensor
    
    // Wait up to 100 ms for data to become available
    unsigned long startTime = millis();
    while (!abhSerial.available()) {
        if (millis() - startTime > 100) { // Timeout after 100 ms
            Serial.println("No data from sensor.");
            return;
        }
    }
    
    parseSensorData(); // Parse data when available
}

void parseSensorData() {
    byte receivedBytes[4];

    // Attempt to read exactly 4 bytes; return if unsuccessful
    if (abhSerial.readBytes(receivedBytes, 4) != 4) {
        Serial.println("Incomplete data received.");
        return;
    }

    // Validate start byte
    if (receivedBytes[0] == 0xFF) { // Check if the first byte is the expected start byte
        // Combine bytes to get the distance in mm
        unsigned int distance = (receivedBytes[1] << 8) | receivedBytes[2]; // Combine high and low bytes
        float distanceCm = distance / 10.0; // Convert to cm
        
        Serial.print("Distance [cm]: ");
        Serial.println(distanceCm, 1); // Print in cm with 1 decimal place

        displayDistance(distanceCm); // Display distance in cm on OLED
    } else {
        displayError("Invalid Start Byte");
    }
}



/*
// Multi read bytes in loop
void getDistance() {
    abhSerial.write(0x01); // Directly send the trigger command to the sensor
    
    unsigned long startTime = millis(); // Start a timer
    while (!abhSerial.available()) {
        if (millis() - startTime > 100) { // Timeout after 100 ms
            Serial.println("No data from sensor.");
            return; // Exit if no data received
        }
    }
    
    Serial.println("Data available from sensor.");
    parseSensorData();  // Call function to parse the data
}


void parseSensorData() {
    byte receivedBytes[4];
    
    for (int i = 0; i < 4; i++) {
        if (abhSerial.available()) {
            receivedBytes[i] = abhSerial.read();
            Serial.print("Received Byte: 0x");
            Serial.println(receivedBytes[i], HEX);
        }
    }

    // Validate start byte
    if (receivedBytes[0] == 0xFF) { // Check if the first byte is the expected start byte
        // Combine bytes to get the distance in mm
        unsigned int distance = (receivedBytes[1] << 8) | receivedBytes[2]; // Combine high and low bytes
        Serial.print("Distance [mm]: "); 
        Serial.println(distance);
        
        // Calculate and print distance in cm
        float distanceCm = distance / 10.0; // Convert to cm
        Serial.print("Distance [cm]: ");
        Serial.println(distanceCm, 1); // Print in cm with 1 decimal place
        
        displayDistance(distanceCm); // Display distance in cm on OLED
    } else {
        displayError("Invalid Start Byte");
    }
}





*/
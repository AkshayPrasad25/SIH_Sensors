// void setup()
// {

//     // Set the serial monitor baudrate to 9600

//     Serial.begin(9600);
// }
 

// void loop()
// {

//     // Variable to store ADC value ( 0 to 1023 )

//     int level;

//     // analogRead function returns the integer 10 bit integer (0 to 1023)

//     level = analogRead(0); 

   
//     delay(2000);
//     // Print text in serial monitor

//     Serial.println("Analog value:");

//     // Print analog value in serial monitor

//     Serial.println(level);
// }

const int sensor_pin = A1; /* Soil moisture sensor O/P pin */

void setup() {

Serial.begin(9600); /* Define baud rate for serial communication */

}

void loop() {

float moisture_percentage;

int sensor_analog;

sensor_analog = analogRead(sensor_pin);

moisture_percentage = ( 100 - ( (sensor_analog/1023.00) * 100 ) );

Serial.print("Moisture Percentage = ");

Serial.print(moisture_percentage);

Serial.print("%\n\n");

delay(1000);

}
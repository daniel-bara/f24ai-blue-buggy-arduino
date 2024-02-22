#include "mcp2515_can.h"
#include <Servo.h>
#include <SPI.h>

// SSAV Chassis Arduino Control Code
// Requires SEEED CAN Bus Protocol Library
// SEEED Can Bus Library Setup
#define CAN_2515
const int SPI_CS_PIN = 10; // SPI Pin, shown in Circuit diagram
const int CAN_INT_PIN = 2; // SPI Pin, shown in Circuit diagram

mcp2515_can CAN(SPI_CS_PIN); // Set Chip Select (CS) pin
// End of SEEED CAN Bus Library Setup
// Servo Setup
Servo SteeringServo;
int Servo_Output = 3;
int steeringRequest = 900;
int steeringRequestRaw = 900;
int realAngle = 0;
// End of Servo Setup
// Motor Setup
int RPWM_Output = 5; // Arduino PWM output pin 5; connect to IBT-2 pin 1(RPWM).
int LPWM_Output = 6; // Arduino PWM output pin 6; connect to IBT-2 pin 2(LPWM).
int forwardPWM = 0;  // Initialises the PWM signal.
// 25
int velocityRequest = 0; // Ensures the motor does not rotate to begin with.
int velocityRequestRaw = 0;
// End of Motor Setup

// Functions

// Motor Control Function
// Controls the drive motor when a valid motor command (CANID 1298) is recieved
int Motion(int velocityRequestRaw)
{
  velocityRequest = map(velocityRequestRaw, 0, 1950, 0, 255); // Maps CAN Bus protocol to applicable PWM signal.255 is equivalent to 100 % motor duty cycle.
  // Forward drive       //////WHY 1950 IS THE MAXIMUM?
  if (velocityRequest >= 0 && velocityRequest <= 255)
  {
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, velocityRequest);
    Serial.print("Spining Forwards at: "); // Prints to Serial Monitor the request value and percentage.
    Serial.println(velocityRequest);
    Serial.print("Percentage: ");
    Serial.println(velocityRequest / 2.55);
  }
  // Reverse drive - Outside of SSAV specification
  else if (velocityRequest < 0)
  {
    Serial.println("ERROR! Invalid Velocity Recieved - Reverse not acceptable ");
  }
  // Error state
  else
  {
    Serial.println("ERROR! Invalid Velocity Recieved - Out of bounds ");
  }
}
// End of Motor Control
// Steering Control Function
// Controls the steering servo when a valid steering command (CANID 1299) is recieved

int Steer(int steeringRequestRaw)
{
  // Turning right - Seperate mapping required for each direction due to SSAV steering geometry
  if (steeringRequest < 900)
  {
    steeringRequest = map(steeringRequestRaw, 0, 900, 35, 85);             /// 35 for extreme right(1/8th model)
    // Maps CAN Bus protocol to Servo Value. Empirically determined
  }
  // Turning left
  else if (steeringRequest >= 900)
  {
    steeringRequest = map(steeringRequestRaw, 900, 1800, 85, 135);        /// 135 for extreme left (1/8th model)
  }
  // Out of bounds check
  if (steeringRequest >= 55 && steeringRequest <= 155)
  {
    SteeringServo.write(steeringRequest); // Uses Servo library to write steering request to Servo. 26
    Serial.print("Turning by: ");         // Prints to Serial Monitor the request value.
    Serial.println(steeringRequest / 10);
  }
  // Error state
  else
  {
    Serial.println("ERROR! Invalid Steering Angle Recieved");
  }
}
// End of Steering Control Function

// Hex Decoder Function.
// Intakes CAN Bus value and provides equivalent decimal value.
int HexaDecoder(int CanBuffer[8])
{
  int RequestRaw;
  if (CanBuffer[1] != 0)
  { // If two bytes are present, then the second byte(being higher order) is multiplied by 16 ^ 2. See report for Hex decoding.
    RequestRaw = (CanBuffer[1] * 16 * 16) + CanBuffer[0];
  }
  else if (CanBuffer[1] == 0)
  { // If one byte are present, then the value is the plain hex value.
    RequestRaw = CanBuffer[0];
  }
  Serial.print("HexDecoder Raw: ");
  Serial.println(RequestRaw);
  return RequestRaw;
}
// End of Hex Decoder

void setup()
{
  SERIAL_PORT_MONITOR.begin(115200);
  // Initiates the MCP2515 daugher board connection.
  while (CAN_OK != CAN.begin(CAN_500KBPS))
  { // init can bus: baudrate = 500 k
    SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
    delay(100);
  }
  SERIAL_PORT_MONITOR.println("CAN init ok!");
  // Initiates the PWM pins for output.
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  SteeringServo.attach(Servo_Output); // Pin Needs checking
}
void loop()
{
  // Defines CAN Bus variables.
  unsigned char len = 0;
  unsigned char buf[8];
  // checkS to see if data is being recieved from the CAN Bus.
  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);            // If CAN Bus is active, it reads the data, len: data length, buf: data buf. 27
    unsigned long canId = CAN.getCanId(); // Records the CAN Bus message ID.
    // Prints the incoming CAN Bus message
    SERIAL_PORT_MONITOR.println("-----------------------------");
    SERIAL_PORT_MONITOR.print("Function ID: ");
    SERIAL_PORT_MONITOR.println(canId, DEC);
    // Loops through the buffer and prints the data
    for (int i = 0; i < len; i++)
    {
      SERIAL_PORT_MONITOR.print(buf[i]);
      SERIAL_PORT_MONITOR.print("\t");
    }
    SERIAL_PORT_MONITOR.println();
    int intBuf = (int)buf; // Converts buffer to integers for Motor and Servo functions.Original buffer required for printing to monitor.
    // Checks CANID
    if (canId == 0)
    {
      Serial.println("Idle, No Function Active");
    }
    // Calls motor function if the correct command is sent
    else if (canId == 1298) ////////////////////////////////////WHY 1298 IS THE ID FOR MOTOR
    {
      Serial.println("Motor Function Active");
      velocityRequestRaw = HexaDecoder(intBuf); // Calls Hex decoder function.
      Motion(velocityRequestRaw);               // Calls motor function.
    }
    // Calls steering function if the correct command is sent
    else if (canId = 1299) //////////////////////////////////////WHY 1299 IS THE ID FOR servo
    {
      Serial.println("Steering Function Active");
      steeringRequestRaw = HexaDecoder(intBuf); // Calls Hex decoder function.
      Steer(steeringRequestRaw);                // Calls steering function.
    }
    else
    {
      Serial.println("Error! Invalid Function Called - No action taken ");
    }
    SERIAL_PORT_MONITOR.println();
  }
}
// END FILE

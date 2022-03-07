
#include "ClearCore.h"
	
    /* Replace with your application code 
    bool ledState = true;
    while (1)
    {
        ConnectorLed.State(ledState);
        ledState = !ledState;
        Delay_ms(5000);
		
		#define motor ConnectorM0  */

// Defines the bit-depth of the ADC readings (8-bit, 10-bit, or 12-bit)
// Supported adcResolution values are 8, 10, and 12
#define adcResolution 12

// Specify which input pin to read from.
// IO-0 through A-12 are all available as digital inputs.
#define inputPinIO ConnectorA11
#define inputPinDir ConnectorA10

// The current state of the input pin
int16_t stateIO;
int16_t stateDir;

// Specifies which motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor ConnectorM0

// Select the baud rate to match the target serial device
#define baudRate 9600

// Specify which serial to use: ConnectorUsb, ConnectorCOM0, or ConnectorCOM1.
#define SerialPort ConnectorUsb

// Define the velocity and acceleration limits to be used for each move
int32_t velocityLimit = 10000; // pulses per sec
int32_t accelerationLimit = 100000; // pulses per sec^2

// Declares our user-defined helper function, which is used to command moves to
// the motor. The definition/implementation of this function is at the  bottom
// of the example.
void MoveDistance(int32_t distance);

// Declares our user-defined helper function, which is used to command moves to
// the motor. The definition/implementation of this function is at the  bottom
// of the example
bool MoveAbsolutePosition(int32_t position);

// Declares our user-defined helper function, which is used to command moves to
// the motor. The definition/implementation of this function is at the  bottom
// of the example
bool MoveAtVelocity(int32_t velocity);

int main() {
   // Sets the input clocking rate. This normal rate is ideal for ClearPath
   // step and direction applications.
   MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

   // Sets all motor connectors into step and direction mode.
   MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
						Connector::CPM_MODE_STEP_AND_DIR);
	
   // Set the motor's HLFB mode to bipolar PWM
   motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
   // Set the HFLB carrier frequency to 482 Hz
   motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

   // Sets the maximum velocity for each move
   motor.VelMax(velocityLimit);

   // Set the maximum acceleration for each move
   motor.AccelMax(accelerationLimit);
   
   // Sets up serial communication and waits up to 5 seconds for a port to open.
   // Serial communication is not required for this example to run.
   SerialPort.Mode(Connector::USB_CDC);
   SerialPort.Speed(baudRate);
   uint32_t timeout = 5000;
   uint32_t startTime = Milliseconds();
   SerialPort.PortOpen();
   while (!SerialPort && Milliseconds() - startTime < timeout) {
	   continue;
   }
   
   // Make a voltage meter display with the I/O pins.
   ConnectorIO0.Mode(Connector::OUTPUT_DIGITAL);
   ConnectorIO1.Mode(Connector::OUTPUT_DIGITAL);
   ConnectorIO1.Mode(Connector::OUTPUT_DIGITAL);
   // Clear out the state of our voltage meter to start.
   ConnectorIO0.State(false);
   ConnectorIO1.State(false);
   ConnectorIO2.State(false);
   
   // Since analog inputs default to analog input mode, there's no need to
   // call Mode().

   // Set the resolution of the ADC.
   AdcMgr.AdcResolution(adcResolution);
		
   // Enables the motor; homing will begin automatically if enabled
   motor.EnableRequest(true);
   SerialPort.SendLine("Motor Enabled");

   // Waits for HLFB to assert (waits for homing to complete if applicable)
   SerialPort.SendLine("Waiting for HLFB...");
   while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
	   continue;
   }
   SerialPort.SendLine("Motor Ready");

   while (true) {
	     
	    // Read the state of the input connector.
	    stateIO = inputPinIO.State();
		stateDir = inputPinDir.State();
		  
		// Read the analog input (A-9 through A-12 may be configured as analog
	    // inputs).
	    int16_t adcResult_12 = ConnectorA12.State();

	    // Convert the reading to a voltage.
	    double inputVoltage_12 = 10.0 * adcResult_12 / ((1 << adcResolution) - 1);
		
		// Display the state of the input connector.
		SerialPort.Send("A-11 Input state: ");
		if (stateIO) {
		SerialPort.SendLine("ON");
		}
		else {
		SerialPort.SendLine("OFF");
		}
		 
		SerialPort.Send("A-10 Input state: ");
		if (stateDir) {
		SerialPort.SendLine("ON");
		}
		else {
		SerialPort.SendLine("OFF");
		}
			
	    // Display the voltage reading to the serial port. 
		SerialPort.Send("A-12 input voltage: ");
		SerialPort.Send(inputVoltage_12);
		SerialPort.SendLine("V12.");
	    // Write the voltage reading to the voltage meter display pins
	    // (IO-0 through IO-5).
	    
		 
		if (inputVoltage_12 > 0.1) {
		     ConnectorIO0.State(true);
	    }
	    else {
		     ConnectorIO0.State(false);
	    }
	    // Move at 1,000 steps/sec for 2000ms
        MoveAtVelocity(0);       
        }
}


/* Sub-function Descriptions
 * ------------------------------------------------------------------------------
 * MoveDistance
 *
 *    Command "distance" number of step pulses away from the current position
 *    Prints the move status to the USB serial port
 *    Returns when step pulses have completed
 *
 * Parameters:
 *    int distance  - The distance, in step pulses, to move
 *
 * Returns: None
 *
 * ------------------------------------------------------------------------------
 * MoveAbsolutePosition
 *
 *    Command step pulses to move the motor's current position to the absolute
 *    position specified by "position"
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int position  - The absolute position, in step pulses, to move to
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 *
 * ------------------------------------------------------------------------------
 * MoveAtVelocity
 *
 *    Command the motor to move at the specified "velocity", in steps/second.
 *    Prints the move status to the USB serial port
 *
 * Parameters:
 *    int velocity  - The velocity, in step steps/sec, to command
 *
 * Returns: None
 */
 
void MoveDistance(int32_t distance) {
    SerialPort.Send("Moving distance: ");
    SerialPort.SendLine(distance);

    // Command the move of incremental distance
    motor.Move(distance);

    // Waits for all step pulses to output
    SerialPort.SendLine("Moving... Waiting for the step output to finish...");
    while (!motor.StepsComplete()) {
        continue;
    }

    SerialPort.SendLine("Steps Complete");    }
bool MoveAbsolutePosition(int32_t position) {
	// Check if an alert is currently preventing motion
	if (motor.StatusReg().bit.AlertsPresent) {
		SerialPort.SendLine("Motor status: 'In Alert'. Move Canceled.");
		return false;
	}

	SerialPort.Send("Moving to absolute position: ");
	SerialPort.SendLine(position);

	// Command the move of absolute distance
	motor.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

	// Waits for HLFB to assert (signaling the move has successfully completed)
	SerialPort.SendLine("Moving.. Waiting for HLFB");
	while (!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
		continue;
	}

	SerialPort.SendLine("Move Done");
	return true;
}
bool MoveAtVelocity(int32_t velocity) {
	// Check if an alert is currently preventing motion
	if (motor.StatusReg().bit.AlertsPresent) {
		SerialPort.SendLine("Motor status: 'In Alert'. Move Canceled.");
		return false;
	}

	SerialPort.Send("Commanding velocity: ");
	SerialPort.SendLine(velocity);

	// Command the velocity move
	motor.MoveVelocity(velocity);

	// Waits for the step command to ramp up/down to the commanded velocity.
	// This time will depend on your Acceleration Limit.
	SerialPort.SendLine("Ramping to speed...");
	while (!motor.StatusReg().bit.AtTargetVelocity) {
		continue;
	}

	SerialPort.SendLine("At Speed");
	return true;
}
//------------------------------------------------------------------------------
// "stdint.h" is included for using standard integer data types like uint8_t, uint16_t, etc.
// "Servo.h" is included to control servo motors in the code.
// "IRremote.h" is included to use infrared remote control functionality in the code.
#include <stdint.h>

#include <Servo.h>

#include <IRremote.h>


// These constants define the pins connected to the color sensor module
// S0, S1, S2, and S3 are pins used to select the color filter frequency
// OUT pin outputs the detected color signal
// LED pin is used to turn on and off the color sensor module's onboard LED
constexpr uint8_t CS_S0 = 2;
constexpr uint8_t CS_S1 = 3;
constexpr uint8_t CS_S2 = 4;
constexpr uint8_t CS_S3 = 5;
constexpr uint8_t CS_OUT = 6;
constexpr uint8_t CS_LED = 7;

// Define the pins used by the ultrasonic sensor to trigger and receive signals
constexpr uint8_t US_TRIG_PIN = 8; // Pin used to send ultrasonic signals
constexpr uint8_t US_ECHO_PIN = 9; // Pin used to receive ultrasonic signals

// These constants define the pins connected to the left and right servos, respectively.
constexpr uint8_t LEFT_SERVO_PIN = 10;
constexpr uint8_t RIGHT_SERVO_PIN = 11;

// infrared sensor receive pin for receiving signals from the remote control.
constexpr uint8_t IR_RECEIVE_PIN = 12;

// These are experimental values of the RGB color of the tapes that were measured and used in the code for tape detection. 
// The color values might vary based on the environment and lighting conditions.
constexpr float RIGHT_TAPE_R = 1.27;
constexpr float RIGHT_TAPE_G = 4.45;
constexpr float RIGHT_TAPE_B = 3.36;
constexpr float LEFT_TAPE_R = 2.38;
constexpr float LEFT_TAPE_G = 2.31;
constexpr float LEFT_TAPE_B = 2.92;
constexpr float MID_TAPE_R = 3.14;
constexpr float MID_TAPE_G = 3.00;
constexpr float MID_TAPE_B = 1.64;

// These are the distance values in centimeters used for the ultrasonic sensor algorithm.
// SMALL_DIST is the distance threshold below which the robot considers that there is an obstacle very close to it.
// BIG_DIST is the distance threshold below which the robot slows down and tries to avoid obstacles.
// NO_WALL_DIST is the maximum distance at which the robot considers there is no wall in front of it.
// NO_WALL_MAX_COUNT is the number of times the robot must measure a distance greater than NO_WALL_DIST to consider there is no wall.
constexpr uint16_t SMALL_DIST = 18;
constexpr uint16_t BIG_DIST = 24;
constexpr uint16_t NO_WALL_DIST = 70;
constexpr uint8_t NO_WALL_MAX_COUNT = 10;

// These are the delays in milliseconds in-between servo movements.
// They are used to control the speed of the robot's movement.
constexpr uint32_t MOVEMENT_SMALL_DELAY = 100;
constexpr uint32_t MOVEMENT_BIG_DELAY = 200;

// This constant specifies the amount of delay in milliseconds to wait for the IR signal before stopping the cart. 
// This value is set to 130ms to ensure that the IR signal is received and processed before stopping the cart.
constexpr uint32_t WAIT_IR_SIGNAL_BEFORE_STOP = 130;

// This constant determines the delay between reading color frequencies from the sensor.
// A small delay value might increase accuracy but also increase the load on the sensor, so this value should be set based on the specific sensor and application requirements.
// In this code, it is set to 20ms.
constexpr uint32_t CS_INTERNAL_DELAY = 20;

// These constants define the angle changes that the servos will make when the robot is turning to follow a wall or a tape. 
// WALL_TURN_DEG determines the degree change in angle of the servos when the robot is following a wall, while TAPE_TURN_DEG determines the degree change when the robot is following a tape.
constexpr uint32_t WALL_TURN_DEG = 10;
constexpr uint32_t TAPE_TURN_DEG = 15;

// This block of code defines three enumerations:
// - SysMode to represent the system's different modes: RemoteControl, WallFollow, and TapeFollow
// - RemoteButtons to represent the remote control buttons: Up, Left, Right, Down, Mode1, Mode2, and Mode3
// - TapeColor to represent the three tapes on the ground: MidTape, LeftTape, and RightTape.
enum class SysMode: uint8_t {
    RemoteControl,
    WallFollow,
    TapeFollow
};
enum class RemoteButtons: uint8_t {
    Up = 0x18, Left = 0x8, Right = 0x5A, Down = 0x46, Mode1 = 0x45, Mode2 = 0x46, Mode3 = 0x47
};
enum class TapeColor: uint8_t {
    MidTape,
    LeftTape,
    RightTape
};

// Initializing Servo objects for left and right servo motors.
Servo leftServo, rightServo;

// initializing state variables for system mode, IR signal waiting and wall count
SysMode currMode = SysMode::RemoteControl;
uint8_t waitIR = 0, noWallCount = 0;

// defining a namespace called Movement to group different movement functions
namespace Movement {
	
    // function to stop the cart movement
    void stopMoving(void) {
        leftServo.write(90);
        rightServo.write(90);
    }
	
    // defining a namespace inside Movement namespace for RemoteControl mode
    namespace RemoteControl {
	
	// function to turn cart left
        void turnLeft(void) {
            leftServo.write(0);
            rightServo.write(0);
        }

	// function to turn cart right
        void turnRight(void) {
            leftServo.write(180);
            rightServo.write(180);
        }

	// function to move cart forward
        void moveForward(void) {
            leftServo.write(180);
            rightServo.write(0);
        }

	// function to move cart backward
        void moveBackward(void) {
            leftServo.write(0);
            rightServo.write(180);
        }
    }
	
    // defining a namespace inside Movement namespace for WallFollow mode
    namespace WallFollow {

	// function to turn cart left when wall is detected
        void turnLeft(void) {
            leftServo.write(90 + WALL_TURN_DEG);
            rightServo.write(0);
        }

	// function to turn cart right when wall is detected
        void turnRight(void) {
            leftServo.write(180);
            rightServo.write(90 - WALL_TURN_DEG);
        }

	// function to move cart straight when wall is detected
        void moveStraight(void) {
            leftServo.write(180);
            rightServo.write(0);
        }
    }

    // defining a namespace inside Movement namespace for TapeFollow mode
    namespace TapeFollow {

	// function to turn cart left when tape is detected
        void turnLeft(void) {
            leftServo.write(90 - TAPE_TURN_DEG);
            rightServo.write(90 - TAPE_TURN_DEG);
        }

	// function to turn cart right when tape is detected
        void turnRight(void) {
            leftServo.write(90 + TAPE_TURN_DEG);
            rightServo.write(90 + TAPE_TURN_DEG);
        }

	// function to move cart straight when tape is detected
        void moveStraight(void) {
            leftServo.write(90 + TAPE_TURN_DEG);
            rightServo.write(90 - TAPE_TURN_DEG);
        }
    }
}

// Namespace for Sensors
// getDistance() returns distance measured by ultrasound sensor
// readColor() returns the tape color detected by the color sensor
namespace Sensors {

    uint16_t getDistance(void) { // ultrasound sensor

        // send ultrasonic signal
        digitalWrite(US_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(US_TRIG_PIN, LOW);

        // get the pulse duration in ms
        const uint32_t duration = pulseIn(US_ECHO_PIN, HIGH);

        // convert to distance by using 0.034 as the speed of sound (cm/ms)
        return duration * 0.034 / 2;
    }

    TapeColor readColor(void) { // color sensor

        // detecting red color frequency
        digitalWrite(CS_S2, LOW);
        digitalWrite(CS_S3, LOW);
        const uint32_t redFrequency = pulseIn(CS_OUT, LOW);
        delay(CS_INTERNAL_DELAY);

        // detecting green color frequency
        digitalWrite(CS_S2, HIGH);
        digitalWrite(CS_S3, HIGH);
        const uint32_t greenFrequency = pulseIn(CS_OUT, LOW);
        delay(CS_INTERNAL_DELAY);

        // detecting blue color frequency
        digitalWrite(CS_S2, LOW);
        digitalWrite(CS_S3, HIGH);
        const uint32_t blueFrequency = pulseIn(CS_OUT, LOW);
        delay(CS_INTERNAL_DELAY);

        // detecting clear (no filter) frequency
        digitalWrite(CS_S2, HIGH);
        digitalWrite(CS_S3, LOW);
        const uint32_t clearFrequency = pulseIn(CS_OUT, LOW);
        delay(CS_INTERNAL_DELAY);

        // calculate color ratios
        const float redRatio = (float) redFrequency / clearFrequency;
        const float greenRatio = (float) greenFrequency / clearFrequency;
        const float blueRatio = (float) blueFrequency / clearFrequency;

        // calculate differences for each tape color
        const uint32_t midTapeDiff = abs(MID_TAPE_R - redRatio) + abs(MID_TAPE_G - greenRatio) + abs(MID_TAPE_B - blueRatio);
        const uint32_t leftTapeDiff = abs(LEFT_TAPE_R - redRatio) + abs(LEFT_TAPE_G - greenRatio) + abs(LEFT_TAPE_B - blueRatio);
        const uint32_t rightTapeDiff = abs(RIGHT_TAPE_R - redRatio) + abs(RIGHT_TAPE_G - greenRatio) + abs(RIGHT_TAPE_B - blueRatio);

        // return value corresponding with minimum difference
        return ((midTapeDiff < leftTapeDiff) ? ((rightTapeDiff < midTapeDiff) ? TapeColor::RightTape : TapeColor::MidTape) : ((leftTapeDiff < rightTapeDiff) ? TapeColor::LeftTape : TapeColor::RightTape));
    }
}

// The following function initializes the hardware components required for the robot's operation.
void setup() {

    // Start serial communication at 9600 baud rate
    Serial.begin(9600);

    // Attach servos to the corresponding pins and stop moving the robot
    leftServo.attach(LEFT_SERVO_PIN);
    rightServo.attach(RIGHT_SERVO_PIN);
    Movement::stopMoving();

    // Set up pins for ultrasound sensor
    pinMode(US_TRIG_PIN, OUTPUT);
    pinMode(US_ECHO_PIN, INPUT);
    digitalWrite(US_TRIG_PIN, LOW);

    // Set up pins for color sensor
    pinMode(CS_S0, OUTPUT);
    pinMode(CS_S1, OUTPUT);
    pinMode(CS_S2, OUTPUT);
    pinMode(CS_S3, OUTPUT);
    pinMode(CS_OUT, INPUT);
    pinMode(CS_LED, OUTPUT);

    // Enable LED lights on color sensor
    digitalWrite(CS_LED, HIGH);

    // Set color sensor frequency to 20%
    digitalWrite(CS_S0, HIGH);
    digitalWrite(CS_S1, LOW);
    digitalWrite(CS_S2, LOW);
    digitalWrite(CS_S3, LOW);

    // Initialize IR receiver for remote control
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {

    // checks the current mode
    if (currMode == SysMode::RemoteControl) {

	// check if infrared signal has been received
        if (IrReceiver.decode()) {
		
	    // check if the signal is a repeat
            const bool isRepeating = (IrReceiver.decodedIRData.flags == IRDATA_FLAGS_IS_REPEAT);
			
	    // get the command from the signal
            const uint8_t command = IrReceiver.decodedIRData.command;

            // check if the signal is a movement command
            if (command == static_cast < uint8_t > (RemoteButtons::Up)) {
                waitIR = WAIT_IR_SIGNAL_BEFORE_STOP; // set waitIR time before stopping
                Movement::RemoteControl::moveForward(); // move forward
            } else if (command == static_cast < uint8_t > (RemoteButtons::Left)) {
                waitIR = WAIT_IR_SIGNAL_BEFORE_STOP; // set waitIR time before stopping
                Movement::RemoteControl::turnLeft(); // turn left
            } else if (command == static_cast < uint8_t > (RemoteButtons::Right)) {
                waitIR = WAIT_IR_SIGNAL_BEFORE_STOP; // set waitIR time before stopping
                Movement::RemoteControl::turnRight(); // turn right
            } else if (command == static_cast < uint8_t > (RemoteButtons::Down)) {
                waitIR = WAIT_IR_SIGNAL_BEFORE_STOP; // set waitIR time before stopping
                Movement::RemoteControl::moveBackward(); // move backward
            }

            // check if the signal is a mode change command
            else if (command == static_cast < uint8_t > (RemoteButtons::Mode2)) {
                if (!isRepeating) { // only execute the first time the button is pressed
                    noWallCount = 0; // reset the noWallCount variable
                    currMode = SysMode::WallFollow; // change to WallFollow mode
                    Movement::stopMoving(); // stop the cart
                    delay(MOVEMENT_SMALL_DELAY); // wait for a short amount of time
                }
            } else if (command == static_cast < uint8_t > (RemoteButtons::Mode3)) {
                if (!isRepeating) { // only execute the first time the button is pressed
                    currMode = SysMode::TapeFollow; // change to TapeFollow mode
                    Movement::stopMoving(); // stop the cart
                    delay(MOVEMENT_SMALL_DELAY); // wait for a short amount of time
                }
            }

            IrReceiver.resume(); // re-enable input reading from the sensor
        }

        // if no valid command is received, stop the cart after a set amount of time
        else {
		
            // decrement waitIR until 1, then stop the cart and set waitIR to 0
            if (waitIR) {
                if (waitIR > 1) {
                    --waitIR; // decrement waitIR time
                    delay(1); // wait for a short amount of time
                } else {
                    Movement::stopMoving();  // stop the cart
                    waitIR = 0; // reset the waitIR variable
                }
            }
        }

    } else if (currMode == SysMode::WallFollow) {
	
	// check if infrared signal has been received
        if (IrReceiver.decode()) {
		
            // check if the signal is a mode change command
            if (IrReceiver.decodedIRData.command == static_cast < uint8_t > (RemoteButtons::Mode1)) {
                currMode = SysMode::RemoteControl; // change to RemoteControl mode
                Movement::stopMoving();
            }
            IrReceiver.resume();

        } else { // start follow the wall task

            // get distance from ultrasonic sensor
            const uint16_t distance = Sensors::getDistance();

            if (distance > NO_WALL_DIST) { // no wall detected

                // the cart stops if there is no wall for some time
                if (noWallCount >= NO_WALL_MAX_COUNT) {
                    Movement::stopMoving();
                    currMode = SysMode::RemoteControl;
                } else {
                    Movement::WallFollow::moveStraight();
                    noWallCount++;
                }

                return; // skips rest of code for performance
            }

            noWallCount = 0; // set count to 0 if it detected a wall

            if (distance < SMALL_DIST) { // turns right if too close
                Movement::WallFollow::turnRight();
                delay(MOVEMENT_BIG_DELAY);
                Movement::WallFollow::moveStraight();
                delay(MOVEMENT_SMALL_DELAY);
            } else if (distance < BIG_DIST) { // goes straight if good distance
                Movement::WallFollow::moveStraight();
                delay(MOVEMENT_BIG_DELAY);
            } else { // turns left if too far
                Movement::WallFollow::turnLeft();
                delay(MOVEMENT_BIG_DELAY);
                Movement::WallFollow::moveStraight();
                delay(MOVEMENT_SMALL_DELAY);
            }
        }

    } else if (currMode == SysMode::TapeFollow) {

        if (IrReceiver.decode()) { // receive infrared signal with sensor
            // change to remote control if button 1 is pressed
            if (IrReceiver.decodedIRData.command ==
                static_cast < uint8_t > (RemoteButtons::Mode1)) {
                currMode = SysMode::RemoteControl;
                Movement::stopMoving();
            }
            IrReceiver.resume();
        } else { // start follow the tape task

            // detect tape color with color sensor
            const TapeColor c = Sensors::readColor();

            // move cart according to tape color
            if (c == TapeColor::MidTape) {
                Movement::TapeFollow::moveStraight();
                delay(MOVEMENT_BIG_DELAY);
            } else if (c == TapeColor::LeftTape) {
                Movement::TapeFollow::turnRight();
                delay(MOVEMENT_SMALL_DELAY);
            } else if (c == TapeColor::RightTape) {
                Movement::TapeFollow::turnLeft();
                delay(MOVEMENT_SMALL_DELAY);
            }
            Movement::stopMoving();
            delay(MOVEMENT_SMALL_DELAY);
        }
    }
}

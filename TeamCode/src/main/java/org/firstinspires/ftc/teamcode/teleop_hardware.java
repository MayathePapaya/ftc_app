package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.matrix.MatrixDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an OpMode
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot, using Matrix Hardware.
 * See PushbotTeleopTank_Iterative for a usage examples.
 *
 * This is coded as an Extension of HardwarePushbot to illustrate that the only additional
 * action REQUIRED for a MATRIX controller is enabling the Servos.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 *  Matrix Controller has been assigned the name:  "matrix controller"
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 *
 * In addition, the Matrix Controller has been assigned the name:  "matrix controller"
 */
public class teleop_hardware
{
    /* Public OpMode members. */
    public DcMotor leftMotor1 = null;
    public DcMotor rightMotor1 = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor2 = null;
    //public DcMotor  armMotor    = null;
    public CRServo beaconPusher = null;
    public CRServo shovel = null;

    public static final double OUT_OF_THE_WAY_SERVO = 0.56;
    public static final double SHOVEL_FORWARD_POWER = 0.45;
    public static final double SHOVEL_BACKWARD_POWER = -0.45;

//    /* Private OpMode members. */
//    private MatrixDcMotorController matrixMotorController = null;
//    private ServoController matrixServoController = null;

    /* Constructor */
    public teleop_hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        leftMotor1 = ahwMap.dcMotor.get("leftMotor1");
        leftMotor2 = ahwMap.dcMotor.get("leftMotor2");
        rightMotor1 = ahwMap.dcMotor.get("rightMotor1");
        rightMotor2 = ahwMap.dcMotor.get("rightMotor2");

        leftMotor1.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);

        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);

        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        beaconPusher = ahwMap.crservo.get("beaconPusher");
        shovel = ahwMap.crservo.get("shovel");

    }



        // Initialize base Motor and Servo objects
        //super.init(ahwMap);

        /*
         * Matrix controllers are special.
         *
         * A Matrix controller is one controller with both motors and servos
         * but software wants to treat it as two distinct controllers, one
         * DcMotorController, and one ServoController.
         *
         * We accomplish this by initializing Motor and Servo controller with the same name
         * given in the configuration.  In the example below the name of the controller is
         * "MatrixController"
         *
         * Normally we don't need to access the controllers themselves, we deal directly with
         * the Motor and Servo objects, but the Matrix interface is different.
         *
         * In order to activate the servos, they need to be enabled on the controller with
         * a call to pwmEnable() and disabled with a call to pwmDisable()
         *
         * Also, the Matrix Motor controller interface provides a call that enables all motors to
         * updated simultaneously (with the same value).
         */

//        // Initialize Matrix Motor and Servo objects
//        matrixMotorController = (MatrixDcMotorController)ahwMap.dcMotorController.get("matrix controller");
//        matrixServoController = ahwMap.servoController.get("matrix controller");
//
//        // Enable Servos
//        matrixServoController.pwmEnable();       // Don't forget to enable Matrix Output
    }

//    /*
//         *
//         * This is an example LinearOpMode that shows how to use
//         * a legacy (NXT-compatible) Hitechnic Color Sensor v2.
//         * It assumes that the color sensor is configured with a name of "sensor_color".
//         *
//         * You can use the X button on gamepad1 to toggle the LED on and off.
//         *
//         * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
//         * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
//         */
//    @Autonomous(name = "Sensor: HT color", group = "Sensor")
//    @Disabled
//    public static class SensorHTColor extends LinearOpMode {
//
//        ColorSensor colorSensor;  // Hardware Device Object
//
//
//        @Override
//        public void runOpMode() {
//
//            // hsvValues is an array that will hold the hue, saturation, and value information.
//            float hsvValues[] = {0F, 0F, 0F};
//
//            // values is a reference to the hsvValues array.
//            final float values[] = hsvValues;
//
//            // get a reference to the RelativeLayout so we can change the background
//            // color of the Robot Controller app to match the hue detected by the RGB sensor.
//            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
//
//            // bPrevState and bCurrState represent the previous and current state of the button.
//            boolean bPrevState = false;
//            boolean bCurrState = false;
//
//            // bLedOn represents the state of the LED.
//            boolean bLedOn = true;
//
//            // get a reference to our ColorSensor object.
//            colorSensor = hardwareMap.colorSensor.get("sensor_color");
//
//            // turn the LED on in the beginning, just so user will know that the sensor is active.
//            colorSensor.enableLed(bLedOn);
//
//            // wait for the start button to be pressed.
//            waitForStart();
//
//            // loop and read the RGB data.
//            // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
//            while (opModeIsActive()) {
//
//                // check the status of the x button on either gamepad.
//                bCurrState = gamepad1.x;
//
//                // check for button state transitions.
//                if ((bCurrState == true) && (bCurrState != bPrevState)) {
//
//                    // button is transitioning to a pressed state.  Toggle LED.
//                    // on button press, enable the LED.
//                    bLedOn = !bLedOn;
//                    colorSensor.enableLed(bLedOn);
//                }
//
//                // update previous state variable.
//                bPrevState = bCurrState;
//
//                // convert the RGB values to HSV values.
//                Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
//
//                // send the info back to driver station using telemetry function.
//                telemetry.addData("LED", bLedOn ? "On" : "Off");
//                telemetry.addData("Clear", colorSensor.alpha());
//                telemetry.addData("Red  ", colorSensor.red());
//                telemetry.addData("Green", colorSensor.green());
//                telemetry.addData("Blue ", colorSensor.blue());
//                telemetry.addData("Hue", hsvValues[0]);
//
//                // change the background color to match the color detected by the RGB sensor.
//                // pass a reference to the hue, saturation, and value array as an argument
//                // to the HSVToColor method.
//                relativeLayout.post(new Runnable() {
//                    public void run() {
//                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
//                    }
//                });
//
//                telemetry.update();
//            }
//        }

//        /**
//         * This is NOT an opmode.
//         * <p>
//         * This class can be used to define all the specific hardware for a single robot.
//         * In this case that robot is a Pushbot.
//         * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
//         * <p>
//         * This hardware class assumes the following device names have been configured on the robot:
//         * Note:  All names are lower case and some have single spaces between words.
//         * <p>
//         * Motor channel:  Left  drive motor:        "left_drive"
//         * Motor channel:  Right drive motor:        "right_drive"
//         * Motor channel:  Manipulator drive motor:  "left_arm"
//         * Servo channel:  Servo to open left claw:  "left_hand"
//         * Servo channel:  Servo to open right claw: "right_hand"
//         */
//        public static class HardwarePushbot {
//            /* Public OpMode members. */
//            public DcMotor leftMotor1 = null;
//            public DcMotor rightMotor1 = null;
//            public DcMotor leftMotor2 = null;
//            public DcMotor rightMotor2 = null;
//            //public DcMotor  armMotor    = null;
//            public Servo beaconPusher = null;
//            public CRServo shovel = null;
//
//            public static final double OUT_OF_THE_WAY_SERVO = 0.56;
//            public static final double SHOVEL_FORWARD_POWER = 0.45;
//            public static final double SHOVEL_BACKWARD_POWER = -0.45;
//
//            /* local OpMode members. */
//            HardwareMap hwMap = null;
//            private ElapsedTime period = new ElapsedTime();
//
//            /* Constructor */
//            public HardwarePushbot() {
//
//            }
//
//            /* Initialize standard Hardware interfaces */
//            public void init(HardwareMap ahwMap) {
//                // Save reference to Hardware map
//                hwMap = ahwMap;
//
//                // Define and Initialize Motors
//                leftMotor1 = hwMap.dcMotor.get("left_drive");
//                rightMotor1 = hwMap.dcMotor.get("right_drive");
//                leftMotor2 = hwMap.dcMotor.get("left_drive");
//                rightMotor2 = hwMap.dcMotor.get("right_drive");
//                //armMotor    = hwMap.dcMotor.get("left_arm");
//                leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//                rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//                leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//                rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//
//                // Set all motors to zero power
//
//                leftMotor1.setPower(0);
//                rightMotor1.setPower(0);
//                leftMotor2.setPower(0);
//                rightMotor2.setPower(0);
//                //armMotor.setPower(0);
//
//
//                // Set all motors to run without encoders.
//                // May want to use RUN_USING_ENCODERS if encoders are installed.
//                leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//                // Define and initialize ALL installed servos.
//                beaconPusher = hwMap.servo.get("left_hand");
//                //shovel = hwMap.servo.get("right_hand");
//                //beaconPusher.setPosition(MID_SERVO);
//                //shovel;
//            }
//
//            /***
//             * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
//             * periodic tick.  This is used to compensate for varying processing times for each cycle.
//             * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
//             *
//             * @param periodMs Length of wait cycle in mSec.
//             */
//            public void waitForTick(long periodMs) {
//
//                long remaining = periodMs - (long) period.milliseconds();
//
//                // sleep for the remaining portion of the regular cycle period.
//                if (remaining > 0) {
//                    try {
//                        Thread.sleep(remaining);
//                    } catch (InterruptedException e) {
//                        Thread.currentThread().interrupt();
//                    }
//                }
//
//                // Reset the cycle clock for the next pass.
//                period.reset();
//            }
//        }

//}
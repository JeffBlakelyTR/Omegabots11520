package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 *
 * Claw Movement Motor: 117 RPM, 68.4 kg-cm -- 1,425.1 PPR at the Output Shaft
 * ForeArm Motor:       60 RPM, 133.2 kg-cm  -- 2,786.2 PPR at the Output Shaft
 * Biceps Motor:        60 RPM, 133.2 kg-cm  -- 2,786.2 PPR at the Output Shaft
 */
public class SpringRobotHardware {

    // Declare OpMode members.
    private ElapsedTime period = new ElapsedTime();

    public DcMotor leftrear = null;
    public DcMotor rightrear = null;
    public DcMotor leftfront = null;
    public DcMotor rightfront = null;
    public DcMotor linearSlide = null;
    public Servo clawL;
    public Servo clawR;

    //public Servo dispatcher;
    public static final double MID_SERVO = 0.5;
    final double DISPATCHER_SPEED = 0.00;
    double dispatcherOffset = 0.3;
    public static final double CONTINUOUS_SERVO_STOP = 0.6;
    public static final double CONTINUOUS_SERVO_FORWARD = 1.0;
    public static final double CONTINUOUS_SERVO_REVERSE = 0.0;
    double dispatchUpPosition = 0.0;
    double dispatchDownPosition = 0.0;

    //public Servo marker;
   // public static final double MID_SERVO = 0.5;

    //public Servo dispatchservo;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    //boolean isEncodersReqd = false;

    /* Constructor */
    public SpringRobotHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        leftrear = hwMap.get(DcMotor.class, "RearL");
        rightrear = hwMap.get(DcMotor.class, "RearR");
        leftfront = hwMap.get(DcMotor.class, "FrontL");
        rightfront = hwMap.get(DcMotor.class, "FrontR");
        linearSlide = hwMap.get(DcMotor.class, "linearSlide");
        clawL  = hwMap.get(Servo.class, "clawL");
        clawR = hwMap.get(Servo.class, "clawR");
        // clawL.setPosition(0.75);
        // clawR.setPosition(0.3);
        // Get the touch sensor and motor from hardwareMap

        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our compass
       // rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "RangeSensor");


        /*linearslide= hwMap.get(DcMotor.class, "linearslide");
        linearrotation = hwMap.get(DcMotor.class, "linearrotation");
        collector = hwMap.get(DcMotor.class, "collector");
        lift = hwMap.get(DcMotor.class, "lift");
        //dispatcher =hwMap.servo.get("dispatcher");

        marker  = hwMap.get(Servo.class, "marker");
        dispatcher.setPosition(0.2);
        marker.setPosition(0.47);
        //dispatchservo = hwMap.servo.get("dispatchservo");
       //dispatchservo.setPosition(CONTINUOUS_SERVO_STOP) ;*/



        // Set the drive motor direction:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // These polarities are for the Neverest 20 motors
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftrear.setDirection(DcMotor.Direction.REVERSE);
        rightrear.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);

        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
        // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)
        /*leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        leftrear.setPower(0);
        rightrear.setPower(0);
        leftfront.setPower(0);
        rightfront.setPower(0);
        linearSlide.setPower(0);

       /* lift.setDirection(DcMotor.Direction.FORWARD);
        linearslide.setDirection(DcMotor.Direction.FORWARD);
        linearrotation.setDirection(DcMotor.Direction.FORWARD);
        collector.setDirection(DcMotor.Direction.FORWARD);

        linearslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearrotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        lift.setPower(0);
        linearslide.setPower(0);
        collector.setPower(0);*/
        // Wait for the game to start (driver presses PLAY)
        // waitForStart();
        // runtime.reset();
    }

    public void enableEncoders() {
        leftrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetEncoders() {
        leftrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*public void resetLinearSlideEncoder() {
        linearslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearslide.setDirection(DcMotor.Direction.FORWARD);
        linearslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearslide.setPower(0);
    }
    public void resetLinearRotationMotorEncoder() {
        linearrotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearrotation.setDirection(DcMotor.Direction.FORWARD);
        linearrotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearrotation.setPower(0);
    }
    public void disableEncoders() {
        leftrear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightrear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }*/


}


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.SpringRobotHardware;

/**
 * This program provides driver station control of the Team 9960 Mecanum Drive Prototype.
 *
 * This robot uses four VEX Mecanum wheels, each direct driven by Neverest 20 motors.
 * It is designed as a linear op mode, and uses RUN_WITH_ENCODER motor operation.
 *
 * The gamepad1 right joystick is used for translation movement, while the left joystick x-axis controls rotation.
 *
 * Claw Movement Motor: 117 RPM, 68.4 kg-cm -- 1,425.1 PPR at the Output Shaft
 * ForeArm Motor:       60 RPM, 133.2 kg-cm  -- 2,786.2 PPR at the Output Shaft
 * Biceps Motor:        60 RPM, 133.2 kg-cm  -- 2,786.2 PPR at the Output Shaft
 *
 */

//@TeleOp(name="RoboNextGenMecanumDrive", group="RoboNextGen Opmode") // @Autonomous(...) is the other common choice
//@Disabled
public class RoboNextGenMecanumDrive extends LinearOpMode {

    /* Declare OpMode members. */
    SpringRobotHardware robot   = new SpringRobotHardware();   // Use a Pushbot's hardware
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // declare motor speed variables
    double RF; double LF; double RR; double LR;
    // declare motor speed variables
    double RF2; double LF2; double RR2; double LR2;
    // declare joystick position variables
    double X1; double Y1; double X2; double Y2;

    int linearSlideInit; int foreArmVar; int bicepArmInit; int bicepArmVar; int clawInit;
    boolean clawServoClosed = true;
    boolean runMotorWithHighSpeed = true;


    // declare joystick position variables for controller2
    double XX1; double YY1; double XX2; double YY2;    // operational constants
    double joyScale = 0.8;
    double motorMax = 0.8; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    double foreArmMotorMax = 0.6;

    double motorMaxForStick0 = 0.0;//default
    double motorMaxForStick1 = 0.6;//default
    double motorMaxForStick2 = 0.5;
    double motorMaxForStick = 0.8;//default
    double motorMaxForButton0 = 0.0;//default
    double motorMaxForButton1 = 0.3;//default
    double motorMaxForButton2 = 0.2;
    double motorSpeedForDpad = 0.9;
    double motorSpeedForDpadRotate = 0.8;
    double motorMaxSpeed = 1.0;
    double motorSpeedForRotation = 0.7;

    double leftStickSpeed = 0.4;
    double leftStickSpeedNormal = 0.4;
    double leftStickSpeedToggle = 0.3;
    double buttonSpeed = 0.5;
    double rightStickSpeed = 0.7;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.enableEncoders();
        linearSlideInit = robot.linearSlide.getCurrentPosition();
        //foreArmVar = robot.foreArmMotor.getCurrentPosition();
       // bicepArmVar = robot.bicepsMotor.getCurrentPosition();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        motorMaxForStick0 = motorMaxForStick1;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // Reset speed variables
            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;


//DRIVE TRAIN CODE FOR MEC WHEELS

    //-------------------GAME PAD-1 CODE STARTS HERE-----------------------------------------------
            if (gamepad1.y) { // Gamepad vertical up button
               /* robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(buttonSpeed);*/

                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(buttonSpeed);
            } else if (gamepad1.a) {// Gamepad vertical down button
                /*robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(buttonSpeed);*/
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(buttonSpeed);
            } else if (gamepad1.b) { // Gamepad horizontal right button
                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(buttonSpeed);
            } else if (gamepad1.x) {// Gamepad horizontal left button
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(buttonSpeed);
            } else if (gamepad1.right_stick_x >=-1.0 && gamepad1.right_stick_x < 0.0) { //Move Forward
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(rightStickSpeed);

            } else  if (gamepad1.right_stick_x > 0.0 &&  gamepad1.right_stick_x < 1.0) { //Move Backward
                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(rightStickSpeed);

            }else  if (gamepad1.right_stick_y >0.0 &&  gamepad1.right_stick_y< 1.0) { // Move horizontally right
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(rightStickSpeed);

            } else  if (gamepad1.right_stick_y >=-1.0 &&  gamepad1.right_stick_y < 0.0) { // Move horizontally left
                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(rightStickSpeed);

            }else if (gamepad1.left_stick_y >=-1.0 && gamepad1.left_stick_y < 0.0) { //Move Forward
                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(leftStickSpeed);

            } else  if (gamepad1.left_stick_y > 0.0 &&  gamepad1.left_stick_y < 1.0) { //Move Backward
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(leftStickSpeed);

            }else  if  (gamepad1.left_stick_x > 0.0 &&  gamepad1.left_stick_x < 1.0) { //Rotate right
                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(leftStickSpeed);



            }else  if (gamepad1.left_stick_x >=-1.0 &&  gamepad1.left_stick_x < 0.0) { //Rotate left
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(leftStickSpeed);




            } else if (gamepad1.dpad_up) { // Rotate right
                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(motorSpeedForDpadRotate);
            } else if (gamepad1.dpad_down) { //Rotate left
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(motorSpeedForDpadRotate);
            } else if (gamepad1.dpad_right) { // Move Forward
                motorMaxForStick1 = motorMaxForStick;
                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(motorSpeedForDpad);
                //resetEncoders();
            } else if (gamepad1.dpad_left) { // Move Backward
                motorMaxForStick1 = motorMaxForStick;
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(motorSpeedForDpad);

            } else if (gamepad1.right_bumper) { // Straif Left Fast
                motorMaxForStick1 = motorMaxForStick;
                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(motorSpeedForDpad);
                //resetEncoders();
            } else if (gamepad1.left_bumper) { // Straif Right Fast
                motorMaxForStick1 = motorMaxForStick;
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(motorSpeedForDpad);

            } else if (gamepad1.dpad_up && gamepad1.dpad_right) {
                robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
                robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
                runMotors(motorMaxForStick);
            } else if (gamepad1.dpad_down && gamepad1.dpad_left) {
                robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
                robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
                runMotors(motorMaxForStick);
            } else{
                robot.leftrear.setPower(0.0);
                robot.rightrear.setPower(0.0);
                robot.leftfront.setPower(0.0);
                robot.rightfront.setPower(0.0);

            }

            if (gamepad1.right_bumper) {
                leftStickSpeed = leftStickSpeedToggle;

               // strafeRight(1,8);
              //  moveForward(2.0) ;
                /*if(runMotorWithHighSpeed){
                    motorMaxForStick0 = motorMaxForStick2;//0.5
                    motorSpeedForDpad = motorMaxForStick2;//0.5
                    motorSpeedForDpadRotate = motorMaxForStick2;//0.5
                    runMotorWithHighSpeed = false;

                }else{
                    motorMaxForStick0 = motorMaxForStick1;//0.8
                    motorSpeedForDpad = motorMaxSpeed;//1.0
                    motorSpeedForDpadRotate = motorMaxForStick1;//0.8
                    runMotorWithHighSpeed = true;
                }*/


            } else if (gamepad1.left_bumper) {
                leftStickSpeed = leftStickSpeedNormal;
               // strafeLeft(2);
                //moveForward(2.0) ;

            }
            /*telemetry.addData("gamepad1.left_stick_y = ", "%.3f", gamepad1.left_stick_y);
            telemetry.addData("gamepad1.left_stick_x = ", "%.3f", gamepad1.left_stick_x);
            telemetry.addData("gamepad1.right_stick_y = ", "%.3f", gamepad1.right_stick_y);
            telemetry.addData("gamepad1.right_stick_x = ", "%.3f", gamepad1.right_stick_x);

            telemetry.addData("gamepad2.left_stick_y = ", "%.3f", gamepad2.left_stick_y);
            telemetry.addData("gamepad2.left_stick_x = ", "%.3f", gamepad2.left_stick_x);
            telemetry.addData("gamepad2.right_stick_y = ", "%.3f", gamepad2.right_stick_y);
            telemetry.addData("gamepad2.right_stick_x = ", "%.3f", gamepad2.right_stick_x);*/


//---------------------------GAME PAD-1 CODE ENDS HERE-------------------------------------------
//FORE ARM MOVEMENT
            /*if(gamepad2.right_stick_y != 0.0 && gamepad2.right_stick_y > 0.0){
                robot.foreArmMotor.setDirection(DcMotor.Direction.REVERSE);
                robot.foreArmMotor.setPower(0.3);
            }else if(gamepad2.right_stick_y != 0.0 && gamepad2.right_stick_y < 0.0){

                robot.foreArmMotor.setDirection(DcMotor.Direction.FORWARD);
                robot.foreArmMotor.setPower(0.3);
            }else{
                robot.foreArmMotor.setPower(0.0);
            }*/

//---------------------------GAME PAD-2 CODE STARTS HERE-------------------------------------------
            if (gamepad2.y) { // Gamepad vertical up button
                robot.linearSlide.setTargetPosition(100);
            } else if (gamepad2.a) {// Gamepad vertical down button
                robot.linearSlide.setTargetPosition(200);
            } else if (gamepad2.b) { // Gamepad horizontal right button

            } else if (gamepad2.x) {// Gamepad horizontal left button

            } else {
                robot.linearSlide.setPower(0.0);
            }


            if (gamepad2.left_bumper) {
                robot.clawL.setPosition(0.75);
                robot.clawR.setPosition(0.3);

            } else if (gamepad2.right_bumper) {
                robot.clawL.setPosition(0.5);
                robot.clawR.setPosition(0.6);



            }


            if (gamepad2.dpad_up ){

            }else if ( gamepad2.dpad_down){
            } else if (gamepad2.dpad_right) { //For High Station Preset
                //runClawWithEncoder(1,"forward");


            } else if (gamepad2.dpad_left) {//For High Station Preset


                //runClawWithEncoder(1,"reverse");
            }
            else{

            }
//For Medium Station Preset
            if (gamepad2.right_stick_y >=-1.0 && gamepad2.right_stick_y < 0.0) { //Preset for Medium Station
               // runForeArmAndBicepsPresetForMediumAndLowStations(4700, "medium");

            } else  if (gamepad2.right_stick_y > 0.0 &&  gamepad2.right_stick_y < 1.0) { //Resetting Preset for Medium Station
                //resetRunForeArmAndBicepsPresetForMediumAndLowStations(4700);
            }

//For Small Station Preset
            if (gamepad2.left_stick_y >=-1.0 && gamepad2.left_stick_y < 0.0) { //Preset for Medium Station
               // runForeArmAndBicepsPresetForMediumAndLowStations(3162, "low");
                //runBicepsForLowPreset();

            } else  if (gamepad2.left_stick_y > 0.0 &&  gamepad2.left_stick_y < 1.0) { //Resetting Preset for Medium Station
                //resetRunForeArmAndBicepsPresetForMediumAndLowStations(3162);
               // resetRunBicepsForLowPreset();
            }


//===============================================================================================




            // Send some useful parameters to the driver station
            /*telemetry.addData("LF", "%.3f", LF);
            telemetry.addData("RF", "%.3f", RF);
            telemetry.addData("LR", "%.3f", LR);
            telemetry.addData("RR", "%.3f", RR);
            telemetry.addData("servoOffset", "%.3f", servoOffset);
            telemetry.addData("MagneticTouch.isPressed()=", robot.magneticTouch.isPressed());
            telemetry.addData("ForeArmMagTouch.isPressed()=", robot.foreArmMagTouch.isPressed());
            telemetry.addData("ForeArmMagTouch.isPressed()=", robot.bicepArmMagTouch.isPressed());

            telemetry.addData("ForeArmInitial", "%s", foreArmInit);
            telemetry.addData("ForeArmFinal", "%s", robot.foreArmMotor.getCurrentPosition());

            telemetry.addData("BicepArmInitial", "%s", bicepArmInit);
            telemetry.addData("BicepArmFinal", "%s", robot.bicepsMotor.getCurrentPosition());

            telemetry.addData("ClawInitial", "%s", clawInit);
            telemetry.addData("ClawFinal", "%s", robot.claw.getCurrentPosition());

            /*telemetry.addData("raw ultrasonic", robot.rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", robot.rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", robot.rangeSensor.cmOptical());*/
            telemetry.addData("leftfront", "%s", robot.leftfront.getCurrentPosition());
            telemetry.addData("rightfront", "%s", robot.rightfront.getCurrentPosition());
            telemetry.addData("leftrear", "%s", robot.leftrear.getCurrentPosition());
            telemetry.addData("rightrear", "%s", robot.rightrear.getCurrentPosition());
            telemetry.addData("linearslide", "%s", robot.linearSlide.getCurrentPosition());


            //telemetry.addData("xVal", xVal);

        }
    }
    private void runMotors( double power){
        robot.leftrear.setPower(power);
        robot.rightrear.setPower(power);
        robot.leftfront.setPower(power);
        robot.rightfront.setPower(power);
    }

    private void stopMotors(){
        robot.leftrear.setPower(0.0);
        robot.rightrear.setPower(0.0);
        robot.leftfront.setPower(0.0);
        robot.rightfront.setPower(0.0);
    }



    public void runClawWithEncoder( int rotations, String direction, int ticks) {
        //robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Initialize the motor to run using encoders
        int fixedTicks1 = 800;
        int fixedTicks2 = 1600;
        int fixedTicks3 = 2200;
        ticks = robot.linearSlide.getCurrentPosition();
            // Put loop blocks here.
            //robot.claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the encoder to 0 position
            // Assign motor to run to position

            if(direction != null && direction.equalsIgnoreCase("forward")){

                    robot.linearSlide.setDirection(DcMotor.Direction.FORWARD);
                    robot.linearSlide.setPower(0.3);

               /* robot.claw.setDirection(DcMotor.Direction.FORWARD);
                robot.claw.setTargetPosition(ticks);
                robot.claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.claw.setPower(0.4);*/
               // robot.linearSlide.setDirection(DcMotor.Direction.FORWARD);
                //robot.linearSlide.setPower(0.4);

            }else if(direction != null && direction.equalsIgnoreCase("reverse")) {



                        robot.linearSlide.setDirection(DcMotor.Direction.REVERSE);
                        robot.linearSlide.setPower(0.3);

                }
                robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            /*while (robot.claw.isBusy()) {
                // While the motor is still making its way to 480, do nothing
            }
            robot.claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.claw.setPower(0.0);*/

    //}

    public void encoderDrive(double speed,String direction) {

        if(direction != null && direction.equalsIgnoreCase("forward")){
            robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
            robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
            robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
            robot.rightrear.setDirection(DcMotor.Direction.REVERSE);

        } else if(direction != null && direction.equalsIgnoreCase("reverse")){
            robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
            robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
            robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
            robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
        }
        robot.leftrear.setPower(speed);
        robot.rightrear.setPower(speed);
        robot.leftfront.setPower(speed);
        robot.rightfront.setPower(speed);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    private void resetEncoders(){
        robot.leftrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightrear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderStrafeLeft() {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget =  1715;
            newLeftTarget2 = 1715;
            newRightTarget = 3330;
            newRightTarget2 = 3220;

//LF2 = D_PAD_POWER; RF2 = -D_PAD_POWER; LR2 = -D_PAD_POWER; RR2 = D_PAD_POWER;//Strafe left
            // LF2 = -D_PAD_POWER; RF2 = D_PAD_POWER; LR2 = D_PAD_POWER; RR2 = -D_PAD_POWER;
            robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
            robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
            robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
            robot.rightrear.setDirection(DcMotor.Direction.FORWARD);

            robot.leftrear.setTargetPosition(newLeftTarget);
            robot.leftfront.setTargetPosition(newLeftTarget2);
            robot.rightrear.setTargetPosition(newRightTarget);
            robot.rightfront.setTargetPosition(newRightTarget2);


            // Turn On RUN_TO_POSITION
            robot.leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftrear.setPower(0.5);
            robot.leftfront.setPower(0.5);

            robot.rightrear.setPower(0.5);
            robot.rightfront.setPower(0.5);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&

                    (robot.leftrear.isBusy() && robot.rightrear.isBusy()) && (robot.leftfront.isBusy() && robot.rightfront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftrear.getCurrentPosition(),
                        robot.rightrear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            // robot.leftDrive.setPower(0);
            //robot.rightDrive.setPower(0);
            robot.leftrear.setPower(0);
            robot.leftfront.setPower(0);

            robot.rightrear.setPower(0);
            robot.rightfront.setPower(0);

            // Turn off RUN_TO_POSITION
            resetEncoders();

            //  sleep(250);   // optional pause after each move
        }


    }
    private void moveForward(double seconds){
        runtime.reset();
        robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
        robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
        robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
        robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
        runMotors(buttonSpeed);

        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMotors();
    }
    private void strafeLeft(double seconds){
        runtime.reset();
        robot.leftfront.setDirection(DcMotor.Direction.FORWARD);
        robot.rightfront.setDirection(DcMotor.Direction.REVERSE);
        robot.leftrear.setDirection(DcMotor.Direction.REVERSE);
        robot.rightrear.setDirection(DcMotor.Direction.FORWARD);
        runMotors(buttonSpeed);

        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMotors();
    }
    private void strafeRight(double seconds){
        runtime.reset();
        robot.leftfront.setDirection(DcMotor.Direction.REVERSE);
        robot.rightfront.setDirection(DcMotor.Direction.FORWARD);
        robot.leftrear.setDirection(DcMotor.Direction.FORWARD);
        robot.rightrear.setDirection(DcMotor.Direction.REVERSE);
        runMotors(buttonSpeed);

        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMotors();
    }
}
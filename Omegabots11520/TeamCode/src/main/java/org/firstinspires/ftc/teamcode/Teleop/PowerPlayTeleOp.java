package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot.AtonRobotHardware;

@TeleOp(name="PowerPlayTeleOp", group="OmegaBots Opmode") // @Autonomous(...) is the other common choice
public class PowerPlayTeleOp extends OpMode {
    public AtonRobotHardware robot;

    @Override
    public void init() {
        this.robot = new AtonRobotHardware(this);
        robot.Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double slowFactor = 1;

        // Claw controls
        // OPEN NARROW
        //if(gamepad2.a){
        //    robot.IntakeMotorL.setClawLPosition(0.6);
        //    robot.IntakeMotorR.setClawRPosition(0.5);
        //}
        // OPEN WIDE
        if(gamepad2.a){
            robot.IntakeMotorL.setClawLPosition(0.75);
            robot.IntakeMotorR.setClawRPosition(0.45);
        }
        //CLOSE
        if(gamepad2.b){
            robot.IntakeMotorL.setClawLPosition(0.5);
            robot.IntakeMotorR.setClawRPosition(0.6);
        }
        //WIDE OPEN
        if(gamepad2.a){
            robot.IntakeMotorL.setClawLPosition(0.75);
            robot.IntakeMotorR.setClawRPosition(0.45);
        }
        //CLOSE
        if(gamepad2.b){
            robot.IntakeMotorL.setClawLPosition(0.5);
            robot.IntakeMotorR.setClawRPosition(0.6);
        }

        if (gamepad1.right_bumper) {
            slowFactor = .5;
        }

        if (gamepad1.left_bumper) {
            slowFactor = .3;
        }

        // Drivetrain
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x * 1.1;
        double turn = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(drive)+Math.abs(strafe)+Math.abs(turn),1);

        double fl = ((drive+strafe+turn)/denominator)*slowFactor;
        double bl = ((drive-strafe+turn)/denominator)*slowFactor;
        double fr = ((drive-strafe-turn)/denominator)*slowFactor;
        double br = ((drive+strafe-turn)/denominator)*slowFactor;
        robot.Drive.setMotorPower(fl, bl, fr, br);

        //---------------------------LINEAR SLIDE UP DOWN STUFF---------------------------------------------

        // DR4B lift
        double lift = gamepad2.left_stick_y*-.8;
        robot.lift.SetMotorPower(lift);
    }
}

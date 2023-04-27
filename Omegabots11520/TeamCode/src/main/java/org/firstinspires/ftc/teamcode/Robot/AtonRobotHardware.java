package org.firstinspires.ftc.teamcode.Robot;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeClaw;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift;

public class AtonRobotHardware {
    public OpMode opMode;
    public HardwareMap hardwareMap;
    public View limit1;
    Telemetry telemetry;

    // public distance_sensor frontDist;
    public Lift lift;
    // public IntakeClaw claw;
    public DriveTrain Drive;
    public IntakeClaw IntakeMotorL;
    public IntakeClaw IntakeMotorR;
   // public distance_sensor left;
    //public distance_sensor right;
    // public Limit_Switch limit1;

    public AtonRobotHardware(OpMode op ) {

        opMode = op;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

        // initialize objects/classes
        //frontDist = new distance_sensor( hardwareMap, "front", opMode.telemetry );
        lift = new Lift(hardwareMap, opMode.telemetry);
        //claw = new IntakeClaw(hardwareMap,"ServoClaw1",opMode.telemetry);
        IntakeMotorL = new IntakeClaw(hardwareMap, "clawL",opMode.telemetry);
        IntakeMotorR = new IntakeClaw(hardwareMap,"clawR",opMode.telemetry);
        Drive = new DriveTrain(hardwareMap, opMode.telemetry);
        //left = new distance_sensor(hardwareMap, "DistanceLeft", opMode.telemetry);
        //right = new distance_sensor(hardwareMap, "DistanceRight", opMode.telemetry);
        // limit1 = new Limit_Switch(hardwareMap, "limit1", opMode.telemetry);
    }
}

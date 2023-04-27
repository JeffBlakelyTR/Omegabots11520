package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeClaw{
    Telemetry telemetry;
    public Servo IntakeMotorL;
    public Servo IntakeMotorR;

    public IntakeClaw(HardwareMap hardwareMap, String IntakeMotorName, Telemetry telemetry) {
        this.telemetry = telemetry;
        setup(hardwareMap, IntakeMotorName);
    }

    public void setup(HardwareMap hardwareMap, String IntakeMotorName) {
        IntakeMotorL = hardwareMap.get(Servo.class, "clawL");
        IntakeMotorR = hardwareMap.get(Servo.class, "clawR");

    }

    public void setClawLPosition(double position) {
        IntakeMotorL.setPosition(position);
    }
    public void setClawRPosition(double position) {
            IntakeMotorR.setPosition(position);
    }
}
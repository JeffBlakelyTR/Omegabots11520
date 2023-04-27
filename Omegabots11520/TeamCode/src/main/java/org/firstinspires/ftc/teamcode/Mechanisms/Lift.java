package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift{
    Telemetry telemetry;
    public DcMotor liftMotor1;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry ) {
        this.telemetry = telemetry;
        setup( hardwareMap );
    }

    public void setup( HardwareMap hardwareMap ) {
        liftMotor1 = hardwareMap.get(DcMotor.class,"linearSlide");

    }

    public void SetMotorPower(double Power){
        liftMotor1.setPower(Power);
    }

    public void driveLiftToPosition(double power, int liftPosition) {

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setTargetPosition(liftPosition);

        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor1.setPower(power);

        while(liftMotor1.isBusy()){
        }
    }
}



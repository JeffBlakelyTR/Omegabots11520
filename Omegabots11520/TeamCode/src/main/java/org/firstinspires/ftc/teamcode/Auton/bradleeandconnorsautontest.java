package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Robot.AtonRobotHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class bradleeandconnorsautontest extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    // Tag ID 18 from the 36h11 family
    int LEFT = 11;
    int MIDDLE = 52;
    int RIGHT = 0;

    AprilTagDetection tagOfInterest = null;

    String park;

    public AtonRobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize robot hardware
        robot = new AtonRobotHardware(this);
        robot.IntakeMotorL.setClawLPosition(0.5);
        robot.IntakeMotorR.setClawRPosition(0.6);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        //Open Webcam
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //Tag Detection
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
        }
        telemetry.update();

        /* Actually do something useful */
        if (tagOfInterest.id == LEFT) {
            //Default path is LEFT
            park = "LEFT";
            telemetry.addLine("Left parking spot detected (11)");
        } else if (tagOfInterest.id == MIDDLE) {
            park = "MIDDLE";
            telemetry.addLine("Middle parking spot detected (52)");
        } else if (tagOfInterest.id == RIGHT) {
            park = "RIGHT";
            telemetry.addLine("Right parking spot detected (0)");
        }

        telemetry.addData("robot", "press play to start");
        telemetry.update();

        waitForStart();

        if(park.equals("LEFT")){
            robot.IntakeMotorL.setClawLPosition(0.5);
            robot.IntakeMotorR.setClawRPosition(0.6);
            robot.Drive.MoveRobotToPosition(0.3, 25);
            sleep(700);
            robot.Drive.pointTurn(90,0.3);
            sleep(700);
            robot.lift.driveLiftToPosition(0.5, 1500);
            sleep(700);
            robot.Drive.MoveRobotToPosition(0.3, 8.5);
            sleep(700);
            robot.IntakeMotorL.setClawLPosition(0.75);
            robot.IntakeMotorR.setClawRPosition(0.45);
            robot.Drive.MoveRobotToPosition(0.3, -3);
            robot.Drive.pointTurn(-90,0.3);
            robot.Drive.MoveRobotToPosition(0.3, -25);
            robot.Drive.MoveRobotToPositionStrafe(0.3, -30);
        }
        else if(park.equals("MIDDLE")){
            robot.IntakeMotorL.setClawLPosition(0.5);
            robot.IntakeMotorR.setClawRPosition(0.6);
            robot.Drive.MoveRobotToPosition(0.3, 25);
            sleep(700);
            robot.Drive.pointTurn(90,0.3);
            sleep(700);
            robot.lift.driveLiftToPosition(0.5, 1500);
            sleep(700);
            robot.Drive.MoveRobotToPosition(0.3, 8.5);
            sleep(700);
            robot.IntakeMotorL.setClawLPosition(0.75);
            robot.IntakeMotorR.setClawRPosition(0.45);
            robot.Drive.MoveRobotToPosition(0.3, -3);
            robot.Drive.pointTurn(-90,0.3);
            robot.Drive.MoveRobotToPosition(0.3, -25);
            robot.Drive.MoveRobotToPositionStrafe(0.3, -5);
        }
        else if(park.equals("RIGHT")){
            robot.IntakeMotorL.setClawLPosition(0.5);
            robot.IntakeMotorR.setClawRPosition(0.6);
            robot.Drive.MoveRobotToPosition(0.3, 25);
            sleep(700);
            robot.Drive.pointTurn(90,0.3);
            sleep(700);
            robot.lift.driveLiftToPosition(0.5, 1500);
            sleep(700);
            robot.Drive.MoveRobotToPosition(0.3, 8.5);
            sleep(700);
            robot.IntakeMotorL.setClawLPosition(0.75);
            robot.IntakeMotorR.setClawRPosition(0.45);
            robot.Drive.MoveRobotToPosition(0.3, -3);
            robot.Drive.pointTurn(-90,0.3);
            robot.Drive.MoveRobotToPosition(0.3, -25);
            robot.Drive.MoveRobotToPositionStrafe(0.3, 25);
        }
        else{
            telemetry.addLine("NO Parking position found please try again.");
        }

        telemetry.update();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

}


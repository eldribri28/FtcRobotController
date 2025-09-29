package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;


import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TurretTurnLaunch")
public class TurretTurnLaunch extends LinearOpMode {

    private Gamepad gamepad;
    private DcMotorEx turretMotor;
    private DcMotorEx launcherMotor;
    private Servo angleServo;
    private Servo launchServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final PIDController bearingPid = new PIDController(0.05, 0.005, 0.05);


    public void runOpMode() {
        initialize();
        angleServo.setPosition(0);
        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {

            double targetRPM = 3700;
            if(gamepad1.right_trigger > 0) {
                launcherMotor.setVelocity(-( targetRPM / 60 ) * 28);
            } else {
                launcherMotor.setVelocity(0);
            }

            launchBall();

            //


            double flywheelRPM = ((launcherMotor.getVelocity()/28) * 60) * (33 / 30);
            telemetry.addData("shooter motor1 power", launcherMotor.getPower());
            telemetry.addData("shooter motor1 current (AMPS)", launcherMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("shooter motor1 velocity", flywheelRPM);
            telemetryAprilTag(0.3, 1.175, 0.090, flywheelRPM, 0.43);
            telemetry.update();

        }
        visionPortal.close();
    }

    private void initAprilTagProcessor() {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "TurretCam"));
        builder.addProcessor(aprilTagProcessor);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(640,480));
        visionPortal = builder.build();
    }

    private void telemetryAprilTag(double launchHeight, double targetHeight, double flywheelDiameterMeters, double flywheelRPM, double velocityTransferEfficiency) {

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        if (currentDetections.size() == 0) {
            turretMotor.setPower(0);
        } else {
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                turretMotor.setPower(-bearingPid.calculate(1, detection.ftcPose.bearing));

                LaunchResult launchResults = LaunchCalculator.CalculateShot(detection.ftcPose.range, 0.3, 1.175, .090, 3500, 0.43);

                double launchAngle = launchResults.getLaunchAngle1();

                if (!Double.isNaN(launchAngle)) {
                    setLaunchAngle(launchAngle);
                }


                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    telemetry.addData("Launch Angle", launchAngle);
                    telemetry.addData("Launch Angle", launchAngle);
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    private void launchBall() {
        if(gamepad.a == true) {
            launchServo.setPosition(0);
        } else {
            launchServo.setPosition(0.7);
        }
    }

    private void setLaunchAngle(double launchAngle) {

        //Base Angle 58deg
        //Extended Angle 15deg

        double positionValue = launchAngle / 43;
        angleServo.setPosition(positionValue);

    }

    private void initialize() {
        initializeHardware();
    }

    private void initializeHardware() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        launchServo = hardwareMap.get(Servo.class, "launchServo");

        angleServo.setDirection(Servo.Direction.REVERSE);

        this.gamepad = gamepad1;

        turretMotor.setMode(RUN_USING_ENCODER);
        launcherMotor.setMode(RUN_USING_ENCODER);
        initAprilTagProcessor();
    }


}

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TurretTurnLaunch")
public class TurretTurnLaunch extends LinearOpMode {

    private Gamepad gamepad;
    private DcMotorEx turretMotor;
    private DcMotorEx launcherMotor;
    private DcMotorEx intakeMotor;
    private Servo angleServo;
    private Servo launchServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final PIDController bearingPid = new PIDController(0.05, 0.005, 0.05);
    private final PIDController turretMotorVelocityPid = new PIDController(1, 0.01, 0.4);
    private boolean cameraInitialized = false;
    private double calculatedLaunchAngle = 0;
    private double targetDistance = 0;

    public void runOpMode() {
        initialize();
        angleServo.setPosition(0);
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            if(!cameraInitialized) {
                setExposureAndGain();
                cameraInitialized = true;
            }

            intakeBall();
            launchBall();

            double flywheelRPM = ((launcherMotor.getVelocity()/28) * 60) * (1.1);
            telemetry.addData("shooter motor1 power", launcherMotor.getPower());
            telemetry.addData("shooter motor1 current (AMPS)", launcherMotor.getCurrent(CurrentUnit.AMPS));
            telemetryAprilTag(flywheelRPM);
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
        builder.setCameraResolution(new Size(640, 480));
        visionPortal = builder.build();
    }

    private void setExposureAndGain() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if(exposureControl != null && gainControl != null)  {
            if(exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure(25, TimeUnit.MILLISECONDS);
            gainControl.setGain(50);
        }
    }

    private void telemetryAprilTag(double flywheelRPM) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if (currentDetections.isEmpty()) {
            turretMotor.setPower(0);
        } else {
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                turretMotor.setPower(-bearingPid.calculate(1, detection.ftcPose.bearing));

                targetDistance = detection.ftcPose.range / 39.37;

                LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult(flywheelRPM, targetDistance );

                calculatedLaunchAngle = 0;

                if(launchResult != null) {
                    calculatedLaunchAngle = launchResult.getLaunchAngle();
                    setLaunchAngle(calculatedLaunchAngle);
                }

                double targetRPM = Math.round((((targetDistance / 1.670) * 600) + 2000) * 0.9);

                //double targetRPM = Math.round(2600 * 0.9);
                if(gamepad1.right_trigger > 0) {
                    launcherMotor.setVelocity(turretMotorVelocityPid.calculate(( targetRPM / 60.0 ) * 28.0));
                } else {
                    launcherMotor.setVelocity(0);
                }


                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    telemetry.addData("Target Range (M)", targetDistance);
                    telemetry.addData("Launch Angle (deg)", calculatedLaunchAngle);
                    telemetry.addData("Target RPM", targetRPM);
                    telemetry.addData("Actual RPM", flywheelRPM);
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
        if(gamepad.a) {
            launchServo.setPosition(0);
        } else {
            launchServo.setPosition(0.7);
        }
    }

    private void intakeBall() {
        if (gamepad1.left_trigger > 0) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    private void setLaunchAngle(double launchAngle) {

        //Base Angle 58deg
        //Extended Angle 15deg

        double positionValue = Math.abs((58.0-launchAngle)/35.0);
        angleServo.setPosition(positionValue);
    }

    private void initialize() {
        initializeHardware();
    }

    private void initializeHardware() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        launchServo = hardwareMap.get(Servo.class, "launchServo");

        angleServo.setDirection(Servo.Direction.REVERSE);

        this.gamepad = gamepad1;

        turretMotor.setMode(RUN_USING_ENCODER);
        launcherMotor.setMode(RUN_USING_ENCODER);
        initAprilTagProcessor();
    }
}

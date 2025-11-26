package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSCalculator;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import android.util.Size;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TurretTurnLaunch", group="sample")
public class TurretTurnLaunch extends LinearOpMode {

    private Gamepad gamepad;
    private DcMotorEx turretMotor;
    private DcMotorEx launcherMotor;
    private DcMotorEx intakeMotor;
    private Servo angleServo;
    private Servo launchServo;
    private NormalizedColorSensor launchColorSensor;
    private NormalizedColorSensor intakeColorSensor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final PIDController bearingPid = new PIDController(0.05, 0.005, 0.05);
    private boolean cameraInitialized = false;
    private double calculatedLaunchAngle = 0;
    private double targetDistance = 0;
    private HardwareManager hardwareManager;

    private SparkFunOTOS otos;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(100, 0, 0, 3);

    public void runOpMode() {
        initialize();
        angleServo.setPosition(0.2);
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            if(!cameraInitialized) {
                setExposureAndGain();
                cameraInitialized = true;
            }

            intakeBall();
            launchBall();
            getCurrentPosition();

            double flywheelRPM = (( launcherMotor.getVelocity() / 28 ) * 60);
            //telemetry.addData("shooter motor1 power", launcherMotor.getPower());
            //telemetry.addData("shooter motor1 current (AMPS)", launcherMotor.getCurrent(CurrentUnit.AMPS));
            telemetryAprilTag(flywheelRPM);
            telemetry.update();

        }
        visionPortal.close();
    }

    private void initAprilTagProcessor() {
        Position cameraPosition = new Position(DistanceUnit.METER, 0,  -0.20, 0.250, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -75, 0, 0);
        AprilTagProcessor.Builder aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        aprilTagProcessorBuilder.setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.364794005);
        aprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
        aprilTagProcessor = aprilTagProcessorBuilder.build();
        aprilTagProcessor.setDecimation(4);
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
            exposureControl.setExposure(5, TimeUnit.MILLISECONDS);
            gainControl.setGain(75);
        }
    }

    public void getCurrentPosition() {

        double currentX = OTOSCalculator.getCurrentPosition(otos).getXPos() * 39.37;
        double currentY = OTOSCalculator.getCurrentPosition(otos).getYPos() * 39.37;
        double currentHeading = OTOSCalculator.getCurrentPosition(otos).getHeading();

        telemetry.addLine(String.format("Current Position: %6.1f %6.1f %6.1f  (meter, deg)", currentX, currentY, currentHeading));

    }

    private void telemetryAprilTag(double flywheelRPM) {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if (currentDetections.isEmpty()) {
            turretMotor.setPower(0);
        } else {
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {

                OTOSCalculator.setCurrentPosition(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, AngleUnit.DEGREES.normalize(detection.robotPose.getOrientation().getYaw()), otos);

                turretMotor.setPower(bearingPid.calculate(1, detection.ftcPose.bearing));

                targetDistance = detection.ftcPose.range / 39.37;

                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    telemetry.addData("Target Range (M)", targetDistance);
                    telemetry.addData("Launch Angle (deg)", calculatedLaunchAngle);
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop
        }
        telemetry.addData("Actual RPM", flywheelRPM);
        telemetry.addData("Launcher Ball Color:", getLauncherBallColor());
        telemetry.addData("Intake Ball Color:", getIntakeBallColor());
        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult(flywheelRPM, targetDistance );

        calculatedLaunchAngle = 0;

        if(launchResult != null) {
            calculatedLaunchAngle = launchResult.getLaunchAngle();
        }

        setLaunchAngle(calculatedLaunchAngle);

        double targetRPM = Math.round(((targetDistance / 1.670) * 800) + 1900);
        telemetry.addData("Target RPM", targetRPM);

        //double targetRPM = Math.round(2600);
        if(gamepad1.right_trigger > 0) {
            launcherMotor.setVelocity(((targetRPM / 60.0 ) * 28.0) + (( 300 / 60 ) * 28));
            if (Math.abs(((launcherMotor.getVelocity() / 28) * 60) - targetRPM) < 150 ) {
                if (getLauncherBallColor() != "None") {
                    autoLaunchBall();
                }
                autoIntakeBall();
            }

        } else {
            launcherMotor.setVelocity(0);
        }


    }

    private void launchBall() {
        if(gamepad.a) {
            launchServo.setPosition(0);
        } else {
            launchServo.setPosition(0.7);
        }
    }

    private void autoLaunchBall() {
            launchServo.setPosition(0);
            sleep(100);
            launchServo.setPosition(0.7);
            sleep(100);
    }

    private void intakeBall() {
        if (gamepad1.left_trigger > 0) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    private void autoIntakeBall() {
        intakeMotor.setPower(1);
        sleep(25);
        intakeMotor.setPower(0);
    }

    private void setLaunchAngle(double launchAngle) {

        //Base Angle 58deg
        //Extended Angle 15deg

        double positionValue = Math.abs(((62.0-launchAngle)/35.0));
        angleServo.setPosition(positionValue);
    }

    private String getLauncherBallColor() {

        NormalizedRGBA colors = launchColorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
        //int alpha = colors.alpha;

        String ballDetected = "";

        if (blue > green && red > 0.2 && blue > 0.3) {
            ballDetected = "Purple";
        } else if (green > red && green > blue && green > 0.3) {
            ballDetected = "Green";
        } else {
            ballDetected = "None";
        }

        telemetry.addLine(String.format("Launcher Ball RGB %6.3f %6.3f %6.3f ", red, green, blue));


        return ballDetected;
    }

    private String getIntakeBallColor() {

        NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
        //int alpha = colors.alpha;

        String ballDetected = "";

        if (blue > green && red > 0.2 && blue > 0.3) {
            ballDetected = "Purple";
        } else if (green > red && green > blue && green > 0.3) {
            ballDetected = "Green";
        } else {
            ballDetected = "None";
        }

        telemetry.addLine(String.format("Intake Ball RGB %6.3f %6.3f %6.3f ", red, green, blue));


        return ballDetected;
    }

    private void initialize() {
        initializeHardware();
    }

    private void initializeHardware() {

        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        launchServo = hardwareMap.get(Servo.class, "launchServo");
        launchColorSensor = hardwareMap.get(NormalizedColorSensor.class, "launchColorSensor");
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");

        //otos = hardwareManager.getOtos();

        launchColorSensor.setGain(30);
        intakeColorSensor.setGain(15);

        angleServo.setDirection(Servo.Direction.FORWARD);

        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.gamepad = gamepad1;

        //// Turns on bulk reading
        //for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
        //    module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //}

        //// RUE limits max motor speed to 85% by default
        //// Raise that limit to 100%
        //MotorConfigurationType motorConfigurationType = launcherMotor.getMotorType().clone();
        //motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        //launcherMotor.setMotorType(motorConfigurationType);

        turretMotor.setMode(RUN_USING_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherMotor.setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()));
        initAprilTagProcessor();
    }
}

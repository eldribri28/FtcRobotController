package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

// SYSTEMS
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.GobildaMotorEnum.YELLOWJACKET_6000;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.CAMERA_EXPOSURE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.CAMERA_GAIN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_ROBOT_POSE_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.LAUNCH_VELO_PID;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.TURRET_ERROR_SMOOTHING_FACTOR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.TURRET_VELO_PID;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.tuningEnabled;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.drive.calculateDrivePower;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.launcher.setLaunchAngle;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.aprilTagLatencyCompensatedPose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.calculateDistanceBetweenPose2D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.calculateTurretError;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.clearPoseHistory;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.convertTo360Degrees;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.getTurretFieldPose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.robotAngularRateToTarget;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.turretFieldPoseToRobotFieldPose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.updatePedroFromRobotPose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.updateRobotPoseFromPedro;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.velocityToTarget;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.trajectory.flywheelVelocityToRpm;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.trajectory.generateTrajectoryLUT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.trajectory.rpmToEncoderVelocity;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.turret.calculateTurretErrorEncoderPosition;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.turret.canRotateTurret;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.turret.getTurretAngleFromEncoder;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.turret.setTurretEncoderToMotorEncoderOffset;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_NO_POWER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_POWER_IN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_POWER_OUT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_GATE_OPEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_H;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine.setExposureAndGain;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.getFlywheelRpm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.IndicatorLedEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.drive;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public abstract class TeleOpBaseLinearOpMode extends LinearOpMode {
    private HardwareManager hardwareManager;
    private ArtifactMotifEnum artifactMotifEnum = ArtifactMotifEnum.UNKNOWN;
    private Follower follower;
    private boolean robotPoseSetFlag = false;
    private Pose2D robotPose = new Pose2D(DistanceUnit.METER, 0,0,AngleUnit.RADIANS,0);
    private Pose2D turretPose = new Pose2D(DistanceUnit.METER, 0,0,AngleUnit.RADIANS,0);
    private Pose2D cameraPose = new Pose2D(DistanceUnit.METER, 0,0,AngleUnit.RADIANS,0);
    private double botHeadingRad = 0;
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double timeOfFlight = 0;
    private double targetRPM = 0;
    private double turretError = 0;
    private double lastTurretError = 0;

    private long turretMotorEncoderZero = 0;
    private boolean launchSolution = false;
    private double launchVelocity = 0;
    private long preShotTimestamp = 0;
    private AprilTagEngine aprilTagEngine;
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    abstract AprilTagEnum getTargetAprilTag();
    private Thread aprilTagEngineThread;
    private PoseHistory robotPoseHistory;
    boolean firing = false;
    long controlLoopStart = 0;

    double lastbuttonstate = 0;
    double lastbuttonstate2 = 0;
    double lastbuttonstate3 = 0;
    double flywheelspeedoffset = 0;



    // START HARDWARE READ VARIABLES
    private double turretAngleEncoder = 0;
    private double trajectoryLUT[][];
    private double launcherMotorVelocity = 0;
    private long turretMotorEncoder = 0;
    private boolean limitSwitchLeft = false;
    private boolean limitSwitchRight = false;
    private boolean gamepad1A = false;
    private boolean gamepad1B = false;
    private boolean gamepad1X = false;
    private boolean gamepad1Y = false;
    private boolean gamepad1DU = false;
    private boolean gamepad1DD = false;
    private boolean gamepad1DL = false;
    private boolean gamepad1DR = false;
    private boolean gamepad1RB = false;
    private boolean gamepad1RT = false;
    private boolean gamepad1LB = false;
    private boolean gamepad1LT = false;
    private double gamepad1LSX = 0;
    private double gamepad1LSY = 0;
    private double gamepad1RSX = 0;
    private double gamepad1RSY = 0;
    private boolean gamepad2A = false;
    private boolean gamepad2B = false;
    private boolean gamepad2X = false;
    private boolean gamepad2Y = false;
    private boolean gamepad2YWasReleased = false;
    private boolean gamepad2DU = false;
    private boolean gamepad2DD = false;
    private boolean gamepad2DL = false;
    private boolean gamepad2DR = false;

    // END HARDWARE READ VARIABLES

    // START HARDWARE WRITE VARIABLES
    private double setRightFrontMotorPower = 0;
    private double setRightRearMotorPower = 0;
    private double setLeftFrontMotorPower = 0;
    private double setLeftRearMotorPower = 0;
    private double setTurretMotorPower = 0;
    private int turretEncoderTarget = 0;
    private double setLauncherMotorVelocity = 0;
    private double setIntakeMotorPower = 0;
    private double setLaunchServo = 0;
    private double setAngleServo = 0;
    private double setIndicatorLed = IndicatorLedEnum.RED.getLedValue();
    private boolean flywheelConstantSpeedActive = false;
    // END HARDWARE WRITE VARIABLES

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            initialize();
            cacheHardwareWrite();
            waitForStart();
            stopLaunchArtifact();
            resetRuntime();
            aprilTagEngineThread.start();
            hardwareManager.postStartInitialization();
            cacheHardwareWrite();
            while (opModeIsActive()) {
                controlLoopStart = System.currentTimeMillis();
                bulkHardwareRead();
                resetIMU();
                drive();
                intakeOrRejectArtifact();
                aprilTagProcessor();
                updateLauncher();
                updateTurret();
                gamepad2();
                gamepadLaunchArtifact();
                tuningMode(tuningEnabled);
                cacheHardwareWrite();

            }
        } finally {
            scheduler.shutdownNow();
            if(aprilTagEngineThread != null) {
                aprilTagEngineThread.interrupt();
                aprilTagEngine.teardown();
            }
            telemetry.update();
        }
    }

    private void tuningMode(boolean enabledFlag) {
        if (enabledFlag) {
            hardwareManager.getLauncherMotor().setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(
                    LAUNCH_VELO_PID.p, LAUNCH_VELO_PID.i, LAUNCH_VELO_PID.d, LAUNCH_VELO_PID.f)); // * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()));
            hardwareManager.getTurretMotor().setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(
                    TURRET_VELO_PID.p, TURRET_VELO_PID.i, TURRET_VELO_PID.d, TURRET_VELO_PID.f));
            setExposureAndGain();
            tuningTelemetry();
        } else {
            updateTelemetry();
        }
        telemetry.update();
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        aprilTagEngineThread = new Thread(aprilTagEngine);
        bulkHardwareRead();
        setIndicatorLed = IndicatorLedEnum.RED.getLedValue();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardwareManager.getTurretMotor().setMode(STOP_AND_RESET_ENCODER);
        hardwareManager.getTurretMotor().setMode(RUN_USING_ENCODER);
        turretMotorEncoderZero = setTurretEncoderToMotorEncoderOffset(turretMotorEncoder, turretAngleEncoder);
        initPedro();
        telemetry.addLine("Start Generate Trajectory LUT...");
        telemetry.update();
        controlLoopStart = System.currentTimeMillis();
        generateLUTs();
        telemetry.addLine("End Generate Trajectory LUT...");
        telemetry.addData("LoopTime", "%d ms", (System.currentTimeMillis() - controlLoopStart));
        setExposureAndGain();

        //while (!isStarted() && !isStopRequested()) {
            bulkHardwareRead();
            //telemetry.addData("Distance 1.000m", "%.3f(m/s), %.3f(deg), %.3f(s)", trajectoryLUT[100][0], trajectoryLUT[100][1], trajectoryLUT[100][2]);
            //telemetry.addData("Distance 1.500m", "%.3f(m/s), %.3f(deg), %.3f(s)", trajectoryLUT[150][0], trajectoryLUT[150][1], trajectoryLUT[150][2]);
            //telemetry.addData("Distance 1.900m", "%.3f(m/s), %.3f(deg), %.3f(s)", trajectoryLUT[190][0], trajectoryLUT[190][1], trajectoryLUT[190][2]);
            //telemetry.addData("Distance 2.000m", "%.3f(m/s), %.3f(deg), %.3f(s)", trajectoryLUT[200][0], trajectoryLUT[200][1], trajectoryLUT[200][2]);
            //telemetry.addData("Distance 2.100m", "%.3f(m/s), %.3f(deg), %.3f(s)", trajectoryLUT[210][0], trajectoryLUT[210][1], trajectoryLUT[210][2]);
            //telemetry.addData("Distance 2.500m", "%.3f(m/s), %.3f(deg), %.3f(s)", trajectoryLUT[250][0], trajectoryLUT[250][1], trajectoryLUT[250][2]);
            //telemetry.addData("Distance 3.000m", "%.3f(m/s), %.3f(deg), %.3f(s)", trajectoryLUT[300][0], trajectoryLUT[300][1], trajectoryLUT[300][2]);
            //telemetry.addData("Distance 3.500m", "%.3f(m/s), %.3f(deg), %.3f(s)", trajectoryLUT[350][0], trajectoryLUT[350][1], trajectoryLUT[350][2]);
            //telemetry.addData("Distance 4.000m", "%.3f(m/s), %.3f(deg), %.3f(s)", trajectoryLUT[400][0], trajectoryLUT[400][1], trajectoryLUT[400][2]);

            telemetry.update();
        //}
    }

    private void initPedro() {
        follower = Constants.createFollower(hardwareMap);
        clearPoseHistory();
    }

    private void generateLUTs() {
        trajectoryLUT = generateTrajectoryLUT();
    }

    private void updateTelemetry() {
        updateRuntime();
        //telemetry.addLine("---SYSTEM INFO---");
        //telemetry.addLine("_________________________________________________________________");
        telemetry.addData("Runtime", 0);
        telemetry.addData("LoopTime", "%d ms", (System.currentTimeMillis() - controlLoopStart));
        //telemetry.addLine("---TARGET INFO---");
        //telemetry.addLine("_________________________________________________________________");
        telemetry.addData("Target name", getTargetAprilTag().name());
        telemetry.addData("Target distance", targetDistance);
        //telemetry.addLine("---APRILTAG INFO---");
        //telemetry.addLine("_________________________________________________________________");
        telemetry.addData("# AprilTags Detected", 0);
        telemetry.addData("Target detection age(millisecond)", 0);
        telemetry.addData("AprilTagEngine exception","");
        //telemetry.addLine("---GAMEPAD 2---");
        //telemetry.addLine("_________________________________________________________________");
        telemetry.addData("Editing", "");
        telemetry.addData("camera exposure value", CAMERA_EXPOSURE);
        telemetry.addData("camera gain value", CAMERA_GAIN);
        telemetry.addData("fly wheel offset", flywheelspeedoffset);

        aprilTagEngine.getTelemetry().forEach((k, v) -> telemetry.addData(k, v));
    }

    private void tuningTelemetry() {
        updateRuntime();
        double cameraX = cameraPose.getX(DistanceUnit.METER);
        double cameraY = cameraPose.getY(DistanceUnit.METER);
        double cameraH = convertTo360Degrees(cameraPose.getHeading(AngleUnit.DEGREES));
        double robotX = robotPose.getX(DistanceUnit.METER);
        double robotY = robotPose.getY(DistanceUnit.METER);
        double robotH = convertTo360Degrees(robotPose.getHeading(AngleUnit.DEGREES));
        double turretX = turretPose.getX(DistanceUnit.METER);
        double turretY = turretPose.getY(DistanceUnit.METER);
        double turretH = convertTo360Degrees(turretPose.getHeading(AngleUnit.DEGREES));
        //telemetry.addData("---SYSTEM INFO--------------------------", "");
        telemetry.addData("Runtime", 0);
        telemetry.addData("LoopTime", "%d ms", (System.currentTimeMillis() - controlLoopStart));
        //telemetry.addData("---TARGET INFO--------------------------", "");
        telemetry.addData("Target Name", getTargetAprilTag().name());
        telemetry.addData("Target Distance", targetDistance);
        //telemetry.addData("---APRILTAG INFO--------------------------", "");
        telemetry.addData("# AprilTags Detected", 0);
        telemetry.addData("Target Detection Age", "%d ms", 0);
        telemetry.addData("AprilTagEngine Exception","");
        //telemetry.addData("---TURRET INFO--------------------------","");
        telemetry.addData("Turret Error", turretError);
        telemetry.addData("Turret Power", setTurretMotorPower);
        telemetry.addData("Turret Angle", 0);
        telemetry.addData("Turret Encoder Target", 0);
        telemetry.addData("Turret Encoder Current", 0);
        //telemetry.addData("---LAUNCHER INFO--------------------------", "");
        telemetry.addData("Launch Angle", launchAngle);
        telemetry.addData("Launcher RPM (Target)", targetRPM);
        telemetry.addData("Launcher RPM (Actual)", flywheelVelocityToRpm(launcherMotorVelocity, YELLOWJACKET_6000));
        //telemetry.addData("---FIELD COORDINATES--------------------------", "");
        telemetry.addData("Camera: x, y, h", "%.2f m, %.2f m, %.2f deg", cameraX, cameraY, cameraH);
        telemetry.addData("Turret: x, y, h","%.2f m, %.2f m, %.2f deg", turretX, turretY, turretH);
        telemetry.addData("Robot: x, y, h", "%.2f m, %.2f m, %.2f deg", robotX, robotY, robotH);
        //telemetry.addData("---GAMEPAD 2--------------------------", "");
        telemetry.addData("Editing", "");
        telemetry.addData("Camera Exposure", CAMERA_EXPOSURE);
        telemetry.addData("Camera Gain", CAMERA_GAIN);
        telemetry.addData("Flywheel Offset", flywheelspeedoffset);
        aprilTagEngine.getTelemetry().forEach((k, v) -> telemetry.addData(k, v));
    }

    public static boolean isTurretAngleWithinThreshold(double turretError, double targetDistance) {
        double bearing = Math.abs(turretError);
        if(targetDistance > 2.0) {
            return bearing <= 0.25;
        }
        return bearing <= 0.5;
    }

    public boolean readyToShoot() {
        return isTurretAngleWithinThreshold(turretError, targetDistance)
                && isLaunchMotorVelocityWithinThreshold()
                && launchSolution;
    }

    private void resetIMU() {
        if (gamepad1DR) {
            hardwareManager.getImu().resetYaw();
        }
    }

    private void intakeOrRejectArtifact() {
        if (gamepad1LB) {
            setIntakeMotorPower = INTAKE_POWER_OUT;
        } else if (gamepad1LT) {
            setIntakeMotorPower = INTAKE_POWER_IN;
        } else {
            setIntakeMotorPower = INTAKE_NO_POWER;
        }
    }

    private void aprilTagProcessor() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if(timedDetection != null && timedDetection.getDetection() != null) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
                CAMERA_FIELD_X = targetDetection.robotPose.getPosition().x;
                CAMERA_FIELD_Y = targetDetection.robotPose.getPosition().y;
                CAMERA_FIELD_H = AngleUnit.normalizeRadians(targetDetection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
                cameraPose = new Pose2D(DistanceUnit.METER, CAMERA_FIELD_X, CAMERA_FIELD_Y, AngleUnit.RADIANS, CAMERA_FIELD_H);
                turretPose = cameraPose;
                robotPose = turretFieldPoseToRobotFieldPose(turretPose, TURRET_ROBOT_POSE_OFFSET, getTurretAngleFromEncoder(turretMotorEncoder, turretMotorEncoderZero));
                updatePedroFromRobotPose(follower, aprilTagLatencyCompensatedPose(targetDetection.frameAcquisitionNanoTime, robotPose), InvertedFTCCoordinates.INSTANCE);
                robotPoseSetFlag = true;
            } else {
                setNoTagDetected();
            }
        } else {
            setNoTagDetected();
        }
    }

    private void setNoTagDetected() {
        updateRobotFieldPose();
        updateTurretFieldPose(robotPose);
        setIndicatorLed = IndicatorLedEnum.RED.getLedValue();
        if (!gamepad1A) {
            setLaunchServo = LAUNCH_GATE_CLOSE;
        }
    }

    private double turretErrorSmoothing(double turretError) {
        double smoothError = (turretError * (1 - TURRET_ERROR_SMOOTHING_FACTOR)) + (lastTurretError * TURRET_ERROR_SMOOTHING_FACTOR);
        lastTurretError = turretError;
        return smoothError;
    }

    private void updateRobotFieldPose() {
        robotPose = updateRobotPoseFromPedro(follower, FTCCoordinates.INSTANCE);
    }

    private void updateTurretFieldPose(Pose2D robotPose) {
        turretPose = getTurretFieldPose(robotPose, TURRET_ROBOT_POSE_OFFSET, getTurretAngleFromEncoder(turretMotorEncoder, turretMotorEncoderZero));
    }

    private void updateTurret() {
        //double angularMovementCompensation = robotAngularRateToTarget(robotPose, follower, getTargetAprilTag());
        double turretErrorRad = turretError;
        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            turretErrorRad = AngleUnit.normalizeRadians(calculateTurretError(turretPose, BLUE_GOAL_POSE));
        } else {
            turretErrorRad = AngleUnit.normalizeRadians(calculateTurretError(turretPose, RED_GOAL_POSE));
        }
        turretError = turretErrorSmoothing(Math.toDegrees(turretErrorRad)); // Try to smooth out jitter
        double turretAngle = getTurretAngleFromEncoder(turretMotorEncoder, turretMotorEncoderZero);
        turretEncoderTarget = calculateTurretErrorEncoderPosition(turretError, turretAngle, turretMotorEncoder);
        if (canRotateTurret(turretError, limitSwitchRight, limitSwitchLeft, turretAngle, turretError) && robotPoseSetFlag) {
            setTurretMotorPower = 1.0;
            telemetry.addData("Turret Angle", Math.toDegrees(turretAngle));
            telemetry.addData("TurretEncoderTarget", turretEncoderTarget);
            telemetry.addData("Turret Encoder Current", turretMotorEncoder);
        } else {
            setTurretMotorPower = 0.0;
        }
        setAngleServo = setLaunchAngle(launchAngle);
    }

    private void updateLauncher() {
        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            targetDistance = calculateDistanceBetweenPose2D(turretPose, BLUE_GOAL_POSE);
        } else {
            targetDistance = calculateDistanceBetweenPose2D(turretPose, RED_GOAL_POSE);
        }
        updateLaunchCalculations(launcherMotorVelocity, targetDistance);
    }

    private void updateLaunchCalculations(double currentFlywheelVelocity, double targetDistance) {
        // Use trajectory Look Up Table to boost speed.
        int targetDistanceToCM = (int)(targetDistance * 100);
        if (targetDistanceToCM > 500) { targetDistanceToCM = 500; }
        if (targetDistanceToCM < 50) { targetDistanceToCM = 100; }
            double targetVelocityOffset = velocityToTarget(robotPose, follower, getTargetAprilTag());
            //double targetVelocityOffset = 0;
            launchVelocity = trajectoryLUT[targetDistanceToCM][0] + targetVelocityOffset;
            double hoodAngle = trajectoryLUT[targetDistanceToCM][1];
            double calculatedTOF = trajectoryLUT[targetDistanceToCM][2];
            if (launchVelocity > 0 || hoodAngle > 0) {
                launchSolution = true;
                targetRPM = getFlywheelRpm(launchVelocity);
                if (targetRPM < 2000) { targetRPM = 2000; }
                setLauncherMotorVelocity = Math.round(rpmToEncoderVelocity(targetRPM + flywheelspeedoffset, YELLOWJACKET_6000));
                launchAngle = hoodAngle;
                timeOfFlight = calculatedTOF;
            } else {
                launchSolution = false;
            }
    }

    private void drive() {
        drive.DriveMotorPower motorPowerResults = calculateDrivePower(gamepad1LSY, gamepad1LSX, gamepad1RSX, gamepad1RB, gamepad1RT, botHeadingRad);
        setRightFrontMotorPower = motorPowerResults.getRightFrontMotorPower();
        setLeftFrontMotorPower = motorPowerResults.getLeftFrontMotorPower();
        setRightRearMotorPower = motorPowerResults.getRightRearMotorPower();
        setLeftRearMotorPower = motorPowerResults.getLeftRearMotorPower();
    }

    private boolean isLaunchMotorVelocityWithinThreshold() {
        double currentRPM = ((launcherMotorVelocity / 28.0) * 60.0);
        return Math.abs(targetRPM - currentRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
    }

    public boolean preShotTimer() {
        if( preShotTimestamp == 0) {
            preShotTimestamp = System.currentTimeMillis() + 100;
        } else if (preShotTimestamp <= System.currentTimeMillis()) {
            return true;
        }
        return false;
    }

    private void startLaunchArtifact(double targetDistance) {
        setLaunchServo = LAUNCH_GATE_OPEN;
        if (preShotTimer()) {
            if (targetDistance < 3) {
                setIntakeMotorPower = INTAKE_POWER_IN;
            } else {
                setIntakeMotorPower = INTAKE_POWER_IN * 0.8;
            }

        }
    }

    private void stopLaunchArtifact() {
        if (!gamepad1A) {
            setLaunchServo = LAUNCH_GATE_CLOSE;
            if(!gamepad1LT && !gamepad1LB) {
                setIntakeMotorPower = INTAKE_NO_POWER;
                preShotTimestamp = 0;
            }
        }
    }

    private void gamepadLaunchArtifact() {
        if (gamepad1A) {
            setIndicatorLed = IndicatorLedEnum.BLUE.getLedValue();
            startLaunchArtifact(targetDistance);
        } else {
            if (readyToShoot()) {
                setIndicatorLed = IndicatorLedEnum.GREEN.getLedValue();
            } else {
                setIndicatorLed = IndicatorLedEnum.YELLOW.getLedValue();
            }
            stopLaunchArtifact();
        }
    }

    private void gamepad2() {

        if (gamepad2A) {
            telemetry.addData("Editing", "Camera Gain");
            if (gamepad2DU && lastbuttonstate == 0) {
                lastbuttonstate = 1;
                CAMERA_GAIN += 1;
                setExposureAndGain();
            } else if (gamepad2DD && lastbuttonstate == 0) {
                lastbuttonstate = 1;
                CAMERA_GAIN -= 1;
                setExposureAndGain();
            } else if (gamepad2DL && lastbuttonstate == 0) {
                lastbuttonstate = 1;
                CAMERA_GAIN = 5;
                setExposureAndGain();
            }
        }

        if (gamepad2X) {
            telemetry.addData("Editing", "Camera Exposure");
            if (gamepad2DU && lastbuttonstate2 == 0) {
                lastbuttonstate2 = 1;
                CAMERA_EXPOSURE += 1;
                setExposureAndGain();
            } else if (gamepad2DD && lastbuttonstate2 == 0) {
                lastbuttonstate2 = 1;
                CAMERA_EXPOSURE -= 1;
                setExposureAndGain();
            } else if (gamepad2DL && lastbuttonstate == 0) {
                lastbuttonstate2 = 1;
                CAMERA_EXPOSURE = 7;
                setExposureAndGain();
            }
        }

        if (gamepad2B) {
            telemetry.addData("Editing", "Flywheel Offset");
            if (gamepad2DU && lastbuttonstate3 == 0) {
                lastbuttonstate3 = 1;
                flywheelspeedoffset += 10;
            } else if (gamepad2DD && lastbuttonstate3 == 0) {
                lastbuttonstate3 = 1;
                flywheelspeedoffset -= 10;
            } else if (gamepad2DL && lastbuttonstate3 == 0) {
                lastbuttonstate3 = 1;
                flywheelspeedoffset = 0;
            }
        }

        if (!gamepad2DD && !gamepad2DU) {
            lastbuttonstate = 0;
            lastbuttonstate2 = 0;
            lastbuttonstate3 = 0;
        }

        if(gamepad2YWasReleased) {
            flywheelConstantSpeedActive = !flywheelConstantSpeedActive;
        }
    }



    private void bulkHardwareRead() {
        YawPitchRollAngles orientation = hardwareManager.getImu().getRobotYawPitchRollAngles();
        botHeadingRad = orientation.getYaw(AngleUnit.RADIANS);
        launcherMotorVelocity = hardwareManager.getLauncherMotor().getVelocity();
        turretMotorEncoder = hardwareManager.getTurretMotor().getCurrentPosition();
        turretAngleEncoder = Math.toRadians(hardwareManager.getTurretEncoder().getAngleDeg(true));
        limitSwitchLeft = hardwareManager.getLimitSwitchLeft().isPressed();
        limitSwitchRight = hardwareManager.getLimitSwitchRight().isPressed();
        gamepad1A = hardwareManager.getGamepad1().a;
        gamepad1B = hardwareManager.getGamepad1().b;
        gamepad1X = hardwareManager.getGamepad1().x;
        gamepad1Y = hardwareManager.getGamepad1().y;
        gamepad1DU = hardwareManager.getGamepad1().dpad_up;
        gamepad1DD = hardwareManager.getGamepad1().dpad_down;
        gamepad1DL = hardwareManager.getGamepad1().dpad_left;
        gamepad1DR = hardwareManager.getGamepad1().dpad_right;
        gamepad1RB = hardwareManager.getGamepad1().right_bumper;
        gamepad1RT = hardwareManager.getGamepad1().right_trigger_pressed;
        gamepad1LB = hardwareManager.getGamepad1().left_bumper;
        gamepad1LT = hardwareManager.getGamepad1().left_trigger_pressed;
        gamepad1LSX = hardwareManager.getGamepad1().left_stick_x;
        gamepad1LSY = hardwareManager.getGamepad1().left_stick_y;
        gamepad1RSX = hardwareManager.getGamepad1().right_stick_x;
        gamepad1RSY = hardwareManager.getGamepad1().right_stick_y;
        gamepad2A = hardwareManager.getGamepad2().a;
        gamepad2B = hardwareManager.getGamepad2().b;
        gamepad2X = hardwareManager.getGamepad2().x;
        gamepad2Y = hardwareManager.getGamepad2().y;
        gamepad2YWasReleased = hardwareManager.getGamepad2().yWasReleased();
        gamepad2DU = hardwareManager.getGamepad2().dpad_up;
        gamepad2DD = hardwareManager.getGamepad2().dpad_down;
        gamepad2DL = hardwareManager.getGamepad2().dpad_left;
        gamepad2DR = hardwareManager.getGamepad2().dpad_right;
    }

    private void cacheHardwareWrite() {
            hardwareManager.getRightFrontMotor().setPower(setRightFrontMotorPower);
            hardwareManager.getRightRearMotor().setPower(setRightRearMotorPower);
            hardwareManager.getLeftFrontMotor().setPower(setLeftFrontMotorPower);
            hardwareManager.getLeftRearMotor().setPower(setLeftRearMotorPower);
            hardwareManager.getTurretMotor().setTargetPosition(turretEncoderTarget);

            if(flywheelConstantSpeedActive) {
                hardwareManager.getTurretMotor().setMode(RUN_USING_ENCODER);
                hardwareManager.getTurretMotor().setPower(0);
                hardwareManager.getLauncherMotor().setVelocity((double) (2400 * 28)/60);
            } else {
                hardwareManager.getTurretMotor().setMode(RUN_TO_POSITION);
                hardwareManager.getTurretMotor().setPower(setTurretMotorPower);
                hardwareManager.getLauncherMotor().setVelocity(setLauncherMotorVelocity);
            }
            hardwareManager.getIntakeMotor().setPower(setIntakeMotorPower);
            hardwareManager.getLaunchServo().setPosition(setLaunchServo);
            hardwareManager.getAngleServo().setPosition(setAngleServo);
            hardwareManager.getIndicatorLed().setPosition(setIndicatorLed);
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
                "Runtime",
                "%02.0f:%02.0f",
                totalSeconds / 60,
                totalSeconds % 60);
    }

}
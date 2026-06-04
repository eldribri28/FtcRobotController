package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

// SYSTEMS
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.CAMERA_TURRET_POSE_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.RED_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_ROBOT_POSE_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.drive.calculateDrivePower;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.launcher.setLaunchAngle;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.aprilTagLatencyCompensatedPose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.calculateAngleBetweenPose2D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.calculateDistanceBetweenPose2D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.calculateTurretError;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.cameraFieldPoseToTurretFieldPose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.clearPoseHistory;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.convertTo360Degrees;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.getTurretFieldPose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.predictFuturePose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.robotAngularRateToTarget;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.turretFieldPoseToRobotFieldPose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.updatePedroFromRobotPose;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.updateRobotPoseFromPedro;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.localization.velocityToTarget;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.trajectory.flywheelVelocityToRpm;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.trajectory.generateTrajectoryLUT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.trajectory.getHoodAngle;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.trajectory.rpmToEncoderVelocity;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.turret.adjustTurretPowerNearStop;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.turret.getTurretAngleFromEncoder;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.turret.rotateTurret;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.turret.setTurretEncoderToMotorEncoderOffset;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_NO_POWER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_POWER_IN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_POWER_OUT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCHER_MOTOR_IDLE_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_GATE_OPEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_LAST_TIMESTAMP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_TARGET_CLOSE_RATE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_TARGET_YAW_RATE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_H;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.getFlywheelRpm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private Pose2D turretShotPose = new Pose2D(DistanceUnit.METER, 0,0,AngleUnit.RADIANS,0);
    private double botHeadingRad = 0;
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double timeOfFlight = 0;
    private double targetRPM = 0;
    private double turretError = 0;
    private long turretMotorEncoderZero = 0;
    private boolean launchSolution = false;
    private double launchVelocity = 0;
    private AprilTagEngine aprilTagEngine;
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    //private ColorManager colorManager;
    abstract AprilTagEnum getTargetAprilTag();
    //private Thread colorManagerThread;
    private Thread aprilTagEngineThread;
    private PoseHistory robotPoseHistory;
    boolean firing = false;
    long controlLoopStart = 0;

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
    // END HARDWARE READ VARIABLES

    // START HARDWARE WRITE VARIABLES
    private double setRightFrontMotorPower = 0;
    private double lastRightFrontMotorPower = 0;
    private double setRightRearMotorPower = 0;
    private double lastRightRearMotorPower = 0;
    private double setLeftFrontMotorPower = 0;
    private double lastLeftFrontMotorPower = 0;
    private double setLeftRearMotorPower = 0;
    private double lastLeftRearMotorPower = 0;
    private double setTurretMotorPower = 0;
    private double lastTurretMotorPower = 0;
    private double setLauncherMotorVelocity = 0;
    private double lastLauncherMotorVelocity = 0;
    private double setIntakeMotorPower = 0;
    private double lastIntakeMotorPower = 0;
    private double setLaunchServo = 0;
    private double lastLaunchServo = 0;
    private double setAngleServo = 0;
    private double lastAngleServo = 0;
    private double setIntakeServo = 0;
    private double lastIntakeServo = 0;
    private double setIndicatorLed = IndicatorLedEnum.RED.getLedValue();;
    private double lastIndicatorLed = 0;
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
                moveIntake();
                gamepadLaunchArtifact();
                cacheHardwareWrite();
                updateTelemetry();
                telemetry.update();
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

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        aprilTagEngineThread = new Thread(aprilTagEngine);
        bulkHardwareRead();
        setIndicatorLed = IndicatorLedEnum.RED.getLedValue();
        setIntakeServo = INTAKE_DOWN;
        //colorManager = new ColorManager(hardwareManager);
        //colorManagerThread = new Thread(colorManager);
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
        telemetry.update();
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
        double cameraX = cameraPose.getX(DistanceUnit.METER);
        double cameraY = cameraPose.getY(DistanceUnit.METER);
        double cameraH = convertTo360Degrees(cameraPose.getHeading(AngleUnit.DEGREES));
        double robotX = robotPose.getX(DistanceUnit.METER);
        double robotY = robotPose.getY(DistanceUnit.METER);
        double robotH = convertTo360Degrees(robotPose.getHeading(AngleUnit.DEGREES));
        double turretX = turretPose.getX(DistanceUnit.METER);
        double turretY = turretPose.getY(DistanceUnit.METER);
        double turretH = convertTo360Degrees(turretPose.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("SYSTEM INFO");
        telemetry.addData("LoopTime", "%d ms", (System.currentTimeMillis() - controlLoopStart));
        telemetry.addLine("TARGET INFO");
        telemetry.addData("Target name", getTargetAprilTag().name());
        telemetry.addData("Target distance", targetDistance);
        telemetry.addData("Turret Error", turretError);
        telemetry.addData("Launch Angle", launchAngle);
        telemetry.addData("Launcher RPM (Target)", targetRPM);
        telemetry.addData("Launcher RPM (Actual)", flywheelVelocityToRpm(launcherMotorVelocity));
        //telemetry.addData("Launch Velocity", launchVelocity);
        telemetry.addData("Turret Limit Switch Left Pressed", limitSwitchLeft);
        telemetry.addData("Turret Limit Switch Right Pressed", limitSwitchRight);
        telemetry.addData("Turret Angle: (deg)", Math.toDegrees(getTurretAngleFromEncoder(turretMotorEncoder, turretMotorEncoderZero)));
        telemetry.addData("Target Close Rate (M/S)", ROBOT_TARGET_CLOSE_RATE);
        telemetry.addData("Target Yaw Rate (Deg/S)", ROBOT_TARGET_YAW_RATE);
        telemetry.addData("Target Last Timestamp (ms)", ROBOT_LAST_TIMESTAMP);
        telemetry.addLine("FIELD COORDINATES");
        telemetry.addData("Camera: x, y, h", "%.2f(m), %.2f(m), %.2f(deg)", cameraX, cameraY, cameraH);
        telemetry.addData("Turret: x, y, h","%.2f(m), %.2f(m), %.2f(deg)", turretX, turretY, turretH);
        telemetry.addData("Robot: x, y, h", "%.2f(m), %.2f(m), %.2f(deg)", robotX, robotY, robotH);
        aprilTagEngine.getTelemetry().forEach((k, v) -> telemetry.addData(k, v));
    }

    public static boolean isTurretAngleWithinThreshold(double turretError, double targetDistance) {
        double bearing = Math.abs(turretError);
        if(targetDistance > 2.0) {
            return bearing <= 0.3;
        }
        return bearing <= 0.75;
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

    private void moveIntake() {
        if (gamepad1DU) {
            setIntakeServo = INTAKE_UP;
        } else if (gamepad1DD) {
            setIntakeServo = INTAKE_DOWN;
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
                turretPose = cameraFieldPoseToTurretFieldPose(cameraPose, CAMERA_TURRET_POSE_OFFSET);
                robotPose = turretFieldPoseToRobotFieldPose(turretPose, TURRET_ROBOT_POSE_OFFSET, getTurretAngleFromEncoder(turretMotorEncoder, turretMotorEncoderZero));
                updatePedroFromRobotPose(follower, aprilTagLatencyCompensatedPose(targetDetection.frameAcquisitionNanoTime, robotPose));
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

    private void updateRobotFieldPose() {
        robotPose = updateRobotPoseFromPedro(follower);
    }

    private void updateTurretFieldPose(Pose2D robotPose) {
        turretPose = getTurretFieldPose(robotPose, TURRET_ROBOT_POSE_OFFSET, getTurretAngleFromEncoder(turretMotorEncoder, turretMotorEncoderZero));
    }

    private void updateTurret() {
        double angularMovementCompensation = robotAngularRateToTarget(robotPose, follower, getTargetAprilTag());
        double turretErrorRad = turretError;
        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            turretErrorRad = AngleUnit.normalizeRadians(calculateTurretError(turretPose, BLUE_GOAL_POSE));
        } else {
            turretErrorRad = AngleUnit.normalizeRadians(calculateTurretError(turretPose, RED_GOAL_POSE));
        }
        turretError = Math.toDegrees(turretErrorRad);
        setTurretMotorPower = rotateTurret(turretError, limitSwitchRight, limitSwitchLeft, getTurretAngleFromEncoder(turretMotorEncoder, turretMotorEncoderZero), robotPoseSetFlag);
        setTurretMotorPower = adjustTurretPowerNearStop(getTurretAngleFromEncoder(turretMotorEncoder, turretMotorEncoderZero), setTurretMotorPower);
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
            //double flywheelRPM = flywheelVelocityToRpm(currentFlywheelVelocity);
            //LaunchCalculator.LaunchResult launchResult = LaunchCalculator.getLaunchData(LAUNCH_HEIGHT, TARGET_HEIGHT, targetDistance, flywheelRPM, ROBOT_TARGET_CLOSE_RATE);
            //double launchVelocity = launchResult.getLaunchVelocity();
            //double hoodAngle = launchResult.getLaunchAngle();
            // Use trajectory Look Up Table to boost speed.
            int targetDistanceToCM = (int)(targetDistance * 100);
            if (targetDistanceToCM <= 500 && targetDistanceToCM >= 1) {
                double targetVelocityOffset = velocityToTarget(robotPose, follower, getTargetAprilTag());
                //double targetVelocityOffset = 0;
                launchVelocity = trajectoryLUT[targetDistanceToCM][0] + targetVelocityOffset;
                double hoodAngle = trajectoryLUT[targetDistanceToCM][1];
                double calculatedTOF = trajectoryLUT[targetDistanceToCM][2];
                if (launchVelocity > 0 || hoodAngle > 0) {
                    launchSolution = true;
                    targetRPM = getFlywheelRpm(launchVelocity);
                    //setLauncherMotorVelocity = Math.round(rpmToEncoderVelocity(targetRPM));
                    launchAngle = hoodAngle;
                    timeOfFlight = calculatedTOF;
                } else {
                    launchSolution = false;
                }
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

    private void startLaunchArtifact() {
        setLaunchServo = LAUNCH_GATE_OPEN;
        setIntakeMotorPower = INTAKE_POWER_IN;
    }

    private void stopLaunchArtifact() {
        if (!gamepad1A) {
            setLaunchServo = LAUNCH_GATE_CLOSE;
            if(!gamepad1LT && !gamepad1LB) {
                setIntakeMotorPower = INTAKE_NO_POWER;
            }
        }
    }

    private void gamepadLaunchArtifact() {
        if (gamepad1A) {
            setIndicatorLed = IndicatorLedEnum.BLUE.getLedValue();
            startLaunchArtifact();
        } else {
            if (readyToShoot()) {
                setIndicatorLed = IndicatorLedEnum.GREEN.getLedValue();
            } else {
                setIndicatorLed = IndicatorLedEnum.YELLOW.getLedValue();
            }
            stopLaunchArtifact();
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
    }

    private void cacheHardwareWrite() {
        //if (setRightFrontMotorPower != lastRightFrontMotorPower) {
            hardwareManager.getRightFrontMotor().setPower(setRightFrontMotorPower);
        //    lastRightFrontMotorPower = setRightFrontMotorPower;
        //}
        //if (setRightRearMotorPower != lastRightRearMotorPower) {
            hardwareManager.getRightRearMotor().setPower(setRightRearMotorPower);
        //    lastRightRearMotorPower = setRightRearMotorPower;
        //}
        //if (setLeftFrontMotorPower != lastLeftFrontMotorPower) {
            hardwareManager.getLeftFrontMotor().setPower(setLeftFrontMotorPower);
        //    lastLeftFrontMotorPower = setLeftFrontMotorPower;
        //}
        //if (setLeftRearMotorPower != lastLeftRearMotorPower) {
            hardwareManager.getLeftRearMotor().setPower(setLeftRearMotorPower);
        //    lastLeftRearMotorPower = setLeftRearMotorPower;
        //}
        //if (setTurretMotorPower != lastTurretMotorPower) {
            hardwareManager.getTurretMotor().setPower(setTurretMotorPower);
        //    lastTurretMotorPower = setTurretMotorPower;
        //}
        if (setLauncherMotorVelocity != lastLauncherMotorVelocity) {
            hardwareManager.getLauncherMotor().setVelocity(setLauncherMotorVelocity);
            lastLauncherMotorVelocity = setLauncherMotorVelocity;
        }
        if (setIntakeMotorPower != lastIntakeMotorPower) {
            hardwareManager.getIntakeMotor().setPower(setIntakeMotorPower);
            lastIntakeMotorPower = setIntakeMotorPower;
        }
        if (setLaunchServo != lastLaunchServo) {
            hardwareManager.getLaunchServo().setPosition(setLaunchServo);
            lastLaunchServo = setLaunchServo;
        }
        if (setAngleServo != lastAngleServo) {
            hardwareManager.getAngleServo().setPosition(setAngleServo);
            lastAngleServo = setAngleServo;
        }
        if (setIntakeServo != lastIntakeServo) {
            hardwareManager.getIntakeServo().setPosition(setIntakeServo);
            lastIntakeServo = setIntakeServo;
        }
        if (setIndicatorLed != lastIndicatorLed) {
            hardwareManager.getIndicatorLed().setPosition(setIndicatorLed);
            lastIndicatorLed = setIndicatorLed;
        }
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
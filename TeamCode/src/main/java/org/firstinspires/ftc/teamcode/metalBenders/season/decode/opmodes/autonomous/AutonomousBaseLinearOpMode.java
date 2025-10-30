package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.GREEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.NONE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.UNKNOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.BOT_HEADING_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_H;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_Y;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ROTATEPID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ROTATEPID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ROTATEPID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVEPID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVEPID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVEPID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ROTATION_ACCURACY;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROTATE_PID_ACCUMULATED_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROTATE_PID_LAST_TIME;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROTATE_PID_LAST_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.DRIVE_PID_ACCUMULATED_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.DRIVE_PID_LAST_TIME;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.DRIVE_PID_LAST_ERROR;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AutonStateEnum;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AutonTelemetry;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.DriveVelocityResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.DriveSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Map;

public abstract class AutonomousBaseLinearOpMode extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
    private HardwareManager hardwareManager;
    private AprilTagEngine aprilTagEngine;
    private ArtifactColorEnum intakeArtifactColor = ArtifactColorEnum.NONE;
    private ArtifactColorEnum launcherArtifactColor = ArtifactColorEnum.NONE;
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double flywheelRPM = 0;
    private double targetRPM = 0;
    private boolean allowedToLaunch = false;
    private boolean launcherEnabled = false;
    private boolean atTargetPosition = true;
    private double stateTimestamp = getRuntime();
    private Limelight3A limelight;
    private SparkFunOTOS otos;
    abstract AprilTagEnum getTargetAprilTag();
    AutonStateEnum state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread aprilTagEngineThread = new Thread(aprilTagEngine);
        try {
            while (!isStarted() && !isStopRequested()) {
                setArtifactColors();
                updateSparkfunOdometry();
                refreshTelemetry();
            }
            waitForStart();
            resetRuntime();
            aprilTagEngineThread.start();
            while (opModeIsActive()) {
                updateRuntime();
                updateSparkfunOdometry();
                setArtifactColors();
                telemetry.addData("Target name", getTargetAprilTag().name());
                telemetry.addData("Target id", getTargetAprilTag().getId());
                switch (state) {
                    case WAIT_LAUNCH_ARTIFACT_1:
                        launcherEnabled = true;
                        if (launcherArtifactColor != NONE && getRuntime() - stateTimestamp < 5) {
                            allowedToLaunch = true;
                        } else {
                            allowedToLaunch = false;
                            state = AutonStateEnum.WAIT_INTAKE_LOAD_ARTIFACT_2;
                            sleep(100);
                            stateTimestamp = getRuntime();
                        }
                        break;
                    case WAIT_INTAKE_LOAD_ARTIFACT_2:
                        if (launcherArtifactColor == NONE && getRuntime() - stateTimestamp < 5) {
                            startIntake();
                            sleep(50);
                            stopIntake();
                            sleep(50);
                        } else {
                            state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_2;
                            sleep(100);
                            stateTimestamp = getRuntime();
                        }
                        break;
                    case WAIT_LAUNCH_ARTIFACT_2:
                        launcherEnabled = true;
                        if (launcherArtifactColor != NONE && getRuntime() - stateTimestamp < 5) {
                            allowedToLaunch = true;
                        } else {
                            allowedToLaunch = false;
                            state = AutonStateEnum.WAIT_INTAKE_LOAD_ARTIFACT_3;
                            sleep(100);
                            stateTimestamp = getRuntime();
                        }
                        break;
                    case WAIT_INTAKE_LOAD_ARTIFACT_3:
                        if (launcherArtifactColor == NONE && getRuntime() - stateTimestamp < 5) {
                            startIntake();
                            sleep(50);
                            stopIntake();
                            sleep(50);
                        } else {
                            state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_3;
                            sleep(100);
                            stateTimestamp = getRuntime();
                        }
                        break;
                    case WAIT_LAUNCH_ARTIFACT_3:
                        launcherEnabled = true;
                        if (launcherArtifactColor != NONE && getRuntime() - stateTimestamp < 5) {
                            allowedToLaunch = true;
                        } else {
                            allowedToLaunch = false;
                            state = AutonStateEnum.WAIT_DRIVE_FROM_LAUNCH_ZONE;
                            sleep(150);
                            stateTimestamp = getRuntime();
                            atTargetPosition = false;
                        }
                        break;
                    case WAIT_DRIVE_FROM_LAUNCH_ZONE:
                        launcherEnabled = false;
                        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET && !atTargetPosition) {
                            startDrive(0.200, 0.200, 0.0, 0.020);
                        } else {
                            stopDrive();
                            state = AutonStateEnum.FINISHED;
                            stateTimestamp = getRuntime();
                        }
                    case FINISHED:
                        launcherEnabled = false;
                        allowedToLaunch = false;
                        if (launcherArtifactColor != NONE && getRuntime() < 20) {
                            state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_1;
                            stateTimestamp = getRuntime();
                        }
                        break;
                    default:
                        stateTimestamp = getRuntime();
                        state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_1;
                }
                autoLaunch();
                timeIsUpMove();
                refreshTelemetry();
            }
        } finally {
            aprilTagEngineThread.interrupt();
            aprilTagEngine.teardown();
            refreshTelemetry();
        }

    }

    private void refreshTelemetry() {
        telemetry.addData("Current State", state);
        telemetry.addData("Time in State (s)", (getRuntime() - stateTimestamp));
        telemetry.addData("Target Flywheel RPM", targetRPM);
        telemetry.addData("Actual Flywheel RPM", flywheelRPM);
        telemetry.addData("Launcher Angle", launchAngle);
        telemetry.addData("Robot Heading Offset (deg)", BOT_HEADING_OFFSET);
        telemetry.addData("Launcher Artifact Color", launcherArtifactColor);
        telemetry.addLine(String.format("Robot Position (x,y,h) (m,m,deg) %6.3f %6.3f %6.3f", ROBOT_FIELD_X, ROBOT_FIELD_Y, ROBOT_FIELD_H));
        Map<String, String> aprilTagTelemetry = aprilTagEngine.getTelemetry();
        for (String key : aprilTagTelemetry.keySet()) {
            String value = aprilTagTelemetry.get(key);
            telemetry.addData(key, value);
        }
        telemetry.update();
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        otos = hardwareManager.getOtos();
        state = AutonStateEnum.WAIT_LAUNCH_ARTIFACT_1;
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
                "Runtime",
                "%02.0f:%02.0f",
                totalSeconds / 60,
                totalSeconds % 60);
    }

    private boolean timeIsUpMove() {
        double totalSeconds = getRuntime();
        if (totalSeconds > 25 && state != AutonStateEnum.FINISHED) {
            state = AutonStateEnum.WAIT_DRIVE_FROM_LAUNCH_ZONE;
            atTargetPosition = false;
            return true;
        } else {
            return false;
        }
    }

    private void setArtifactColors() {
        intakeArtifactColor = getArtifactColor(hardwareManager.getIntakeColorSensor(), "Intake");
        launcherArtifactColor = getArtifactColor(hardwareManager.getLaunchColorSensor(), "Launcher");
    }

    private ArtifactColorEnum getArtifactColor(RevColorSensorV3 colorSensor, String sensorName) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
        double distanceCM = colorSensor.getDistance(DistanceUnit.CM);

        ArtifactColorEnum artifactColorEnum;
        if (blue > green && red > 0.15 && blue > 0.25) {
            artifactColorEnum = PURPLE;
        } else if (green > red && green > blue && green > 0.25) {
            artifactColorEnum = GREEN;
        } else if (distanceCM < 3) {
            artifactColorEnum = UNKNOWN;
        } else {
            artifactColorEnum = NONE;
        }
        telemetry.addData(sensorName + " - RGB", "%6.3f %6.3f %6.3f", red, green, blue);
        return artifactColorEnum;
    }

    private void autoLaunch() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if(timedDetection != null && timedDetection.getDetection() != null) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {

                /*
                Aquire target lock
                 */
                if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(targetDetection.ftcPose.bearing)) {
                    telemetry.addData("Turret Angle", (targetDetection.ftcPose.bearing));
                    double setPower = turretBearingPid.calculate(1, targetDetection.ftcPose.bearing);
                    telemetry.addData("Turret Power", setPower);
                    hardwareManager.getTurretMotor().setPower(setPower);
                } else {
                    hardwareManager.getTurretMotor().setPower(0);
                }

                /*
                Generate launch system parameters and generate firing solution
                 */
                targetDistance = targetDetection.ftcPose.range;
                flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                targetRPM = Math.round(((targetDistance / 1.670) * 900.0) + 1750.0);
                LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult1(flywheelRPM, targetDistance);

                if (launcherEnabled) {
                    if (launchResult != null) {
                        setLaunchAngle(launchResult.getLaunchAngle());
                    }
                    hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
                    if (Math.abs(((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0) - targetRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL && allowedToLaunch) {
                        autoLaunchArtifact();
                    }
                } else {
                    hardwareManager.getLauncherMotor().setVelocity(0);
                }

            } else {
                //stop rotating turret if detection is too old
                hardwareManager.getTurretMotor().setPower(0);
            }
        } else {
            //stop rotating turret if there is no detection
            hardwareManager.getTurretMotor().setPower(0);
        }
    }

    private void startIntake() {
        hardwareManager.getIntakeMotor().setPower(1);
    }
    private void stopIntake() {
        hardwareManager.getIntakeMotor().setPower(1);
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((70.0 - launchAngle) / 35.0));
        if(positionValue >= 0 && positionValue <= 0.8) {
            hardwareManager.getAngleServo().setPosition(positionValue);
        }
        this.launchAngle = launchAngle;
    }

    private void autoLaunchArtifact() {
        if (launcherArtifactColor != ArtifactColorEnum.NONE) {
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_UP);
            sleep(150);
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_UP);
            sleep(150);
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
            sleep(150);
        }
    }

    /*
    Check to see if the turret is against a limit switch
     */
    private boolean canRotateTurret(double input) {
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()) {
            return false;
        } else if (input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()){
            return false;
        } else {
            return true;
        }
    }

    private double getLimeLightCoordinates() {
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            // Getting numbers from Python
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                double targetFound = pythonOutputs[0];
                double targetBearing = pythonOutputs[1];
                double targetDistance = pythonOutputs[2];
                ArtifactColorEnum targetColor;

                if (pythonOutputs[5] == 1) {
                    targetColor = ArtifactColorEnum.GREEN;
                } else if (pythonOutputs[5] == 2) {
                    targetColor = ArtifactColorEnum.PURPLE;
                } else {
                    targetColor = ArtifactColorEnum.NONE;
                }

                telemetry.addData("LL Target Found", targetFound);
                telemetry.addData("LL Target Bearing (deg)", targetBearing);
                telemetry.addData("LL Target Distance (m)", targetDistance / 1000);
                telemetry.addData("LL Target Color", targetColor);
                telemetry.update();
            }
        }
        return 0;
    }


    private void updateSparkfunOdometry() {
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        ROBOT_FIELD_X = pos.x;
        ROBOT_FIELD_Y = pos.y;
        ROBOT_FIELD_H = pos.h;
    }

    /* Auton Driving Functions ************************************************************************************ */
     /*
    Drive to target coordinates
    */
    public void startDrive(double x, double y, double h, double tgtTol) {
        double targetH = AngleUnit.DEGREES.normalize(h);
        if (Math.abs(getDriveDistance(x,y)) < tgtTol) {
            DriveVelocityResult motorVelocities = calculateMotorSpeeds(x, y, h);
            hardwareManager.getLeftFrontMotor().setVelocity(motorVelocities.lfDriveVelocity());
            hardwareManager.getLeftRearMotor().setVelocity(motorVelocities.lrDriveVelocity());
            hardwareManager.getRightFrontMotor().setVelocity(motorVelocities.rfDriveVelocity());
            hardwareManager.getRightRearMotor().setVelocity(motorVelocities.rrDriveVelocity());
            atTargetPosition = false;
        } else {
            rotateRobot(h);
        }

    }

    private void stopDrive() {
        hardwareManager.getLeftFrontMotor().setVelocity(0);
        hardwareManager.getLeftRearMotor().setVelocity(0);
        hardwareManager.getRightFrontMotor().setVelocity(0);
        hardwareManager.getRightRearMotor().setVelocity(0);
    }

    /*
    Rotate robot to desired heading
     */
    public void rotateRobot(double h) {
        double steeringCorrection;
        double rotateVelocity;
        double headingError = AngleUnit.DEGREES.normalize(h - Math.cos(normalizedFieldBotHeading()));
        // Keep looping while we are still active and not on heading.
        if (Math.abs(headingError) >= ROTATION_ACCURACY) {
            headingError = AngleUnit.DEGREES.normalize(h - Math.cos(normalizedFieldBotHeading()));
            if (headingError > 0) {
                rotateVelocity = rotatePidPower(headingError) * AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND;
            } else {
                rotateVelocity = -(rotatePidPower(headingError) * AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND);
            }
            // Clip the speed to the maximum permitted value.
            hardwareManager.getLeftFrontMotor().setVelocity(-rotateVelocity);
            hardwareManager.getLeftRearMotor().setVelocity(-rotateVelocity);
            hardwareManager.getRightFrontMotor().setVelocity(rotateVelocity);
            hardwareManager.getRightRearMotor().setVelocity(rotateVelocity);
            atTargetPosition = false;
        } else {
            stopDrive();
            atTargetPosition = true;
        }
    }

    /*
    Calculate motor velocities to get to target
     */
    private DriveVelocityResult calculateMotorSpeeds(double targetX, double targetY, double targetH) {
        double RX;
        double headingError = AngleUnit.DEGREES.normalize(targetH - Math.cos(normalizedFieldBotHeading() / 180 * Math.PI));
        double x = Math.sin(getDriveAngle(targetX,targetY) / 180 * Math.PI) * drivePidPower(getDriveDistance(targetX,targetY));
        double y = Math.cos(getDriveAngle(targetX,targetY) / 180 * Math.PI) * drivePidPower(getDriveDistance(targetX,targetY));
        if (headingError > 0) {
            RX = rotatePidPower(headingError) * 0.8;
        } else {
            RX = -(rotatePidPower(headingError) * 0.8);
        }
        double cosHeading = Math.cos(normalizedFieldBotHeading() / 180 * Math.PI);
        double sinHeading = Math.sin(normalizedFieldBotHeading() / 180 * Math.PI);
        double rotX = y * sinHeading - x * cosHeading;
        double rotY = y * cosHeading + x * sinHeading;
        rotX = rotX * 1.1;  // Counteract imperfect strafing
        //double Denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(RX))), 1));
        double lfDriveVelocity = (int) (AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND * -((rotY + rotX + RX))); // Denominator));
        double rfDriveVelocity = (int) (AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND * -(((rotY - rotX) - RX))); // Denominator));
        double lrDriveVelocity = (int) (AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND * -(((rotY - rotX) + RX))); // Denominator));
        double rrDriveVelocity = (int) (AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND * -(((rotY + rotX) - RX))); // Denominator));

        return new DriveVelocityResult(lfDriveVelocity, rfDriveVelocity, lrDriveVelocity, rrDriveVelocity);
    }

    /*
    Function to convert -180:180 degree field headings into 0-360 degree headings
     */
    private double normalizedFieldBotHeading() {
        YawPitchRollAngles orientation = hardwareManager.getImu().getRobotYawPitchRollAngles();
        ROBOT_FIELD_H = Double.parseDouble(JavaUtil.formatNumber(AngleUnit.DEGREES.normalize(orientation.getYaw(AngleUnit.DEGREES) + BOT_HEADING_OFFSET), 3));
        return ROBOT_FIELD_H;
    }


    /*
    Function to control rotation speed based on error magnitude
     */
    private double rotatePidPower(double h) {
        double headingError = AngleUnit.DEGREES.normalize(h - normalizedFieldBotHeading());
        // Determine the heading current error.
        if (headingError > 180) {
            headingError = headingError - 360;
        } else if (headingError <= -180) {
            headingError = headingError + 360;
        }
        ROTATE_PID_ACCUMULATED_ERROR = (int) (ROTATE_PID_ACCUMULATED_ERROR + headingError);
        if (Math.abs(headingError) < 1) {
            ROTATE_PID_ACCUMULATED_ERROR = 0;
        }
        // Ensure sign of ROTATE_PID_ACCUMULATED_ERROR matches headingError
        if (headingError < 0) {
            ROTATE_PID_ACCUMULATED_ERROR = Math.abs(ROTATE_PID_ACCUMULATED_ERROR) * -1;
        } else {
            ROTATE_PID_ACCUMULATED_ERROR = Math.abs(ROTATE_PID_ACCUMULATED_ERROR) * 1;
        }
        double rotatePID_slope = 0;
        if (ROTATE_PID_LAST_TIME > 0) {
            rotatePID_slope = (int) ((headingError - ROTATE_PID_LAST_ERROR) / (System.currentTimeMillis() - ROTATE_PID_LAST_TIME));
            ROTATE_PID_LAST_TIME = System.currentTimeMillis();
            ROTATE_PID_LAST_ERROR = headingError;
        }
        return Range.clip(Math.abs(Range.clip(headingError, 0, 0.2)) + Range.scale(Math.abs(headingError * ROTATEPID_P + ROTATE_PID_ACCUMULATED_ERROR * ROTATEPID_I + ROTATEPID_D * rotatePID_slope), 0, 180, 0.1, 0.8), 0.2, 1);
    }

    /*
    PID Function to control drive power based on error
     */
    private double drivePidPower(double driveDistance) {
        DRIVE_PID_ACCUMULATED_ERROR = (int) (DRIVE_PID_ACCUMULATED_ERROR + (System.currentTimeMillis() - DRIVE_PID_LAST_TIME) * driveDistance);
        if (driveDistance < 1) {
            DRIVE_PID_ACCUMULATED_ERROR = 0;
        }
        double drivePidSlope = 0;
        if (DRIVE_PID_LAST_TIME > 0) {
            drivePidSlope = (int) ((driveDistance - DRIVE_PID_LAST_ERROR) / (System.currentTimeMillis() - DRIVE_PID_LAST_TIME));
            DRIVE_PID_LAST_TIME = System.currentTimeMillis();
            DRIVE_PID_LAST_ERROR = driveDistance;
        }
        return Range.scale(Math.abs(driveDistance * DRIVEPID_P + DRIVE_PID_ACCUMULATED_ERROR * DRIVEPID_I + DRIVEPID_D * drivePidSlope), 0, 60, 0.1, 1);
    }

    /*
    Calculate the distance to the target coordinates
     */
    private double getDriveDistance(double targetX, double targetY) {
        double deltaX = ROBOT_FIELD_X - targetX;
        double deltaY = ROBOT_FIELD_Y - targetY;
        return Math.abs(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));
    }

    /*
    calculate the angle to the target coordinates
     */
    private double getDriveAngle(double targetX, double targetY) {
        double deltaX = ROBOT_FIELD_X - targetX;
        double deltaY = ROBOT_FIELD_Y - targetY;
        return Math.atan2(deltaY, deltaX) / Math.PI * 180;
    }


}

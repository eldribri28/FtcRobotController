package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.UNKNOWN;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.APRIL_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.NO_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.VIABLE_LAUNCH_SOLUTION;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_DRIVE_VELOCITY_TICKS_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CURRENT_ENCODER;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CHASSIS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.IndicatorLedEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.security.PrivateKey;
import java.util.Map;


public abstract class TeleOpBaseLinearOpMode extends LinearOpMode {
    private HardwareManager hardwareManager;
    private ArtifactColorEnum intakeArtifactColor = UNKNOWN;
    private ArtifactColorEnum launcherArtifactColor = UNKNOWN;
    private ArtifactColorEnum launcherArtifactColor2 = UNKNOWN;
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double flywheelRPM = 0;
    private double targetRPM = 0;
    private AprilTagEngine aprilTagEngine;

    abstract AprilTagEnum getTargetAprilTag();

    private SparkFunOTOS otos;

    private double currentX = 0;
    private double currentY = 0;
    private double currentOtosH = 0;
    private double aprilTagBearing = 0;
    private double TURRET_ERROR = 0;
    private double launchTimer = 0;
    private double previousProfileTime = 0;

    /*
    Drive Motor Ramping
     */
    private double DRIVE_INCREMENT = 0.15;     // amount to ramp motor each CYCLE_MS cycle
    private double DRIVE_CYCLE_S = 0.050;     // period of each cycle
    private double lastDriveTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread aprilTagEngineThread = new Thread(aprilTagEngine);
        try {
            waitForStart();
            resetRuntime();
            aprilTagEngineThread.start();
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
            while (opModeIsActive()) {
                updateTelemetry();

                resetIMU();
                drive();
                intakeOrRejectArtifact();
                setArtifactColors();
                clearArtifactFromLaunch();

                turretRotateLimit();

                if (updateAprilTagFieldPosition()) {
                    TURRET_ERROR = aprilTagBearing;
                } else {
                    TURRET_ERROR = calculateTurretError();
                }
                moveTurret(TURRET_ERROR);
                getFieldPosition();


                launch();
                telemetry.update();
            }
        } finally {
            aprilTagEngineThread.interrupt();
            aprilTagEngine.teardown();
            telemetry.update();
        }
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        hardwareManager.getRedLed().off();
        hardwareManager.getGreenLed().off();
        hardwareManager.getTurretMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareManager.getTurretMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        otos = hardwareManager.getOtos();
        otos.setAngularUnit(AngleUnit.DEGREES);
        TURRET_CHASSIS_OFFSET = 0;
        TURRET_LEFT_LIMIT_ENCODER_VALUE = 0;
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
                "Runtime",
                "%02.0f:%02.0f",
                totalSeconds / 60,
                totalSeconds % 60);
    }

    private void updateTelemetry() {
        updateRuntime();
        telemetry.addData("Target name", getTargetAprilTag().name());
        telemetry.addData("Motif detected", aprilTagEngine.getArtifactMotif().name());
        telemetry.addData("Target distance", targetDistance);
        telemetry.addData("Launch Angle", launchAngle);
        telemetry.addData("Turret Error (deg)", TURRET_ERROR);
        telemetry.addLine(String.format("Current Position: %6.1f %6.1f %6.1f  (meter, deg)", currentX, currentY, currentOtosH));
        telemetry.addData("Target Calculated Heading (deg)", getTargetBearing(currentX, currentY));

        Map<String, String> aprilTagTelemetry = aprilTagEngine.getTelemetry();
        for (String key : aprilTagTelemetry.keySet()) {
            String value = aprilTagTelemetry.get(key);
            telemetry.addData(key, value);
        }
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 0.4;
        YawPitchRollAngles orientation = hardwareManager.getImu().getRobotYawPitchRollAngles();
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        //calculate power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftRearPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightRearPower = (rotY + rotX - rx) / denominator;

        /*
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }
        */

        double leftFrontVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * leftFrontPower;
        double rightFrontVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * rightFrontPower;
        double leftRearVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * leftRearPower;
        double rightRearVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * rightRearPower;
        if (leftFrontVelocity == 0) {
            hardwareManager.getLeftFrontMotor().setPower(0);
        } else {
            hardwareManager.getLeftFrontMotor().setVelocity(leftFrontVelocity);
        }
        if (rightFrontVelocity == 0) {
            hardwareManager.getRightFrontMotor().setPower(0);
        } else {
            hardwareManager.getRightFrontMotor().setVelocity(rightFrontVelocity);
        }
        if (leftRearVelocity == 0) {
            hardwareManager.getLeftRearMotor().setPower(0);
        } else {
            hardwareManager.getLeftRearMotor().setVelocity(leftRearVelocity);
        }
        if (rightRearVelocity == 0) {
            hardwareManager.getRightRearMotor().setPower(0);
        } else {
            hardwareManager.getRightRearMotor().setVelocity(rightRearVelocity);
        }

        telemetry.addData("Bearing", -botHeading);
        telemetry.addData("L-Stick X", -gamepad1.left_stick_x);
        telemetry.addData("L-Stick y", -gamepad1.left_stick_y);
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);



        /*
        if (getRuntime() - lastDriveTime >=  DRIVE_CYCLE_S) {

            double leftFrontVelocity = driveMotorRamp(leftFrontPower, hardwareManager.getLeftFrontMotor().getPower());
            double rightFrontVelocity = driveMotorRamp(rightFrontPower, hardwareManager.getRightFrontMotor().getPower());
            double leftRearVelocity = driveMotorRamp(leftRearPower, hardwareManager.getLeftRearMotor().getPower());
            double rightRearVelocity = driveMotorRamp(rightRearPower, hardwareManager.getRightRearMotor().getPower());

            if (leftFrontVelocity == 0) {
                hardwareManager.getLeftFrontMotor().setPower(0);
            } else {
                hardwareManager.getLeftFrontMotor().setPower(leftFrontVelocity * 0.5);
            }
            if (rightFrontVelocity == 0) {
                hardwareManager.getRightFrontMotor().setPower(0);
            } else {
                hardwareManager.getRightFrontMotor().setPower(rightFrontVelocity * 0.5);
            }
            if (leftRearVelocity == 0) {
                hardwareManager.getLeftRearMotor().setPower(0);
            } else {
                hardwareManager.getLeftRearMotor().setPower(leftRearVelocity * 0.5);
            }
            if (rightRearVelocity == 0) {
                hardwareManager.getRightRearMotor().setPower(0);
            } else {
                hardwareManager.getRightRearMotor().setPower(rightRearVelocity * 0.5);
            }

        }

        */

        //telemetry.addData("Commanded Motor (Left Front)", "%.2f", leftFrontVelocity);
        //telemetry.addData("Commanded Motor (Right Front)", "%.2f", rightFrontVelocity);
        //telemetry.addData("Commanded Motor (Left Rear)", "%.2f", leftRearVelocity);
        //telemetry.addData("Commanded Motor (Right Rear)", "%.2f", rightRearVelocity);

        //telemetry.addData("Actual Motor (Left Front)", "%.2f", hardwareManager.getLeftFrontMotor().getVelocity());
        //telemetry.addData("Actual Motor (Right Front)", "%.2f", hardwareManager.getRightFrontMotor().getVelocity());
        //telemetry.addData("Actual Motor (Left Rear)", "%.2f", hardwareManager.getLeftRearMotor().getVelocity());
        //telemetry.addData("Actual Motor (Right Rear)", "%.2f", hardwareManager.getRightRearMotor().getVelocity());
    }

    private void resetIMU() {
        if (hardwareManager.getGamepad1().dpad_right) {
            hardwareManager.getImu().resetYaw();
        }
    }

    private void intakeOrRejectArtifact() {
        if (hardwareManager.getGamepad1().left_trigger > 0) {
            hardwareManager.getIntakeMotor().setPower(1);
        } else if (hardwareManager.getGamepad1().left_bumper) {
            hardwareManager.getIntakeMotor().setPower(-1);
        } else {
            hardwareManager.getIntakeMotor().setPower(0);
        }
    }


    private void clearArtifactFromLaunch() {
        if (hardwareManager.getGamepad1().right_bumper) {
            hardwareManager.getLauncherMotor().setPower(0.4);
            sleep(100);
            autoLaunchArtifact();
        }
    }

    private void launch() {
        autoLaunch();
        autoLaunchServoDown();
    }

    private boolean canRotateTurret(double input) {
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()) {
            return false;
        } else if (input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()) {
            return false;
        } else {
            return true;
        }
    }

    private void turretRotateLimit() {
        if (hardwareManager.getLimitSwitchRight().isPressed()) {
            TURRET_LEFT_LIMIT_ENCODER_VALUE = (long) AngleUnit.DEGREES.normalize(Math.round(TURRET_CURRENT_ENCODER + (TURRET_TICKS_PER_DEGREE * 90)));
        } else if (hardwareManager.getLimitSwitchLeft().isPressed()) {
            TURRET_LEFT_LIMIT_ENCODER_VALUE = (long) AngleUnit.DEGREES.normalize(Math.round(TURRET_CURRENT_ENCODER - (TURRET_TICKS_PER_DEGREE * 90)));
        }
    }

    private void setArtifactColors() {
        intakeArtifactColor = getArtifactColor(hardwareManager.getIntakeColorSensor(), "Intake");
        launcherArtifactColor = getArtifactColor(hardwareManager.getLaunchColorSensor(), "Launcher");
        launcherArtifactColor2 = getArtifactColor(hardwareManager.getLaunchColorSensor2(), "Launcher2");
        if (launcherArtifactColor == ArtifactColorEnum.GREEN || launcherArtifactColor2 == ArtifactColorEnum.GREEN) {
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.GREEN.getLedValue());
        } else if (launcherArtifactColor == ArtifactColorEnum.PURPLE || launcherArtifactColor2 == ArtifactColorEnum.PURPLE) {
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.PURPLE.getLedValue());
        } else {
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.WHITE.getLedValue());
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.BLACK.getLedValue());
        }
    }

    private ArtifactColorEnum getArtifactColor(RevColorSensorV3 colorSensor, String sensorName) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
        double distanceCM = colorSensor.getDistance(DistanceUnit.CM);

        ArtifactColorEnum artifactColorEnum;
        if (blue > green && red > 0.2 && blue > 0.2) {
            artifactColorEnum = ArtifactColorEnum.PURPLE;
        } else if (green > red && green > blue && green > 0.2) {
            artifactColorEnum = ArtifactColorEnum.GREEN;
        } else {
            artifactColorEnum = ArtifactColorEnum.NONE;
        }
        telemetry.addData(sensorName + " - RGB", "%6.3f %6.3f %6.3f", red, green, blue);
        return artifactColorEnum;
    }

    private void autoLaunch() {
        boolean bearingGood = false;

        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if (timedDetection != null && timedDetection.getDetection() != null) {
            setLedStates(APRIL_TAG_DETECTED);
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
                if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(targetDetection.ftcPose.bearing) && targetDetection.id == getTargetAprilTag().getId()) {
                    if (Math.abs(targetDetection.ftcPose.bearing) < 3) {
                        bearingGood = true;
                    }
                    telemetry.addData("Turret Angle", (targetDetection.ftcPose.bearing));
                    telemetry.addData("Target ID", targetDetection.id);
                }
                targetDistance = targetDetection.ftcPose.range;

                flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                telemetry.addData("flywheel RPM", flywheelRPM);

                LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult1(flywheelRPM, targetDistance);
                if (launchResult != null) {
                    setLaunchAngle(launchResult.getLaunchAngle());
                    setLedStates(VIABLE_LAUNCH_SOLUTION);
                }

                targetRPM = Math.round(((targetDistance / 1.670) * 920.0) + 1750.0);
                hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
                telemetry.addData("target RPM", targetRPM);

                if (hardwareManager.getGamepad1().right_trigger > 0) {
                    if (Math.abs(((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0) - targetRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL && bearingGood && TURRET_ERROR != 0) {
                        autoLaunchArtifact();
                    }
                } else {
                    hardwareManager.getLauncherMotor().setVelocity(0);
                }
            }
        } else {
            setLedStates(NO_TAG_DETECTED);
        }
    }

    private void setLedStates(LedStateEnum ledStateEnum) {
        if (ledStateEnum == APRIL_TAG_DETECTED) {
            hardwareManager.getRedLed().on();
            hardwareManager.getGreenLed().off();
        } else if (ledStateEnum == VIABLE_LAUNCH_SOLUTION) {
            hardwareManager.getRedLed().on();
            hardwareManager.getGreenLed().off();
        } else {
            hardwareManager.getRedLed().off();
            hardwareManager.getGreenLed().off();
        }
    }

    private void autoLaunchArtifact() {
        if (getRuntime() - launchTimer > 0.15) {
            //if ((launcherArtifactColor != ArtifactColorEnum.NONE && launcherArtifactColor2 != ArtifactColorEnum.NONE)) {
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_UP);
            launchTimer = getRuntime();
            //}
        }
    }

    private void autoLaunchServoDown() {
        if (getRuntime() - launchTimer > 0.150) {
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
            launchTimer = getRuntime();
        }
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((70.0 - launchAngle) / 35.0));
        if (positionValue >= 0 && positionValue <= 0.8) {
            hardwareManager.getAngleServo().setPosition(positionValue);
        }
        this.launchAngle = launchAngle;
    }

    /*
    Set the field position based on apriltag
    */
    private void setFieldPosition(AprilTagDetection targetDetection) {
        if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
            TURRET_CHASSIS_OFFSET = getTurretChassisOffset(TURRET_CURRENT_ENCODER);
            double chassisFieldHeading = AngleUnit.DEGREES.normalize(targetDetection.robotPose.getOrientation().getYaw() - TURRET_CHASSIS_OFFSET);
            OTOSCalculator.setCurrentPosition(targetDetection.robotPose.getPosition().x, targetDetection.robotPose.getPosition().y, chassisFieldHeading, otos);
        }
    }

    /*
    If a tag is in view, update the robot position
     */
    private boolean updateAprilTagFieldPosition() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if (timedDetection != null && timedDetection.getDetection() != null && timedDetection.getAgeInMillis() < AGED_DATA_LIMIT_MILLISECONDS) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS) {
                setFieldPosition(targetDetection);
                if (targetDetection.id == getTargetAprilTag().getId()) {
                    aprilTagBearing = targetDetection.ftcPose.bearing;
                    return true;
                }
                return false;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    /*
    Lookup current field positon and heading
     */
    private void getFieldPosition() {
        currentX = OTOSCalculator.getCurrentPosition(otos).getXPos();
        currentY = OTOSCalculator.getCurrentPosition(otos).getYPos();
        currentOtosH = OTOSCalculator.getCurrentPosition(otos).getHeading();
    }

    /*
    Move the turret to the target
     */
    private void moveTurret(double turretError) {
        double setPower = turretBearingPid.calculate(0, turretError);
        setPower = Range.clip(setPower, -1, 1);
        if (canRotateTurret(setPower) && Math.abs(turretError) > 1) {
            hardwareManager.getTurretMotor().setPower(setPower);
        } else {
            hardwareManager.getTurretMotor().setVelocity(0);
        }
    }

    /*
    Calculate the current turret error if the camera doesn't see a tag
     */
    private double calculateTurretError() {
        double turretError = 0;
        double turretHeading = 0;
        double targetHeading = getTargetBearing(currentX, currentY);
        if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
            turretHeading = AngleUnit.DEGREES.normalize(currentOtosH + TURRET_ANGLE);
            turretError = AngleUnit.DEGREES.normalize(turretHeading - targetHeading);
        }
        return -turretError;
    }

    /*
    Locate target bearing based on robot position
     */
    private double getTargetBearing(double x, double y) {
        double deltaX = 0;
        double deltaY = 0;

        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            deltaX = (-1.371) - x;
            deltaY = (-1.371) - y;
        } else if (getTargetAprilTag() == AprilTagEnum.RED_TARGET) {
            deltaX = (-1.371) - x;
            deltaY = (1.371) - y;
        }

        return (AngleUnit.DEGREES.normalize(Math.toDegrees(Math.atan2(deltaY, deltaX)) - 90));
    }

    private double driveMotorRamp(double targetDriveVelocity, double currentDriveVelocity) {
        double velocity;

        if (targetDriveVelocity == 0) {
            return 0;
        } else if (targetDriveVelocity > currentDriveVelocity) {
             velocity = currentDriveVelocity + DRIVE_INCREMENT;
             if ( velocity > targetDriveVelocity ) { velocity = targetDriveVelocity; }
        } else if (targetDriveVelocity < currentDriveVelocity && currentDriveVelocity + DRIVE_INCREMENT > targetDriveVelocity) {
            velocity = currentDriveVelocity - DRIVE_INCREMENT;
            if ( velocity < targetDriveVelocity ) { velocity = targetDriveVelocity; }
        } else {
            velocity = targetDriveVelocity;
        }

        return velocity;
    }
}


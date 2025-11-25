package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.APRIL_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.NO_LAUNCH_SOLUTION;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.NO_TAG_DETECTED;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum.VIABLE_LAUNCH_SOLUTION;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCHER_MOTOR_IDLE_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_FAR_LAUNCH_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_LAUNCH_MOTOR_VELOCITY_START;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_NEAR_LAUNCH_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_DRIVE_VELOCITY_METER_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_DRIVE_VELOCITY_TICKS_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CHASSIS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.calculateTransitTime;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.calculateVelocity;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.LedStateEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.ColorManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


public abstract class TeleOpBaseLinearOpMode extends LinearOpMode {
    private HardwareManager hardwareManager;
    private ArtifactMotifEnum artifactMotifEnum = ArtifactMotifEnum.UNKNOWN;
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double flywheelRPM = 0;
    private double targetRPM = 0;
    private AprilTagEngine aprilTagEngine;
    private boolean isManualLaunchOverrideActive = false;
    private double manualLaunchVelocity = MANUAL_LAUNCH_MOTOR_VELOCITY_START;
    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    private ColorManager colorManager;
    abstract AprilTagEnum getTargetAprilTag();
    private boolean isShotTimeoutActive = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread aprilTagEngineThread = new Thread(aprilTagEngine);
        try {
            waitForStart();
            resetRuntime();
            aprilTagEngineThread.start();
            hardwareManager.postStartInitialization();
            while (opModeIsActive()) {
                updateTelemetry();
                turretRotateLimit();
                resetIMU();
                drive();
                intakeOrRejectArtifact();
                clearArtifactFromLaunch();
                colorManager.setArtifactColors();
                setArtifactMotifEnum();
                setManualLaunchOverride(aprilTagEngineThread);
                launch();
                telemetry.update();
            }
        } finally {
            scheduler.shutdownNow();
            aprilTagEngineThread.interrupt();
            aprilTagEngine.teardown();
            telemetry.update();
        }
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        colorManager = new ColorManager(hardwareManager);
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
        telemetry.addData("Manual Launch Controls Active", isManualLaunchOverrideActive);
        telemetry.addData("Target name", getTargetAprilTag().name());
        telemetry.addData("Motif detected", artifactMotifEnum.name());
        telemetry.addData("Intake artifact color", colorManager.getIntakeArtifactColor().name());
        telemetry.addData("Launcher artifact color", colorManager.getLauncherArtifactColor().name());
        telemetry.addData("Target distance", targetDistance);
        telemetry.addData("Launch Angle", launchAngle);
        telemetry.addData("Turret Limit Switch Left Pressed", hardwareManager.getLimitSwitchLeft().isPressed());
        telemetry.addData("Turret Limit Switch Right Pressed", hardwareManager.getLimitSwitchRight().isPressed());
        telemetry.addData("Turret Angle: (deg)", getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition()));
        OTOSResult otosData = OTOSCalculator.getCurrentPosition(hardwareManager.getOtos());
        telemetry.addLine(String.format("Current Position: %6.1f %6.1f %6.1f  (meter, deg)", otosData.getXPos(), otosData.getYPos(), otosData.getHeading()));

        telemetry.addData("Target Calculated Heading (deg)", getTargetBearing(otosData.getXPos(), otosData.getYPos()));
        //telemetry.addData("Turret Calculated Heading (deg)", calculateTurretAngleFromOtos(hardwareManager.getTurretMotor().getCurrentPosition(), otosData.getHeading()));

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

        double driverSelectedMultipler = 1;
        if(gamepad1.right_bumper) {
            driverSelectedMultipler = 2.0;
        } else if (gamepad1.b) {
            driverSelectedMultipler = 0.5;
        }
        double leftFrontVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * leftFrontPower * driverSelectedMultipler;
        double rightFrontVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * rightFrontPower * driverSelectedMultipler;
        double leftRearVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * leftRearPower * driverSelectedMultipler;
        double rightRearVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * rightRearPower * driverSelectedMultipler;

        telemetry.addData("Bearing", -botHeading);
        telemetry.addData("L-Stick X", -gamepad1.left_stick_x);
        telemetry.addData("L-Stick y", -gamepad1.left_stick_y);
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);

        if(leftFrontVelocity == 0) {
            hardwareManager.getLeftFrontMotor().setPower(0);
        } else {
            hardwareManager.getLeftFrontMotor().setVelocity(leftFrontVelocity);
        }
        if(rightFrontVelocity == 0) {
            hardwareManager.getRightFrontMotor().setPower(0);
        } else {
            hardwareManager.getRightFrontMotor().setVelocity(rightFrontVelocity);
        }
        if(leftRearVelocity == 0) {
            hardwareManager.getLeftRearMotor().setPower(0);
        } else {
            hardwareManager.getLeftRearMotor().setVelocity(leftRearVelocity);
        }
        if(rightRearVelocity == 0) {
            hardwareManager.getRightRearMotor().setPower(0);
        } else {
            hardwareManager.getRightRearMotor().setVelocity(rightRearVelocity);
        }

        telemetry.addData("Commanded Motor (Left Front)", "%.2f", leftFrontVelocity);
        telemetry.addData("Commanded Motor (Right Front)", "%.2f", rightFrontVelocity);
        telemetry.addData("Commanded Motor (Left Rear)", "%.2f", leftRearVelocity);
        telemetry.addData("Commanded Motor (Right Rear)", "%.2f", rightRearVelocity);
        telemetry.addData("Actual Motor (Left Front)", "%.2f", hardwareManager.getLeftFrontMotor().getVelocity());
        telemetry.addData("Actual Motor (Right Front)", "%.2f", hardwareManager.getRightFrontMotor().getVelocity());
        telemetry.addData("Actual Motor (Left Rear)", "%.2f", hardwareManager.getLeftRearMotor().getVelocity());
        telemetry.addData("Actual Motor (Right Rear)", "%.2f", hardwareManager.getRightRearMotor().getVelocity());
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

    private void setManualLaunchOverride(Thread aprilTagEngineThread) {
        if(hardwareManager.getGamepad2().dpad_left && hardwareManager.getGamepad2().x) {
            isManualLaunchOverrideActive = true;
            aprilTagEngineThread.interrupt();
            if(getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
                hardwareManager.getTurretMotor().setPower(0.8);
            } else {
                hardwareManager.getTurretMotor().setPower(-0.8);
            }
        } else if (hardwareManager.getGamepad1().dpad_left && hardwareManager.getGamepad1().x) {
            isManualLaunchOverrideActive = false;
            aprilTagEngineThread.start();
        }
    }

    private void launch() {
        if(isManualLaunchOverrideActive) {
            manualLaunch();
        } else {
            autoLaunch();
        }
    }
    private void manualLaunch() {
        if(hardwareManager.getGamepad2().dpad_up) {
            manualLaunchVelocity = MANUAL_FAR_LAUNCH_VELOCITY;
        }
        if(hardwareManager.getGamepad2().dpad_down) {
            manualLaunchVelocity = MANUAL_NEAR_LAUNCH_VELOCITY;
        }
        hardwareManager.getLauncherMotor().setVelocity(manualLaunchVelocity);

        if(hardwareManager.getGamepad2().right_trigger > 0) {
            autoLaunchArtifact();
            autoIntakeArtifact();
        }
    }

    private boolean canRotateTurret(double input) {
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()) {
            return false;
        } else if (input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()){
            return false;
        } else {
            return true;
        }
    }

    private void turretRotateLimit() {
        if(hardwareManager.getLimitSwitchRight().isPressed() || hardwareManager.getLimitSwitchLeft().isPressed()) {
            hardwareManager.getTurretMotor().setPower(0);
            int directionMultiplier = 0;
            if (hardwareManager.getLimitSwitchRight().isPressed()) {
                TURRET_LEFT_LIMIT_ENCODER_VALUE = Math.round(hardwareManager.getTurretMotor().getCurrentPosition() + (TURRET_TICKS_PER_DEGREE * 90));
                directionMultiplier = 1;
            } else if (hardwareManager.getLimitSwitchLeft().isPressed()) {
                TURRET_LEFT_LIMIT_ENCODER_VALUE = Math.round(hardwareManager.getTurretMotor().getCurrentPosition() - (TURRET_TICKS_PER_DEGREE * 90));
                directionMultiplier = -1;
            }
            if(isManualLaunchOverrideActive) {
                hardwareManager.getTurretMotor().setTargetPositionTolerance(5);
                hardwareManager.getTurretMotor().setTargetPosition((int) (TURRET_LEFT_LIMIT_ENCODER_VALUE + Math.round(TURRET_TICKS_PER_DEGREE * 45.0)));
                hardwareManager.getTurretMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareManager.getTurretMotor().setPower(0.8 * directionMultiplier);
            }
        }
    }

    private void setArtifactMotifEnum() {
        //get the motif if unknown, may continue to be unknown motif tag is seen
        if(artifactMotifEnum == ArtifactMotifEnum.UNKNOWN) {
            artifactMotifEnum = aprilTagEngine.getArtifactMotif();
        }
    }

    /*
    Target lead calculations to move while shooting.  Assumes infinite acceleration of chassis and no obstacles.
     */
    private double targetLeadCalculation() {

        double transitTime = calculateTransitTime(targetDistance, calculateVelocity(flywheelRPM), launchAngle);
        double angleChange = 0;

        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET && flywheelRPM > 0) {

            double deltaY = targetDistance - (hardwareManager.getGamepad1().left_stick_y * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);
            double deltaX = 0 - (hardwareManager.getGamepad1().left_stick_x * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);

            angleChange = (Math.toDegrees(Math.atan2(deltaY, deltaX)) - 90) * 0.5;

        } else if (getTargetAprilTag() == AprilTagEnum.RED_TARGET && flywheelRPM > 0) {

            double deltaY = targetDistance - (hardwareManager.getGamepad1().left_stick_y * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);
            double deltaX = 0 - (hardwareManager.getGamepad1().left_stick_x * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);

            angleChange = (-Math.toDegrees(Math.atan2(deltaY, deltaX)) + 90) * 0.5;

        }

        telemetry.addData("Turret Lead (deg)", angleChange);
        return angleChange;

    }

    private void autoLaunch() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if(timedDetection != null && timedDetection.getDetection() != null) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
                setLedStates(APRIL_TAG_DETECTED);
                if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(targetDetection.ftcPose.bearing) && targetDetection.id == getTargetAprilTag().getId()) {
                    telemetry.addData("Turret Angle", (targetDetection.ftcPose.bearing));
                    telemetry.addData("Target ID", targetDetection.id);
                    double setPower = turretBearingPid.calculate(0, targetDetection.ftcPose.bearing);
                    telemetry.addData("Turret Power", setPower);
                    hardwareManager.getTurretMotor().setPower(setPower);

                    if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
                        TURRET_CHASSIS_OFFSET = getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition());
                        double chassisFieldHeading = (targetDetection.robotPose.getOrientation().getYaw() - TURRET_CHASSIS_OFFSET);
                        telemetry.addData("AprilTag Turret Field Heading (deg)", targetDetection.robotPose.getOrientation().getYaw());
                        telemetry.addData("Calculated Chassis Field Heading (deg)", chassisFieldHeading);
                        OTOSCalculator.setCurrentPosition(targetDetection.robotPose.getPosition().x, targetDetection.robotPose.getPosition().y, chassisFieldHeading, hardwareManager.getOtos());
                    }
                } else {
                    hardwareManager.getTurretMotor().setPower(0);
                }
                targetDistance = targetDetection.ftcPose.range;
                flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                telemetry.addData("flywheel RPM", flywheelRPM);

                LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult(flywheelRPM, targetDistance);
                if (launchResult != null) {
                    setLaunchAngle(launchResult.getLaunchAngle());
                    setLedStates(VIABLE_LAUNCH_SOLUTION);
                } else {
                    setLedStates(NO_LAUNCH_SOLUTION);
                }

                telemetry.addData("target RPM", targetRPM);
                if (hardwareManager.getGamepad1().right_trigger > 0) {
                    targetRPM = Math.round(((targetDistance / 1.670) * 925) + 1725);
                    if (readyToShoot(targetDetection)) {
                        autoLaunchArtifact();
                    }
                } else {
                    targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
                }
            } else {
                setNoTagDetected();
            }
        } else {
            setNoTagDetected();
        }
        hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
    }

    private void setNoTagDetected() {
        hardwareManager.getTurretMotor().setPower(0);
//        targetRPM = LAUNCHER_MOTOR_IDLE_VELOCITY;
        setLedStates(NO_TAG_DETECTED);
    }

    public boolean readyToShoot(AprilTagDetection targetDetection) {
        return isTurretAngleWithinThreshold(targetDetection)
                && isLaunchMotorVelocityWithinThreshold()
                && !isShotTimeoutActive;
    }

    private boolean isTurretAngleWithinThreshold(AprilTagDetection detection) {
        double bearing = Math.abs(detection.ftcPose.bearing);
        if(targetDistance > 60.0) {
            return bearing <= 0.5;
        }
        return bearing <= 2.0;
    }

    private boolean isLaunchMotorVelocityWithinThreshold() {
        double ratio = targetRPM / ((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0);
        return ratio > 0.995 && ratio <1.005;
    }

    private void setLedStates(LedStateEnum ledStateEnum) {
        if(ledStateEnum == APRIL_TAG_DETECTED) {
            hardwareManager.getRedLed().on();
            hardwareManager.getGreenLed().off();
        } else if (ledStateEnum == VIABLE_LAUNCH_SOLUTION) {
            hardwareManager.getRedLed().off();
            hardwareManager.getGreenLed().on();
        } else {
            hardwareManager.getRedLed().off();
            hardwareManager.getGreenLed().off();
        }
    }

    private void autoLaunchArtifact() {
        hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_UP);
        isShotTimeoutActive = true;
        Runnable servoDown = () -> {
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
            isShotTimeoutActive = false;
        };
        scheduler.schedule(servoDown, 150, TimeUnit.MILLISECONDS);
    }

    private void clearArtifactFromLaunch() {
        if((isManualLaunchOverrideActive && hardwareManager.getGamepad2().x)
                || (!isManualLaunchOverrideActive && hardwareManager.getGamepad1().x)) {
            hardwareManager.getLauncherMotor().setVelocity(LAUNCHER_MOTOR_IDLE_VELOCITY);
            Runnable autoLaunch = this::autoLaunchArtifact;
            scheduler.schedule(autoLaunch, 200, TimeUnit.MILLISECONDS);
            autoLaunchArtifact();
        }
        if (hardwareManager.getGamepad1().a) {
            Runnable autoLaunch = this::autoLaunchArtifact;
            scheduler.schedule(autoLaunch, 200, TimeUnit.MILLISECONDS);
            autoLaunchArtifact();
        }
    }

    private void autoIntakeArtifact() {
        hardwareManager.getIntakeMotor().setPower(1);
        Runnable stopIntake = () -> {
            hardwareManager.getIntakeMotor().setPower(0);
        };
        scheduler.schedule(stopIntake, 100, TimeUnit.MILLISECONDS);
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((69 - launchAngle) / 35.0));
        if(positionValue >= 0 && positionValue <= 0.8) {
            hardwareManager.getAngleServo().setPosition(positionValue);
        }
        this.launchAngle = launchAngle;
    }

    public double getTargetBearing(double x, double y) {
        double deltaX = 0;
        double deltaY = 0;

        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            deltaX = x - (-72);
            deltaY = y - (-72);
        } else if (getTargetAprilTag() == AprilTagEnum.RED_TARGET) {
            deltaX = x - (-72);
            deltaY = y - (72);
        }
        return (Math.toDegrees(Math.atan2(deltaY, deltaX)));// - getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition() + 360) % 360) ;
    }
}


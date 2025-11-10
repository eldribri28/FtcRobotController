package org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.UP;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_DOWN;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class HardwareManager {
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx rightRearMotor;
    private final DcMotorEx leftRearMotor;
    private final DcMotorEx intakeMotor;
    private final DcMotorEx turretMotor;
    private final DcMotorEx launcherMotor;
    private final Servo angleServo;
    private final Servo launchServo;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final WebcamName turretCam;
    private final IMU imu;
    private final SparkFunOTOS otos;
    private final RevColorSensorV3 launchColorSensor;
    private final RevColorSensorV3 launchColorSensor2;
    private final LED redLED;
    private final LED greenLED;
    private final Servo indicatorLED;
    private final RevColorSensorV3 intakeColorSensor;
    private final TouchSensor limitSwitchLeft;
    private final TouchSensor limitSwitchRight;
    private final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(100, 0, 0, 3);
    private final Limelight3A limelight;

    public HardwareManager(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RFmotor");
        this.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LFmotor");
        this.rightRearMotor = hardwareMap.get(DcMotorEx.class, "RRmotor");
        this.leftRearMotor = hardwareMap.get(DcMotorEx.class, "LRmotor");
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        this.turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        this.angleServo = hardwareMap.get(Servo.class, "angleServo");
        this.turretCam = hardwareMap.get(WebcamName.class, "TurretCam");
        this.launchServo = hardwareMap.get(Servo.class, "launchServo");
        this.launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        this.launchColorSensor = hardwareMap.get(RevColorSensorV3.class, "launchColorSensor");
        this.launchColorSensor2 = hardwareMap.get(RevColorSensorV3.class, "launchColorSensor2");
        this.intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor");
        this.limitSwitchLeft = hardwareMap.get(TouchSensor.class, "limitSwitchLeft");
        this.limitSwitchRight = hardwareMap.get(TouchSensor.class, "limitSwitchRight");
        this.otos = hardwareMap.get(SparkFunOTOS.class, "sparkFunOTOS");
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.redLED = hardwareMap.get(LED.class, "redLed");
        this.greenLED = hardwareMap.get(LED.class, "greenLed");
        this.indicatorLED = hardwareMap.get(Servo.class, "signalLed");
        initializeHardware(hardwareMap);
    }

    private void initializeHardware(HardwareMap hardwareMap) {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        for(DcMotor motor : List.of(leftFrontMotor, leftRearMotor)) {
            motor.setMode(STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setMode(RUN_USING_ENCODER);
        }
        for(DcMotor motor : List.of(rightFrontMotor, rightRearMotor)) {
            motor.setMode(STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(RUN_USING_ENCODER);
        }
        for(DcMotor motor : List.of(intakeMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(FLOAT);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        for(DcMotor motor : List.of(turretMotor)) {
            motor.setMode(RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(FLOAT);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        for(DcMotor motor : List.of(launcherMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(FLOAT);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        launcherMotor.setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()));
        angleServo.setDirection(Servo.Direction.FORWARD);
        launchColorSensor.setGain(22);
        launchColorSensor2.setGain(22);
        intakeColorSensor.setGain(20);

        redLED.off();
        greenLED.off();

        initializeLimelight();
        initializeIMU();
        initializeOTOS();
    }

    public void postStartInitialization() {
        launchServo.setPosition(LAUNCH_SERVO_DOWN);
    }

    private void initializeLimelight() {
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    private void initializeIMU() {
        ImuOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RIGHT, UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    /**
     * Configures the SparkFun OTOS.
     */
    public void initializeOTOS() {

        SparkFunOTOS.Pose2D offset;

        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.DEGREES);
        offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1);
        otos.setAngularScalar(1);
        otos.calibrateImu();
        otos.resetTracking();
    }

    public IMU getImu() {
        return imu;
    }

    public SparkFunOTOS getOtos() { return otos; }

    public WebcamName getTurretCam() {
        return turretCam;
    }

    public Gamepad getGamepad1() {
        return gamepad1;
    }

    public Gamepad getGamepad2() {
        return gamepad2;
    }

    public Servo getLaunchServo() {
        return launchServo;
    }

    public Servo getAngleServo() {
        return angleServo;
    }

    public DcMotorEx getTurretMotor() {
        return turretMotor;
    }

    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public DcMotorEx getLeftRearMotor() {
        return leftRearMotor;
    }

    public DcMotorEx getRightRearMotor() {
        return rightRearMotor;
    }

    public DcMotorEx getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public DcMotorEx getRightFrontMotor() {
        return rightFrontMotor;
    }

    public DcMotorEx getLauncherMotor() {
        return launcherMotor;
    }

    public RevColorSensorV3 getIntakeColorSensor() {
        return intakeColorSensor;
    }

    public RevColorSensorV3 getLaunchColorSensor() {
        return launchColorSensor;
    }
    public RevColorSensorV3 getLaunchColorSensor2() {
        return launchColorSensor2;
    }

    public TouchSensor getLimitSwitchLeft() {
        return limitSwitchLeft;
    }

    public TouchSensor getLimitSwitchRight() {
        return limitSwitchRight;
    }

    public Limelight3A getLimelight() { return limelight; }

    public LED getRedLed() { return redLED; }

    public LED getGreenLed() { return greenLED; }

    public Servo getIndicatorLed() { return indicatorLED; }
}

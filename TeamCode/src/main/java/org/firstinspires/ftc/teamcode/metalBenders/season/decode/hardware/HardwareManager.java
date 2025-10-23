package org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.UP;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;



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
    private final Gamepad gamepad;
    private final WebcamName turretCam;
    private final IMU imu;
    private final SparkFunOTOS otos;
    private final NormalizedColorSensor launchColorSensor;
    private final NormalizedColorSensor intakeColorSensor;
    private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(100, 0, 0, 3);

    public HardwareManager(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
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
        this.launchColorSensor = hardwareMap.get(NormalizedColorSensor.class, "launchColorSensor");
        this.intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
        this.otos = hardwareMap.get(SparkFunOTOS.class, "sparkFunOTOS");
        this.imu = hardwareMap.get(IMU.class, "imu");
        initializeHardware(hardwareMap);
    }

    private void initializeHardware(HardwareMap hardwareMap) {
        for(DcMotor motor : List.of(leftFrontMotor, leftRearMotor, turretMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        for(DcMotor motor : List.of(rightFrontMotor, rightRearMotor, intakeMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        launcherMotor.setMode(RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor.setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()));
        angleServo.setDirection(Servo.Direction.FORWARD);
        launchColorSensor.setGain(30);
        intakeColorSensor.setGain(15);
        initializeIMU();
        initializeOTOS();
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

    public Gamepad getGamepad() {
        return gamepad;
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

    public NormalizedColorSensor getIntakeColorSensor() {
        return intakeColorSensor;
    }

    public NormalizedColorSensor getLaunchColorSensor() {
        return launchColorSensor;
    }
}

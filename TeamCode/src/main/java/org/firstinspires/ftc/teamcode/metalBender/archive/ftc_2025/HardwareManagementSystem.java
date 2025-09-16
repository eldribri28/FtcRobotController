package org.firstinspires.ftc.teamcode.metalBender.archive.ftc_2025;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.metalBender.archive.ftc_2025.DriveModeConstants.*;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.metalBender.archive.core.AS5600;

import java.util.List;

public class HardwareManagementSystem {
    private final IMU imu;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx rightRearMotor;
    private final DcMotorEx leftRearMotor;
    private final CRServo intakeWheelServo;
    private final Servo wristServo;
    private final DcMotor liftMotor1;
    private final DcMotor liftMotor2;
    private final DcMotorEx extendArmMotor;
    private final DcMotorEx raiseArmMotor;
    private final TouchSensor armLimitSensor;
    private final AS5600 as5600sensor;
    private final Gamepad gamepad;

    public HardwareManagementSystem(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.imu = hardwareMap.get(IMU.class, DriveModeConstants.IMU);
        this.rightFrontMotor = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_MOTOR);
        this.leftFrontMotor = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_MOTOR);
        this.rightRearMotor = hardwareMap.get(DcMotorEx.class, RIGHT_REAR_MOTOR);
        this.leftRearMotor = hardwareMap.get(DcMotorEx.class, LEFT_REAR_MOTOR);
        this.as5600sensor = hardwareMap.get(AS5600.class, AS5600_SENSOR);
        this.liftMotor1 = hardwareMap.get(DcMotor.class, LIFT_MOTOR1);
        this.liftMotor2 = hardwareMap.get(DcMotor.class, LIFT_MOTOR2);
        this.extendArmMotor = hardwareMap.get(DcMotorEx.class, EXTEND_ARM_MOTOR);
        this.raiseArmMotor = hardwareMap.get(DcMotorEx.class, RAISE_ARM_MOTOR);
        this.intakeWheelServo = hardwareMap.get(CRServo.class, GRIPPER_SERVO);
        this.wristServo = hardwareMap.get(Servo.class, WRIST_SERVO);
        this.armLimitSensor = hardwareMap.get(TouchSensor.class, ARM_LIMIT_SENSOR);
        initializeHardware();
        initializeIMU();
    }

    private void initializeHardware() {
        for(DcMotor motor : List.of(extendArmMotor, raiseArmMotor, rightFrontMotor, rightRearMotor, leftFrontMotor, leftRearMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
        }
        extendArmMotor.setMode(STOP_AND_RESET_ENCODER);
        extendArmMotor.setTargetPositionTolerance(40);
    }

    private void initializeIMU() {
        ImuOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(DOWN, BACKWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        try {
            sleep(1000);
        } catch (InterruptedException ex) {
            //do nothing
        }
    }

    public IMU getImu() {
        return imu;
    }

    public DcMotorEx getRightFrontMotor() {
        return rightFrontMotor;
    }

    public DcMotorEx getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public DcMotorEx getRightRearMotor() {
        return rightRearMotor;
    }

    public DcMotorEx getLeftRearMotor() {
        return leftRearMotor;
    }

    public CRServo getIntakeWheelServo() {
        return intakeWheelServo;
    }

    public Servo getWristServo() {
        return wristServo;
    }

    public DcMotor getLiftMotor1() {
        return liftMotor1;
    }

    public DcMotor getLiftMotor2() {
        return liftMotor2;
    }

    public DcMotorEx getExtendArmMotor() {
        return extendArmMotor;
    }

    public DcMotorEx getRaiseArmMotor() {
        return raiseArmMotor;
    }

    public TouchSensor getArmLimitSensor() {
        return armLimitSensor;
    }

    public AS5600 getAs5600sensor() {
        return as5600sensor;
    }

    public Gamepad getGamepad() {
        return gamepad;
    }
}

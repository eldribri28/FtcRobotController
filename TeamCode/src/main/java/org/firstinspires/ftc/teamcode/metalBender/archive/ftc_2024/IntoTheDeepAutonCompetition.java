package org.firstinspires.ftc.teamcode.metalBender.archive.ftc_2024;

import android.graphics.Color;
import android.util.Size;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "IntoTheDeepAutonCompetition (Blocks to Java)", preselectTeleOp = "Field Centric Drive Mode")
public class IntoTheDeepAutonCompetition extends LinearOpMode {

//  private DcMotor LFmotor;
//  private DcMotor LRmotor;
//  private DcMotor RFmotor;
//  private DcMotor RRmotor;
//  private DcMotor ExtendArm;
//  private DcMotor RaiseArm;
//  private Servo Wrist;
//  private CRServo Grip1;
//  private TouchSensor ArmLimit;
//  private ColorSensor IntakeColor;
//  private IMU imu;
//  private SparkFunOTOS OTOS;
//  private Limelight3A Limelight;
//  private DistanceSensor IntakeColor_DistanceSensor;
//
//  VisionPortal.Builder myVisionPortalBuilder;
//  double ArmTargetElevation;
//  double driveDistance;
//  int extensionTarget;
//  long intakeTimestamp;
//  boolean debugMode;
//  YawPitchRollAngles orientation;
//  double botHeading;
//  double headingError;
//  SparkFunOTOS.Pose2D pos;
//  double TagCalculatedHeading;
//  double imuHeading;
//  int limelightRetryCount;
//  String colorSensorResult;
//  String startLocation;
//  double driveAngle;
//  int extensionCurrentPosition;
//  String WristState;
//  double X;
//  double currentX;
//  List XfList;
//  double Xf;
//  int drivePID_accumulatedError;
//  double elevationError;
//  long Last_LimeLightTime;
//  double elevationMaxPower;
//  int extensionError;
//  ImuOrientationOnRobot orientationOnRobot;
//  double Y;
//  int targetX;
//  double currentY;
//  List YfList;
//  double Yf;
//  int rotatePID_accumulatedError;
//  int Left_ViewportID;
//  int extensionTicksPerInch;
//  int elevation90DegOffset;
//  double targetBotHeading;
//  double cosHeading;
//  int targetY;
//  VisionPortal Left_VisionPortal;
//  AprilTagProcessor LeftAprilTagProcessor;
//  List TagHeadingList;
//  int drivePID_slope;
//  int Right_ViewportID;
//  long extensionResetTime;
//  int limelightMaxRetries;
//  double RX;
//  double sinHeading;
//  VisionPortal Right_VisionPortal;
//  AprilTagProcessor RightAprilTagProcessor;
//  List decisionMargin;
//  double botHeading_Offset;
//  int drivePID_D;
//  int elevationPID_accumulatedError;
//  int elevationPID_D;
//  int elevationPID_slope;
//  double block_xOffset;
//  double block_yOffset;
//  double rotX;
//  int rotationAccuracy;
//  SparkFunOTOS.Pose2D currentPosition;
//  long drivePID_lastTime;
//  int drivePID_P;
//  int drivePID_I;
//  int elevationPID_P;
//  int elevationPID_I;
//  double rotY;
//  int rotateVelocityMax;
//  int rotatePID_D;
//  int rotatePID_slope;
//  String CurrentStep;
//  int LFdriveSpeed;
//  Position RightCameraPosition;
//  double Denominator;
//  List<AprilTagDetection> myAprilTagDetections;
//  double drivePID_lastError;
//  int rotatePID_P;
//  int rotatePID_I;
//  int RFdriveSpeed;
//  YawPitchRollAngles RightCameraOrientation;
//  long elevationPID_lastTime;
//  Position LeftCameraPosition;
//  YawPitchRollAngles LeftCameraOrientation;
//  int LRdriveSpeed;
//  double elevationPID_lastError;
//  int RRdriveSpeed;
//  long rotatePID_lastTime;
//  int maxSpeed;
//  double rotatePID_lastError;
//  int rotatePower;
//
//  /**
//   * Describe this function...
//   */
//  private void MotorPIDF() {
//    ((DcMotorEx) LFmotor).setVelocityPIDFCoefficients(20, 0, 0, 10);
//    ((DcMotorEx) LRmotor).setVelocityPIDFCoefficients(20, 0, 0, 10);
//    ((DcMotorEx) RFmotor).setVelocityPIDFCoefficients(20, 0, 0, 10);
//    ((DcMotorEx) RRmotor).setVelocityPIDFCoefficients(20, 0, 0, 10);
//    ((DcMotorEx) ExtendArm).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(12, 0, 0, 10, MotorControlAlgorithm.PIDF));
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Arm_Elevation(double elevationTarget, boolean blocking) {
//    ArmTargetElevation = elevationTarget;
//    if (ArmElevation() < 20) {
//      elevationMaxPower = 0.9;
//    } else {
//      elevationMaxPower = 0.9;
//    }
//    if (blocking) {
//      while (Math.abs(ArmTargetElevation - ArmElevation()) >= 1 && !isStopRequested()) {
//        if (ArmTargetElevation - ArmElevation() < 0) {
//          RaiseArm.setPower(-(elevationMaxPower * elevationPID_Power()));
//        } else if (ArmTargetElevation - ArmElevation() > 0) {
//          if (ArmElevation() < 10) {
//            RaiseArm.setPower(elevationMaxPower * elevationPID_Power() + 0.3);
//          } else {
//            RaiseArm.setPower(elevationMaxPower * elevationPID_Power());
//          }
//        }
//      }
//      RaiseArm.setPower(0);
//    } else {
//      if (ArmTargetElevation - ArmElevation() <= -1) {
//        RaiseArm.setPower(-(elevationMaxPower * 0.6 * elevationPID_Power()));
//      } else if (ArmTargetElevation - ArmElevation() >= 1) {
//        RaiseArm.setPower(elevationMaxPower * 0.6 * elevationPID_Power());
//      } else {
//        RaiseArm.setPower(0);
//      }
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void DriveLimeLight() {
//    boolean LimelightAtTarget;
//
//    LimelightAtTarget = false;
//    while (!LimelightAtTarget && opModeIsActive()) {
//      UpdateLimeLight();
//      targetTranslationLimelight(block_xOffset - 1.5, block_yOffset - 15);
//      Calculate_Motor_Speeds_Limelight();
//      if ((Math.abs(block_xOffset - 1.5) > 1 || Math.abs(block_yOffset - 15) > 1) && !isStopRequested()) {
//        ((DcMotorEx) LFmotor).setVelocity(LFdriveSpeed);
//        ((DcMotorEx) RFmotor).setVelocity(RFdriveSpeed);
//        ((DcMotorEx) LRmotor).setVelocity(LRdriveSpeed);
//        ((DcMotorEx) RRmotor).setVelocity(RRdriveSpeed);
//      } else {
//        ((DcMotorEx) LFmotor).setVelocity(0);
//        ((DcMotorEx) RFmotor).setVelocity(0);
//        ((DcMotorEx) LRmotor).setVelocity(0);
//        ((DcMotorEx) RRmotor).setVelocity(0);
//        LimelightAtTarget = true;
//      }
//      Update_Telemetry();
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void targetTranslationLimelight(double driveX, double driveY) {
//    driveDistance = Math.abs(Math.sqrt(Math.pow(driveX, 2) + Math.pow(driveY, 2)));
//    driveAngle = Math.atan2(driveX, driveY) / Math.PI * 180;
//  }
//
//  /**
//   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
//   * Comment Blocks show where to place Initialization code (runs once, after touching the
//   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
//   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
//   * Stopped).
//   */
  @Override
  public void runOpMode() {
//    LFmotor = hardwareMap.get(DcMotor.class, "LFmotor");
//    LRmotor = hardwareMap.get(DcMotor.class, "LRmotor");
//    RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
//    RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
//    ExtendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
//    RaiseArm = hardwareMap.get(DcMotor.class, "Raise Arm");
//    Wrist = hardwareMap.get(Servo.class, "Wrist");
//    Grip1 = hardwareMap.get(CRServo.class, "Grip1");
//    ArmLimit = hardwareMap.get(TouchSensor.class, "ArmLimit");
//    IntakeColor = hardwareMap.get(ColorSensor.class, "IntakeColor");
//    imu = hardwareMap.get(IMU.class, "imu");
//    OTOS = hardwareMap.get(SparkFunOTOS.class, "OTOS");
//    Limelight = hardwareMap.get(Limelight3A.class, "Limelight");
//    IntakeColor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "IntakeColor");
//
//    resetRuntime();
//    telemetry.setMsTransmissionInterval(11);
//    // Put initialization blocks here.
//    setup();
//    MotorPIDF();
//    initIMU();
//    initAprilTag();
//    initOTOS();
//    camera_scan();
//    waitForStart();
//    if (opModeIsActive()) {
//      // Disable TensorFlow Processor
//      ExtendArm.setPower(0.6);
//      sleep(100);
//      ArmRetractLimit();
//      if (startLocation.equals("A-3")) {
//        A_3();
//      } else if (startLocation.equals("A-4")) {
//        A_4();
//      } else if (startLocation.equals("F-3")) {
//        F_3();
//      } else if (startLocation.equals("F-4")) {
//        F_4();
//      } else {
//        im_lost();
//      }
//    }
//    while (opModeIsActive()) {
//      // Put loop blocks here.
//      telemetry.update();
//    }
  }
//
//  /**
//   * Describe this function...
//   */
//  private void arm_extend(int arm_Extend, double armTol) {
//    extensionTarget = -(arm_Extend * extensionTicksPerInch);
//    extensionCurrentPosition = ExtendArm.getCurrentPosition();
//    extensionError = Math.abs((extensionCurrentPosition - extensionTarget) / extensionTicksPerInch);
//    ExtendArm.setTargetPosition(extensionTarget);
//    ExtendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    ExtendArm.setPower(1);
//    while (extensionError >= armTol && !isStopRequested()) {
//      extensionCurrentPosition = ExtendArm.getCurrentPosition();
//      extensionError = Math.abs((extensionCurrentPosition - extensionTarget) / extensionTicksPerInch);
//      Update_Telemetry();
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Wrist_Down() {
//    Wrist.setPosition(0.05);
//    WristState = "Down";
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Wrist_Up() {
//    Wrist.setPosition(1);
//    WristState = "Up";
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Intake_In() {
//    Grip1.setDirection(CRServo.Direction.REVERSE);
//    Grip1.setPower(1);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void ArmRetractLimit() {
//    boolean flagArmLimit;
//
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    if (ArmLimit.isPressed() && !flagArmLimit && System.currentTimeMillis() - extensionResetTime > 1000) {
//      ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      flagArmLimit = true;
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      extensionResetTime = System.currentTimeMillis();
//    } else {
//      flagArmLimit = false;
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Intake_Out() {
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    intakeTimestamp = System.currentTimeMillis();
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    while (!CheckIntakeSensor().equals("None") && System.currentTimeMillis() - intakeTimestamp < 300 && opModeIsActive()) {
//      Grip1.setDirection(CRServo.Direction.FORWARD);
//      Grip1.setPower(0.6);
//      Update_Telemetry();
//    }
//    Intake_Stop();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Intake_Stop() {
//    Grip1.setPower(0);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Wrist_Middle() {
//    Wrist.setPosition(0.4);
//    WristState = "Middle";
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void setup() {
//    int colorSensorGain;
//    int cameraAngle;
//    int targetHeading;
//    double gearRatio;
//    double ticksPerRev;
//    double wheelCircumference;
//    long ticksPerInch;
//    double drivePower;
//
//    debugMode = true;
//    // Color Sensor Setup
//    colorSensorGain = 5;
//    colorSensorResult = "None";
//    limelightMaxRetries = 2;
//    // If supported by the sensor, turn the light on in the beginning (it
//    // might already be on anyway, we just make sure it is if we can).
//    IntakeColor.enableLed(true);
//    // Setup Camera Position
//    RightCameraPosition = new Position(DistanceUnit.INCH, 6, 0, 4.75, 0);
//    RightCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, -60, -90, 0, 0);
//    LeftCameraPosition = new Position(DistanceUnit.INCH, -6, -1, 4.75, 0);
//    LeftCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 60, -90, 0, 0);
//    cameraAngle = 0;
//    // Setup Arm Elevation PID Variables
//    // With the arm at 90 degrees, how many degrees need to be added or subtracted from the arm elevation to equal 90 degrees
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    extensionResetTime = System.currentTimeMillis();
//    elevation90DegOffset = 41;
//    extensionTicksPerInch = 60;
//    elevationError = 0;
//    ArmTargetElevation = ArmElevation();
//    elevationPID_P = 5;
//    elevationPID_I = 0;
//    elevationPID_D = 0;
//    elevationMaxPower = 0.8;
//    elevationPID_lastError = 0;
//    elevationPID_lastTime = 0;
//    elevationPID_slope = 0;
//    elevationPID_accumulatedError = 0;
//    // Setup Rotate PID Variables
//    headingError = 0;
//    targetHeading = 0;
//    rotatePID_P = 4;
//    rotatePID_I = 0;
//    rotatePID_D = 0;
//    rotatePower = 0;
//    rotateVelocityMax = 1100;
//    rotationAccuracy = 1;
//    rotatePID_lastError = 0;
//    rotatePID_lastTime = 0;
//    rotatePID_slope = 0;
//    rotatePID_accumulatedError = 0;
//    // Imu Settings
//    orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
//    // Location variables
//    currentX = 0;
//    currentY = 0;
//    botHeading_Offset = Math.PI;
//    startLocation = "";
//    // Calculate Gear Ratios and find Ticks per Inch
//    gearRatio = 13.1;
//    ticksPerRev = gearRatio * 28;
//    wheelCircumference = 12.86;
//    ticksPerInch = Math.round(ticksPerRev / wheelCircumference);
//    // Drive system variables
//    // Setup Drive PID Variables
//    driveDistance = 0;
//    drivePID_P = 4;
//    drivePID_I = 0;
//    drivePID_D = 0;
//    drivePID_lastError = 0;
//    drivePID_lastTime = 0;
//    drivePID_slope = 0;
//    drivePID_accumulatedError = 0;
//    drivePower = 0.3;
//    maxSpeed = 1100;
//    LFdriveSpeed = 0;
//    RFdriveSpeed = 0;
//    LRdriveSpeed = 0;
//    RRdriveSpeed = 0;
//    // Setup Motors
//    Grip1.setPwmEnable();
//    Grip1.setPower(0);
//    LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    RaiseArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    LRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    RaiseArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    LRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    LFmotor.setDirection(DcMotor.Direction.FORWARD);
//    LRmotor.setDirection(DcMotor.Direction.FORWARD);
//    RFmotor.setDirection(DcMotor.Direction.FORWARD);
//    RRmotor.setDirection(DcMotor.Direction.FORWARD);
//    ExtendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    RaiseArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    Wrist_Down();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double ArmElevation() {
//    double ArmElevation2;
//    double correctedElevation;
//
//    // AS5600 Encoder Angle In Degrees
//    ArmElevation2 = AS5600Block.AngleInDeg("AS5600sensor");
//    correctedElevation = ArmElevation2 - elevation90DegOffset;
//    return correctedElevation;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void initIMU() {
//    imu.initialize(new IMU.Parameters(orientationOnRobot));
//    imu.resetYaw();
//    orientation = imu.getRobotYawPitchRollAngles();
//    sleep(1000);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double Field_Bot_Heading_Normalized__deg_() {
//    orientation = imu.getRobotYawPitchRollAngles();
//    botHeading = Double.parseDouble(JavaUtil.formatNumber(AngleUnit.DEGREES.normalize(orientation.getYaw(AngleUnit.DEGREES) + botHeading_Offset), 3));
//    return botHeading;
//  }
//
//  /**
//   * Configures the SparkFun OTOS.
//   */
//  private void initOTOS() {
//    SparkFunOTOS.Pose2D offset;
//
//    OTOS.setLinearUnit(DistanceUnit.INCH);
//    OTOS.setAngularUnit(AngleUnit.DEGREES);
//    offset = new SparkFunOTOS.Pose2D(0, 0, 0);
//    OTOS.setOffset(offset);
//    OTOS.setLinearScalar(1);
//    OTOS.setAngularScalar(1);
//    OTOS.calibrateImu();
//    OTOS.resetTracking();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Calculate_Motor_Speeds() {
//    headingError = AngleUnit.DEGREES.normalize(targetBotHeading - Field_Bot_Heading_Normalized__deg_());
//    X = Math.sin(driveAngle / 180 * Math.PI) * drivePID_Power();
//    Y = Math.cos(driveAngle / 180 * Math.PI) * drivePID_Power();
//    if (headingError > 0) {
//      RX = rotatePID_Power() * 0.8;
//    } else {
//      RX = -(rotatePID_Power() * 0.8);
//    }
//    cosHeading = Math.cos(Field_Bot_Heading_Normalized__deg_() / 180 * Math.PI);
//    sinHeading = Math.sin(Field_Bot_Heading_Normalized__deg_() / 180 * Math.PI);
//    rotX = Y * sinHeading - X * cosHeading;
//    rotY = Y * cosHeading + X * sinHeading;
//    Denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(Y), Math.abs(X), Math.abs(RX))), 1));
//    LFdriveSpeed = (int) (maxSpeed * -((rotY + rotX + RX) / Denominator));
//    RFdriveSpeed = (int) (maxSpeed * -(((rotY - rotX) - RX) / Denominator));
//    LRdriveSpeed = (int) (maxSpeed * -(((rotY - rotX) + RX) / Denominator));
//    RRdriveSpeed = (int) (maxSpeed * -(((rotY + rotX) - RX) / Denominator));
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Calculate_Motor_Speeds_Limelight() {
//    headingError = AngleUnit.DEGREES.normalize(targetBotHeading - Field_Bot_Heading_Normalized__deg_());
//    X = Math.sin(driveAngle / 180 * Math.PI) * drivePID_Power();
//    Y = Math.cos(driveAngle / 180 * Math.PI) * drivePID_Power();
//    cosHeading = Math.cos(0 / 180 * Math.PI);
//    sinHeading = Math.sin(0 / 180 * Math.PI);
//    rotX = Y * sinHeading - X * cosHeading;
//    rotY = Y * cosHeading + X * sinHeading;
//    Denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(Y), Math.abs(X))), 1));
//    LFdriveSpeed = (int) (maxSpeed * 1 * ((rotY + rotX) / Denominator));
//    RFdriveSpeed = (int) (maxSpeed * 1 * ((rotY - rotX) / Denominator));
//    LRdriveSpeed = (int) (maxSpeed * 1 * ((rotY - rotX) / Denominator));
//    RRdriveSpeed = (int) (maxSpeed * 1 * ((rotY + rotX) / Denominator));
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void targetTranslation() {
//    double deltaX;
//    double deltaY;
//
//    deltaX = currentX - targetX;
//    deltaY = currentY - targetY;
//    driveDistance = Math.abs(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));
//    driveAngle = Math.atan2(deltaY, deltaX) / Math.PI * 180;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void UpdateLimeLight() {
//    LLStatus status;
//    LLResult result;
//    // TODO: Enter the type for variable named block_X
//    UNKNOWN_TYPE block_X;
//    // TODO: Enter the type for variable named block_Y
//    UNKNOWN_TYPE block_Y;
//    double block_Yaw;
//    double block_Distance;
//    // TODO: Enter the type for variable named block_AspectRatio
//    UNKNOWN_TYPE block_AspectRatio;
//
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    if (Last_LimeLightTime + 50 < System.currentTimeMillis()) {
//      status = Limelight.getStatus();
//      result = Limelight.getLatestResult();
//      telemetry.addData("LimeLight", result.getPythonOutput());
//      block_X = JavaUtil.inListGet(result.getPythonOutput(), JavaUtil.AtMode.FROM_START, (int) 0, false);
//      block_Y = JavaUtil.inListGet(result.getPythonOutput(), JavaUtil.AtMode.FROM_START, (int) 1, false);
//      block_Yaw = ((Double) JavaUtil.inListGet(result.getPythonOutput(), JavaUtil.AtMode.FROM_START, (int) 2, false)).doubleValue();
//      block_Distance = ((Double) JavaUtil.inListGet(result.getPythonOutput(), JavaUtil.AtMode.FROM_START, (int) 3, false)).doubleValue() / 25.4;
//      block_xOffset = ((Double) JavaUtil.inListGet(result.getPythonOutput(), JavaUtil.AtMode.FROM_START, (int) 4, false)).doubleValue() / 25.4;
//      block_yOffset = block_Distance * Math.cos(block_Yaw / 180 * Math.PI);
//      block_AspectRatio = JavaUtil.inListGet(result.getPythonOutput(), JavaUtil.AtMode.FROM_START, (int) 5, false);
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      Last_LimeLightTime = System.currentTimeMillis();
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void camera_scan() {
//    AutonOverride();
//    while (!isStarted() && !isStopRequested()) {
//      runAprilTagScan();
//    }
//    // Enable or disable the AprilTag processor.
//    Left_VisionPortal.setProcessorEnabled(LeftAprilTagProcessor, false);
//    // Enable or disable the AprilTag processor.
//    Right_VisionPortal.setProcessorEnabled(RightAprilTagProcessor, false);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Drive(int TgtX, int TgtY, int TgtHead, int TgtTol) {
//    pos = OTOS.getPosition();
//    currentX = JavaUtil.formatNumber(pos.x, 2);
//    currentY = JavaUtil.formatNumber(pos.y, 2);
//    targetX = TgtX;
//    targetY = TgtY;
//    targetBotHeading = AngleUnit.DEGREES.normalize(TgtHead);
//    targetTranslation();
//    while (Math.abs(driveDistance) > TgtTol && opModeIsActive()) {
//      pos = OTOS.getPosition();
//      currentX = JavaUtil.formatNumber(pos.x, 2);
//      currentY = JavaUtil.formatNumber(pos.y, 2);
//      targetTranslation();
//      if (Math.abs(driveDistance) <= TgtTol) {
//        break;
//      }
//      Calculate_Motor_Speeds();
//      ((DcMotorEx) LFmotor).setVelocity(LFdriveSpeed);
//      ((DcMotorEx) LRmotor).setVelocity(LRdriveSpeed);
//      ((DcMotorEx) RFmotor).setVelocity(RFdriveSpeed);
//      ((DcMotorEx) RRmotor).setVelocity(RRdriveSpeed);
//      Update_Telemetry();
//    }
//    ((DcMotorEx) LFmotor).setVelocity(0);
//    ((DcMotorEx) LRmotor).setVelocity(0);
//    ((DcMotorEx) RFmotor).setVelocity(0);
//    ((DcMotorEx) RRmotor).setVelocity(0);
//    rotateRobot();
//    Update_Telemetry();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void rotateRobot() {
//    double steeringCorrection;
//    double rotateVelocity;
//
//    steeringCorrection = rotatePID_Power();
//    // Keep looping while we are still active and not on heading.
//    while (opModeIsActive() && Math.abs(headingError) >= rotationAccuracy) {
//      if (headingError > 0) {
//        rotateVelocity = rotatePID_Power() * rotateVelocityMax;
//      } else {
//        rotateVelocity = -(rotatePID_Power() * rotateVelocityMax);
//      }
//      // Clip the speed to the maximum permitted value.
//      ((DcMotorEx) LFmotor).setVelocity(-rotateVelocity);
//      ((DcMotorEx) LRmotor).setVelocity(-rotateVelocity);
//      ((DcMotorEx) RFmotor).setVelocity(rotateVelocity);
//      ((DcMotorEx) RRmotor).setVelocity(rotateVelocity);
//      // Display drive status for the driver.
//      Update_Telemetry();
//    }
//    // Stop all motion;
//    ((DcMotorEx) LFmotor).setVelocity(0);
//    ((DcMotorEx) LRmotor).setVelocity(0);
//    ((DcMotorEx) RFmotor).setVelocity(0);
//    ((DcMotorEx) RRmotor).setVelocity(0);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void runAprilTagScan() {
//    AprilTagDetection recognition;
//
//    // (Note we only process first visible target).
//    XfList = JavaUtil.createListWith();
//    YfList = JavaUtil.createListWith();
//    TagHeadingList = JavaUtil.createListWith();
//    decisionMargin = JavaUtil.createListWith();
//    AutonOverride();
//    if (!isStarted() && !isStopRequested()) {
//      // Get a list of AprilTag detections.
//      myAprilTagDetections = LeftAprilTagProcessor.getDetections();
//      // Iterate through list and call a function to display info for each recognized AprilTag.
//      for (AprilTagDetection recognition_item : myAprilTagDetections) {
//        recognition = recognition_item;
//        if (recognition && !isStarted() && !isStopRequested()) {
//          if (recognition.decisionMargin > 40) {
//            decisionMargin.add(recognition.decisionMargin);
//            XfList.add(recognition.robotPose.getPosition().x);
//            YfList.add(recognition.robotPose.getPosition().y);
//            TagHeadingList.add(AngleUnit.DEGREES.normalize(recognition.robotPose.getOrientation().getYaw() + 90));
//          }
//        } else {
//          break;
//        }
//      }
//      if (JavaUtil.listLength(XfList) > 0) {
//        TagCalculatedHeading = JavaUtil.averageOfList(TagHeadingList);
//        Xf = JavaUtil.averageOfList(XfList);
//        Yf = JavaUtil.averageOfList(YfList);
//        processTarget();
//      }
//      Update_Telemetry();
//    }
//    AutonOverride();
//    if (!isStarted() && !isStopRequested()) {
//      // Get a list of AprilTag detections.
//      myAprilTagDetections = RightAprilTagProcessor.getDetections();
//      // Iterate through list and call a function to display info for each recognized AprilTag.
//      for (AprilTagDetection recognition_item2 : myAprilTagDetections) {
//        recognition = recognition_item2;
//        if (recognition && !isStarted() && !isStopRequested()) {
//          if (recognition.decisionMargin > 40) {
//            decisionMargin.add(recognition.decisionMargin);
//            XfList.add(recognition.robotPose.getPosition().x);
//            YfList.add(recognition.robotPose.getPosition().y);
//            TagHeadingList.add(AngleUnit.DEGREES.normalize(recognition.robotPose.getOrientation().getYaw() + 90));
//          }
//        } else {
//          break;
//        }
//      }
//      if (JavaUtil.listLength(XfList) > 0) {
//        Average_Location();
//        processTarget();
//      }
//      Update_Telemetry();
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Highest_Margin() {
//    int highestMargin;
//    double i;
//
//    highestMargin = 0;
//    double i_end = JavaUtil.listLength(XfList);
//    double i_inc = 1;
//    if (1 > i_end) {
//      i_inc = -i_inc;
//    }
//    for (i = 1; i_inc >= 0 ? i <= i_end : i >= i_end; i += i_inc) {
//      if (JavaUtil.inListGet(decisionMargin, JavaUtil.AtMode.FROM_START, (int) (i - 1), false) > highestMargin) {
//        highestMargin = ((Integer) JavaUtil.inListGet(decisionMargin, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).intValue();
//        TagCalculatedHeading = AngleUnit.DEGREES.normalize(((Double) JavaUtil.inListGet(TagHeadingList, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue());
//        Xf = ((Double) JavaUtil.inListGet(XfList, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue();
//        Yf = ((Double) JavaUtil.inListGet(YfList, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue();
//      }
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Average_Location() {
//    TagCalculatedHeading = AngleUnit.DEGREES.normalize(JavaUtil.averageOfList(TagHeadingList));
//    Xf = JavaUtil.averageOfList(XfList);
//    Yf = JavaUtil.averageOfList(YfList);
//  }
//
//  /**
//   * This function displays location on the field and rotation about the Z
//   * axis on the field. It uses results from the isTargetVisible function.
//   */
//  private void processTarget() {
//    imuHeading = orientation.getYaw(AngleUnit.DEGREES);
//    currentX = Double.parseDouble(JavaUtil.formatNumber(Xf, 2));
//    currentY = Double.parseDouble(JavaUtil.formatNumber(Yf, 2));
//    botHeading = Double.parseDouble(JavaUtil.formatNumber(TagCalculatedHeading, 3));
//    botHeading_Offset = botHeading - imuHeading;
//    currentPosition = new SparkFunOTOS.Pose2D(currentX, currentY, TagCalculatedHeading);
//    OTOS.setPosition(currentPosition);
//    if (!isStarted() && !isStopRequested()) {
//      setStartPosition();
//    }
//    Update_Telemetry();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private YawPitchRollAngles Convert_180_to_360(double inputAngle) {
//    double outputAngle;
//
//    if (inputAngle < 0) {
//      outputAngle = Double.parseDouble(JavaUtil.formatNumber(inputAngle + 360, 3));
//    } else {
//      outputAngle = inputAngle;
//    }
//    return outputAngle;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double drivePID_Power() {
//    // Determine the heading current error.
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    drivePID_accumulatedError = (int) (drivePID_accumulatedError + (System.currentTimeMillis() - drivePID_lastTime) * driveDistance);
//    if (driveDistance < 1) {
//      drivePID_accumulatedError = 0;
//    }
//    drivePID_slope = 0;
//    if (drivePID_lastTime > 0) {
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      drivePID_slope = (int) ((driveDistance - drivePID_lastError) / (System.currentTimeMillis() - drivePID_lastTime));
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      drivePID_lastTime = System.currentTimeMillis();
//      drivePID_lastError = driveDistance;
//    }
//    return Range.scale(Math.abs(driveDistance * drivePID_P + drivePID_accumulatedError * drivePID_I + drivePID_D * drivePID_slope), 0, 60, 0.1, 1);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double elevationPID_Power() {
//    // Determine the current error.
//    elevationError = Math.abs(ArmTargetElevation - ArmElevation());
//    rotatePID_accumulatedError = (int) (rotatePID_accumulatedError + headingError);
//    if (Math.abs(elevationError) < 0.5) {
//      elevationPID_accumulatedError = 0;
//    }
//    // Ensure sign of rotatePID_acuumulatedError matches headingError
//    elevationPID_slope = 0;
//    if (elevationPID_lastTime > 0) {
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      elevationPID_slope = (int) ((elevationError - elevationPID_lastError) / (System.currentTimeMillis() - elevationPID_lastTime));
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      elevationPID_lastTime = System.currentTimeMillis();
//      elevationPID_lastError = elevationError;
//    }
//    return Range.scale(Math.abs(elevationError * elevationPID_P + elevationPID_accumulatedError * elevationPID_I + elevationPID_D * elevationPID_slope), 0, 180, 0.4, 1);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private double rotatePID_Power() {
//    // Determine the heading current error.
//    headingError = targetBotHeading - Field_Bot_Heading_Normalized__deg_();
//    if (headingError > 180) {
//      headingError = headingError - 360;
//    } else if (headingError <= -180) {
//      headingError = headingError + 360;
//    }
//    rotatePID_accumulatedError = (int) (rotatePID_accumulatedError + headingError);
//    if (Math.abs(headingError) < 1) {
//      rotatePID_accumulatedError = 0;
//    }
//    // Ensure sign of rotatePID_acuumulatedError matches headingError
//    if (headingError < 0) {
//      rotatePID_accumulatedError = Math.abs(rotatePID_accumulatedError) * -1;
//    } else {
//      rotatePID_accumulatedError = Math.abs(rotatePID_accumulatedError) * 1;
//    }
//    rotatePID_slope = 0;
//    if (rotatePID_lastTime > 0) {
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      rotatePID_slope = (int) ((headingError - rotatePID_lastError) / (System.currentTimeMillis() - rotatePID_lastTime));
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      rotatePID_lastTime = System.currentTimeMillis();
//      rotatePID_lastError = headingError;
//    }
//    return Range.clip(Math.abs(Range.clip(headingError, 0, 0.2)) + Range.scale(Math.abs(headingError * rotatePID_P + rotatePID_accumulatedError * rotatePID_I + rotatePID_D * rotatePID_slope), 0, 180, 0.1, 0.8), 0.2, 1);
//  }
//
//  /**
//   * By default, distances are returned in millimeters by Vuforia.
//   * Convert to other distance units (CM, M, IN, and FT).
//   */
//  private double displayValue(double originalValue,
//      // TODO: Enter the type for argument named units
//      UNKNOWN_TYPE units) {
//    double convertedValue;
//
//    // Vuforia returns distances in mm.
//    if (units == "CM") {
//      convertedValue = originalValue / 10;
//    } else if (units == "M") {
//      convertedValue = originalValue / 1000;
//    } else if (units == "IN") {
//      convertedValue = originalValue / 25.4;
//    } else if (units == "FT") {
//      convertedValue = (originalValue / 25.4) / 12;
//    } else {
//      convertedValue = originalValue;
//    }
//    return convertedValue;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Update_Telemetry() {
//    double cameraX;
//    double cameraY;
//
//    if (opModeIsActive()) {
//      Arm_Elevation(ArmTargetElevation, false);
//    }
//    pos = OTOS.getPosition();
//    telemetry.addData("onModeIsActive", opModeIsActive());
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    telemetry.addData("Current Time Vs Ijtake Time", System.currentTimeMillis() - intakeTimestamp);
//    telemetry.addData("Intake Sensor", CheckIntakeSensor());
//    telemetry.addData("Extension Power", ExtendArm.getPower());
//    telemetry.addData("Elevation Power", RaiseArm.getPower());
//    telemetry.addData("Extension Target", extensionTarget);
//    telemetry.addData("Extension Current", extensionCurrentPosition);
//    telemetry.addData("Extension Error (in)", extensionError);
//    telemetry.addData("Extension Ticks", ExtendArm.getCurrentPosition());
//    telemetry.addData("Arm Angle", ArmElevation());
//    telemetry.addData("Step", CurrentStep);
//    telemetry.addData("Start Location", startLocation);
//    telemetry.addData("Robot X, Y (in) : Heading", currentX + ", " + currentY + " : " + Double.parseDouble(JavaUtil.formatNumber(Convert_180_to_360(Field_Bot_Heading_Normalized__deg_()), 2)));
//    telemetry.addData("AprilTag X, Y (in) : Heading", JavaUtil.formatNumber(Xf, 2) + ", " + JavaUtil.formatNumber(Yf, 2) + " : " + JavaUtil.formatNumber(Convert_180_to_360(TagCalculatedHeading), 2));
//    telemetry.addData("Odometry X, Y (in) : Heading", JavaUtil.formatNumber(pos.x, 2) + ", " + JavaUtil.formatNumber(pos.y, 2) + " : " + JavaUtil.formatNumber(Convert_180_to_360(pos.h), 2));
//    telemetry.addData("Target X, Y (in) : Heading", JavaUtil.formatNumber(targetX, 2) + ", " + JavaUtil.formatNumber(targetY, 2) + " : " + JavaUtil.formatNumber(Convert_180_to_360(targetBotHeading), 2));
//    telemetry.addData("RX:", RX);
//    telemetry.addData("Decision Margin", decisionMargin);
//    telemetry.addData("All Y", YfList);
//    if (debugMode == true) {
//      telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
//      telemetry.addData("Distance (in)", Double.parseDouble(JavaUtil.formatNumber(driveDistance, 2)));
//      telemetry.addData("Drive Angle", Double.parseDouble(JavaUtil.formatNumber(Convert_180_to_360(driveAngle), 2)));
//      telemetry.addData("Heading Error", Double.parseDouble(JavaUtil.formatNumber(headingError, 2)));
//      telemetry.addData("Camera X, Y (in)", Double.parseDouble(JavaUtil.formatNumber(cameraX, 2)) + ", " + Double.parseDouble(JavaUtil.formatNumber(cameraY, 2)));
//      telemetry.addData("IMU Heading", Double.parseDouble(JavaUtil.formatNumber(Convert_180_to_360(orientation).getYaw(AngleUnit.DEGREES), 2)));
//      telemetry.addData("Heading Offset", Double.parseDouble(JavaUtil.formatNumber(botHeading_Offset, 2)));
//      telemetry.addData("Rotate Power", Double.parseDouble(JavaUtil.formatNumber(rotatePower, 3)));
//      if (!startLocation.equals("")) {
//        telemetry.addData("AprilTags Heading", Double.parseDouble(JavaUtil.formatNumber(Convert_180_to_360(TagCalculatedHeading), 2)));
//      }
//      telemetry.addData("LF Drive Speed:", LFdriveSpeed + " - V:" + ((DcMotorEx) LFmotor).getVelocity());
//      telemetry.addData("RF Drive Speed:", RFdriveSpeed + " - V:" + ((DcMotorEx) RFmotor).getVelocity());
//      telemetry.addData("LR Drive Speed:", LRdriveSpeed + " - V:" + ((DcMotorEx) LRmotor).getVelocity());
//      telemetry.addData("RR Drive Speed:", RRdriveSpeed + " - V:" + ((DcMotorEx) RRmotor).getVelocity());
//    }
//    telemetry.update();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void setStartPosition() {
//    if (currentX < 0 && currentY > 0) {
//      startLocation = "A-3";
//    } else if (currentX > 0 && currentY > 0) {
//      startLocation = "A-4";
//    } else if (currentX < 0 && currentY < 0) {
//      startLocation = "F-3";
//    } else if (currentX > 0 && currentY < 0) {
//      startLocation = "F-4";
//    } else {
//      startLocation = "";
//    }
//  }
//
//  /**
//   * Initialize AprilTag Detection.
//   */
//  private void initAprilTag() {
//    AprilTagProcessor.Builder LeftAprilTagProcessorBuilder;
//    AprilTagProcessor.Builder RightAprilTagProcessorBuilder;
//    List myPortalList;
//
//    // Next, create a VisionPortal.Builder and set attributes related to the camera.
//    myPortalList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL));
//    Left_ViewportID = ((Integer) JavaUtil.inListGet(myPortalList, JavaUtil.AtMode.FROM_START, (int) 0, false)).intValue();
//    Right_ViewportID = ((Integer) JavaUtil.inListGet(myPortalList, JavaUtil.AtMode.FROM_START, (int) 1, false)).intValue();
//    // First, create an AprilTagProcessor.Builder.
//    LeftAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//    RightAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//    LeftAprilTagProcessorBuilder.setCameraPose(LeftCameraPosition, LeftCameraOrientation);
//    RightAprilTagProcessorBuilder.setCameraPose(RightCameraPosition, RightCameraOrientation);
//    // Set whether or not to draw the axes on detections.
//    LeftAprilTagProcessorBuilder.setDrawAxes(true);
//    // Set whether or not to draw the axes on detections.
//    RightAprilTagProcessorBuilder.setDrawAxes(true);
//    // Set the lens intrinsics, obtained from a camera calibration.
//    LeftAprilTagProcessorBuilder.setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.364794005);
//    // Set the lens intrinsics, obtained from a camera calibration.
//    RightAprilTagProcessorBuilder.setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.364794005);
//    // Create an AprilTagProcessor by calling build.
//    LeftAprilTagProcessor = LeftAprilTagProcessorBuilder.build();
//    RightAprilTagProcessor = RightAprilTagProcessorBuilder.build();
//    Left_Viewport();
//    Right_Viewport();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Left_Viewport() {
//    // Create a VisionPortal.Builder object so you can specify attributes about the cameras.
//    myVisionPortalBuilder = new VisionPortal.Builder();
//    // Set the camera to the specified webcam name.
//    myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "LeftCam"));
//    // Set the stream format.
//    myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
//    // Set the camera resolution.
//    myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
//    myVisionPortalBuilder.addProcessor(LeftAprilTagProcessor);
//    // Set the LiveView container id.
//    myVisionPortalBuilder.setLiveViewContainerId(Left_ViewportID);
//    Left_VisionPortal = myVisionPortalBuilder.build();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void Right_Viewport() {
//    // Create a VisionPortal.Builder object so you can specify attributes about the cameras.
//    myVisionPortalBuilder = new VisionPortal.Builder();
//    // Set the camera to the specified webcam name.
//    myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "RightCam"));
//    // Set the stream format.
//    myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
//    // Set the camera resolution.
//    myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
//    myVisionPortalBuilder.addProcessor(RightAprilTagProcessor);
//    // Set the LiveView container id.
//    myVisionPortalBuilder.setLiveViewContainerId(Right_ViewportID);
//    Right_VisionPortal = myVisionPortalBuilder.build();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void F_3() {
//    Limelight.pipelineSwitch(2);
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    Last_LimeLightTime = System.currentTimeMillis();
//    Limelight.start();
//    // Step 1: drive forward and place sample
//    Arm_Elevation(7, false);
//    Drive(-54, -54, 45, 1);
//    CurrentStep = "Extend Arm";
//    Arm_Elevation(7, true);
//    Wrist_Up();
//    arm_extend(26, 3);
//    sleep(100);
//    Intake_Out();
//    sleep(100);
//    // Step 2: drive to get sample
//    Wrist_Down();
//    sleep(200);
//    arm_extend(5, 3);
//    Arm_Elevation(90, false);
//    Drive(-49, -50, 90, 1);
//    Arm_Elevation(90, true);
//    limelightGetSample();
//    // Step 3: Pickup Sample
//    Drive(-55, -53, 45, 1);
//    Arm_Elevation(7, true);
//    Wrist_Up();
//    arm_extend(28, 3);
//    sleep(100);
//    Intake_Out();
//    sleep(100);
//    Wrist_Down();
//    sleep(200);
//    arm_extend(5, 3);
//    Arm_Elevation(90, false);
//    Drive(-54, -48, 90, 1);
//    Arm_Elevation(90, true);
//    limelightGetSample();
//    Drive(-55, -53, 45, 1);
//    Arm_Elevation(7, true);
//    Wrist_Up();
//    arm_extend(28, 3);
//    sleep(100);
//    Intake_Out();
//    sleep(100);
//    Wrist_Down();
//    sleep(200);
//    arm_extend(5, 3);
//    Arm_Elevation(90, false);
//    Drive(-53, -53, 90, 1);
//    Arm_Elevation(90, true);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void AutonOverride() {
//    if (gamepad1.a) {
//      imuHeading = orientation.getYaw(AngleUnit.DEGREES);
//      currentX = Double.parseDouble(JavaUtil.formatNumber(-12, 2));
//      currentY = Double.parseDouble(JavaUtil.formatNumber(-57, 2));
//      botHeading = Double.parseDouble(JavaUtil.formatNumber(90, 3));
//      botHeading_Offset = botHeading - imuHeading;
//      currentPosition = new SparkFunOTOS.Pose2D(currentX, currentY, botHeading);
//      OTOS.setPosition(currentPosition);
//      if (!isStarted() && !isStopRequested()) {
//        setStartPosition();
//      }
//      Update_Telemetry();
//    } else if (gamepad1.b) {
//      imuHeading = orientation.getYaw(AngleUnit.DEGREES);
//      currentX = Double.parseDouble(JavaUtil.formatNumber(12, 2));
//      currentY = Double.parseDouble(JavaUtil.formatNumber(-57, 2));
//      botHeading = Double.parseDouble(JavaUtil.formatNumber(90, 3));
//      botHeading_Offset = botHeading - imuHeading;
//      currentPosition = new SparkFunOTOS.Pose2D(currentX, currentY, botHeading);
//      OTOS.setPosition(currentPosition);
//      if (!isStarted() && !isStopRequested()) {
//        setStartPosition();
//      }
//      Update_Telemetry();
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void A_4() {
//    Limelight.pipelineSwitch(2);
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    Last_LimeLightTime = System.currentTimeMillis();
//    Limelight.start();
//    // Step 1: drive forward and place sample
//    Arm_Elevation(7, false);
//    Drive(55, 54, 225, 1);
//    CurrentStep = "Extend Arm";
//    Arm_Elevation(7, true);
//    Wrist_Up();
//    arm_extend(29, 3);
//    sleep(100);
//    Intake_Out();
//    sleep(100);
//    // Step 2: drive to get sample
//    Wrist_Down();
//    sleep(200);
//    arm_extend(5, 3);
//    Arm_Elevation(90, false);
//    Drive(49, 50, 270, 1);
//    Arm_Elevation(90, true);
//    limelightGetSample();
//    // Step 3: Pickup Sample
//    Drive(55, 53, 225, 1);
//    Arm_Elevation(7, true);
//    Wrist_Up();
//    arm_extend(28, 3);
//    sleep(100);
//    Intake_Out();
//    sleep(100);
//    Wrist_Down();
//    sleep(200);
//    arm_extend(5, 3);
//    Arm_Elevation(90, false);
//    Drive(54, 48, 270, 1);
//    Arm_Elevation(90, true);
//    limelightGetSample();
//    Drive(55, 53, 225, 1);
//    Arm_Elevation(7, true);
//    Wrist_Up();
//    arm_extend(28, 3);
//    sleep(100);
//    Intake_Out();
//    sleep(100);
//    Wrist_Down();
//    sleep(200);
//    arm_extend(5, 3);
//    Arm_Elevation(90, false);
//    Drive(53, 53, 270, 1);
//    Arm_Elevation(90, true);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void A_3() {
//    Limelight.pipelineSwitch(4);
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    Last_LimeLightTime = System.currentTimeMillis();
//    Limelight.start();
//    Wrist_Up();
//    Arm_Elevation(7, false);
//    Drive(-10, 36, 90, 1);
//    arm_extend(7, 0.5);
//    Arm_Elevation(7, true);
//    sleep(500);
//    Drive(-10, 33, 90, 2);
//    sleep(500);
//    arm_extend(14, 0.5);
//    sleep(100);
//    Intake_Out();
//    Drive(-10, 45, 90, 1);
//    arm_extend(3, 0.5);
//    Arm_Elevation(93, false);
//    Wrist_Down();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void F_4() {
//    Limelight.pipelineSwitch(3);
//    // Get the current time in milliseconds. The value returned represents
//    // the number of milliseconds since midnight, January 1, 1970 UTC.
//    Last_LimeLightTime = System.currentTimeMillis();
//    Limelight.start();
//    Wrist_Up();
//    Arm_Elevation(7, false);
//    Drive(10, -36, 270, 1);
//    arm_extend(7, 0.5);
//    Arm_Elevation(7, true);
//    sleep(500);
//    Drive(10, -33, 270, 2);
//    sleep(500);
//    arm_extend(14, 0.5);
//    sleep(100);
//    Intake_Out();
//    Drive(12, -45, 270, 1);
//    arm_extend(3, 0.5);
//    Arm_Elevation(93, false);
//    Wrist_Down();
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void limelightGetSample() {
//    limelightRetryCount = 0;
//    while (CheckIntakeSensor().equals("None") && limelightRetryCount < limelightMaxRetries && opModeIsActive()) {
//      DriveLimeLight();
//      arm_extend(12, 0.25);
//      Intake_In();
//      Arm_Elevation(110, true);
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      intakeTimestamp = System.currentTimeMillis();
//      // Get the current time in milliseconds. The value returned represents
//      // the number of milliseconds since midnight, January 1, 1970 UTC.
//      while (CheckIntakeSensor().equals("None") && System.currentTimeMillis() - intakeTimestamp < 300 && opModeIsActive()) {
//      }
//      Intake_Stop();
//      arm_extend(3, 3);
//      Arm_Elevation(90, false);
//      sleep(100);
//      limelightRetryCount += 1;
//    }
//    Arm_Elevation(6.5, false);
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void limelightGetSpecimen() {
//    limelightRetryCount = 0;
//    while (opModeIsActive() && CheckIntakeSensor().equals("None") && limelightRetryCount < limelightMaxRetries) {
//      DriveLimeLight();
//    }
//  }
//
//  /**
//   * Describe this function...
//   */
//  private String CheckIntakeSensor() {
//    NormalizedRGBA intakeSensorNormalizedColor;
//    int intakeSensorColor;
//    float intakeSensorHue;
//    double intakeSensorDistance;
//
//    intakeSensorNormalizedColor = ((NormalizedColorSensor) IntakeColor).getNormalizedColors();
//    intakeSensorColor = intakeSensorNormalizedColor.toColor();
//    intakeSensorHue = JavaUtil.rgbToHue(Color.red(intakeSensorColor), Color.green(intakeSensorColor), Color.blue(intakeSensorColor));
//    intakeSensorDistance = Double.parseDouble(JavaUtil.formatNumber(IntakeColor_DistanceSensor.getDistance(DistanceUnit.INCH), 3));
//    if (intakeSensorHue < 50 && intakeSensorDistance < 3) {
//      colorSensorResult = "Red";
//    } else if (intakeSensorHue < 150 && intakeSensorDistance < 3) {
//      colorSensorResult = "Yellow";
//    } else if (intakeSensorHue < 225 && intakeSensorDistance < 3) {
//      colorSensorResult = "Blue";
//    } else {
//      colorSensorResult = "None";
//    }
//    return colorSensorResult;
//  }
//
//  /**
//   * Describe this function...
//   */
//  private void im_lost() {
//    startLocation = "I'm Lost";
//    currentX = 0;
//    currentY = 0;
//    botHeading = Double.parseDouble(JavaUtil.formatNumber(AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 180), 3));
//    imuHeading = orientation.getYaw(AngleUnit.RADIANS);
//    botHeading_Offset = AngleUnit.RADIANS.normalize(imuHeading - botHeading);
//    currentPosition = new SparkFunOTOS.Pose2D(currentX, currentY, 0);
//    OTOS.setPosition(currentPosition);
//    while (opModeIsActive()) {
//      Update_Telemetry();
//    }
//  }
}

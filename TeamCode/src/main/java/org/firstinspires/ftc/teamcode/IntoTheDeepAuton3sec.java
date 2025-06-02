package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

@Autonomous(name = "IntoTheDeepAuton3sec (Blocks to Java)")
public class IntoTheDeepAuton3sec extends LinearOpMode {

  private HuskyLens HuskyLens;
  private DcMotor RaiseArm;
  private DcMotor ExtendArm;
  private DcMotor LFmotor;
  private DcMotor LRmotor;
  private DcMotor RFmotor;
  private DcMotor RRmotor;
  private DcMotor LiftMotor;
  private IMU imu_IMU;
  private SparkFunOTOS OTOS;
  private DcMotor ExtendBucket;
  private VoltageSensor ControlHub_VoltageSensor;
  private TouchSensor ArmLimit;
  private Servo Gripper;
  private Servo Ramp;
  private TouchSensor BucketLimit;

  VisionPortal.Builder myVisionPortalBuilder;
  boolean debugMode;
  YawPitchRollAngles orientation;
  double botHeading;
  double headingError;
  int maxSpeed;
  double targetX;
  double TagCalculatedHeading;
  double imuHeading;
  SparkFunOTOS.Pose2D pos;
  double ArmElevation;
  List<HuskyLens.Block> myHuskyLensBlocks;
  String startLocation;
  String currentMatch;
  int targetY;
  List XfList;
  double Xf;
  double currentX;
  int drivePID_accumulatedError;
  double rotatePowerMax;
  ImuOrientationOnRobot orientationOnRobot;
  double driveDistance;
  VisionPortal Left_VisionPortal;
  AprilTagProcessor LeftAprilTagProcessor;
  double targetBotHeading;
  List YfList;
  double Yf;
  double currentY;
  String GripperState;
  int Left_ViewportID;
  int Arm90DegOffset;
  double driveAngle;
  VisionPortal Right_VisionPortal;
  AprilTagProcessor RightAprilTagProcessor;
  double rotatePower;
  List TagHeadingList;
  int drivePID_slope;
  int rotatePID_accumulatedError;
  int Right_ViewportID;
  double RX;
  List decisionMargin;
  double botHeading_Offset;
  int drivePID_D;
  int rotationAccuracy;
  List<AprilTagDetection> myAprilTagDetections;
  SparkFunOTOS.Pose2D currentPosition;
  double drivePID_P;
  int drivePID_I;
  Position RightCameraPosition;
  long drivePID_lastTime;
  double rotatePID_D;
  int rotatePID_slope;
  YawPitchRollAngles RightCameraOrientation;
  double turnGain;
  double drivePID_lastError;
  double rotatePID_P;
  int rotatePID_I;
  Position LeftCameraPosition;
  YawPitchRollAngles LeftCameraOrientation;
  double LFdriveSpeed;
  long rotatePID_lastTime;
  double RFdriveSpeed;
  double rotatePID_lastError;
  double LRdriveSpeed;
  double RRdriveSpeed;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    HuskyLens = hardwareMap.get(HuskyLens.class, "HuskyLensAsHuskyLens");
    RaiseArm = hardwareMap.get(DcMotor.class, "Raise Arm");
    ExtendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
    LFmotor = hardwareMap.get(DcMotor.class, "LFmotor");
    LRmotor = hardwareMap.get(DcMotor.class, "LRmotor");
    RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
    RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
    LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
    imu_IMU = hardwareMap.get(IMU.class, "imu");
    OTOS = hardwareMap.get(SparkFunOTOS.class, "OTOS");
    ExtendBucket = hardwareMap.get(DcMotor.class, "ExtendBucket");
    ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    ArmLimit = hardwareMap.get(TouchSensor.class, "ArmLimit");
    Gripper = hardwareMap.get(Servo.class, "GripperAsServo");
    Ramp = hardwareMap.get(Servo.class, "Ramp");
    BucketLimit = hardwareMap.get(TouchSensor.class, "BucketLimit");

    // Put initialization blocks here.
    setup();
    SetDrivePIDF();
    initIMU();
    initAprilTag();
    initOTOS();
    camera_scan();
    while (!isStarted()) {
      Update_Telemetry();
    }
    waitForStart();
    if (opModeIsActive()) {
      resetRuntime();
      sleep(3000);
      // Disable TensorFlow Processor
      if (startLocation.equals("A-3")) {
        A_3();
      } else if (startLocation.equals("A-4")) {
        A_4();
      } else if (startLocation.equals("F-3")) {
        F_3();
      } else if (startLocation.equals("F-4")) {
        F_4();
      } else {
        im_lost();
      }
      while (opModeIsActive()) {
        Update_Telemetry();
      }
    }
    while (opModeIsActive()) {
      // Put loop blocks here.
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void setup() {
    double driveGain;
    double ArmTicksPerInch;
    int cameraAngle;
    int targetHeading;
    int targetTagX;
    int targetTagY;
    int targetTagYaw;
    double gearRatio;
    double ticksPerRev;
    double wheelCircumference;
    long ticksPerInch;
    double drivePower;

    debugMode = true;
    // SystemTime as a Date Time
    currentMatch = GetTimeStamp.newTimestamp(null);
    // With the arm at 90 degrees, how many degrees need to be added or subtracted from the arm elevation to equal 90 degrees
    Arm90DegOffset = 74;
    ArmTicksPerInch = 30.55;
    // Setup Camera Position
    RightCameraPosition = new Position(DistanceUnit.INCH, 6.25, 2.5, 5, 0);
    RightCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, -58, -90, 0, 0);
    LeftCameraPosition = new Position(DistanceUnit.INCH, -6.25, -2.5, 5, 0);
    LeftCameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 60, -90, 0, 0);
    cameraAngle = 0;
    HuskyLens.selectAlgorithm();
    // Setup Rotate PID Variables
    headingError = 0;
    targetHeading = 0;
    rotatePID_P = 0.005;
    rotatePID_I = 0;
    rotatePID_D = 0.01;
    rotatePower = 0;
    rotatePowerMax = 0.55;
    rotationAccuracy = 1;
    rotatePID_lastError = 0;
    rotatePID_lastTime = 0;
    rotatePID_slope = 0;
    rotatePID_accumulatedError = 0;
    // Imu Settings
    orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    // Target Tag Location
    targetTagX = 0;
    targetTagY = 0;
    targetTagYaw = 0;
    // Location variables
    currentX = 0;
    currentY = 0;
    botHeading_Offset = Math.PI;
    startLocation = "";
    // Calculate Gear Ratios and find Ticks per Inch
    gearRatio = 13.1;
    ticksPerRev = gearRatio * 28;
    wheelCircumference = 12.86;
    ticksPerInch = Math.round(ticksPerRev / wheelCircumference);
    // Drive system variables
    // Setup Drive PID Variables
    driveDistance = 0;
    drivePID_P = 0.2;
    drivePID_I = 0;
    drivePID_D = 18;
    drivePID_lastError = 0;
    drivePID_lastTime = 0;
    drivePID_slope = 0;
    drivePID_accumulatedError = 0;
    drivePower = 0.9;
    maxSpeed = 900;
    LFdriveSpeed = 0;
    RFdriveSpeed = 0;
    LRdriveSpeed = 0;
    RRdriveSpeed = 0;
    // turnGain increases the rotation rate based on heading error of the robot while driving
    // Larger number ramp up the turn speed faster e.g. 25deg error x 0.01 = 25% max rotation power
    turnGain = 0.01;
    // driveGain adjusts the max speed based on drive distance. Larger numbers ramp up speed faster. 10in x 0.05 = 50% max power.
    // Larger numbers ramp up speed faster. e.g. 10in distance x 0.05 driveGain = 50% max power.
    driveGain = 0.18;
    // Setup Motors
    RaiseArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RaiseArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ExtendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    ((DcMotorEx) ExtendArm).setTargetPositionTolerance(10);
    LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RRmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    ((DcMotorEx) LiftMotor).setTargetPositionTolerance(5);
    LFmotor.setDirection(DcMotor.Direction.FORWARD);
    LRmotor.setDirection(DcMotor.Direction.FORWARD);
    RFmotor.setDirection(DcMotor.Direction.FORWARD);
    RRmotor.setDirection(DcMotor.Direction.FORWARD);
    GripperState = "Closed";
  }

  /**
   * Describe this function...
   */
  private void initIMU() {
    imu_IMU.initialize(new IMU.Parameters(orientationOnRobot));
    imu_IMU.resetYaw();
    orientation = imu_IMU.getRobotYawPitchRollAngles();
    sleep(1000);
  }

  /**
   * Describe this function...
   */
  private double Field_Bot_Heading_Normalized__deg_() {
    orientation = imu_IMU.getRobotYawPitchRollAngles();
    botHeading = Double.parseDouble(JavaUtil.formatNumber(AngleUnit.DEGREES.normalize(orientation.getYaw(AngleUnit.DEGREES) + botHeading_Offset), 3));
    return botHeading;
  }

  /**
   * Configures the SparkFun OTOS.
   */
  private void initOTOS() {
    SparkFunOTOS.Pose2D offset;

    OTOS.setLinearUnit(DistanceUnit.INCH);
    OTOS.setAngularUnit(AngleUnit.DEGREES);
    offset = new SparkFunOTOS.Pose2D(0, 0, 0);
    OTOS.setOffset(offset);
    OTOS.setLinearScalar(1);
    OTOS.setAngularScalar(1);
    OTOS.calibrateImu();
    OTOS.resetTracking();
  }

  /**
   * Describe this function...
   */
  private void Calculate_Motor_Speeds() {
    // TODO: Enter the type for variable named drivePID_maxspeed
    UNKNOWN_TYPE drivePID_maxspeed;
    double X;
    double Y;
    double cosHeading;
    double sinHeading;
    double rotX;
    double rotY;
    double Denominator;

    headingError = AngleUnit.DEGREES.normalize(targetBotHeading - Field_Bot_Heading_Normalized__deg_());
    drivePID_maxspeed = 0;
    X = Math.sin(driveAngle / 180 * Math.PI);
    Y = Math.cos(driveAngle / 180 * Math.PI);
    RX = Range.clip(headingError * turnGain, -0.6, 0.6);
    cosHeading = Math.cos(Field_Bot_Heading_Normalized__deg_() / 180 * Math.PI);
    sinHeading = Math.sin(Field_Bot_Heading_Normalized__deg_() / 180 * Math.PI);
    rotX = Y * sinHeading - X * cosHeading;
    rotY = Y * cosHeading + X * sinHeading;
    Denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(Y), Math.abs(X), Math.abs(RX))), 1));
    LFdriveSpeed = drivePID_Power() * -((rotY + rotX + RX) / Denominator);
    RFdriveSpeed = drivePID_Power() * -(((rotY - rotX) - RX) / Denominator);
    LRdriveSpeed = drivePID_Power() * -(((rotY - rotX) + RX) / Denominator);
    RRdriveSpeed = drivePID_Power() * -(((rotY + rotX) - RX) / Denominator);
  }

  /**
   * Describe this function...
   */
  private void targetTranslation() {
    double deltaX;
    double deltaY;

    deltaX = currentX - targetX;
    deltaY = currentY - targetY;
    driveDistance = Math.abs(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));
    driveAngle = Math.atan2(deltaY, deltaX) / Math.PI * 180;
  }

  /**
   * Describe this function...
   */
  private void camera_scan() {
    while (!opModeIsActive() && !isStopRequested()) {
      runAprilTagScan();
      Update_Telemetry();
    }
    // Enable or disable the AprilTag processor.
    Left_VisionPortal.setProcessorEnabled(LeftAprilTagProcessor, false);
    // Enable or disable the AprilTag processor.
    Right_VisionPortal.setProcessorEnabled(RightAprilTagProcessor, false);
  }

  /**
   * Describe this function...
   */
  private void Drive(double TgtX, int TgtY, int TgtHead, double TgtTol) {
    int target_elevation;
    boolean blocking;

    targetX = TgtX;
    targetY = TgtY;
    targetBotHeading = AngleUnit.DEGREES.normalize(TgtHead);
    targetTranslation();
    while (driveDistance > TgtTol && !isStopRequested()) {
      pos = OTOS.getPosition();
      currentX = JavaUtil.formatNumber(pos.x, 2);
      currentY = JavaUtil.formatNumber(pos.y, 2);
      targetTranslation();
      Calculate_Motor_Speeds();
      ((DcMotorEx) LFmotor).setVelocity(LFdriveSpeed);
      ((DcMotorEx) RFmotor).setVelocity(RFdriveSpeed);
      ((DcMotorEx) LRmotor).setVelocity(LRdriveSpeed);
      ((DcMotorEx) RRmotor).setVelocity(RRdriveSpeed);
      if (!blocking) {
        Arm_Elevation(target_elevation, false);
      }
      Update_Telemetry();
    }
    if (!blocking) {
      Arm_Elevation(target_elevation, true);
    }
    ((DcMotorEx) LFmotor).setVelocity(0);
    ((DcMotorEx) RFmotor).setVelocity(0);
    ((DcMotorEx) LRmotor).setVelocity(0);
    ((DcMotorEx) RRmotor).setVelocity(0);
    rotateRobot();
    Update_Telemetry();
  }

  /**
   * Describe this function...
   */
  private void rotateRobot() {
    double steeringCorrection;

    steeringCorrection = rotatePID_Power();
    // Keep looping while we are still active and not on heading.
    while (opModeIsActive() && Math.abs(headingError) > rotationAccuracy) {
      rotatePower = rotatePID_Power();
      // Clip the speed to the maximum permitted value.
      LFmotor.setPower(-rotatePower);
      LRmotor.setPower(-rotatePower);
      RFmotor.setPower(rotatePower);
      RRmotor.setPower(rotatePower);
      // Display drive status for the driver.
      Update_Telemetry();
    }
    // Stop all motion;
    LFmotor.setPower(0);
    RFmotor.setPower(0);
    LRmotor.setPower(0);
    RRmotor.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void runAprilTagScan() {
    AprilTagDetection recognition;

    // (Note we only process first visible target).
    XfList = JavaUtil.createListWith();
    YfList = JavaUtil.createListWith();
    TagHeadingList = JavaUtil.createListWith();
    decisionMargin = JavaUtil.createListWith();
    // Get a list of AprilTag detections.
    myAprilTagDetections = LeftAprilTagProcessor.getDetections();
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection recognition_item : myAprilTagDetections) {
      recognition = recognition_item;
      if (recognition && !opModeIsActive() && !isStopRequested()) {
        if (recognition.decisionMargin > 40) {
          decisionMargin.add(recognition.decisionMargin);
          XfList.add(recognition.robotPose.getPosition().x);
          YfList.add(recognition.robotPose.getPosition().y);
          TagHeadingList.add(AngleUnit.DEGREES.normalize(recognition.robotPose.getOrientation().getYaw() + 90));
        }
      } else {
        break;
      }
    }
    if (JavaUtil.listLength(XfList) > 0) {
      TagCalculatedHeading = JavaUtil.averageOfList(TagHeadingList);
      Xf = JavaUtil.averageOfList(XfList);
      Yf = JavaUtil.averageOfList(YfList);
      processTarget();
      Update_Telemetry();
    }
    // Get a list of AprilTag detections.
    myAprilTagDetections = RightAprilTagProcessor.getDetections();
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection recognition_item2 : myAprilTagDetections) {
      recognition = recognition_item2;
      if (recognition && !opModeIsActive() && !isStopRequested()) {
        if (recognition.decisionMargin > 40) {
          decisionMargin.add(recognition.decisionMargin);
          XfList.add(recognition.robotPose.getPosition().x);
          YfList.add(recognition.robotPose.getPosition().y);
          TagHeadingList.add(AngleUnit.DEGREES.normalize(recognition.robotPose.getOrientation().getYaw() + 90));
        }
      } else {
        break;
      }
    }
    if (JavaUtil.listLength(XfList) > 0) {
      Average_Location();
      processTarget();
      Update_Telemetry();
    }
  }

  /**
   * Describe this function...
   */
  private void Highest_Margin() {
    int highestMargin;
    double i;

    highestMargin = 0;
    double i_end = JavaUtil.listLength(XfList);
    double i_inc = 1;
    if (1 > i_end) {
      i_inc = -i_inc;
    }
    for (i = 1; i_inc >= 0 ? i <= i_end : i >= i_end; i += i_inc) {
      if (JavaUtil.inListGet(decisionMargin, JavaUtil.AtMode.FROM_START, (int) (i - 1), false) > highestMargin) {
        highestMargin = ((Integer) JavaUtil.inListGet(decisionMargin, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).intValue();
        TagCalculatedHeading = AngleUnit.DEGREES.normalize(((Double) JavaUtil.inListGet(TagHeadingList, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue());
        Xf = ((Double) JavaUtil.inListGet(XfList, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue();
        Yf = ((Double) JavaUtil.inListGet(YfList, JavaUtil.AtMode.FROM_START, (int) (i - 1), false)).doubleValue();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Average_Location() {
    TagCalculatedHeading = AngleUnit.DEGREES.normalize(JavaUtil.averageOfList(TagHeadingList));
    Xf = JavaUtil.averageOfList(XfList);
    Yf = JavaUtil.averageOfList(YfList);
  }

  /**
   * This function displays location on the field and rotation about the Z
   * axis on the field. It uses results from the isTargetVisible function.
   */
  private void processTarget() {
    imuHeading = orientation.getYaw(AngleUnit.DEGREES);
    currentX = Double.parseDouble(JavaUtil.formatNumber(Xf, 2));
    currentY = Double.parseDouble(JavaUtil.formatNumber(Yf, 2));
    botHeading = Double.parseDouble(JavaUtil.formatNumber(TagCalculatedHeading, 3));
    botHeading_Offset = botHeading - imuHeading;
    currentPosition = new SparkFunOTOS.Pose2D(currentX, currentY, TagCalculatedHeading);
    OTOS.setPosition(currentPosition);
    if (!isStarted() && !isStopRequested()) {
      setStartPosition();
    }
    Update_Telemetry();
  }

  /**
   * Describe this function...
   */
  private YawPitchRollAngles Convert_180_to_360(double inputAngle) {
    double outputAngle;

    if (inputAngle < 0) {
      outputAngle = Double.parseDouble(JavaUtil.formatNumber(inputAngle + 360, 3));
    } else {
      outputAngle = inputAngle;
    }
    return outputAngle;
  }

  /**
   * Describe this function...
   */
  private double drivePID_Power() {
    // Determine the heading current error.
    drivePID_accumulatedError = (int) (drivePID_accumulatedError + driveDistance);
    if (driveDistance < 1) {
      drivePID_accumulatedError = 0;
    }
    drivePID_slope = 0;
    if (drivePID_lastTime > 0) {
      // Get the current time in milliseconds. The value returned represents
      // the number of milliseconds since midnight, January 1, 1970 UTC.
      drivePID_slope = (int) ((driveDistance - drivePID_lastError) / (System.currentTimeMillis() - drivePID_lastTime));
      // Get the current time in milliseconds. The value returned represents
      // the number of milliseconds since midnight, January 1, 1970 UTC.
      drivePID_lastTime = System.currentTimeMillis();
      drivePID_lastError = driveDistance;
    }
    return maxSpeed * Range.clip(driveDistance * drivePID_P + drivePID_accumulatedError * drivePID_I + drivePID_D * drivePID_slope, 0.2, 1);
  }

  /**
   * Describe this function...
   */
  private double rotatePID_Power() {
    // Determine the heading current error.
    headingError = targetBotHeading - Field_Bot_Heading_Normalized__deg_();
    if (headingError > 180) {
      headingError = headingError - 360;
    } else if (headingError <= -180) {
      headingError = headingError + 360;
    }
    rotatePID_accumulatedError = (int) (rotatePID_accumulatedError + headingError);
    if (Math.abs(headingError) < 1) {
      rotatePID_accumulatedError = 0;
    }
    // Ensure sign of rotatePID_acuumulatedError matches headingError
    if (headingError < 0) {
      rotatePID_accumulatedError = Math.abs(rotatePID_accumulatedError) * -1;
    } else {
      rotatePID_accumulatedError = Math.abs(rotatePID_accumulatedError) * 1;
    }
    rotatePID_slope = 0;
    if (rotatePID_lastTime > 0) {
      // Get the current time in milliseconds. The value returned represents
      // the number of milliseconds since midnight, January 1, 1970 UTC.
      rotatePID_slope = (int) ((headingError - rotatePID_lastError) / (System.currentTimeMillis() - rotatePID_lastTime));
      // Get the current time in milliseconds. The value returned represents
      // the number of milliseconds since midnight, January 1, 1970 UTC.
      rotatePID_lastTime = System.currentTimeMillis();
      rotatePID_lastError = headingError;
    }
    return Range.clip(Range.clip(headingError, -0.11, 0.11) + 0.9 * Range.clip(headingError * rotatePID_P + rotatePID_accumulatedError * rotatePID_I + rotatePID_D * rotatePID_slope, -1, 1), -rotatePowerMax, rotatePowerMax);
  }

  /**
   * By default, distances are returned in millimeters by Vuforia.
   * Convert to other distance units (CM, M, IN, and FT).
   */
  private double displayValue(double originalValue,
      // TODO: Enter the type for argument named units
      UNKNOWN_TYPE units) {
    double convertedValue;

    // Vuforia returns distances in mm.
    if (units == "CM") {
      convertedValue = originalValue / 10;
    } else if (units == "M") {
      convertedValue = originalValue / 1000;
    } else if (units == "IN") {
      convertedValue = originalValue / 25.4;
    } else if (units == "FT") {
      convertedValue = (originalValue / 25.4) / 12;
    } else {
      convertedValue = originalValue;
    }
    return convertedValue;
  }

  /**
   * Describe this function...
   */
  private void SetDrivePIDF() {
    ((DcMotorEx) ExtendArm).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 3, 0, 7, MotorControlAlgorithm.PIDF));
    ((DcMotorEx) ExtendBucket).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 3, 0, 7, MotorControlAlgorithm.PIDF));
  }

  /**
   * Describe this function...
   */
  private void Update_Telemetry() {
    double cameraX;
    double cameraY;

    pos = OTOS.getPosition();
    if (opModeIsActive()) {
      // AS5600 Encoder Angle In Degrees
      ArmElevation = AS5600Block.AngleInDeg("AS5600sensor");
    }
    telemetry.addData("Start Location", startLocation);
    telemetry.addData("Robot X, Y (in) : Heading", currentX + ", " + currentY + " : " + Double.parseDouble(JavaUtil.formatNumber(Convert_180_to_360(Field_Bot_Heading_Normalized__deg_()), 2)));
    telemetry.addData("AprilTag X, Y (in) : Heading", JavaUtil.formatNumber(Xf, 2) + ", " + JavaUtil.formatNumber(Yf, 2) + " : " + JavaUtil.formatNumber(Convert_180_to_360(TagCalculatedHeading), 2));
    telemetry.addData("Odometry X, Y (in) : Heading", JavaUtil.formatNumber(pos.x, 2) + ", " + JavaUtil.formatNumber(pos.y, 2) + " : " + JavaUtil.formatNumber(Convert_180_to_360(pos.h), 2));
    telemetry.addData("Target X, Y (in) : Heading", JavaUtil.formatNumber(targetX, 2) + ", " + JavaUtil.formatNumber(targetY, 2) + " : " + JavaUtil.formatNumber(Convert_180_to_360(targetBotHeading), 2));
    telemetry.addData("RX:", RX);
    telemetry.addData("Decision Margin", decisionMargin);
    telemetry.addData("All Y", YfList);
    if (debugMode == true) {
      telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
      telemetry.addData("Distance (in)", Double.parseDouble(JavaUtil.formatNumber(driveDistance, 2)));
      telemetry.addData("Drive Angle", Double.parseDouble(JavaUtil.formatNumber(Convert_180_to_360(driveAngle), 2)));
      telemetry.addData("Heading Error", Double.parseDouble(JavaUtil.formatNumber(headingError, 2)));
      telemetry.addData("Camera X, Y (in)", Double.parseDouble(JavaUtil.formatNumber(cameraX, 2)) + ", " + Double.parseDouble(JavaUtil.formatNumber(cameraY, 2)));
      telemetry.addData("IMU Heading", Double.parseDouble(JavaUtil.formatNumber(Convert_180_to_360(orientation).getYaw(AngleUnit.DEGREES), 2)));
      telemetry.addData("Heading Offset", Double.parseDouble(JavaUtil.formatNumber(botHeading_Offset, 2)));
      telemetry.addData("Rotate Power", Double.parseDouble(JavaUtil.formatNumber(rotatePower, 3)));
      if (!startLocation.equals("")) {
        telemetry.addData("AprilTags Heading", Double.parseDouble(JavaUtil.formatNumber(Convert_180_to_360(TagCalculatedHeading), 2)));
      }
      telemetry.addData("LF Drive Speed:", LFdriveSpeed + " - V:" + ((DcMotorEx) LFmotor).getVelocity());
      telemetry.addData("RF Drive Speed:", RFdriveSpeed + " - V:" + ((DcMotorEx) RFmotor).getVelocity());
      telemetry.addData("LR Drive Speed:", LRdriveSpeed + " - V:" + ((DcMotorEx) LRmotor).getVelocity());
      telemetry.addData("RR Drive Speed:", RRdriveSpeed + " - V:" + ((DcMotorEx) RRmotor).getVelocity());
    }
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void SendToLog() {
    pos = OTOS.getPosition();
    // Writes a number to specified file on RC device. Includes telemetry.
    DataloggerBlock.writeToFile("MatchLog." + currentMatch + ".txt", JavaUtil.formatNumber(pos.x, 2), JavaUtil.formatNumber(pos.y, 2), JavaUtil.formatNumber(Convert_180_to_360(pos.h), 2), JavaUtil.formatNumber(targetX, 2), JavaUtil.formatNumber(targetY, 2), JavaUtil.formatNumber(Convert_180_to_360(targetBotHeading), 2), "" + Double.parseDouble(JavaUtil.formatNumber(driveDistance, 2)), "" + ArmElevation, "" + ExtendArm.getCurrentPosition(), "" + ExtendBucket.getCurrentPosition(), GripperState, "" + ControlHub_VoltageSensor.getVoltage());
  }

  /**
   * Describe this function...
   */
  private void Arm_Elevation(int target_elevation, boolean blocking) {
    double correctedElevation;

    // AS5600 Encoder Angle In Degrees
    ArmElevation = AS5600Block.AngleInDeg("AS5600sensor");
    correctedElevation = target_elevation + Arm90DegOffset;
    if (blocking) {
      while (Math.abs(correctedElevation - ArmElevation) > 0.4 && !isStopRequested()) {
        if (correctedElevation - ArmElevation < 0) {
          RaiseArm.setPower(-0.6);
        } else if (correctedElevation - ArmElevation > 0) {
          RaiseArm.setPower(0.6);
        }
        // AS5600 Encoder Angle In Degrees
        ArmElevation = AS5600Block.AngleInDeg("AS5600sensor");
      }
      RaiseArm.setPower(0);
    } else {
      if (correctedElevation - ArmElevation < -0.4) {
        RaiseArm.setPower(-0.6);
      } else if (correctedElevation - ArmElevation > 0.4) {
        RaiseArm.setPower(0.6);
      } else {
        RaiseArm.setPower(0);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void arm_extend(int arm_Extend) {
    double ArmMoveStartTime;

    ExtendArm.setTargetPosition(arm_Extend);
    ExtendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ArmMoveStartTime = getRuntime();
    while (Math.abs(ExtendArm.getCurrentPosition() - arm_Extend) > 175 && !isStopRequested()) {
      ExtendArm.setPower(0.45);
    }
  }

  /**
   * Describe this function...
   */
  private void setStartPosition() {
    if (currentX < 0 && currentY > 0) {
      startLocation = "A-3";
    } else if (currentX > 0 && currentY > 0) {
      startLocation = "A-4";
    } else if (currentX < 0 && currentY < 0) {
      startLocation = "F-3";
    } else if (currentX > 0 && currentY < 0) {
      startLocation = "F-4";
    } else {
      startLocation = "";
    }
  }

  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    AprilTagProcessor.Builder LeftAprilTagProcessorBuilder;
    AprilTagProcessor.Builder RightAprilTagProcessorBuilder;
    List myPortalList;

    // Next, create a VisionPortal.Builder and set attributes related to the camera.
    myPortalList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
    Left_ViewportID = ((Integer) JavaUtil.inListGet(myPortalList, JavaUtil.AtMode.FROM_START, (int) 0, false)).intValue();
    Right_ViewportID = ((Integer) JavaUtil.inListGet(myPortalList, JavaUtil.AtMode.FROM_START, (int) 1, false)).intValue();
    // First, create an AprilTagProcessor.Builder.
    LeftAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    RightAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    LeftAprilTagProcessorBuilder.setCameraPose(LeftCameraPosition, LeftCameraOrientation);
    RightAprilTagProcessorBuilder.setCameraPose(RightCameraPosition, RightCameraOrientation);
    // Set whether or not to draw the axes on detections.
    LeftAprilTagProcessorBuilder.setDrawAxes(true);
    // Set whether or not to draw the axes on detections.
    RightAprilTagProcessorBuilder.setDrawAxes(true);
    // Set the lens intrinsics, obtained from a camera calibration.
    LeftAprilTagProcessorBuilder.setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.364794005);
    // Set the lens intrinsics, obtained from a camera calibration.
    RightAprilTagProcessorBuilder.setLensIntrinsics(539.0239404, 539.0239404, 316.450283269, 236.364794005);
    // Create an AprilTagProcessor by calling build.
    LeftAprilTagProcessor = LeftAprilTagProcessorBuilder.build();
    RightAprilTagProcessor = RightAprilTagProcessorBuilder.build();
    Left_Viewport();
    Right_Viewport();
  }

  /**
   * Describe this function...
   */
  private void Left_Viewport() {
    // Create a VisionPortal.Builder object so you can specify attributes about the cameras.
    myVisionPortalBuilder = new VisionPortal.Builder();
    // Set the camera to the specified webcam name.
    myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "LeftWebcam"));
    // Set the stream format.
    myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
    // Set the camera resolution.
    myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
    myVisionPortalBuilder.addProcessor(LeftAprilTagProcessor);
    // Set the LiveView container id.
    myVisionPortalBuilder.setLiveViewContainerId(Left_ViewportID);
    Left_VisionPortal = myVisionPortalBuilder.build();
  }

  /**
   * Describe this function...
   */
  private void Right_Viewport() {
    // Create a VisionPortal.Builder object so you can specify attributes about the cameras.
    myVisionPortalBuilder = new VisionPortal.Builder();
    // Set the camera to the specified webcam name.
    myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "RightWebcam"));
    // Set the stream format.
    myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
    // Set the camera resolution.
    myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
    myVisionPortalBuilder.addProcessor(RightAprilTagProcessor);
    // Set the LiveView container id.
    myVisionPortalBuilder.setLiveViewContainerId(Right_ViewportID);
    Right_VisionPortal = myVisionPortalBuilder.build();
  }

  /**
   * Describe this function...
   */
  private List<HuskyLens.Block> HuskyScan(int HuskyBlockColor) {
    myHuskyLensBlocks = HuskyLens.blocks();
    return myHuskyLensBlocks;
  }

  /**
   * Describe this function...
   */
  private void ArmRetractLimit() {
    if (ArmLimit.isPressed()) {
      ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
  }

  /**
   * Describe this function...
   */
  private void Open_Gripper() {
    Gripper.setPosition(0.5);
    sleep(400);
    GripperState = "Open";
  }

  /**
   * Describe this function...
   */
  private void Close_Gripper() {
    Gripper.setPosition(-0.4);
    sleep(400);
    GripperState = "Closed";
  }

  /**
   * Describe this function...
   */
  private void HuskyCenterSample(int HuskyBlockColor) {
    boolean blockIsCentered;
    int block_X;
    HuskyLens.Block myHuskyLensBlock;
    int block_Y;
    double huskySpeed;

    blockIsCentered = false;
    while (blockIsCentered == !true && !isStopRequested()) {
      myHuskyLensBlocks = HuskyScan(HuskyBlockColor);
      if (JavaUtil.listLength(myHuskyLensBlocks) != 0) {
        myHuskyLensBlock = (((HuskyLens.Block) JavaUtil.inListGet(myHuskyLensBlocks, JavaUtil.AtMode.FROM_START, (int) 0, false)));
        block_X = myHuskyLensBlock.left + myHuskyLensBlock.width / 2;
        block_Y = myHuskyLensBlock.top + myHuskyLensBlock.height / 2;
        if (Math.abs(160 - block_X) > 3) {
          huskySpeed = Range.clip((Math.abs(160 - block_X) / 160) * 0.35, 0.08, 0.2);
          if (block_X > 158) {
            LFmotor.setPower(huskySpeed);
            LRmotor.setPower(-huskySpeed);
            RFmotor.setPower(-huskySpeed);
            RRmotor.setPower(huskySpeed);
          } else if (block_X < 163) {
            LFmotor.setPower(-huskySpeed);
            LRmotor.setPower(huskySpeed);
            RFmotor.setPower(huskySpeed);
            RRmotor.setPower(-huskySpeed);
          } else {
            LFmotor.setPower(0);
            LRmotor.setPower(0);
            RFmotor.setPower(0);
            RRmotor.setPower(0);
            blockIsCentered = true;
          }
        } else {
          blockIsCentered = true;
        }
      }
      telemetry.update();
    }
    LFmotor.setPower(0);
    LRmotor.setPower(0);
    RFmotor.setPower(0);
    RRmotor.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void Retract_Ramp() {
    Ramp.setPosition(1);
  }

  /**
   * Describe this function...
   */
  private void Extend_Ramp() {
    Ramp.setPosition(0);
    sleep(150);
  }

  /**
   * Describe this function...
   */
  private void A_3() {
    // Step 1: drive forward and place sample
    Drive(-9, 48, 270, 1);
    Arm_Elevation(48, true);
    arm_extend(-440);
    Drive(-9, 36, 270, 1);
    Open_Gripper();
    // Step 2: Drive Away from sub
    Drive(-20, 48, 270, 2);
    Close_Gripper();
    arm_extend(5);
    Arm_Elevation(90, true);
    // Step 3: Drive to red samples
    Drive(-37, 48, 270, 1);
    Drive(-36, 36, 270, 4);
    Drive(-36, 24, 270, 4);
    Drive(-36, 12, 270, 1);
    Drive(-47, 12, 270, 1);
    Drive(-53, 55, 270, 1);
  }

  /**
   * Describe this function...
   */
  private void A_4() {
    // Step 1: drive forward and place sample
    Drive(4.5, 46, 270, 2);
    Arm_Elevation(48, true);
    arm_extend(-440);
    Drive(3, 36, 270, 1.5);
    // Step 2: Drive Away from sub
    Drive(12, 48, 270, 5);
    // Step 3: Drive to yellow samples
    Arm_Elevation(99, false);
    Drive(43, 41, 270, 1);
    Open_Gripper();
    arm_extend(-330);
    Arm_Elevation(96, true);
    HuskyCenterSample(2);
    Arm_Elevation(113, true);
    Close_Gripper();
    // Step 4: Place sample in bucket
    Extend_Ramp();
    Arm_Elevation(93, true);
    arm_extend(5);
    Open_Gripper();
    Retract_Ramp();
    Close_Gripper();
    // Step 5: Drive to basket
    Drive(52, 52, 225, 1);
    // Step 6: Raise and dump
    RaiseBucket(-1450);
    // Step 7: Drive to yellow samples
    Drive(55, 55, 225, 1);
    Arm_Elevation(100, false);
    Drive(52, 52, 225, 2);
    RaiseBucket(0);
    Drive(54, 42, 270, 1);
    // Step 8: Pickup sample
    Open_Gripper();
    arm_extend(-340);
    HuskyCenterSample(2);
    Arm_Elevation(113, true);
    Close_Gripper();
    Arm_Elevation(88, true);
    Extend_Ramp();
    arm_extend(5);
    Open_Gripper();
    Retract_Ramp();
    Close_Gripper();
    // Step 9: Drive to basket
    Drive(54, 54, 225, 1);
    RaiseBucket(-1450);
    Drive(55, 55, 225, 1);
    Drive(53, 53, 225, 1);
    RaiseBucket(0);
  }

  /**
   * Describe this function...
   */
  private void F_3() {
    // Step 1: drive forward and place sample
    Drive(-4.5, -46, 90, 2);
    Arm_Elevation(48, true);
    arm_extend(-440);
    Drive(-3, -36, 90, 1.5);
    // Step 2: Drive Away from sub
    Drive(-12, -48, 90, 5);
    // Step 3: Drive to yellow samples
    Arm_Elevation(99, false);
    Drive(-43, -41, 90, 1);
    Open_Gripper();
    arm_extend(-330);
    Arm_Elevation(96, true);
    HuskyCenterSample(2);
    Arm_Elevation(113, true);
    Close_Gripper();
    // Step 4: Place sample in bucket
    Extend_Ramp();
    Arm_Elevation(93, true);
    arm_extend(5);
    Open_Gripper();
    Retract_Ramp();
    Close_Gripper();
    // Step 5: Drive to basket
    Drive(-52, -52, 45, 1);
    // Step 6: Raise and dump
    RaiseBucket(-1450);
    // Step 7: Drive to yellow samples
    Drive(-55, -55, 45, 1);
    Arm_Elevation(100, false);
    Drive(-52, -52, 45, 2);
    RaiseBucket(0);
    Drive(-54, -42, 90, 1);
    // Step 8: Pickup sample
    Open_Gripper();
    arm_extend(-340);
    HuskyCenterSample(2);
    Arm_Elevation(113, true);
    Close_Gripper();
    Arm_Elevation(88, true);
    Extend_Ramp();
    arm_extend(5);
    Open_Gripper();
    Retract_Ramp();
    Close_Gripper();
    // Step 9: Drive to basket
    Drive(-54, -54, 45, 1);
    RaiseBucket(-1450);
    Drive(-55, -55, 45, 1);
    Drive(-53, -53, 45, 1);
    RaiseBucket(0);
  }

  /**
   * Describe this function...
   */
  private void F_4() {
    // Step 1: drive forward and place sample
    Drive(9, -48, 90, 1);
    Arm_Elevation(48, true);
    arm_extend(-440);
    Drive(9, -36, 90, 1);
    Open_Gripper();
    // Step 2: Drive Away from sub
    Drive(20, -48, 90, 2);
    Close_Gripper();
    arm_extend(5);
    Arm_Elevation(90, true);
    // Step 3: Drive to red samples
    Drive(37, -48, 90, 1);
    Drive(36, -36, 90, 4);
    Drive(36, -24, 90, 4);
    Drive(36, -12, 90, 1);
    Drive(48, -12, 90, 1);
    Drive(53, -55, 90, 1);
  }

  /**
   * Describe this function...
   */
  private void RaiseBucket(int BucketPos) {
    ExtendBucket.setTargetPosition(BucketPos);
    ExtendBucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ExtendBucket.setPower(0.9);
    if (Math.abs(BucketPos) < 10) {
    } else {
      while (Math.abs(ExtendBucket.getCurrentPosition() - BucketPos) > 400 && !isStopRequested()) {
      }
    }
    if (BucketPos == 0 && BucketLimit.isPressed()) {
      ExtendBucket.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void im_lost() {
    startLocation = "I'm Lost";
    currentX = 0;
    currentY = 0;
    botHeading = Double.parseDouble(JavaUtil.formatNumber(AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 180), 3));
    imuHeading = orientation.getYaw(AngleUnit.RADIANS);
    botHeading_Offset = AngleUnit.RADIANS.normalize(imuHeading - botHeading);
    currentPosition = new SparkFunOTOS.Pose2D(currentX, currentY, 0);
    OTOS.setPosition(currentPosition);
    while (opModeIsActive()) {
      Update_Telemetry();
    }
  }
}

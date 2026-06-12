package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    private static final double MASS = 11.3;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(MASS)
            .forwardZeroPowerAcceleration(-33.02200435169401)
            .lateralZeroPowerAcceleration(-71.13360661576846)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.052, 0, 0, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.45, 0, 0, .045))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0, 0.6, 0))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.14533700143376554, 0.00050739213817853));

    //public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5, 3);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, // tValueConstraint
            0.5,   // velocityConstraint
            1.5,   // translationalConstraint (Distance from target in inches)
            Math.toRadians(5), // headingConstraint (Rotation error in radians)
            55,    // timeoutConstraint (ms)
            1.25,  // brakingStrength
            10,    // BEZIER_CURVE_SEARCH_LIMIT
            1      // brakingStart
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RFmotor")
            .rightRearMotorName("RRmotor")
            .leftRearMotorName("LRmotor")
            .leftFrontMotorName("LFmotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(61.05163237804503)
            .yVelocity(44.712808443805365);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.75)
            .strafePodX(-4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}
package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class LimelightEngine {

    public static void setIndex(int index, Limelight3A limelight) {
        limelight.pipelineSwitch(index); // Switch to pipeline number 0
    }

    public static void getResult(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();

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
            telemetry.addData("LL Target Distance (m)", targetDistance);
            telemetry.addData("LL Target Color", targetColor);
            telemetry.update();
        }

    }

}

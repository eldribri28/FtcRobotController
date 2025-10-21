package org.firstinspires.ftc.teamcode.metalBenders.ignore.archive.core;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;


public class AS5600Block extends BlocksOpModeCompanion{
    
    private static AS5600 AS5600sensor;
    
    @ExportToBlocks (
           comment = "AS5600 Encoder Angle In Ticks",
           tooltip = "...",
           parameterLabels = {"AS5600 Name"}
    )
    public static double AngleInTicks(String AS5600Name)
    {
        AS5600sensor = hardwareMap.get(AS5600.class, AS5600Name);
        double dataRaw = AS5600sensor.getAngleTicks();
        return dataRaw;
    }

    @ExportToBlocks (
           comment = "AS5600 Encoder Angle In Degrees",
           tooltip = "...",
           parameterLabels = {"AS5600 Name"}
    )    
    public static double AngleInDeg(String AS5600Name)
    {
        AS5600sensor = hardwareMap.get(AS5600.class, AS5600Name);
        double dataRaw = AS5600sensor.getAngleDeg();
        return dataRaw;
    }    

    @ExportToBlocks (
           comment = "AS5600 Encoder Angle In Radians",
           tooltip = "...",
           parameterLabels = {"AS5600 Name"}
    )
    public static double AngleInRad(String AS5600Name)
    {
        AS5600sensor = hardwareMap.get(AS5600.class, AS5600Name);
        double dataRaw = AS5600sensor.getAngleRad();
        return dataRaw;
    }    

    @ExportToBlocks (
           comment = "AS5600 Encoder Raw Angle In Ticks",
           tooltip = "...",
           parameterLabels = {"AS5600 Name"}
    )
    public static double RawAngleInTicks(String AS5600Name)
    {
        AS5600sensor = hardwareMap.get(AS5600.class, AS5600Name);
        double dataRaw = AS5600sensor.getRawAngleTicks();
        return dataRaw;
    }

    @ExportToBlocks (
           comment = "AS5600 Encoder Raw Angle In Degrees",
           tooltip = "...",
           parameterLabels = {"AS5600 Name"}
    )    
    public static double RawAngleInDeg(String AS5600Name)
    {
        AS5600sensor = hardwareMap.get(AS5600.class, AS5600Name);
        double dataRaw = AS5600sensor.getRawAngleDeg();
        return dataRaw;
    }
    
    @ExportToBlocks (
           comment = "AS5600 Encoder Raw Angle In Radians",
           tooltip = "...",
           parameterLabels = {"AS5600 Name"}
    )
    public static double getRawAngleInRad(String AS5600Name)
    {
        AS5600sensor = hardwareMap.get(AS5600.class, AS5600Name);
        double dataRaw = AS5600sensor.getRawAngleRad();
        return dataRaw;
    }


}

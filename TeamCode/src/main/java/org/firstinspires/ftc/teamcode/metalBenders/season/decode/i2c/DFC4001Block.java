package org.firstinspires.ftc.teamcode.metalBenders.season.decode.i2c;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class DFC4001Block extends BlocksOpModeCompanion{
    
    private static DFC4001 DFC4001sensor;

    @ExportToBlocks (
           comment = "DFC4001 Data0",
           tooltip = "...",
           parameterLabels = {"DFC4001 Name"}
    )
    public static double Status(String DFC4001Name)
    {
        DFC4001sensor = hardwareMap.get(DFC4001.class, DFC4001Name);
        double dataRaw = DFC4001sensor.getStatus();
        return dataRaw;
    }
}

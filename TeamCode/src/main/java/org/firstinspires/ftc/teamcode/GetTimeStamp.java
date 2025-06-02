package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;


public class GetTimeStamp extends BlocksOpModeCompanion{
    @ExportToBlocks (
           comment = "SystemTime as a Date Time",
           tooltip = "...",
           parameterLabels = {}
    )
    
    public static String newTimestamp(String args[])
    {
        long currentDateTime = System.currentTimeMillis();
        Date currentDate = new Date(currentDateTime);
        DateFormat df = new SimpleDateFormat("MMMddyyyy.HHmmss");

        return String.valueOf(df.format(currentDate));
    }
}

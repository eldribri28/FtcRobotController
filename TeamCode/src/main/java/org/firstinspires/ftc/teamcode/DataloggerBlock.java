package org.firstinspires.ftc.teamcode;

// these are (usually!) added automatically by OnBotJava when needed
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import java.io.File;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class DataloggerBlock extends BlocksOpModeCompanion {


    // This Annotation must appear immediately before any myBlock method.
    // It's optional to add a comment, tooltip, and/or parameterLabels.
    // Comment must appear on a single line, no rollovers.
    @ExportToBlocks (
    comment = "Writes a number to specified file on RC device. Includes telemetry.",
    tooltip = "Write number to file on RC device.",
    parameterLabels = {
        "Full Filename (.txt)",
        "Odometry X", "Odometry Y", "Odometry Heading",
        "Target X", "Target Y", "Target Heading", "Target Distance",
        "Arm Elevation", "Arm Extension", "Bucket Extension", "Gripper Position",
        "Battery Voltage"
    }
    )
    // This myBlock method writes a number (as text) to a file.
    // It has 2 inputs and no outputs (keyword void).
    public static void writeToFile (
        String toFileName, 
        String Odo_X, 
        String Odo_Y, 
        String Odo_H, 
        String Tgt_X, 
        String Tgt_Y, 
        String Tgt_H, 
        String Tgt_D,
        String Arm_Elevation,
        String Arm_Extension,
        String Bucket_Extension,
        String Gripper_Position,
        String Battery_Voltage
        ) {
        try{
            // Using the properties of the specified "to" file name,
            // declare a filename to be used in this method.  See Note 1 above.
            //File myFileName = AppUtil.getInstance().getSettingsFile("/sdcard/FIRST/java/src/Datalogs/" + toFileName);
    
            // Write the provided number to the newly declared filename.
            // See Note 3 above.
            String timestamp = String.valueOf(System.currentTimeMillis());
            String content = timestamp + "," + Odo_X + "," + Odo_Y + "," + Odo_H + "," + Tgt_X + "," + Tgt_Y + "," + Tgt_H + "," + Tgt_D + "," + Arm_Elevation + "," + Arm_Extension + "," + Bucket_Extension + "," + Gripper_Position + "," + Battery_Voltage + "\n"; 
            //ReadWriteFile.writeFile(myFileName, content);
            
            File file = new File("/sdcard/FIRST/java/src/Datalogs/" + toFileName);
            // If file doesn't exists, then create it
            if (!file.exists()) {
                file.createNewFile();
            }
            FileWriter fw = new FileWriter(file, true);
            BufferedWriter bw = new BufferedWriter(fw);
    
            // Write in file
            bw.write(content);
    
            // Close connection
            bw.close();      
            fw.close(); 
        }
        catch(Exception e){
            System.out.println(e);
        }
    }   // end of method writeToFile()


}

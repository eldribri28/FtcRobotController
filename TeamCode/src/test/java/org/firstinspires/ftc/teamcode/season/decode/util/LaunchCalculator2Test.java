package org.firstinspires.ftc.teamcode.season.decode.util;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator2;
import org.junit.Test;

public class LaunchCalculator2Test {

    @Test
    public void testGetLaunchData() {
        LaunchCalculator2.LaunchResult launchResult = LaunchCalculator2.getLaunchData(0.3, 1.3, 2);
        System.out.println();
        System.out.println(launchResult);
        System.out.println();
    }
}

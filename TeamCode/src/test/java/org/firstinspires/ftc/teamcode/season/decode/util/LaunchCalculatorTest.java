package org.firstinspires.ftc.teamcode.season.decode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.junit.Test;

import java.lang.reflect.Method;

public class LaunchCalculatorTest {

    @Test
    public void testDeterminePreferredLaunchResult_returnsNullWhenNoViableSolutionCalculated() throws Exception {
        LaunchResult launchResult1 = new LaunchResult(0.0, 0.0, 0.0, false);
        LaunchResult launchResult2 = new LaunchResult(0.0, 0.0, 0.0, false);

        Method method = LaunchCalculator.class.getDeclaredMethod("determinePreferredLaunchResult", LaunchResult.class, LaunchResult.class);
        method.setAccessible(true);
        LaunchResult launchResult = (LaunchResult) method.invoke(null, launchResult1, launchResult2);
        assertNull(launchResult);
    }

    @Test
    public void testDeterminePreferredLaunchResult_returnsSingleViableSolution() throws Exception {
        LaunchResult launchResult1 = new LaunchResult(0.0, 0.0, 0.0, false);
        LaunchResult launchResult2 = new LaunchResult(1.0, 2.0, 0.0, true);

        Method method = LaunchCalculator.class.getDeclaredMethod("determinePreferredLaunchResult", LaunchResult.class, LaunchResult.class);
        method.setAccessible(true);
        LaunchResult launchResult = (LaunchResult) method.invoke(null, launchResult1, launchResult2);
        assertNotNull(launchResult);
        assertEquals(launchResult2, launchResult);

        launchResult = (LaunchResult) method.invoke(null, launchResult2, launchResult1);
        assertNotNull(launchResult);
        assertEquals(launchResult2, launchResult);
    }

    @Test
    public void testDeterminePreferredLaunchResult_returnsLaunchResultWithHighestLandingAngleWhenBothAreViable() throws Exception {
        LaunchResult launchResult1 = new LaunchResult(0.0, 0.0, 0.0, true);
        LaunchResult launchResult2 = new LaunchResult(0.0, 10.0, 0.0, true);

        Method method = LaunchCalculator.class.getDeclaredMethod("determinePreferredLaunchResult", LaunchResult.class, LaunchResult.class);
        method.setAccessible(true);
        LaunchResult launchResult = (LaunchResult) method.invoke(null, launchResult1, launchResult2);
        assertNotNull(launchResult);
        assertEquals(launchResult2, launchResult);

        launchResult = (LaunchResult) method.invoke(null, launchResult2, launchResult1);
        assertNotNull(launchResult);
        assertEquals(launchResult2, launchResult);
    }

    @Test
    public void testDeterminePreferredLaunchResult_returnsLaunchResultWhenBothAreViableWithSameLandingAngle() throws Exception {
        LaunchResult launchResult1 = new LaunchResult(0.0, 10.0, 0.0, true);
        LaunchResult launchResult2 = new LaunchResult(0.0, 10.0, 0.0, true);

        Method method = LaunchCalculator.class.getDeclaredMethod("determinePreferredLaunchResult", LaunchResult.class, LaunchResult.class);
        method.setAccessible(true);
        LaunchResult launchResult = (LaunchResult) method.invoke(null, launchResult1, launchResult2);
        assertNotNull(launchResult);
        assertEquals(launchResult2, launchResult);

        launchResult = (LaunchResult) method.invoke(null, launchResult2, launchResult1);
        assertNotNull(launchResult);
        assertEquals(launchResult1, launchResult);
    }
}

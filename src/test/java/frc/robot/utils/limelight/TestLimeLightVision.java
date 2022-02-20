package frc.robot.utils.limelight;

import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

public class TestLimeLightVision {

    private static final double TARGET_HEIGT = 104;
    private static final double CAMERA_HEIGHT = 38;
    private static final double CAMERA_ANGLE = 30;

    private static LimeLightVision classUnderTest;

    @BeforeClass
    public static void init() throws Exception {
        classUnderTest = new LimeLightVision(true, CAMERA_HEIGHT, TARGET_HEIGT, CAMERA_ANGLE);
    }

    @Test
    public void testZeroAngle() {
        double directDistancetoTarget = classUnderTest.calcDirectDistanceToTarget(0); 
        Assert.assertEquals(132, directDistancetoTarget, 0.001);
    }

    @Test
    public void testZeroAngleHorizontal() {
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(0); 
        Assert.assertEquals(114.315, horizontalDistance, 0.001);
    }

    @Test
    public void test5DegreesPos(){
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(5); 
        Assert.assertEquals(94.257, horizontalDistance, 0.001);    
    }

    @Test
    public void test10DegreesPos(){
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(10); 
        Assert.assertEquals(78.656, horizontalDistance, 0.001);    
    }

    @Test
    public void test5DegreesNeg(){
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(-5); 
        Assert.assertEquals(141.537, horizontalDistance, 0.001);    
    }

    @Test
    public void test10DegreesNeg(){
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(-10); 
        Assert.assertEquals(181.334, horizontalDistance, 0.001);    
    }

}   

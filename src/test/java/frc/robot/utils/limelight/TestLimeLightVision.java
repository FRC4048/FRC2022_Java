package frc.robot.utils.limelight;

import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

public class TestLimeLightVision {

    private static final double TARGET_HEIGT = 104;
    private static final double CAMERA_HEIGHT = 38.5;
    private static final double CAMERA_ANGLE = 29;

    private static LimeLightVision classUnderTest;

    @BeforeClass
    public static void init() throws Exception {
        classUnderTest = new LimeLightVision(true, CAMERA_HEIGHT, TARGET_HEIGT, CAMERA_ANGLE);
    }

    @Test
    public void testZeroAngle() {
        double directDistancetoTarget = classUnderTest.calcDirectDistanceToTarget(0); 
        Assert.assertEquals(135.105, directDistancetoTarget, 0.001);
    }

    @Test
    public void testZeroAngleHorizontal() {
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(0); 
        Assert.assertEquals(118.165, horizontalDistance, 0.001);
    }

    @Test
    public void test5DegreesPos(){
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(5); 
        Assert.assertEquals(97.107, horizontalDistance, 0.001);    
    }

    @Test
    public void test10DegreesPos(){
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(10); 
        Assert.assertEquals(80.886, horizontalDistance, 0.001);    
    }

    @Test
    public void test5DegreesNeg(){
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(-5); 
        Assert.assertEquals(147.115, horizontalDistance, 0.001);    
    }

    @Test
    public void test10DegreesNeg(){
        double horizontalDistance = classUnderTest.calcHorizontalDistanceToTarget(-10); 
        Assert.assertEquals(190.226, horizontalDistance, 0.001);    
    }

}   

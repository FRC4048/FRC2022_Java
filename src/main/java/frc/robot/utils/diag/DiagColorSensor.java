/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.diag;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utils.ColorSensor;
import frc.robot.utils.ColorSensor.ColorValue;

/**
 * Add your docs here.
 */
public class DiagColorSensor implements Diagnosable {

    private ColorSensor colorsensor;
    private String name;
    private NetworkTableEntry networkTableEntry;
    private Map<ColorSensor.ColorValue, Boolean> colorMap;

    public DiagColorSensor(String name, ColorSensor colorsensor) {
        this.name = name;
        this.colorsensor = colorsensor;
        colorMap = new HashMap<ColorSensor.ColorValue, Boolean>();
        reset();
    }

    @Override
    public void setShuffleBoardTab(ShuffleboardTab shuffleBoardTab) {
        networkTableEntry = shuffleBoardTab.add(name, false).getEntry();
    }

    @Override
    public void refresh() {
        ColorValue colorValue = colorsensor.getColor();
        colorMap.put(colorValue, true);
        boolean allColors = colorMap.get(ColorValue.RED) && colorMap.get(ColorValue.BLUE) 
                            && colorMap.get(ColorValue.GREEN) && colorMap.get(ColorValue.YELLOW) 
                            && colorMap.get(ColorValue.UNKNOWN);
        if (networkTableEntry != null) {
            networkTableEntry.setBoolean(allColors);
        }
    }

    @Override
    public void reset() {
        colorMap.put(ColorValue.RED, false);
        colorMap.put(ColorValue.GREEN, false);
        colorMap.put(ColorValue.BLUE, false);
        colorMap.put(ColorValue.YELLOW, false);
        colorMap.put(ColorValue.UNKNOWN, false);
    }

}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
/**
 * Add your docs here.
 */
public class ColorSensor {
    private final I2C.Port I2CPort = I2C.Port.kOnboard;

    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2CPort);
    private final ColorMatch colorMatcher = new ColorMatch();

    private static final double CONFIDENCE = 0.75;

    static public class Target {
        public static final Color BLUE = ColorMatch.makeColor(0.143, 0.427, 0.429);
        public static final Color GREEN = ColorMatch.makeColor(0.197, 0.561, 0.240);
        public static final Color RED = ColorMatch.makeColor(0.561, 0.232, 0.114);
        public static final Color YELLOW = ColorMatch.makeColor(0.361, 0.524, 0.113);
    }

    // Shuffelboard
    private static final String keyDetectedColor = "ColorSensor/Color";
    private static final String keyConfidence = "ColorSensor/Confidence";
 
    public ColorSensor(){
        SmartDashboard.putString(keyDetectedColor, "");
        SmartDashboard.putNumber(keyConfidence, 0);

        colorMatcher.addColorMatch(Target.BLUE);
        colorMatcher.addColorMatch(Target.GREEN);
        colorMatcher.addColorMatch(Target.RED);
        colorMatcher.addColorMatch(Target.YELLOW);
    }


    public Color readColor(){
        ColorMatchResult match = colorMatcher.matchClosestColor( colorSensor.getColor() );

        SmartDashboard.putString(keyDetectedColor, match.color.toString());
        SmartDashboard.putNumber(keyConfidence, match.confidence);

        if (match.confidence < CONFIDENCE) {

            return null;
        } else {
            return match.color;
        }
    }
}


package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Ally {
    // Usage:
    // ```java
    // switch(Ally.getAlliance()) {
    //     case Red: // Do something if red
    //     case Blue: // Do something if blue
    // }
    // ```
    public static DriverStation.Alliance dsAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            switch (DriverStation.getAlliance().get()) {
                case Red: return Alliance.Red;
                case Blue: return Alliance.Blue;
                default: return Alliance.Red;
            }
        } else {
            return Alliance.Red;
        }
    }
}
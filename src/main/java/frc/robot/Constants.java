package frc.robot;

import edu.wpi.first.units.Units;

public class Constants {
    public static final class IntakeConstants {
        public static final int intakePivotID = 51;
        public static final int intakeRollersID = 52;
        public static final int limSwitchDIO = 7;

        public static double kP = 12;
        public static double kI = 0.0;
        public static double kD = 0.1;
    
        public static double kS = 1.1;
        public static double kG = 0.2;
        public static double kV = 0.58;
        public static double kDt = 0.02;
    
        public static double kMaxVelocity = 1.75;
        public static double kMaxAcceleration = 0.75;
    };

    public static final class ShooterPivotConstants {
        public static final int shooterPivotID = 60;
        public static final int shooterEncoder = 4;

        public static double kP = 45;
        public static double kI = 0.01;
        public static double kD = 0.1;

        public static double kS = 0.95;
        public static double kV = 1.2;
        public static double kG = 0.83;
        public static double kDt = 0.02;

        public static double kMaxVelocity = 1.75;
        public static double kMaxAcceleration = 0.75;
        public static double kMaxRotation = 65; // IN DEGREES

        public static double gearRatio = 100.0;

        public static double encoderMagnentOffset = 0.0966796875;  // TODO: update value
    }

    public static final class ShooterFlywheelConstants {

        public static final double shooterRPM = 4000;

        public static final int shooterFlywheelLeftID = 9;
        public static final int shooterFlywheelRightID = 10;

        public static double kP = 2.4206;
        public static double kI = 0.0;
        public static double kD = 0.05;

        public static double kS = 0.12517;
        public static double kV = 0.12235;
        public static double kA = 0.010447;
    }

    public static final class TelescopeConstants {
        public static int telescopeID = 59;
    }
    
    public static final class WinchConstants {
        public static int winchID = 58;
    }

    public static final class TrapConstants {
        public static int trapID = 57;
    }
}

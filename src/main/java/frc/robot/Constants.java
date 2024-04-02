package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Constants {
    public static final class IntakeConstants {
        public static final int intakePivotID = 51;
        public static final int intakeRollersID = 52;
        public static final int limSwitchDIO = 9;
        public static final int limSwitchDIO2 = 8;

        public static double kP = 14;
        public static double kI = 0.0;
        public static double kD = 0.4;
    
        public static double kS = 0.90;
        public static double kG = 0.3;
        public static double kV = 0.200;
        public static double kDt = 0.02;
    
        public static double kMaxVelocity = 4.9;
        public static double kMaxAcceleration = 5;
    };

    public static final class ShooterPivotConstants {
        public static final int shooterPivotID = 60;
        public static final int shooterEncoder = 4;

        public static double kP = 125.0;
        public static double kI = 10.0;
        public static double kD = 0.0;

        public static double kS = 3.0;
        public static double kV = 4.0;
        public static double kG = 0.12;
        public static double kDt = 0.02;

        public static double kMaxVelocity = 1;
        public static double kMaxAcceleration = 1;

        public static double kMinRotation = 0; // IN DEGREES
        public static double kMaxRotation = 65; // IN DEGREES

        public static double gearRatio = 240.0;

        public static double encoderMagnentOffset = 0.18115234375;  // TODO: update value
    }

    public static final class ShooterFlywheelConstants {
        public static final int shooterFlywheelLeftID = 9;
        public static final int shooterFlywheelRightID = 10;

        public static final class Fast {
            public static final double shooterRPM = 4000;
            public static final double shooterAcceleration = 100;
           
            public static double kP = 4.0;
            public static double kI = 0.0;
            public static double kD = 0.00;
    
            public static double kS = 0.067504;
            public static double kV = 0.12574;
            public static double kA = 0.020513;
        }
        
        public static final class Slow {
            public static final double shooterRPM = 240;
            public static final double shooterAcceleration = 1;

            public static double kP = 0.5;
            public static double kI = 0.0;
            public static double kD = 0.00;

            public static double kS = 0.6;
            public static double kV = 0.3;
        }
    }

    public static final class TelescopeConstants {
        public static int telescopeIDLeft = 59;
        public static int telescopeIDRight = 58;
    }
    
    public static final class WinchConstants {
        public static int winchID = 58;
    }

    public static final class TrapConstants {
        public static int trapID = 57;
    }

    public static final class photonConstants {
        public static Matrix<N3, N1> multiTagSTDDEVS = VecBuilder.fill(0.5, 0.5, 1);
        public static Matrix<N3, N1> singleTagSTDDEVS = VecBuilder.fill(1.0, 1.0, 2);

    }
}

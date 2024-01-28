package frc.robot.ControlSys;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class launchpad implements Subsystem {

    public boolean cur[][] = new boolean[9][9];
    public NetworkTableInstance table;
    public NetworkTable launch;

    public launchpad() {
        table = NetworkTableInstance.getDefault();
        launch = table.getTable("launchpad");
    }

    public boolean pressed(int row, int col) {
        return cur[row][col];
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 9; i++) {
            for (int a = 0; a < 9; a++) {
                cur[i][a] = launch.getEntry(i+"|"+a).getBoolean(false);
            }
        }
    }
}

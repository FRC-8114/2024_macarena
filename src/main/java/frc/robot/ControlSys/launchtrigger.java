package frc.robot.ControlSys;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class launchtrigger extends Trigger {
    public launchtrigger(launchpad launch, int row, int col) {
        super(() -> launch.pressed(row, col));
    }
}

package frc.robot.Util;

import edu.wpi.first.wpilibj.AddressableLED;

public final class LEDs {
    static AddressableLED lead = new AddressableLED(6);

    public LEDs () {
        lead.setLength(0);
    }
}

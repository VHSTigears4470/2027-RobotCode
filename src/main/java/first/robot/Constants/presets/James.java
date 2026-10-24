package first.robot.Constants.presets;

import org.wpilib.driverstation.XboxController.*;

public enum James implements Layout {
    LAYOUT; 

    @Override public int SHOOTER() { return Button.B.value; }
    @Override public int INTAKE()  { return Button.A.value; }
}

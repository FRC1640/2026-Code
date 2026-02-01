package frc.robot.util.wrapper.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * class for SubsystemBase + Wrapper for SubsystemInfo
 */
public abstract class SubsystemPlatform extends SubsystemBase {

    public static final SubsystemInfo info = null;

    public SubsystemPlatform() {
        super();
    }

    public static SubsystemInfo getInfo() {
        return info;
    }
}

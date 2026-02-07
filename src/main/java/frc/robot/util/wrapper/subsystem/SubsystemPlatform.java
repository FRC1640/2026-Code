package frc.robot.util.wrapper.subsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * class for SubsystemBase + Wrapper for SubsystemInfo
 */
public abstract class SubsystemPlatform extends SubsystemBase {

  public SubsystemPlatform(SubsystemInfo info) {
    super();
    setName(info.getName());
  }

  public abstract Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue);
}

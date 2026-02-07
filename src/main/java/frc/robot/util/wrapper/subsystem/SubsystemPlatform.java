package frc.robot.util.wrapper.subsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * class for SubsystemBase + Wrapper for SubsystemInfo
 */
public abstract class SubsystemPlatform extends SubsystemBase {

  private SubsystemInfo info;

  public SubsystemPlatform(SubsystemInfo info) {
    super();
    this.info = info;
    setName(info.getName());
  }

  public SubsystemInfo getInfo() {
    return info;
  }

  public abstract Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue);
}

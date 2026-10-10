package frc.robot.util.sysid;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.module.Module;

public class SwerveDriveSysidRoutine {

  /**
   * Creates a new sysid routine with swerve modules
   *
   * @param fl
   *            front left module
   * @param fr
   *            front right module
   * @param bl
   *            back left module
   * @param br
   *            front right module
   * @param subsystem
   *            subsystem for requirments
   * @param config
   *            config for sysid
   */
  public static SysIdRoutine createDriveRoutine(Module fl, Module fr, Module bl, Module br, SubsystemBase subsystem,
      SysIdRoutine.Config config) {
    return new SysIdRoutine(config, new SysIdRoutine.Mechanism((Voltage volts) -> {
      fl.setDriveVoltage(-volts.in(Volts));
      fr.setDriveVoltage(-volts.in(Volts));
      bl.setDriveVoltage(-volts.in(Volts));
      br.setDriveVoltage(-volts.in(Volts));
    }, null, subsystem));
  }

  public static SysIdRoutine createSteerRoutine(SubsystemBase subsystem,
      SysIdRoutine.Config config, Module... modules) {
    return new SysIdRoutine(config, new SysIdRoutine.Mechanism((Voltage voltage) -> {
      for (Module module : modules) {
        module.setSteerVoltage(voltage.in(Volts));
      }
    }, null, subsystem));
  }
}

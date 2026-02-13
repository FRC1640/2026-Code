package frc.robot.subsystems.shooter.deflector;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.shooter.ShooterControl;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;
import frc.robot.constants.RobotConstants.RobotTypes;

public class DeflectorSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = RobotTypes.deflectorSubsystem;

  private DeflectorIO io;
  private DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  public DeflectorSubsystem(DeflectorIO io) {
    super(info);
    this.io = io;
  }

  /*
   * Commands
   */

  public Command setAngleCommand(DoubleSupplier angle) {
    return run(() -> io.setAngle(angle.getAsDouble()));
  }

  public Command setAngleCommand(Supplier<TurretSetpoint> setpoint) {
    return run(() -> io.setAngle(setpoint.get()));
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return runVoltageCommand(() -> leftJoystickValue.getAsDouble() * -8);
  }
  /*
   * Commands
   */
  public Command runDeflectorToAngle(DoubleSupplier angle) {
    return run(() -> io.setAngle(angle.getAsDouble()));
  }

  public Command runDeflectorToSetpoint() {
    return run(() -> io.setAngle(ShooterControl.getInstance().getSetpoint()));
  }

  private void stop() {
    io.setVoltage(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deflector", inputs);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static DeflectorIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new DeflectorIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new DeflectorIOReal();
      case SIM -> new DeflectorIOSim();
      case REPLAY -> new DeflectorIO() {};
    };
  } // spotless formatting
}

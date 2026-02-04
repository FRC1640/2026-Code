package frc.robot.subsystems.shooter.deflector;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;

public class DeflectorSubsystem extends SubsystemBase {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.deflectorSubsystem;

  private DeflectorIO io;
  private DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  public DeflectorSubsystem(DeflectorIO io) {
    this.io = io;
  }

  /*
   * Commands
   */
  public Command runHoodToAngle(DoubleSupplier angle) {
    return run(() -> io.setAngle(angle.getAsDouble()));
  }

  public Command runHoodToSetpoint(Supplier<TurretSetpoint> setpoint) {
    return run(() -> io.setAngle(setpoint.get()));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deflector", inputs);
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

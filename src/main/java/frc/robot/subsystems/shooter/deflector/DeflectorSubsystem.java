package frc.robot.subsystems.shooter.deflector;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class DeflectorSubsystem extends SubsystemPlatform {

  private DeflectorIO io;
  private DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.deflectorSubsystem;

  public DeflectorSubsystem(DeflectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deflector", inputs);
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setDeflectorMotorVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  private void stop() {
    io.setDeflectorMotorVoltage(0);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return runVoltageCommand(() -> leftJoystickValue.getAsDouble() * -8);
  }

  @Override
  public String getName() {
    return "Deflector Subsystem";
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

  public static DeflectorIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info)) {
      return new DeflectorIO() {

      };
    }
    return switch (Robot.getMode()) {
      case REAL -> new DeflectorIOReal();
      case SIM -> new DeflectorIOSim();
      case REPLAY -> new DeflectorIO() {
      };
    };
  }
}

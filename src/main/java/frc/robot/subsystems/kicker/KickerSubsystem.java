package frc.robot.subsystems.kicker;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;
import frc.robot.constants.RobotConstants.RobotTypes;

public class KickerSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = RobotTypes.kickerSubsystem;

  private KickerIO io;
  private KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public KickerSubsystem(KickerIO io) {
    super(info);
    this.io = io;
  }

  /*----------
  | COMMANDS |
  ----------*/

  public Command runVelocityCommand(DoubleSupplier velocity) {
    return run(() -> io.setVelocity(velocity.getAsDouble())).finallyDo(this::stop);
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

  private void stop() {
    io.setVoltage(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static KickerIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info)) return new KickerIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new KickerIOReal();
      case SIM -> new KickerIOSim();
      case REPLAY -> new KickerIO() {};
    };
  } // spotless formatting
}

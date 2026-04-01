package frc.robot.subsystems.spindexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class SpindexerSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = RobotTypes.spindexerSubsystem;

  private SpindexerIO io;
  private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public SpindexerSubsystem(SpindexerIO io) {
    super(info);
    this.io = io;
  }

  /*----------
  | COMMANDS |
  ----------*/

  public Command runCommand() {
    return runVoltageCommand(() -> SpindexerConstants.runVoltage);
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

  public boolean isJammed() {
    return inputs.isJammed;
  }

  private void stop() {
    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static SpindexerIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new SpindexerIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new SpindexerIOReal();
      case SIM -> new SpindexerIOSim();
      case REPLAY -> new SpindexerIO() {
      };
    };
  } // spotless formatting
}

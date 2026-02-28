package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class ClimberSubsystem extends SubsystemPlatform {
  public static final SubsystemInfo info = RobotTypes.climberSubsystem;

  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO io) {
    super(info);
    this.io = io;
  }

  public Command setPositionCommand(double position) {
    return run(() -> io.setPosition(position)).finallyDo(this::stop);
  }

  public Command setHeightCommand(double height) {
    return run(() -> io.setHeight(height)).finallyDo(this::stop);
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier joystick0, DoubleSupplier joystick1) {
    return runVoltageCommand(() -> joystick0.getAsDouble());
  }

  private void stop() {
    io.setVoltage(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(info.getName(), inputs);
  }

  public SubsystemInfo getInfo() {
    return info;
  }

  public static final ClimberIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new ClimberIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new ClimberIOReal();
      case SIM -> new ClimberIOSim();
      case REPLAY -> new ClimberIO() {
      };
    };
  }
}

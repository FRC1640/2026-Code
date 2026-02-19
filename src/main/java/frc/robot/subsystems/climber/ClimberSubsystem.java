package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
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

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier joystick0, DoubleSupplier joystick1) {
    return Commands.none();
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
    return switch (Robot.getMode()) {
      case REAL -> new ClimberIOReal();
      case SIM -> new ClimberIOSim();
      case REPLAY -> new ClimberIO() {};
    };
  }
}

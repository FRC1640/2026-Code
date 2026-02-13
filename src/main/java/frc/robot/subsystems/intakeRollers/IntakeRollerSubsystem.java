package frc.robot.subsystems.intakeRollers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.subsystems.intakeRollers.IntakeRollerIOInputsAutoLogged;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class IntakeRollerSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.rollerSubsystem;

  private IntakeRollerIO io;
  private IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  public IntakeRollerSubsystem(IntakeRollerIO io) {
    super(info);
    this.io = io;
  }

  private void stop() {
    io.runVoltage(0);
  }

  public Command runVelocityCommand(double velocity) {
    return run(() -> io.runVelocity(velocity)).finallyDo(this::stop);
  }

  public Command runVoltageCommand(double voltage) {
    return run(() -> io.runVoltage(voltage)).finallyDo(this::stop);
  }

  public Command runCommand() {
    return runVoltageCommand(IntakeRollerConstants.intakeVoltage);
  }

  public Command stopCommand() {
    return runVoltageCommand(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static IntakeRollerIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new IntakeRollerIO() {
      };
    return switch (Robot.getMode()) {
      case REAL -> new IntakeRollerIOReal();
      case SIM -> new IntakeRollerIOSim();
      case REPLAY -> new IntakeRollerIO() {
      };
    };
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return run(() -> {
      io.runVoltage(leftJoystickValue.getAsDouble() * -8);
    }).finallyDo(this::stop);
  }
}

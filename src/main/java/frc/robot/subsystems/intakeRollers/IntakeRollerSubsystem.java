package frc.robot.subsystems.intakeRollers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class IntakeRollerSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = RobotTypes.intakeRollerSubsystem;

  private IntakeRollerIO io;
  private IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  public IntakeRollerSubsystem(IntakeRollerIO io) {
    super(info);
    this.io = io;
  }

  public Command runCommand() {
    return runVoltageCommand(IntakeRollerConstants.intakeVoltage);
  }

  public Command runVelocityCommand(double velocity) {
    return run(() -> io.setVelocityRadPerSec(velocity)).finallyDo(this::stop);
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  public Command runVoltageCommand(double voltage) {
    return runVoltageCommand(() -> voltage);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return run(() -> {
      io.setVoltage(leftJoystickValue.getAsDouble() * -8);
    }).finallyDo(this::stop);
  }

  private void stop() {
    io.setVoltage(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRollers", inputs);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static IntakeRollerIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new IntakeRollerIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new IntakeRollerIOReal();
      case SIM -> new IntakeRollerIOSim();
      case REPLAY -> new IntakeRollerIO() {};
    };
  } // spotless formatting
}

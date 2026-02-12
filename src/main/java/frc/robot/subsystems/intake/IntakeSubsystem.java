package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class IntakeSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.intakeSubsystem;

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    super(info);
    this.io = io;
  }

  private void stop() {
    io.setVoltage(0, inputs);
  }

  public Command setIntakePositionCommand(double pos) {
    return run(() -> io.setPosition(pos, inputs)).finallyDo(this::stop);
  }

  public Command runVoltageCommand(DoubleSupplier voltage, IntakeIOInputs inputs) {
    return run(() -> io.setVoltage(voltage.getAsDouble(), inputs)).finallyDo(this::stop);
  }

  public Command intakeDownCommand() {
    return setIntakePositionCommand(IntakeConstants.intakeDownPosition);
  }

  public Command intakeUpCommand() {
    return setIntakePositionCommand(IntakeConstants.intakeUpPosition);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static IntakeIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new IntakeIO() {
      };
    return switch (Robot.getMode()) {
      case REAL -> new IntakeIOReal();
      case SIM -> new IntakeIOSim();
      case REPLAY -> new IntakeIO() {
      };
    };
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return run(() -> {
      io.setVoltage(leftJoystickValue.getAsDouble() * -8, inputs);
    }).finallyDo(this::stop);
  }
}

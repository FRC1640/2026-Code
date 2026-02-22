package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.util.command.TimedCommand;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class IntakeSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = RobotConstants.RobotTypes.intakeSubsystem;

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    super(info);
    this.io = io;
  }

  public Command setPositionCommand(double pos) {
    return setPositionCommand(() -> pos);
  }

  public Command setPositionCommand(DoubleSupplier pos) {
    return run(() -> io.setPosition(pos.getAsDouble())).finallyDo(this::stop);
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  public Command intakeDownCommand() {
    return setPositionCommand(IntakeConstants.activePositionRadians);
  }

  public Command intakeUpCommand() {
    return setPositionCommand(IntakeConstants.stowedPositionRadians);
  }

  public Command oscillateIntakeCommand(double angleDegrees, double amp, double freq) {
    return new TimedCommand((t) -> io.setPosition(angleDegrees + amp * Math.sin(t * 2 * Math.PI * freq)))
        .finallyDo(this::stop);
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
}

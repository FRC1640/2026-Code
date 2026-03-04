package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
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

  private double holdPosition = 0;

  public IntakeSubsystem(IntakeIO io) {
    super(info);
    this.io = io;
  }

  public Command setPositionRadiansCommand(double pos) {
    return IntakeSubsystem.this.setPositionRadiansCommand(() -> pos);
  }

  public Command setPositionRadiansCommand(DoubleSupplier pos) {
    return run(() -> io.setPosition(pos.getAsDouble())).finallyDo(this::stop);
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  public Command intakeDownCommand() {
    return setPositionRadiansCommand(IntakeConstants.activePositionRadians);
  }

  public Command intakeUpCommand() {
    return setPositionRadiansCommand(IntakeConstants.stowedPositionRadians);
  }

  public Command intakeHoldCommand(double angleRadians) {
    return run(() -> io.setPositionHold(holdPosition)).beforeStarting(() -> holdPosition = angleRadians)
        .finallyDo(this::stop);
  }

  public Command intakeHoldCommand() {
    return intakeHoldCommand(inputs.positionRadians);
  }

  public Command oscillateIntakeCommand(double pos, double amp, double freq) {
    return new TimedCommand((t) -> io.setState(pos + amp * Math.sin(2 * Math.PI * freq * t),
        (2 * Math.PI * freq) * amp * Math.cos(2 * Math.PI * freq * t)), this).finallyDo(this::stop);
  }

  public Command simpleOscillateIntakeCommand() {
    return intakeDownCommand().until(() -> isDown())
        .andThen(IntakeSubsystem.this.setPositionRadiansCommand(() -> Units.degreesToRadians(50)))
        .until(() -> isAtPosition(Units.degreesToRadians(50))).repeatedly();
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return run(() -> {
      io.setVoltage(leftJoystickValue.getAsDouble() * -8);
    }).finallyDo(this::stop);
  }

  public boolean isDown() {
    return isAtPosition(IntakeConstants.activePositionRadians);
  }

  public boolean isAtPosition(double positionRadians) {
    return MathUtil.isNear(positionRadians, inputs.positionRadians, IntakeConstants.intakeSetpointToleranceRadians);
  }

  public boolean isAtPosition(double positionRadians, double toleranceRadians) {
    return MathUtil.isNear(positionRadians, inputs.positionRadians, toleranceRadians);
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

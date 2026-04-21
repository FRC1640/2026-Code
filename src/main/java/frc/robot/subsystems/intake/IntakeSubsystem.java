package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

  private Debouncer currentDebouncer;

  public IntakeSubsystem(IntakeIO io) {
    super(info);
    this.io = io;
    this.currentDebouncer = new Debouncer(0.3);
  }

  public Command setPositionRadiansCommand(double pos) {
    return IntakeSubsystem.this.setPositionRadiansCommand(() -> pos);
  }

  public Command setPositionRadiansCommand(DoubleSupplier pos) {
    return run(() -> io.setPosition(pos.getAsDouble())).finallyDo(this::stop);
  }

  public Command setVelocityRadPerSecCommand(DoubleSupplier velocityRadPerSec) {
    return run(() -> io.setVelocity(velocityRadPerSec.getAsDouble())).finallyDo(this::stop);
  }

  public Command setVelocityDegreesPerSecCommand(DoubleSupplier velocityDegreesPerSec) {
    return setVelocityRadPerSecCommand(() -> Units.degreesToRadians(velocityDegreesPerSec.getAsDouble()));
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  public Command intakeDownCommand() {
    return runVoltageCommand(() -> 2).until(() -> MathUtil.isNear(io.getPositionRadians(), IntakeConstants.activePositionRadians, 0.2)).finallyDo(this::stop); // 0.2 Rad -> 11.45916 deg
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
    return simpleOscillateIntakeCommand(60);
  }

  public Command simpleOscillateIntakeCommand(double maxAngleDegrees) {
    return simpleOscillateIntakeCommand(maxAngleDegrees, 1);
  }

  public Command simpleOscillateIntakeCommand(double maxAngleDegrees, double timeout) {
    return new WaitUntilCommand(() -> isDown()).withTimeout(timeout).deadlineFor(intakeDownCommand())
        .andThen(new WaitUntilCommand(
            () -> isAtPosition(Units.degreesToRadians(maxAngleDegrees), Units.degreesToRadians(8)))
                .withTimeout(timeout)
                .deadlineFor(IntakeSubsystem.this
                    .setPositionRadiansCommand(() -> Units.degreesToRadians(maxAngleDegrees))))
        .repeatedly();
  }

  public Command automaticOscillateIntakeCommand(double amplitudeDegrees, double errorToleranceDegrees) {
    return setPositionRadiansCommand(Units.degreesToRadians(amplitudeDegrees))
        .until(() -> isAtPosition(Units.degreesToRadians(amplitudeDegrees),
            Units.degreesToRadians(errorToleranceDegrees)))
        .until(() -> currentDebouncer
            .calculate(inputs.motorCurrent > IntakeConstants.oscillationCurrentThreshold))
        .andThen(setPositionRadiansCommand(IntakeConstants.activePositionRadians)
            .until(() -> isAtPosition(IntakeConstants.activePositionRadians,
                Units.degreesToRadians(errorToleranceDegrees))))
        .repeatedly();
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

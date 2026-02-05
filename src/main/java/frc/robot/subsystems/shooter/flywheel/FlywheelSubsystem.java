package frc.robot.subsystems.shooter.flywheel;

import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class FlywheelSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.flywheelSubsystem;

import frc.robot.util.limits.ExponentialMovingAverage;

public class FlywheelSubsystem extends SubsystemBase {
  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private SysIdRoutine sysIdRoutine;

  private ExponentialMovingAverage currentEMA;
  private boolean jamDetected = false;

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
    setName(info.getName());

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(8), Seconds.of(15),
            (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.magnitude()), null, this)); // TODO: maybe
    // change
    // this?

    currentEMA = new ExponentialMovingAverage(2.0, 10.0,
        () -> Math.average(inputs.motorCurrent, inputs.motorFollowerCurrent), "FlywheelCurrent");
  }

  public boolean isJamDetected() {
    return jamDetected;
  }

  public void clearJamDetected() {
    jamDetected = false;
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stopVoltage);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return runVoltageCommand(() -> leftJoystickValue.getAsDouble() * -8);
  }
  /*
   * Commands
   */
  public Command runFlywheelSpeed(DoubleSupplier speed) {
    return run(() -> io.setVelocity(speed.getAsDouble())).finallyDo(this::stop);
  }

  public Command runFlywheelSpeed(Supplier<TurretSetpoint> setpoint) {
    return run(() -> io.setVelocity(setpoint.get()));
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  public void stopVoltage() {
    io.setVoltage(0);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public static FlywheelIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info)) {
      return new FlywheelIO() {

      };
    }
    return switch (Robot.getMode()) {
      case REAL -> new FlywheelIOReal();
      case SIM -> new FlywheelIOSim();
      case REPLAY -> new FlywheelIO() {
      };
    };
  }
}

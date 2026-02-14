package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.ShotControl.TurretSetpoint;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class HoodSubsystem extends SubsystemPlatform {
  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = RobotTypes.hoodSubsystem;

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public HoodSubsystem(HoodIO io) {
    super(info);
    this.io = io;
  }

  /*----------
  | COMMANDS |
  ----------*/

  public Command aimCommand() {
    return setAngleCommand(() -> ShotControl.getInstance().getSetpoint());
  }

  public Command downCommand() {
    return setAngleDegCommand(() -> HoodConstants.downPosition);
  }

  public Command setAngleRadCommand(DoubleSupplier angle) {
    return run(() -> io.setAngleRad(angle.getAsDouble()));
  }

  public Command setAngleDegCommand(DoubleSupplier angle) {
    return run(() -> io.setAngleRad(Math.toRadians(angle.getAsDouble())));
  }

  public Command setAngleCommand(Supplier<TurretSetpoint> setpoint) {
    return run (() -> io.setAngle(setpoint.get())); // internally converts to radians
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
  /*
   * Commands
   */
  public Command runHoodToAngle(DoubleSupplier angle) {
    return run(() -> io.setAngleRad(angle.getAsDouble()));
  }

  public Command runHoodToSetpoint() {
    return run(() -> io.setAngle(ShotControl.getInstance().getSetpoint()));
  }

  private void stop() {
    io.setVoltage(0);
  }

  public boolean isAtSetpoint() {
    return Math.abs(Math.toDegrees(inputs.angle) - ShotControl.getInstance().getSetpoint().hoodAngleDeg()) < Math.toDegrees(HoodConstants.angleToleranceRad);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public static SubsystemInfo getInfo() {
    return info;
  }

  // custom formatting
  public static HoodIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info))
      return new HoodIO() {};
    return switch (Robot.getMode()) {
      case REAL -> new HoodIOReal();
      case SIM -> new HoodIOSim();
      case REPLAY -> new HoodIO() {};
    };
  } // spotless formatting
}

package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
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

  public Command runHoodToSetpointCommand() {
    return setAngleCommand(() -> ShotControl.getInstance().getSetpoint());
  }

  public Command downCommand() {
    return setAngleDegCommand(() -> HoodConstants.downAngleRadians);
  }

  public Command setAngleRadCommand(DoubleSupplier angle) {
    return run(() -> io.setAngleRadians(angle.getAsDouble())).finallyDo(this::stop);
  }

  public Command setAngleDegCommand(DoubleSupplier angle) {
    return setAngleRadCommand(() -> Units.degreesToRadians(angle.getAsDouble()));
  }

  public Command setAngleCommand(Supplier<TurretSetpoint> setpoint) {
    return setAngleDegCommand(() -> setpoint.get().hoodAngleDeg());
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return runVoltageCommand(() -> leftJoystickValue.getAsDouble() * -1);
  }

  private void stop() {
    io.setVoltage(0);
  }

  public boolean isAtSetpoint() {
    return Math.abs(inputs.angleVerticalDegrees - ShotControl.getInstance().getSetpoint().hoodAngleDeg()) < Math
        .toDegrees(HoodConstants.angleToleranceRadians);
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

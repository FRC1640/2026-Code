package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.turret.TurretConstants.turretAngleLimits;
import static frc.robot.subsystems.shooter.turret.TurretConstants.velocityLimitRate;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.shooter.ShooterControl;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class TurretSubsystem extends SubsystemPlatform {

  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private SysIdRoutine sysIdRoutine;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    setName(info.getName());
    ShooterControl.setTurretAngleSupplier(() -> inputs.angle);

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(8), Seconds.of(15),
            (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.magnitude()), null, this)); // TODO: maybe
    // change
    // this?
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  public Command trackCommand() {
    return run(this::track).finallyDo(this::stop);
  }

  private void track() {
    TurretSetpoint setpoint = ShooterControl.getInstance().getSetpointLocal();
    double finalAngle = 0;
    double finalVelocity = 0;
    // limit angle setpoint
    if (turretAngleLimits.inRange(setpoint.turretAngle())) {
      finalAngle = setpoint.turretAngle();
    } else {
      finalAngle = turretAngleLimits.clampPosition(setpoint.turretAngle());
    }
    // limit velocity setpoint to slow down near limit
    double intervalPos = (finalAngle - turretAngleLimits.low) / (turretAngleLimits.high - turretAngleLimits.low);
    double scaledVelocity = setpoint.turretOmega() * trapezoidScale(intervalPos);
    boolean approachingLimit = (intervalPos > 0.5) ? setpoint.turretOmega() > 0 : setpoint.turretOmega() < 0;
    if (approachingLimit) {
      finalVelocity = scaledVelocity;
    } else if (turretAngleLimits.inRange(setpoint.turretAngle())) {
      finalVelocity = setpoint.turretOmega();
    } else {
      finalVelocity = 0;
    }
    Logger.recordOutput("Shooter/velocitySetpointScale", scaledVelocity / finalVelocity);
    io.setTurretState(finalAngle, finalVelocity);
  }

  public void stop() {
    io.setVoltage(0);
  }

  public Rotation2d getAngle() {
    return new Rotation2d(inputs.angle);
  }

  private double trapezoidScale(double x) {
    return (0 <= x && x <= 1 / velocityLimitRate)
        ? x * velocityLimitRate
        : (1 - (1 / velocityLimitRate) <= x && x <= 1) ? -velocityLimitRate * (x - 1) : 1;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Shooter/turretDirection", RobotOdometry.instance.getPose("Main")
        .plus(new Transform2d(new Translation2d(1, new Rotation2d(inputs.angle)), new Rotation2d())));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public static TurretIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info)) {
      return new TurretIO() {

      };
    }
    return switch (Robot.getMode()) {
      case REAL -> new TurretIOReal();
      case SIM -> new TurretIOSim();
      case REPLAY -> new TurretIO() {
      };
    };
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return runVoltageCommand(() -> leftJoystickValue.getAsDouble() * -8);
  }
}

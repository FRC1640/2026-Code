package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.turretAngleLimits;
import static frc.robot.subsystems.shooter.turret.TurretConstants.velocityLimitRate;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.shooter.ShooterControl;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

public class TurretSubsystem extends SubsystemBase {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public TurretSubsystem(TurretIO io) {
    this.io = io;
  }

  public Command trackCommand() {
    return run(this::track).finallyDo(this::stop);
  }

  private void track() {
    TurretSetpoint setpoint = ShooterControl.getInstance().getSetpoint();
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
    boolean approachingLimit = (intervalPos > 0.5)
      ? setpoint.turretOmega() > 0
      : setpoint.turretOmega() < 0;
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

  private void stop() {
    io.setTurretVoltage(0);
  }

  private double trapezoidScale(double x) {
    return (0 <= x && x <= 1 / velocityLimitRate)
      ? x * velocityLimitRate
      : (1 - (1 / velocityLimitRate) <= x && x <= 1)
        ? -velocityLimitRate * (x - 1)
        : 1;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Shooter/turretDirection",
      RobotOdometry.instance.getPose("Main")
        .plus(new Transform2d(new Translation2d(1, new Rotation2d(inputs.turretAngle)),
              new Rotation2d())));
  }
}

package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.shooter.ShooterControl;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;
import static frc.robot.subsystems.shooter.turret.TurretConstants.turretAngleLimits;
import static frc.robot.subsystems.shooter.turret.TurretConstants.velocityLimitRate;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;

public class TurretSubsystem extends SubsystemBase {

  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.turretSubsystem;
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

  private void stop() {
    io.setTurretVoltage(0);
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
        .plus(new Transform2d(new Translation2d(1, new Rotation2d(inputs.turretAngle)), new Rotation2d())));
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
}

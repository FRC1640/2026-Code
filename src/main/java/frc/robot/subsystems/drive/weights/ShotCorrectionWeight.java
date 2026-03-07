package frc.robot.subsystems.drive.weights;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShotCorrectionWeight implements DriveWeight {

  private static double velocityConstantRotationRadiansPerSecond = 5;
  private TurretSubsystem turretSubsystem;

  public ShotCorrectionWeight(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Logger.recordOutput("Correction/CorrectionAmount",
        turretSubsystem.getMultiplierDrive() * velocityConstantRotationRadiansPerSecond);
    return new ChassisSpeeds(0, 0, Units
        .degreesToRadians(turretSubsystem.getMultiplierDrive() * velocityConstantRotationRadiansPerSecond));
  }

  @Override
  public Vector<N3> getWeight() {
    return VecBuilder.fill(0, 0, 1);
  }

  public boolean isDone() {
    return Math.abs(getSpeeds().omegaRadiansPerSecond) <= 0.02;
  }
}

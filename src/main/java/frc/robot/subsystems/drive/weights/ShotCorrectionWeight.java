package frc.robot.subsystems.drive.weights;

import static frc.robot.subsystems.turret.TurretConstants.turretAngleLimits;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShotCorrectionWeight implements DriveWeight {
  private static final String name = "ShotCorrectionWeight";

  private static double velocityConstantRotationRadiansPerSecond = 5;
  private TurretSubsystem turretSubsystem;

  public ShotCorrectionWeight(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    //Logger.recordOutput("Correction/CorrectionAmount",
    //    turretSubsystem.getMultiplierDrive() * velocityConstantRotationRadiansPerSecond);
    //return new ChassisSpeeds(0, 0, Units
    //    .degreesToRadians(turretSubsystem.getMultiplierDrive() * velocityConstantRotationRadiansPerSecond));
    return new ChassisSpeeds(0,0,(turretSubsystem.getMultiplierDrive()!=0)?ShotControl.getInstance().getSetpoint().turretAngleRad():0);
  }

  @Override
  public Vector<N3> getWeight() {
    return VecBuilder.fill(0, 0, 1);
  }

  public boolean isDone() {
    return Math.abs(getSpeeds().omegaRadiansPerSecond) <= 0.02;
  }

  public boolean needsCorrection() {
		boolean needsCorrection = false;
		double error = 0;
		if (ShotControl.getInstance().getSetpoint().turretAngleRad() > turretAngleLimits.low && ShotControl.getInstance().getSetpoint().turretAngleRad() < turretAngleLimits.high){
			needsCorrection = true;
			if (ShotControl.getInstance().getSetpoint().turretAngleRad() > (turretAngleLimits.low + turretAngleLimits.high)/2){
				error = turretAngleLimits.high - ShotControl.getInstance().getSetpoint().turretAngleRad();
			} else {
				error = turretAngleLimits.low - ShotControl.getInstance().getSetpoint().turretAngleRad();
			}
		} else {
			error = 0;
		}
    return turretSubsystem.getMultiplierDrive() != 0;
  }

  @Override
  public String getName() {
    return name;
  }
}

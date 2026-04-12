package frc.robot.subsystems.drive.weights;

import static frc.robot.subsystems.turret.TurretConstants.turretAngleLimits;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShotCorrectionWeight implements DriveWeight {
  private static final String name = "ShotCorrectionWeight";

  private static double velocityConstantRotationRadiansPerSecond = 5;
  private TurretSubsystem turretSubsystem;
	private double error = 0;
	private PIDController pid = RobotPIDConstants.constructPID(RobotPIDConstants.shotCorrectTurnPID);

  public ShotCorrectionWeight(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    //Logger.recordOutput("Correction/CorrectionAmount",
    //    turretSubsystem.getMultiplierDrive() * velocityConstantRotationRadiansPerSecond);
    //return new ChassisSpeeds(0, 0, Units
    //    .degreesToRadians(turretSubsystem.getMultiplierDrive() * velocityConstantRotationRadiansPerSecond));
		double error = 0;
		if (ShotControl.getInstance().getSetpoint().turretAngleRad() > turretAngleLimits.low && ShotControl.getInstance().getSetpoint().turretAngleRad() < turretAngleLimits.high){
			if (ShotControl.getInstance().getSetpoint().turretAngleRad() > (turretAngleLimits.low + turretAngleLimits.high)/2){
				error = turretAngleLimits.high - ShotControl.getInstance().getSetpoint().turretAngleRad();
			} else {
				error = turretAngleLimits.low - ShotControl.getInstance().getSetpoint().turretAngleRad();
			}
		} 
		return new ChassisSpeeds(0,0,pid.calculate(error));
  }

  @Override
  public Vector<N3> getWeight() {
    return needsCorrection()?VecBuilder.fill(0, 0, 1):VecBuilder.fill(0,0,0);
  }

  public boolean isDone() {
    return Math.abs(getSpeeds().omegaRadiansPerSecond) <= 0.02;
  }

  public boolean needsCorrection() {
		boolean needsCorrection = false;
		if (ShotControl.getInstance().getSetpoint().turretAngleRad() > turretAngleLimits.low && ShotControl.getInstance().getSetpoint().turretAngleRad() < turretAngleLimits.high){
			needsCorrection = true;
		}
    return needsCorrection;
  }

  @Override
  public String getName() {
    return name;
  }
}

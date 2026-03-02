package frc.robot.util.projectileLogger;

import frc.robot.RobotCommands;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.ShotControl.ShotType;
import frc.robot.subsystems.ShotControl.TurretSetpoint;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ProjectileLogger {

  public ProjectileLogger(RobotCommands robotCommands) {
    SmartDashboard.putNumber("Shooter Velocity RPM 0", 0);
    SmartDashboard.putNumber("Shooter Velocity RPM f", 0);
    SmartDashboard.putNumber("Shooter Velocity RPM Step", 0);
    SmartDashboard.putNumber("Hood Angle Deg 0", 0);
    SmartDashboard.putNumber("Hood Angle Deg f", 0);
    SmartDashboard.putNumber("Hood Angle Deg Step", 0);
  }

  // BALL PROJECTILE LOGGER COMMAND
  public static Command bplCommand(RobotCommands robotCommands) {
    double shooterVelocityRPM0 = SmartDashboard.getNumber("Shooter Velocity RPM 0", 0);
    double shooterVelocityRPMf = SmartDashboard.getNumber("Shooter Velocity RPM f", 0);
    double RPMStep = SmartDashboard.getNumber("Shooter Velocity RPM Step", 0);
    double hoodAngleDeg0 = SmartDashboard.getNumber("Hood Angle Deg 0", 0);
    double hoodAngleDegf = SmartDashboard.getNumber("Hood Angle Deg f", 0);
    double DegStep = SmartDashboard.getNumber("Hood Angle Deg Step", 0);
    int shooterSteps = (int) Math.ceil(shooterVelocityRPMf - shooterVelocityRPM0 / RPMStep);
    int hoodSteps = (int) Math.ceil(hoodAngleDegf - hoodAngleDeg0 / DegStep);

    ArrayList<Command> commands = new ArrayList<Command>(shooterSteps * hoodSteps);
    for (double shooterVelocityRPM = shooterVelocityRPM0; shooterVelocityRPM <= shooterVelocityRPMf; shooterVelocityRPM += RPMStep) {
      ShotControl.getInstance().setShotType(ShotType.MANUAL);
      final double localshooterVelocityRPM = shooterVelocityRPM;
      for (double hoodAngleDeg = hoodAngleDeg0; hoodAngleDeg <= hoodAngleDegf; hoodAngleDeg += DegStep) {
        final double localHoodAngleDeg = hoodAngleDeg; // mutable variables cannot be used in lambdas.
        commands.add(new InstantCommand(() -> {
          ShotControl.getInstance()
              .setSetpoint(new TurretSetpoint(0, 0, localHoodAngleDeg, localshooterVelocityRPM));
        }).andThen(robotCommands.bplShootCommand(3), new WaitCommand(3)));
      }
    }

    return new SequentialCommandGroup((Command[]) commands.toArray());
  }
}

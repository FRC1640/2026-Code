package frc.robot.util.projectileLogger;

import frc.robot.RobotCommands;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.ShotControl.ShotSetpoint;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ProjectileLogger {

  public ProjectileLogger(RobotCommands robotCommands) {
    SmartDashboard.putNumber("Distance Minimum", 0);
    SmartDashboard.putNumber("Distance Maximum", 0);
    SmartDashboard.putNumber("Distance Step", 0);

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
      System.out.println(shooterVelocityRPM);
      ShotControl.getInstance().setManual(true);
      final double localshooterVelocityRPM = shooterVelocityRPM;
      for (double hoodAngleDeg = hoodAngleDeg0; hoodAngleDeg <= hoodAngleDegf; hoodAngleDeg += DegStep) {
        final double localHoodAngleDeg = hoodAngleDeg; // mutable variables cannot be used in lambdas.
        commands.add(new InstantCommand(() -> {
          ShotControl.getInstance()
              .setManualSetpoint(new ShotSetpoint(0, 0, localHoodAngleDeg, localshooterVelocityRPM));
        }).andThen(robotCommands.bplShootCommand(2), new WaitCommand(10)));
      }
    }
    Command[] placeholder = new Command[commands.size()];
    return new SequentialCommandGroup(commands.toArray(placeholder));
  }

  // BALL PROJECTILE LOGGER COMMAND 2 (Shasun)
  public static Command bplCommandSubhash(RobotCommands robotCommands) {
    double distMin = SmartDashboard.getNumber("Distance Minimum", 0);
    double distMax = SmartDashboard.getNumber("Distance Maximum", 0);
    double distStep = SmartDashboard.getNumber("Distance Step", 0);
    int steps = (int) Math.ceil(distMax - distMin / distStep);
    ArrayList<Command> commands = new ArrayList<Command>(steps);
    for (int i = 0; i <= steps; i++) {
      final double _i = i;
      ShotControl.getInstance().setManual(true);
      commands.add(new InstantCommand(() -> {
        ShotControl.getInstance()
            .setManualSetpoint(new ShotSetpoint(0, 0,
                ShotControl.AZInterpolator.getHoodAngle(distMin + (_i * distStep)),
                ShotControl.AZInterpolator.getShooterVelocity(distMin + (_i * distStep))));
      }).andThen(robotCommands.bplShootCommand(2), new WaitCommand(10)));
    }
    Command[] placeholder = new Command[commands.size()];
    return new SequentialCommandGroup(commands.toArray(placeholder));
  }

}

package test;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;

public class SwerveClampingTest {

  public static void run(int samples, double dreamLevel) {
    try {
      File log = new File(Filesystem.getDeployDirectory() + "/test/swerve_clamping_test.csv");
      PrintWriter writer = new PrintWriter(log);

      ChassisSpeeds speeds = new ChassisSpeeds();
      ChassisSpeeds speedsClamped;
      Translation2d centerOfRotation = new Translation2d();
      for (int i = 0; i < samples; i++) {
        boolean driveMax = Math.random() > 0.5;
        if (driveMax) {
          speeds.vxMetersPerSecond = 1;
          speeds.omegaRadiansPerSecond = Math.random();
        } else {
          speeds.vxMetersPerSecond = Math.random();
          speeds.omegaRadiansPerSecond = 1;
        }
        speedsClamped = DriveSubsystem.inceptionMode(speeds, centerOfRotation, dreamLevel);
        writer.println(speeds.vxMetersPerSecond + ", " + speeds.omegaRadiansPerSecond
            + ", " + speedsClamped.vxMetersPerSecond + ", " + speedsClamped.omegaRadiansPerSecond);
      }
      
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    };
  }

  /* public static void runFull(int samples, double dreamLevel, boolean slowMode, boolean fastMode) {
    try {
      File log = new File(Filesystem.getDeployDirectory() + "/test/swerve_clamping_test.csv");
      PrintWriter writer = new PrintWriter(log);

      JoystickInputContainer input = new JoystickInputContainer();
      ChassisSpeeds speedsClamped;
      Translation2d centerOfRotation = new Translation2d();

      JoystickDriveWeight joystickDriveWeight = new JoystickDriveWeight(
        () -> input.getXPercent(), () -> 0.0, () -> input.getOmegaPercent(),
        () -> slowMode, () -> fastMode,
        () -> true, null, () -> false);

      DriveWeightCommand.addPersistentWeight(joystickDriveWeight);

      for (int i = 0; i < samples; i++) {
        boolean driveMax = Math.random() > 0.5;
        if (driveMax) {
          input.setXPercent(1.0);
          input.setOmegaPercent(Math.random());
        } else {
          input.setXPercent(Math.random());
          input.setOmegaPercent(1.0);
        }

        ChassisSpeeds speedsPercent = DriveWeightCommand.getAllSpeeds();
        speedsPercent.vxMetersPerSecond /= DriveConstants.maxSpeed;
        speedsPercent.vyMetersPerSecond /= DriveConstants.maxSpeed;
        speedsPercent.omegaRadiansPerSecond /= DriveConstants.maxOmega;

        speedsClamped = DriveSubsystem.inceptionMode(speedsPercent, centerOfRotation, dreamLevel);

        writer.println(input.getXPercent() + ", " + input.getOmegaPercent()
            + ", " + speedsClamped.vxMetersPerSecond + ", " + speedsClamped.omegaRadiansPerSecond);
      }
      
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    };
  } */

  private static class JoystickInputContainer {
    private double xPercent = 0.0;
    private double omegaPercent = 0.0;

    public void setXPercent(double xPercent) {
      this.xPercent = xPercent;
    }

    public void setOmegaPercent(double omegaPercent) {
      this.omegaPercent = omegaPercent;
    }

    public double getXPercent() {
      return xPercent;
    }

    public double getOmegaPercent() {
      return omegaPercent;
    }
  }
}

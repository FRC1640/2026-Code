package test;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SwerveClampingTest {
  public static void run(int iterations, double dreamLevel) {
    try {
      File log = new File(Filesystem.getDeployDirectory() + "/test/swerve_clamping_test.csv");
      PrintWriter writer = new PrintWriter(log);

      ChassisSpeeds speeds = new ChassisSpeeds();
      ChassisSpeeds speedsClamped;
      Translation2d centerOfRotation = new Translation2d();
      for (int i = 0; i < iterations; i++) {
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
}

package test;

import java.io.Closeable;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SwerveClampingTest {

  public static void run(int samples, double dreamLevel) {
    try {
      File log = new File(Filesystem.getDeployDirectory() + "/test/swerve_clamping_test.csv");
      log.createNewFile();
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

  public static class SwerveTestInputRemapper extends CommandXboxController {
    public SwerveTestInputRemapper(int port) {
      super(port);
    }

    @Override
    public double getLeftX() {
      return MathUtil.applyDeadband(Math.hypot(super.getLeftX(), super.getLeftY()), 0.03);
    }

    @Override
    public double getLeftY() {
      return 0;
    }

    @Override
    public double getRightX() {
      return MathUtil.applyDeadband(Math.hypot(super.getRightX(), super.getRightY()), 0.03);
    }

    @Override
    public double getRightY() {
      return 0;
    }
  }

  public static class ChassisSpeedsLogger implements Closeable {
    private File logFile;
    private PrintWriter writer;

    private TimeInterpolatableBuffer<ChassisSpeeds> input;
    private TimeInterpolatableBuffer<ChassisSpeeds> output;
    private double lastTimeSeconds = 0;

    public ChassisSpeedsLogger() {
      input = createChassisSpeedsBuffer(1);
      output = createChassisSpeedsBuffer(1);
      try {
        logFile = new File(Filesystem.getDeployDirectory() + "/test/swerve_clamping_test__input.csv");
        logFile.createNewFile();
        writer = new PrintWriter(logFile);
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    private static TimeInterpolatableBuffer<ChassisSpeeds> createChassisSpeedsBuffer(int historySizeSeconds) {
      return TimeInterpolatableBuffer.createBuffer(
        (x1, x2, t) -> new ChassisSpeeds(
          x1.vxMetersPerSecond + t * (x2.vxMetersPerSecond - x1.vxMetersPerSecond),
          x1.vyMetersPerSecond + t * (x2.vyMetersPerSecond - x1.vyMetersPerSecond),
          x1.omegaRadiansPerSecond + t * (x2.omegaRadiansPerSecond - x1.omegaRadiansPerSecond)),
        historySizeSeconds);
    }

    public void addInputMeasurement(double timeSeconds, ChassisSpeeds speeds) {
      input.addSample(timeSeconds, speeds);
      if (timeSeconds > lastTimeSeconds) lastTimeSeconds = timeSeconds;
    }

    public void addOutputMeasurement(double timeSeconds, ChassisSpeeds speeds) {
      output.addSample(timeSeconds, speeds);
      if (timeSeconds > lastTimeSeconds) lastTimeSeconds = timeSeconds;
    }

    public void log() {
      ChassisSpeeds inputSpeeds = input.getSample(lastTimeSeconds).orElse(null);
      ChassisSpeeds outputSpeeds = output.getSample(lastTimeSeconds).orElse(null);
      if (inputSpeeds == null || outputSpeeds == null) {
        return;
      }
      writer.println(inputSpeeds.vxMetersPerSecond + ", " + inputSpeeds.omegaRadiansPerSecond
            + ", " + outputSpeeds.vxMetersPerSecond + ", " + outputSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void close() {
      writer.close();
    }
  }
}

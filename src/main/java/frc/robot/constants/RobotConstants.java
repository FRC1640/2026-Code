package frc.robot.constants;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import frc.robot.subsystems.drive.DriveConstants;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.sensors.apriltag.CameraConstant;
import frc.robot.util.WPICal.AprilTagPositionSwitcher.AprilTagSetting;
import frc.robot.util.robotswitcher.RobotType;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

public class RobotConstants {
  public class AutopilotConstants {
    private static final APConstraints kConstraints = new APConstraints().withAcceleration(DriveConstants.accelLimit).withJerk(2.0).withVelocity(DriveConstants.maxSpeed);

    private static final APProfile kProfile = new APProfile(kConstraints).withErrorXY(Centimeters.of(2))
        .withErrorTheta(Degrees.of(0.5)).withBeelineRadius(Centimeters.of(8));

    public static final com.therekrab.autopilot.Autopilot kAutopilot = new com.therekrab.autopilot.Autopilot(
        kProfile);
  }
  public class RobotInformation {
    // change it for running autons to prime25
    public static final RobotType robot = RobotTypes.frank25;
  }

  public class RobotTypes {

    // SUBSYSTEMS
    public static final SubsystemInfo intakeSubsystem = new SubsystemInfo("Intake");
    public static final SubsystemInfo turretSubsystem = new SubsystemInfo("Turret");
    public static final SubsystemInfo shooterSubsystem = new SubsystemInfo("Shooter");
    public static final SubsystemInfo hoodSubsystem = new SubsystemInfo("Hood");
    public static final SubsystemInfo spindexerSubsystem = new SubsystemInfo("Spindexer");
    public static final SubsystemInfo kickerSubsystem = new SubsystemInfo("Kicker");
    public static final SubsystemInfo driveSubsystem = new SubsystemInfo("Drive");
    public static final SubsystemInfo intakeRollerSubsystem = new SubsystemInfo("Intake Roller");
    // ROBOTS
    public static final RobotType duex26 = new RobotType("Duex26", driveSubsystem, kickerSubsystem,
        spindexerSubsystem, hoodSubsystem, shooterSubsystem, turretSubsystem, intakeSubsystem);

    public static final RobotType frank25 = new RobotType("Frank25", driveSubsystem)
        .addAprilTagCamera(CameraSettings.frankOdometryCamera);

    public static final RobotType prime25 = new RobotType("Prime25", driveSubsystem);
  }

  public enum TestingSetting {
    none, sysid, pid, pit, motor
  }

  public static enum OutputMode {
    REAL, SIM, REPLAY
  }

  public static enum RobotState {
    DISABLED, AUTONOMOUS, TELEOP, TEST
  }

  public class AprilTagPositionSettings {

    public static final AprilTagSetting fieldPositionType = AprilTagSetting.WPILibWelded;
    public static final String WPICalOutputJson = "ImportedLayout.json";
  }

  // TODO fix stuff
  public class RobotDimensions {

    public static final double robotWidth = 0.81;
    public static final double robotLength = 0.81; // 0.927
    public static final double robotLengthLocalAlign = 0.79 + 0.16 + 0.005 * 2;
    public static final Translation2d robotXY = new Translation2d(robotWidth / 2, robotLength / 2);
  }

  public class CameraSettings {

    public static final Matrix<N3, N1> defaultDriveStandardDev = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> defaultVisionStandardDev = VecBuilder.fill(2, 2, 9999999);

    public static final double bumpVisionStdDevFactor = 0.1;

    // TRANSFORM IS RELATIVE TO TURRET
    public static final CameraConstant frankTurretCamera = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(Units.inchesToMeters(6.05), 0, 0), new Rotation3d()), 1,
        "Arducam_OV2311_USB_Camera", "TurretCamera");

    public static final CameraConstant frankOdometryCamera = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(Units.inchesToMeters(2.6375), Units.inchesToMeters(-14.075), Units
            .inchesToMeters(7.875)), new Rotation3d(0, -17 * Math.PI / 180,
                -Math.PI / 2)/* .rotateBy(new Rotation3d(73 * Math.PI / 180, 0, 0)) */),
        1, "Park", "Right Reef Camera");
  }

  public static class WarningThresholdConstants {

    public static final double maxVortexMotorCurrent = 90;
    public static final double maxNeoMotorCurrent = 80;
    public static final double maxNeo550MotorCurrent = 70;
    public static final double maxMotorTemp = 60; // in degrees celcius
    public static final double minBatteryVoltage = 12.1;
  }
}

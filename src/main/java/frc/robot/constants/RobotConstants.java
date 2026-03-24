package frc.robot.constants;

import java.util.List;

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

public class RobotConstants {

  public class RobotInformation {
    // change it for running autons to prime25

    public static final RobotType robot = RobotTypes.prime26;
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
    public static final SubsystemInfo climberSubsystem = new SubsystemInfo("Climber");

    // ROBOTS
    public static final RobotType prime26 = new RobotType("Prime26", driveSubsystem, hoodSubsystem,
        intakeRollerSubsystem, turretSubsystem, shooterSubsystem, spindexerSubsystem, kickerSubsystem,
        intakeSubsystem).addAprilTagCamera(CameraConstants.primeLeftCamera)
            .addAprilTagCamera(CameraConstants.primeRightCamera)
            .addAprilTagCamera(CameraConstants.primeBackCamera);
    public static final RobotType duex26 = new RobotType("Duex26", driveSubsystem, kickerSubsystem,
        spindexerSubsystem, intakeSubsystem, intakeRollerSubsystem, turretSubsystem, shooterSubsystem,
        hoodSubsystem).addAprilTagCamera(CameraConstants.duexLeftCamera);

    public static final RobotType frank25 = new RobotType("Frank25", driveSubsystem);
    public static final RobotType prime25 = new RobotType("Prime25", driveSubsystem);
  }

  public enum TestingSetting {
    none, sysid, pid, pit, motor, shotControl
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

  public class AutonConstants {
    public static final List<String> excludedAutons = List.of("BUMP Depot Side Shoot + Climb",
        "BUMP Outpost Side Shoot + HP", "Bump Start Outpost + Climb", "Bump Depot Start",
        "Depot Side Shoot + Climb", "Middle", "Test", "Outpost Side Shoot + Climb", "Single Shoot + Outpost",
        "Outpost Full + Double Sweep");
  }
  public static final double zoneSwitchingHysteresis = 0.5; // in meters, how far into the next zone the robot needs
  // to be before we switch setpoint zones

  public class CameraConstants {
    /**
     * Default standard deviation vector for drive x, y, and theta.
     */
    public static final Matrix<N3, N1> defaultDriveStandardDev = VecBuilder.fill(0.1, 0.1, 0.1);
    /**
     * Default standard deviation vector for vision x, y, theta.
     */
    public static final Matrix<N3, N1> defaultVisionStandardDev = VecBuilder.fill(2, 2, 9999999);

    /**
     * Factor by which to reduce vision standard deviations to correct for bump
     * error.
     */
    public static final double bumpVisionStdDevFactor = 0.1;

    /*------------------
    | CAMERA CONSTANTS |
    ------------------*/
    /**
     * Odometry camera underneath Frank climber, for testing with Frank.
     */
    public static final CameraConstant frankOdometryCamera = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(Units.inchesToMeters(2.6375), Units.inchesToMeters(-14.075), Units
            .inchesToMeters(7.875)), new Rotation3d(0, -17 * Math.PI / 180,
                -Math.PI / 2)/* .rotateBy(new Rotation3d(73 * Math.PI / 180, 0, 0)) */),
        1, "Park", "Right Reef Camera");

    /**
     * Right deux camera.
     */
    public static final CameraConstant deuxRightCamera = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(Units.inchesToMeters(-3.7), Units.inchesToMeters(-13.57),
            Units.inchesToMeters(8.875)), new Rotation3d(0, -Math.PI / 9, -Math.PI / 2)),
        1, "Arducam_OV2311_USB_Camera", "Deux Right Camera");

    /**
     * Back deux camera, mounted on turret base.
     */
    public static final CameraConstant deuxBackCamera = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(-Units.inchesToMeters(13.123), -Units.inchesToMeters(10.075),
            Units.inchesToMeters(11.443)), new Rotation3d(0, -Math.PI / 9, Math.PI)),
        1, "Dodds", "Deux Back Camera");

    /**
     * Right deux camera, mounted on turret base.
     */
    public static final CameraConstant duexLeftCamera = new CameraConstant(new SimCameraProperties(),
        new Transform3d(
            new Translation3d(-Units.inchesToMeters(7.073), Units.inchesToMeters(12.342),
                Units.inchesToMeters(9.591)),
            new Rotation3d(0, -Units.degreesToRadians(20), Math.PI / 2)),
        1, "PC_Camera", "Deux Left Camera");

    /**
     * Back prime camera, mounted beside turret base.
     */
    public static final CameraConstant primeBackCamera = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(-0.313055, 0.173355, 0.263525),
            new Rotation3d(-Units.degreesToRadians(2), -Math.PI / 9, Math.PI)),
        1, "PC_Camera", "Prime Back Camera");

    /**
     * Left prime camera, mounted on turret base.
     */
    public static final CameraConstant primeLeftCamera = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(-0.17798, 0.32398, 0.25),
            new Rotation3d(0, -Units.degreesToRadians(20), Math.PI / 2)),
        1, "Markward", "Prime Left Camera");

    /**
     * Right prime camera, mounted on turret base.
     */
    public static final CameraConstant primeRightCamera = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(-0.09498, -0.32448, 0.244),
            new Rotation3d(0, -Units.degreesToRadians(15), -Math.PI / 2)),
        1, "Arducam_OV9281_USB_Camera (1)", "Prime Right Camera");

  }

  public static class WarningThresholdConstants {

    public static final double maxVortexMotorCurrent = 90;
    public static final double maxNeoMotorCurrent = 80;
    public static final double maxNeo550MotorCurrent = 70;
    public static final double maxMotorTemp = 60; // in degrees celcius
    public static final double minBatteryVoltage = 12.1;
  }
}

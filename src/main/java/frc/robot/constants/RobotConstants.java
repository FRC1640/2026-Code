package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.WPICal.AprilTagPositionSwitcher.AprilTagSetting;
import frc.robot.util.robotswitcher.RobotType;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;

public class RobotConstants {

  public class RobotInformation {
    // change it for running autons to prime25
    public static final RobotType robot = RobotTypes.duex26;
  }

  public class RobotTypes {

    public static final RobotType duex26 = new RobotType("Duex26", Subsystems.driveSubsystem,
        Subsystems.hopperSubsystem, Subsystems.indexerSubsystem, Subsystems.deflectorSubsystem,
        Subsystems.flywheelSubsystem, Subsystems.turretSubsystem, Subsystems.intakeSubsystem);

    public static final RobotType frank25 = new RobotType("Frank25", Subsystems.driveSubsystem,
        Subsystems.turretSubsystem);
    public static final RobotType prime25 = new RobotType("Prime25", Subsystems.driveSubsystem);
  }

  public class Subsystems {

    public static final SubsystemInfo driveSubsystem = new SubsystemInfo("Drive");
    public static final SubsystemInfo hopperSubsystem = new SubsystemInfo("Hopper");
    public static final SubsystemInfo indexerSubsystem = new SubsystemInfo("Indexer");

    public static final SubsystemInfo deflectorSubsystem = new SubsystemInfo("Deflector");
    public static final SubsystemInfo flywheelSubsystem = new SubsystemInfo("Flywheel");
    public static final SubsystemInfo turretSubsystem = new SubsystemInfo("Turret");
    public static final SubsystemInfo intakeSubsystem = new SubsystemInfo("Intake");

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
  }

  public static class WarningThresholdConstants {

    public static final double maxVortexMotorCurrent = 90;
    public static final double maxNeoMotorCurrent = 80;
    public static final double maxNeo550MotorCurrent = 70;
    public static final double maxMotorTemp = 60; // in degrees celcius
    public static final double minBatteryVoltage = 12.1;
  }
}

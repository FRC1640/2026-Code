package frc.robot.constants;

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

public class RobotConstants {
  public enum TestingSetting {
    none, sysid, pid, pit,
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

    // TRANSFORM IS RELATIVE TO TURRET
    public static final CameraConstant turretCameraConstant = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(0, Units.inchesToMeters(6.05), 0), new Rotation3d()), 1,
        "Arducam_OV2311_USB_Camera", "TurretCamera");

    public static final CameraConstant reefCameraRight = new CameraConstant(new SimCameraProperties(),
        new Transform3d(new Translation3d(Units.inchesToMeters(2.6375), Units.inchesToMeters(-14.075),
            Units.inchesToMeters(7.875)), new Rotation3d(0, 73 * Math.PI / 180, -Math.PI / 2)),
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

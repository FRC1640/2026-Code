package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.util.FeedForwardConstants;
import frc.robot.util.logging.PID.PIDStorage;
import frc.robot.util.logging.PPID.PPIDStorage;
import frc.robot.util.robotswitcher.Switchable;

public class RobotPIDConstants {
  /*---------------
  | PID CONSTANTS |
  ---------------*/

  public static final PIDConstants drivePid = new PIDConstants(0.17189, 0.0, 0);
  public static final FeedForwardConstants driveFF = new FeedForwardConstants(0.12506, 2, 0.27879);
  public static final PIDConstants steerPid = new PIDConstants(0.725, 0.0, 0.005);
  public static final PIDConstants intakeAnglePidReal = Switchable.of(new PIDConstants(3.5, 0, 0))
      .addAlt(RobotTypes.prime26, new PIDConstants(0, 0, 0)).get();
  public static final FeedForwardConstants intakeFFReal = Switchable.of(new FeedForwardConstants(0, 0, 0)).get();
  public static final PIDConstants intakeHoldPidReal = Switchable.of(new PIDConstants(14, 0, 0))
      .addAlt(RobotTypes.prime26, new PIDConstants(0, 0, 0)).get();
  public static final PIDConstants intakePPIDReal = Switchable.of(new PIDConstants(7, 0, 0))
      .addAlt(RobotTypes.prime26, new PIDConstants(0, 0, 0)).get();
  public static final Constraints intakeAngleConstraintsReal = Switchable.of(new Constraints(Math.PI, Math.PI)).get();
  public static final PIDConstants turretAnglePidReal = Switchable.of(new PIDConstants(5.5, 0, 0))
      .addAlt(RobotTypes.prime26, new PIDConstants(0, 0, 0)).get();
  public static final PIDConstants turretProfiledPidReal = Switchable.of(new PIDConstants(3.4, 0, 0))
      .addAlt(RobotTypes.prime26, new PIDConstants(0, 0, 0)).get();
  public static final Constraints turretAngleConstraintsReal = Switchable
      .of(new Constraints(4 * Math.PI, 6 * Math.PI)).get();
  public static final FeedForwardConstants turretAngleFF = Switchable.of(new FeedForwardConstants(0.236, 1.098, 0))
      .addAlt(RobotTypes.prime26, new FeedForwardConstants(0, 0, 0)).get();

  public static final PIDConstants intakeSim = new PIDConstants(0.2, 0, 0.005);
  public static final Constraints intakeAngleConstraintsSim = new Constraints(Math.PI, Math.PI);
  public static final FeedForwardConstants intakeFFSim = new FeedForwardConstants(0, 0.01984, 0);
  public static final PIDConstants rollerSim = new PIDConstants(0, 0, 0);
  public static final PIDConstants turretAnglePidSim = new PIDConstants(0.6, 0, 0);
  public static final PIDConstants turretVelocityPidSim = new PIDConstants(0.06, 0, 0);
  public static final PIDConstants shooterVelocityPidSim = new PIDConstants(0.005, 0, 0);
  public static final FeedForwardConstants shooterVelocityFFSim = new FeedForwardConstants(0, 0.02, 0);
  public static final PIDConstants hoodAnglePidSim = new PIDConstants(0.2, 0, 0.005);
  public static final PIDConstants hoodVelocityPidSim = new PIDConstants(0.1, 0, 0);
  public static final PIDConstants kickerVelocityPidSim = new PIDConstants(0.1, 0, 0);
  public static final PIDConstants climberPidSim = new PIDConstants(0.2, 0, 0.005);

  // IMPORTED FOR LOCAL ALIGN
  public static final PIDConstants localTagAlign = new PIDConstants(1.1, 0.005, 0.005);
  public static final PIDConstants localTagAlignVelocity = new PIDConstants(0.25, 0, 0);
  // public static final PIDConstants localTagAlignY = new PIDConstants(0.25, 0,
  // 0);
  public static final PIDConstants localAnglePid = new PIDConstants(0.85, 0, 0);
  public static final PIDConstants localDriveProfiledPid = new PIDConstants(0.5, 0, 0);
  public static final PIDConstants rotateToAnglePIDRadians = new PIDConstants(0.5, 0.001, 0.0001);
  public static final PIDConstants linearDrivePID = new PIDConstants(0.25, 0, 0);
  public static final PIDConstants linearDrivePIDProfiled = new PIDConstants(0.1, 0, 0);

  // DriveToPoint
  public static final PIDConstants autoDrivePID = new PIDConstants(5, 0, 0);
  public static final PIDConstants autoTurnPID = new PIDConstants(5, 0, 0.1);

  // PATHPLANNER AUTO CONSTANTS
  public static final PIDConstants pathplannerTranslationPid = new PIDConstants(3.6, 0.0, 0.0);
  public static final PIDConstants pathplannerRotationPid = new PIDConstants(5.0, 0.0, 0.0);

  /*-----------------------
  * CONSTRUCTION HELPERS |
  *---------------------*/
  public static final PIDController constructPID(PIDConstants constants) {
    PIDController j = new PIDController(constants.kP, constants.kI, constants.kD);
    PIDStorage.addPID(j);
    return j;
  }

  public static final PIDController constructPID(PIDConstants constants, boolean logEnabled) {
    PIDController j = new PIDController(constants.kP, constants.kI, constants.kD);
    if (logEnabled) {
      PIDStorage.addPID(j);
    }
    return j;
  }

  public static final PIDController constructPID(PIDConstants constants, String name) {
    PIDController j = new PIDController(constants.kP, constants.kI, constants.kD);
    return j;
  }

  public static final PIDController constructPID(PIDConstants constants, String name, boolean logEnabled) {
    PIDController j = new PIDController(constants.kP, constants.kI, constants.kD);
    if (logEnabled) {
      PIDStorage.addPID(name, j);
    }
    return j;
  }

  public static final SimpleMotorFeedforward constructFFSimpleMotor(FeedForwardConstants constants) {
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
    // FeedForwardTrack.feedTrack.add(feedforward);
    // FeedForwardTrack.idName.add("SimpleMotorFeedForward" +
    // FeedForwardTrack.feedTrack.size());
    return feedforward;
  }

  public static final SimpleMotorFeedforward constructFFSimpleMotor(FeedForwardConstants constants, String name) {
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
    // FeedForwardTrack.feedTrack.add(feedforward);
    // FeedForwardTrack.idName.add(name);
    return feedforward;
  }

  public static final ProfiledPIDController constructProfiledPIDController(PIDConstants pidConstants,
      TrapezoidProfile.Constraints constraints) {
    ProfiledPIDController k = new ProfiledPIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD,
        constraints, 0.02);
    PPIDStorage.addPID(k);
    return k;
  }

  public static final ProfiledPIDController constructProfiledPIDController(PIDConstants pidConstants,
      TrapezoidProfile.Constraints constraints, boolean logEnabled) {
    ProfiledPIDController k = new ProfiledPIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD,
        constraints, 0.02);
    if (logEnabled) {
      PPIDStorage.addPID(k);
    }
    return k;
  }

  public static final ProfiledPIDController constructProfiledPIDController(PIDConstants pidConstants,
      TrapezoidProfile.Constraints constraints, String name) {
    ProfiledPIDController k = new ProfiledPIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD,
        constraints, 0.02);
    PPIDStorage.addPID(name, k);
    return k;
  }

  public static final ProfiledPIDController constructProfiledPIDController(PIDConstants pidConstants,
      TrapezoidProfile.Constraints constraints, String name, boolean logEnabled) {
    ProfiledPIDController k = new ProfiledPIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD,
        constraints, 0.02);
    if (logEnabled) {
      PPIDStorage.addPID(name, k);
    }
    return k;
  }

  public static final ElevatorFeedforward constructFFElevator(FeedForwardConstants constants) {
    ElevatorFeedforward k = new ElevatorFeedforward(constants.kS, constants.kG, constants.kV, constants.kA);
    // ElevatorFeedForwardTrack.elevatorFeedTrack.add(k);
    // ElevatorFeedForwardTrack.idName.add(
    // "ElevatorFeedForward" + (ElevatorFeedForwardTrack.elevatorFeedTrack.size()));
    return k;
  }

  public static final MAXMotionConfig constructMaxMotionPos(PIDConstants constant) {
    MAXMotionConfig config = new MAXMotionConfig();

    config.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    return config;
  }
}

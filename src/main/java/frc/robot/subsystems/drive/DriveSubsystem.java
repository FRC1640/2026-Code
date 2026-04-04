package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.drive.DriveConstants.PivotId;
import frc.robot.subsystems.module.Module;
import frc.robot.subsystems.module.ModuleIO;
import frc.robot.subsystems.module.ModuleIOReal;
import frc.robot.subsystems.module.ModuleIOSim;
import frc.robot.subsystems.module.ModuleInfo;
import frc.robot.util.sysid.SwerveDriveSysidRoutine;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class DriveSubsystem extends SubsystemPlatform {

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  public RobotConfig config;
  public Gyro gyro;
  public SysIdRoutine sysIdRoutine;
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;
  public static final Lock odometryLock = new ReentrantLock();
  public Rotation2d totalRot = new Rotation2d();
  private FollowPath.Builder pathBuilder;

  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = RobotTypes.driveSubsystem;

  public DriveSubsystem(Gyro gyro) {
    super(info);

    this.gyro = gyro;

    modules[0] = new Module(DriveSubsystem.getIOByMode(DriveConstants.FL), PivotId.FL);
    modules[1] = new Module(DriveSubsystem.getIOByMode(DriveConstants.FR), PivotId.FR);
    modules[2] = new Module(DriveSubsystem.getIOByMode(DriveConstants.BL), PivotId.BL);
    modules[3] = new Module(DriveSubsystem.getIOByMode(DriveConstants.BR), PivotId.BR);

    // custom format
    sysIdRoutine =
        new SwerveDriveSysidRoutine()
            .createNewRoutine(
                modules[0],
                modules[1],
                modules[2],
                modules[3],
                this,
                new SysIdRoutine.Config(
                    Volts.per(Seconds).of(2),
                    Volts.of(7),
                    Seconds.of(5),
                    (state) -> Logger.recordOutput("SysIdTestState", state.toString())));
    // spotless format

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = null;
    }
    setpointGenerator = new SwerveSetpointGenerator(config, // The robot configuration. This is the same config used
        // for generating
        // trajectories and running path following commands.
        DriveConstants.maxSteerSpeed);
    previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getActualSwerveStates(), DriveFeedforwards.zeros(4));
  }

  public void configureBLine() {
    FollowPath.setBooleanLoggingConsumer((pair) -> Logger.recordOutput(pair.getFirst(), pair.getSecond()));
    FollowPath.setDoubleLoggingConsumer((pair) -> Logger.recordOutput(pair.getFirst(), pair.getSecond()));
    FollowPath.setPoseLoggingConsumer((pair) -> Logger.recordOutput(pair.getFirst(), pair.getSecond()));
    FollowPath.setTranslationListLoggingConsumer((pair) -> Logger.recordOutput(pair.getFirst(), pair.getSecond()));
    this.pathBuilder = new FollowPath.Builder((SubsystemBase) this, () -> RobotOdometry.instance.getPose("Main"),
        this::getChassisSpeeds, (speeds) -> runVelocity(speeds, false, 3, () -> false),
        new PIDController(4.5, 0.0, 1.4), new PIDController(3.0, 0.0, 0.0), new PIDController(2.0, 0.0, 1.0))
            .withDefaultShouldFlip().withPoseReset((pose) -> {
              CommandScheduler.getInstance().schedule(new InstantCommand(() -> {
                RobotOdometry.instance.resetGyro(pose.getRotation());
                RobotOdometry.instance.setPose("Main", pose);
              }));
            });
  }

  public FollowPath.Builder getPathBuilder() {
    return this.pathBuilder;
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    for (var module : modules) {
      module.periodic();
    }
    gyro.periodic();
    odometryLock.unlock();

    double totalDriveCurrent = 0;
    double totalSteerCurrent = 0;
    for (Module module : modules) {
      totalDriveCurrent += module.getDriveCurrent();
      totalSteerCurrent += module.getSteerCurrent();
    }
    Logger.recordOutput("Subsystems/Drive/totalDriveCurrent", totalDriveCurrent);
    Logger.recordOutput("Subsystems/Drive/totalSteerCurrent", totalSteerCurrent);
  }

  private void stop() {
    for (Module m : modules) {
      m.stop();
    }
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  public SwerveModuleState[] getActualSwerveStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(getActualSwerveStates());
  }

  public double chassisSpeedsMagnitude() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/VelocityAngle")
  public Rotation2d chassisSpeedsAngle() {
    ChassisSpeeds speeds = getChassisSpeeds();
    if (chassisSpeedsMagnitude() < 0.001) {
      return new Rotation2d();
    }
    return new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(gyro.getAngleRotation2d());
  }

  public Module[] getModules() {
    return modules;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  public void runVelocity(ChassisSpeeds speeds, boolean fieldCentric, double dreamLevel,
      BooleanSupplier limitSpeeds) {

    double scale = 1;
    ChassisSpeeds percent = new ChassisSpeeds(speeds.vxMetersPerSecond / DriveConstants.maxSpeed,
        speeds.vyMetersPerSecond / DriveConstants.maxSpeed,
        speeds.omegaRadiansPerSecond / DriveConstants.maxOmega);

    ChassisSpeeds doubleCone = inceptionMode(percent, new Translation2d(), dreamLevel);
    ChassisSpeeds speedsOptimized = fieldCentric
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(doubleCone.vxMetersPerSecond * DriveConstants.maxSpeed * scale,
                doubleCone.vyMetersPerSecond * DriveConstants.maxSpeed * scale,
                doubleCone.omegaRadiansPerSecond * DriveConstants.maxOmega * scale),
            gyro.getAngleRotation2d())
        : new ChassisSpeeds(doubleCone.vxMetersPerSecond * DriveConstants.maxSpeed * scale,
            doubleCone.vyMetersPerSecond * DriveConstants.maxSpeed * scale,
            doubleCone.omegaRadiansPerSecond * DriveConstants.maxOmega * scale);

    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, // The previous setpoint
        speedsOptimized, // The desired target speeds
        0.02 // The loop time of the robot code, in seconds
    );
    Logger.recordOutput("Drive/SwerveStates/SetpointStates", previousSetpoint.moduleStates());
    Logger.recordOutput("Drive/SwerveStates/Input", DriveConstants.kinematics.toSwerveModuleStates(speeds));
    Logger.recordOutput("Drive/SwerveStates/DoubleCone",
        DriveConstants.kinematics.toSwerveModuleStates(speedsOptimized));
    // speedsOptimized = ChassisSpeeds.discretize(speedsOptimized, 0.02);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredStateMetersPerSecond(previousSetpoint.moduleStates()[i]);
      // DriveConstants.kinematics.toSwerveModuleStates(speedsOptimized)[i]);
    }
  }

  private void setSteerPosition(Rotation2d rotation) {
    for (int i = 0; i < 4; i++) {
      modules[i].setSteerPosition(rotation);
    }
  }

  private boolean areModulesAtRotations(Rotation2d rotation) {
    for (int i = 0; i < 4; i++) {
      if (!MathUtil.isNear(rotation.getRadians(), modules[i].getPosition().angle.getRadians(),
          Units.degreesToRadians(25))) {
        return false;
      }
    }
    return true;
  }

  private void setSteerVoltages(double voltage) {
    for (int i = 0; i < 4; i++) {
      modules[i].setSteerVoltage(0);
    }
  }

  public Command setSteerPositionCommand(Rotation2d rotation2d) {
    return new RunCommand(() -> setSteerPosition(rotation2d)).until(() -> areModulesAtRotations(rotation2d))
        .andThen(() -> setSteerVoltages(0));
  }

  public static ChassisSpeeds inceptionMode(ChassisSpeeds speedsPercent, Translation2d centerOfRotation,
      double dreamLevel) {

    double xSpeed = speedsPercent.vxMetersPerSecond;
    double ySpeed = speedsPercent.vyMetersPerSecond;
    double rot = speedsPercent.omegaRadiansPerSecond;
    double translationalSpeed = Math.hypot(xSpeed, ySpeed);
    double linearRotSpeed = Math.abs(rot * computeMaxNorm(DriveConstants.positions, centerOfRotation));
    double k;
    if (linearRotSpeed == 0 || translationalSpeed == 0) {
      k = 1;
    } else {
      k = Math.pow(Math.max(linearRotSpeed, translationalSpeed) / (linearRotSpeed + translationalSpeed),
          dreamLevel);
    }
    return new ChassisSpeeds(k * xSpeed, k * ySpeed, k * rot);
  }

  public static double computeMaxNorm(Translation2d[] translations, Translation2d centerOfRotation) {
    return Arrays.stream(translations).map((translation) -> translation.minus(centerOfRotation))
        .mapToDouble(Translation2d::getNorm).max()
        .orElseThrow(() -> new NoSuchElementException("No max norm."));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public Command runVelocityCommand(Supplier<ChassisSpeeds> speeds, BooleanSupplier limitSpeeds) {
    return new RunCommand(() -> runVelocity(speeds.get(), true, 3, limitSpeeds), this).finallyDo(() -> stop());
  }

  public Consumer<ChassisSpeeds> runVelocityConsumer() {
    return (speeds) -> runVelocity(speeds, true, 3, () -> false);
  }

  public static ModuleIO getIOByMode(ModuleInfo modInfo) {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info)) {
      return new ModuleIO() {

      };
    }
    return switch (Robot.getMode()) {
      case REAL -> new ModuleIOReal(modInfo);
      case SIM -> new ModuleIOSim(modInfo);
      case REPLAY -> new ModuleIO() {
      };
    };
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return Commands.none();
  }
}

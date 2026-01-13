// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.util.logging.PID.PIDLog;
import frc.robot.util.logging.PPID.PPIDLog;
import frc.robot.util.periodic.PeriodicScheduler;

public class Robot extends LoggedRobot {
  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum RobotState {
    DISABLED,
    AUTONOMOUS,
    TELEOP,
    TEST
  }

  private static RobotState state = RobotState.DISABLED;

  public static RobotState getState() {
    return state;
  }

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    // Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("RobotMode", getMode().toString());
    // Logger.recordMetadata("MACAddress", getMACAddress());
    // switch (BuildConstants.DIRTY) {
    // case 0:
    // Logger.recordMetadata("GitDirty", "All changes committed");
    // break;
    // case 1:
    // Logger.recordMetadata("GitDirty", "Uncomitted changes");
    // break;
    // default:
    // Logger.recordMetadata("GitDirty", "Unknown");
    // break;
    // }
    System.out.println(getMode().toString());
    // Set up data receivers & replay source
    switch (getMode()) {
      // Running on a real robot, log to a USB stick
      case REAL:
        LoggedPowerDistribution.getInstance(21, ModuleType.kRev);
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

        break;

      // Running a physics simulator, log to local folder
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter("logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        break;
    }

    // Start AdvantageKit Logger
    Logger.start();
    Logger.registerURCL(URCL.startExternal());
    m_robotContainer = new RobotContainer();

    WebServer.start(
        5800,
        Filesystem.getDeployDirectory()
            .getPath()); // instructed to add to get elastic config to load automatically
  }

  @Override
  public void robotInit() {
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    // PPIDLog.log();
    // PIDLog.log();
    CommandScheduler.getInstance().run();
    PeriodicScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    state = RobotState.DISABLED;
    DriveWeightCommand.removeAllWeights();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    state = RobotState.AUTONOMOUS;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    state = RobotState.TELEOP;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    state = RobotState.TEST;
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  public static boolean isReplay() {
    // if (RobotConfigConstants.robotType == RobotType.Replay) {
    // return true; // TODO put back if using robotswitch
    // }
    String replay = System.getProperty("REPLAY");
    return replay != null && replay.equalsIgnoreCase("true");
  }

  public static Mode getMode() {
    if (isReal()) {
      return Mode.REAL;
    }
    if (isReplay()) {
      return Mode.REPLAY;
    }

    return Mode.SIM;
  }
}

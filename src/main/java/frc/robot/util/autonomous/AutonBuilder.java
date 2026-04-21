package frc.robot.util.autonomous;

import java.util.HashMap;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class AutonBuilder {

  private static AutonBuilder instance;

  public record Auton(Command command, RobotCommands robotCommands) {
    public Auton(Command command, RobotCommands robotCommands) {
      this.robotCommands = robotCommands;
      this.command = // Commands.sequence(
          // robotCommands.setSteerPositionCommand(
          // firstPath != null ? firstPath.getInitialModuleDirection() :
          // Rotation2d.fromDegrees(180)),
          (command).finallyDo(AutonBuilder.getInstance().autoEndCallback);

    }
  }

  public final HashMap<String, Auton> autons = new HashMap<String, Auton>();

  public final Runnable autoEndCallback;

  // subscriber for dashboard wait time
  private final DoubleSubscriber autoWaitSub;

  public AutonBuilder(RobotCommands robotCommands, FollowPath.Builder pathBuilder, Runnable autoEndCallback) {
    AutonBuilder.instance = this;
    this.autoEndCallback = autoEndCallback;

    // connect to /Autonomous/WaitTime
    NetworkTable autoTable = NetworkTableInstance.getDefault().getTable("Autonomous");
    autoWaitSub = autoTable.getDoubleTopic("WaitTime").subscribe(0.0);

    // custom format

    /*--------
    | AUTONS |
    --------*/

    // None
    autons.put("None", new Auton(Commands.none(), robotCommands));

    // custom format
    /*
     * autons.put("OTrench 1Sweep Depot",
     * new Auton(Commands.sequence(
     * pathBuilder.build(new Path("dt1sde1"))), robotCommands));
     */ // spotless format

    // custom format
    /*
     * autons.put("OTrench 1Sweep Outpost",
     * new Auton(Commands.sequence(
     * pathBuilder.build(new Path("ot1sou1"))), robotCommands));
     */ // spotless format

    autons.put("Center Outpost Depot", new Auton(Commands.sequence(pathBuilder.build(new Path("ceoude1")),
        new WaitCommand(2),
        new InstantCommand(
            () -> CommandScheduler.getInstance().schedule(robotCommands.autoOscillateCommand(65, 0))),
        pathBuilder.build(new Path("ceoude2"))), robotCommands));

    autons.put("OTrench 2Sweep OBump",
        new Auton(
            Commands.sequence(pathBuilder.build(new Path("ot2sob1")).finallyDo(
                () -> CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
            robotCommands));

    autons.put("DTrench 2Sweep DBump",
        new Auton(
            Commands.sequence(pathBuilder.build(new Path("dt2sdb1")).finallyDo(
                () -> CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
            robotCommands));

    // TODO: add autons here!!!! MAKE SURE YOU PRESERVE THE HOOD AND PROPER
    // SHOOTERIDLE USE.

    // spotless format

  }

  public static AutonBuilder getInstance() {
    return instance;
  }

  // safely get wait time from dashboard
  private double getAutoWaitTime() {
    double wait = autoWaitSub.get();

    if (Double.isNaN(wait) || Double.isInfinite(wait)) {
      return 0.0;
    }

    if (wait < 0.0)
      wait = 0.0;
    if (wait > 15.0)
      wait = 15.0;

    return wait;
  }

  // wrap selected auton with wait first, then run auto
  public Command wrapSelectedAuton(Command selectedAuton) {
    Command autonToRun = selectedAuton != null ? selectedAuton : Commands.none();
    double waitTime = getAutoWaitTime();

    System.out.println("Auto wait time: " + waitTime + " seconds");

    return Commands.sequence(
        new WaitCommand(waitTime),
        autonToRun);
  }
}
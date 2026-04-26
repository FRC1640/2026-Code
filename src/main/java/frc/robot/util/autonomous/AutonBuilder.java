package frc.robot.util.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.util.helpers.AllianceManager;

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

    // autons.put("Center Outpost + Depot", new
    // Auton(Commands.sequence(pathBuilder.build(new Path("collect_outpost")),
    // new WaitCommand(2),
    // new InstantCommand(
    // () ->
    // CommandScheduler.getInstance().schedule(robotCommands.autoOscillateCommand(65,
    // 0))),
    // pathBuilder.build(new Path("outpost_depot"))), robotCommands));

    autons.put("Outpost 2Sweep",
        new Auton(
            Commands.sequence(pathBuilder.build(new Path("outpost_2sweep")).finallyDo(
                () -> CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
            robotCommands));

    autons.put("Depot 2Sweep",
        new Auton(
            Commands.sequence(pathBuilder.build(new Path("depot_2sweep")).finallyDo(
                () -> CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
            robotCommands));

    autons.put("Depot Trench/OP Pair", new Auton(
        Commands.sequence(
            pathBuilder.build(new Path("hub_trench_route")),
            new WaitCommand(0.0),
            pathBuilder.build(new Path("hub_intake_return")),
            pathBuilder.build(new Path("collect_depot"))),
        robotCommands));

    Path outposthtrPath = new Path("hub_trench_route");
    outposthtrPath.mirror();
    Path outposthirPath = new Path("hub_intake_return");
    outposthirPath.mirror();

    autons.put("Outpost Trench/OP Pair", new Auton(
        Commands.sequence(
            pathBuilder.build(outposthtrPath),
            new WaitCommand(0.0),
            pathBuilder.build(outposthirPath),
            pathBuilder.build(new Path("collect_outpost"))),
        robotCommands));

    autons.put("Depot Bump Pair", new Auton(
        Commands.sequence(
            pathBuilder.build(new Path("hub_trench_route")),
            new WaitCommand(3.0),
            pathBuilder.build(new Path("depot_hub_trench_sweep")),
            pathBuilder.build(new Path("hub_trench_sweep2"))),
        robotCommands));

    Path outposthts2Path = new Path("hub_trench_sweep2");
    outposthts2Path.mirror();

    autons.put("Outpost Bump Pair", new Auton(
        Commands.sequence(
            pathBuilder.build(outposthtrPath),
            new WaitCommand(3.0),
            pathBuilder.build(new Path("outpost_hub_trench_sweep")),
            pathBuilder.build(outposthts2Path)),
        robotCommands));

    autons.put("Straight To Depot", new Auton(
        pathBuilder.build(new Path("collect_depot")),
        robotCommands));

    // TODO: test the following: Center 1Sweep Depot, Center 2Sweep Depot, Center
    // 1Sweep Outpost, Center 2Sweep Outpost, Straight To Outpost, Straight To
    // Depot, Depot 2Sweep, Outpost 2Sweep

    // add autons here!!!! MAKE SURE YOU PRESERVE THE HOOD AND PROPER
    // SHOOTERIDLE USE.

    // spotless format

  }

  public static AutonBuilder getInstance() {
    return instance;
  }

  // safely get wait time from dashboard
  private double getAutoWaitTime() {
    double wait = autoWaitSub.get();

    if (Double.isNaN(wait) || Double.isInfinite(wait) || (wait < 0.0)) {
      return 0.0;
    }

    if (wait > 15.0)
      wait = 15.0;

    return wait;
  }

  // wrap selected auton with wait first, then run auto
  public Command wrapSelectedAuton(Command selectedAuton) {
    Command autonToRun = selectedAuton != null ? selectedAuton : Commands.none();
    double waitTime = getAutoWaitTime();

    System.out.println("Auto wait time: " + waitTime + " seconds");

    return Commands.sequence(new WaitCommand(waitTime), autonToRun);
  }
}

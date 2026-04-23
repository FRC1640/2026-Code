package frc.robot.util.autonomous;

import java.util.HashMap;

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

  public AutonBuilder(RobotCommands robotCommands, FollowPath.Builder pathBuilder, Runnable autoEndCallback) {
    AutonBuilder.instance = this;
    this.autoEndCallback = autoEndCallback;

    // custom format

    /*--------
    | AUTONS |
    --------*/

    // None
    autons.put("None", new Auton(Commands.none(), robotCommands));

    // Double Sweep Depot: Trench -> Sweep -> Hub -> Shoot for 8 seconds -> Sweep ->
    // Hub -> Shoot for 8 seconds
    // custom format
    /*
    autons.put("Double Sweep Depot",
        new Auton(
            Commands.sequence(
                pathBuilder.build(new Path("depot ds 1")),
                new InstantCommand(() -> CommandScheduler.getInstance().schedule(
                    robotCommands.autoShootCommand()
                        .alongWith(robotCommands.autoOscillateCommand(0.75))
                        .withTimeout(8))),
                new WaitCommand(8),
                pathBuilder.build(new Path("depot ds 2")),
                new InstantCommand(() -> CommandScheduler.getInstance().schedule(
                    robotCommands.autoShootCommand()
                        .alongWith(robotCommands.autoOscillateCommand(0.75))
                        .withTimeout(8))),
                new WaitCommand(8)), robotCommands));
    */ // spotless format

    // custom format
    /*
    autons.put("Double Sweep Outpost",
        new Auton(
            Commands.sequence(
                pathBuilder.build(new Path("outpost ds 1")),
                new InstantCommand(() -> CommandScheduler.getInstance().schedule(
                    robotCommands.autoShootCommand()
                        .alongWith(robotCommands.autoOscillateCommand(0.75))
                        .withTimeout(8))),
                new WaitCommand(8),
                pathBuilder.build(new Path("outpost ds 2")),
                new InstantCommand(() -> CommandScheduler.getInstance().schedule(
                    robotCommands.autoShootCommand()
                        .alongWith(robotCommands.autoOscillateCommand(0.75))
                        .withTimeout(8))),
                new WaitCommand(8)), robotCommands));
    */ // spotless format

    // custom format
    /*
    autons.put("Pass the Ball Bro Depot",
        new Auton(Commands.sequence(
            pathBuilder.build(new Path("depot ssf 1"))), robotCommands));
    */ // spotless format

    // custom format
    /*
    autons.put("Pass the Ball Bro Outpost",
        new Auton(Commands.sequence(
            pathBuilder.build(new Path("outpost ssf 1"))), robotCommands));
    */ // spotless format

    autons.put("Center", new Auton(Commands.sequence(pathBuilder.build(new Path("center 1")), new WaitCommand(2),
        new InstantCommand(
            () -> CommandScheduler.getInstance().schedule(robotCommands.autoOscillateCommand(65, 0))),
        pathBuilder.build(new Path("center 2"))), robotCommands));

    // 2056 Outpost: Trench -> Sweep -> Bump -> S.W.I.M -> Trench -> Sweep -> Bump
    // -> S.W.I.M -> Trench
    // custom format
    /*
    autons.put("2056 Outpost",
        new Auton(
            Commands.sequence(
                pathBuilder.build(new Path("outpost alt dss 1")).finallyDo(() -> CommandScheduler.getInstance().schedule(
                    robotCommands.setSwerveToZeroCommand()))), robotCommands));
    */ // spotless format

    autons.put("Outpost SWIM Double Sweep",
        new Auton(
            Commands.sequence(pathBuilder.build(new Path("outpost dss 1")).finallyDo(
                () -> CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
            robotCommands));

    // custom format
    /*
    autons.put("2056 Depot",
        new Auton(
            Commands.sequence(
                pathBuilder.build(new Path("depot alt dss 1")).finallyDo(() -> CommandScheduler.getInstance().schedule(
                    robotCommands.setSwerveToZeroCommand()))), robotCommands));
    */ // spotless format

    autons.put("Depot SWIM Double Sweep",
        new Auton(
            Commands.sequence(pathBuilder.build(new Path("outpost dss 1flip")).finallyDo(
                () -> CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
            robotCommands));

    autons.put("Outpost Third Robot",
        new Auton(Commands.sequence(pathBuilder.build(new Path("perpendicular_hub_intake_wait")),
            pathBuilder.build(new Path("hub_intake_return")),
            pathBuilder.build(new Path("collect_depot"))), robotCommands));

    // Double Sweep Outpost: Trench -> Sweep -> Hub -> Shoot for 8 seconds -> Sweep
    // -> Hub -> Shoot for 8 seconds
    // autons.put("Double Sweep Outpost",
    // new Auton(
    // Commands.sequence(
    // Commands.deadline(robotCommands.waitForShotCommand(true),
    // pathBuilder.build(new Path("double sweep outpost 1"))),
    // Commands.parallel(
    // robotCommands.autoShootCommand().withTimeout(6),
    // robotCommands.autoOscillateCommand(0.75),
    // Commands.deadline(robotCommands.waitForShotCommand(true),
    // pathBuilder.build(new Path("double sweep outpost 2"))),
    // Commands.parallel(
    // robotCommands.autoShootCommand().withTimeout(6),
    // robotCommands.autoOscillateCommand(0.75)))),
    // new Path("double sweep outpost 1"), robotCommands));

    // TODO: add autons here!!!! MAKE SURE YOU PRESERVE THE HOOD AND PROPER
    // SHOOTERIDLE USE.

    // spotless format

  }

  public static AutonBuilder getInstance() {
    return instance;
  }
}

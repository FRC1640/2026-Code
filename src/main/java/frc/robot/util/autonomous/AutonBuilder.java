package frc.robot.util.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class AutonBuilder {

  private static AutonBuilder instance;

  public record Auton(Command command, Path firstPath) {
    public Auton(Command command, Path firstPath) {
      this.firstPath = firstPath;
      this.command = command;
    }
  }

  public final HashMap<String, Auton> autons = new HashMap<String, Auton>();

  public AutonBuilder(RobotCommands robotCommands, FollowPath.Builder pathBuilder) {
    AutonBuilder.instance = this;

    // custom format

    /*--------
    | AUTONS |
    --------*/

    // None
    autons.put("None", new Auton(Commands.none(), null));

    // Example: Preload -> Near Hub -> Shoot for 8 seconds -> Outpost
    autons.put("Example", new Auton(
        Commands.sequence(
            // robotCommands.setSteerPositionCommand(new
            // Path("e1").getInitialModuleDirection()),
            pathBuilder.build(new Path("e1")),
            Commands.deadline(
                new WaitCommand(8),
                robotCommands.autoShootCommand()),
            pathBuilder.build(new Path("e2"))),
        new Path("e1")));

    // Leave: Leave Starting Line
    autons.put("Leave", new Auton(
        Commands.sequence(
            // robotCommands.setSteerPositionCommand(new
            // Path("l1").getInitialModuleDirection()),
            pathBuilder.build(new Path("l1"))),
        new Path("l1")));

    autons.put("Double Sweep Outpost",
        new Auton(
            Commands.sequence(
              pathBuilder.build(new Path("outpost sweep 2")),
              pathBuilder.build(new Path("outpost sweep 3")),
              robotCommands.autoShootCommand().withTimeout(6)
            ),
            new Path("outpost sweep 2")
          )
        );

    // TODO: add autons here!!!!

    // spotless format

  }

  public static AutonBuilder getInstance() {
    return instance;
  }
}

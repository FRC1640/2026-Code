package frc.robot.util.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class AutonBuilder {

  private RobotCommands robotCommands;
  private FollowPath.Builder pathBuilder;

  public static final HashMap<String, Command> autons = new HashMap<String, Command>();

  public AutonBuilder(RobotCommands robotCommands, FollowPath.Builder pathBuilder) {
    this.robotCommands = robotCommands;
    this.pathBuilder = pathBuilder;

    // custom format
    autons.put("None", Commands.none());
    autons.put("Example", 
      Commands.sequence(
        pathBuilder.build(new Path("ExamplePath1")),
        Commands.deadline(
          new WaitCommand(4),
          robotCommands.shootCommand()
        ),
        pathBuilder.build(new Path("ExamplePath2"))
    ));
    //TODO: add autons here!!!!

    // spotless format

  }
}

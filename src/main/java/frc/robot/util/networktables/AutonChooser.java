package frc.robot.util.networktables;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonChooser {
  private SendableChooser<String> dropdown = new SendableChooser<String>();

  public AutonChooser() {
    autonInit();
  }

  // the string[] is now in RobotConstants.Autons

  private void autonInit() {
    dropdown.setDefaultOption("None", "None");

    Path dir = Paths.get(Filesystem.getDeployDirectory() + "/pathplanner/autos");

    try (Stream<Path> walk = Files.list(dir)) {

      List<String> fileNames = walk.filter(Files::isRegularFile).map(Path::getFileName).map(Path::toString)
          .collect(Collectors.toList());

      System.out.println("Files in Pathplanner Auto Folder:");

      for (String autonName : fileNames) {
        dropdown.addOption(autonName, autonName);
        System.out.println(autonName);
      }

    } catch (IOException e) {
      System.err.println("An error occurred while accessing Autos");
    }

    SmartDashboard.putData("Choose Auton", dropdown);
  }

  public Command getAuto() {
    String selected = dropdown.getSelected();
    if (selected.equals("None")) {
      return Commands.none();
    }
    return AutoBuilder.buildAuto(selected);
  }

  public String getString() {
    return dropdown.getSelected();
  }
}

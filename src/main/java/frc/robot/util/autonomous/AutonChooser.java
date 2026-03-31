package frc.robot.util.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;

public class AutonChooser {

  private SendableChooser<String> dropdown = new SendableChooser<String>();

  public AutonChooser() {
    autonInit();
  }

  private void autonInit() {
    dropdown.setDefaultOption("None", "None");

    for (String autonName : AutonBuilder.getInstance().autons.keySet()) {
      dropdown.addOption(autonName, autonName);
      System.out.println(autonName);
    }

    SmartDashboard.putData("Choose Auton", dropdown);

  }

  public Command getAuto() {
    String selected = dropdown.getSelected();
    if (selected.equals("None")) {
      return Commands.none();
    }

    return AutonBuilder.getInstance().autons.get(selected).command();
  }

  public Path getFirstPath() {
    String selected = dropdown.getSelected();
    if (selected.equals("None")) {
      return null;
    }

    return AutonBuilder.getInstance().autons.get(selected).firstPath();
  }

  public String getString() {
    return dropdown.getSelected();
  }
}

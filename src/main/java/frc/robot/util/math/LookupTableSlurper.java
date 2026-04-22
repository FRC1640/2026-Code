package frc.robot.util.math;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;

public class LookupTableSlurper {
  public enum LookupTableType {
    PRIME26AZ, PRIME26NZ, DEUX26AZ, DEUX26NZ
  }

  public static HashMap<LookupTableType, String> FileMap = new HashMap<LookupTableType, String>() {
    {
      put(LookupTableType.PRIME26AZ, "prime26az.csv");
      put(LookupTableType.PRIME26NZ, "prime26nz.csv");
      put(LookupTableType.DEUX26AZ, "deux26az.csv");
      put(LookupTableType.DEUX26NZ, "deux26nz.csv");
    }
  };

  public static ShotInterpolator slurpShotInterpolator(LookupTableType tableType) {
    ShotInterpolator interpolator = new ShotInterpolator();

    String filePath = FileMap.get(tableType);

    try {
      String content = Files.readString(Paths.get(filePath));
      String[] lines = content.split("\\R"); // handles all line endings

      for (String line : lines) {
        if (line.isEmpty())
          continue;

        String[] values = line.split(",");
        double distance = Double.parseDouble(values[0].trim());
        double hoodAngle = Double.parseDouble(values[1].trim());
        double shooterVelocity = Double.parseDouble(values[2].trim());
        double timeOfFlight = Double.parseDouble(values[3].trim());

        interpolator.put(distance, hoodAngle, shooterVelocity, timeOfFlight);
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
    return interpolator;
  }

}

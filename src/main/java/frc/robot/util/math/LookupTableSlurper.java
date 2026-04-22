package frc.robot.util.math;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collector;
import java.util.stream.Collectors;

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
      Files.readAllLines(Paths.get(filePath))
          .stream()
          .map(line -> line.split(","))
          .collect(Collectors.toList())
          .forEach(values -> {
            double distance = Double.parseDouble(values[0]);
            double hoodAngle = Double.parseDouble(values[1]);
            double shooterVelocity = Double.parseDouble(values[2]);
            double timeOfFlight = Double.parseDouble(values[3]);
            interpolator.put(distance, hoodAngle, shooterVelocity, timeOfFlight);

          });
    } catch (IOException e) {
      e.printStackTrace();
    }
    return interpolator;
  }

}

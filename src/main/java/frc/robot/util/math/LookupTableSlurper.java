package frc.robot.util.math;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;

public class LookupTableSlurper {
  public enum LookupTableType {
    PRIME26AZ, PRIME26NZ, DEUX26AZ, DEUX26NZ
  }

  public static HashMap<LookupTableType, String> FileMap = new HashMap<LookupTableType, String>() {
    {
      put(LookupTableType.PRIME26AZ, "src/main/resources/lookuptables/prime26az.csv");
      put(LookupTableType.PRIME26NZ, "src/main/resources/lookuptables/prime26nz.csv");
      put(LookupTableType.DEUX26AZ, "src/main/resources/lookuptables/deux26az.csv");
      put(LookupTableType.DEUX26NZ, "src/main/resources/lookuptables/deux26nz.csv");
    }
  };

  public static ShotInterpolator slurpShotInterpolator(LookupTableType tableType) {
    ShotInterpolator interpolator = new ShotInterpolator();

    String line = "";
    String filePath = FileMap.get(tableType);

    try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
      while ((line = br.readLine()) != null) {

        String[] values = line.split(",");

        double distance = Double.parseDouble(values[0]);
        double hoodAngle = Double.parseDouble(values[1]);
        double shooterVelocity = Double.parseDouble(values[2]);
        double timeOfFlight = Double.parseDouble(values[3]);
        interpolator.put(distance, hoodAngle, shooterVelocity, timeOfFlight);

      }
    } catch (IOException e) {
      e.printStackTrace();
    }
    return interpolator;
  }

}

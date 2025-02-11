// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final CTREConfigs Configs = new CTREConfigs();
  private static final RobotConfig robotConfig;
  static {
    RobotConfig tempConfig = null;
      try {
          tempConfig = RobotConfig.fromGUISettings();
      } catch (Exception e) {
          // Handle exception as needed
          tempConfig = null;
          e.printStackTrace();
      }
      robotConfig = tempConfig;
  }
  
  public static final class ElevatorConstants {
    // Trapezoidal Profile Variables
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double MAX_VELOCITY = 0.0;
    public static final double MAX_ACCEL = 0.0;

    // Height Setpoints (meters)
    public static final double HEIGHT_OF_INTAKE = 0.0;
    public static final double MAX_HEIGHT = 0.0 - HEIGHT_OF_INTAKE;
  }

  public static final class WristConstants {
    // Trapezoidal Profile Variables
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double MAX_VELOCITY = 0.0;
    public static final double MAX_ACCEL = 0.0;

    // Setpoints (rotations)
    public static final double MAX_HEIGHT = 0.0;
  }

  public static RobotConfig getPathPlannerConfig() {
    return robotConfig;
  }
}

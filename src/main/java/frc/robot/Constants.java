// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Constants for the drivetrain
   */
  public static class DriveConstants {
    public static final int kDriverControllerPort = 0;
    public static final int leftLeadDeviceID = 0;
    public static final int rightLeadDeviceID = 0;

    public static final int driveMotorsCurrentLimit = 50;
    public static final double driveMotorsRampRate = 0.125;

    public static final double defaultSpeed = 1.0;
    public static final double slowSpeed = 0.3;
    public static final double slowForward = 0.5;
  }

  /**
   * Constants for the operator controller
   */
  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 1;

    public static final int rightTriggerRawAxis = 2;
    public static final int leftTriggerRawAxis = 3;
  }

  /**
   * Constants for the driver controller
   */
  public static class DriverConstants {
    public static final int selectControllerbutton = 7;
    public static final int startControllerButton = 8;
  }

  /**
   * Constants for autonomous
   */
  public static class AutoConstants {
    public static final double startingPosition = 0;
    public static final double distanceForward = 0;
    public static final double driveTolerance = 0.025;
  }
}
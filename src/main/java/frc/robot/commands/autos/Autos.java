// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public final class Autos {

  /** Simple  auto that drives forward a constant distance */
  public static CommandBase driveForwardAuto(TankDriveSubsystem driveSubsystem) {
    return new DriveForwardAuto(driveSubsystem, AutoConstants.distanceForward, false);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSubsystem;


/** Only used for autos that require you to drive forward a set distance */
public class DriveForwardAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TankDriveSubsystem driveSubsystem;
  private double distance;
  private boolean finished;
  private boolean slow;

  /**
   * Tells the robot to drive a set distance.
   *
   * @param subsystem The subsystem used by this command.
   * @param distance The distance to drive forward.
   * @param slow Boolean to tell it to drive slower.
   */
  public DriveForwardAuto(TankDriveSubsystem subsystem, double distance, boolean slow) {
    driveSubsystem = subsystem;
    this.distance = distance;
    this.slow = slow;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = driveSubsystem.driveForward(distance, slow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finished)
      return true;
    return false;
  }
}
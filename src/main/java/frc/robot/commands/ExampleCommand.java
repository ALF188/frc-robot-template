// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command to control the tank drive */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  // Create variables:
    ExampleSubsystem exampleSubsystem;
  /**
   * Creates an {@link ExampleSubsystem}
   *
   * @link ExampleSubsystem.java
   */
  public ExampleCommand(ExampleSubsystem subsystem) {
    exampleSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    // Must do for all subsystems passed to this function
    addRequirements(subsystem);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    exampleSubsystem.exampleFunction();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  // Returning false every time causes the susbsystem to run in the background
  @Override
  public boolean isFinished() {
    return false;
  }
}
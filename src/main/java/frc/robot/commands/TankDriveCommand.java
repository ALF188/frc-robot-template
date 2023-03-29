// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TankDriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command to control the tank drive */
public class TankDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TankDriveSubsystem driveSubsystem;
  private DoubleSupplier forward;
  private DoubleSupplier rotation;
  private boolean slow;

  /**
   * Controls the tank drive using axis from the driver controller.
   *
   * @param subsystem The subsystem used by this command.
   * @param forward The speed forward (from joystick)
   * @param rotation The speed to rotate (from joystick)
   * @param slow Boolean if the robot should turn and drive slower
   */
  public TankDriveCommand(TankDriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation, boolean slow) {
    driveSubsystem = subsystem;
    this.forward = forward;
    this.rotation = rotation;
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
    driveSubsystem.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble(), slow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
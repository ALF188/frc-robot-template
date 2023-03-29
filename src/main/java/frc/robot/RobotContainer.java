// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autos.Autos;
import frc.robot.subsystems.*;

public class RobotContainer {
  // Define subsystems
  private final TankDriveSubsystem driveSubsystem;

  // SmartDashboard autonomous selector
  private SendableChooser<Command> autoChooser;

  // Xbox Controller for command based 
  private final CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.kDriverControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initalize subystems
    driveSubsystem = new TankDriveSubsystem();

    // Set default commands
    driveSubsystem.setDefaultCommand(
      new TankDriveCommand(driveSubsystem, () -> -driverController.getLeftY(), () -> -driverController.getRightX(), false));

    // Create the SmartDashboard autonomous selector
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Forward Auto", Autos.driveForwardAuto(driveSubsystem));
    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Sets up triggers for button clicks
    // Example:
    // driverController.rightBumper().onTrue(new ExampleCommand(driveSubsystem));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

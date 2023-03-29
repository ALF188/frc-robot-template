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
  // The robot's subsystems and commands are defined here...
  private final TankDriveSubsystem driveSubsystem;

  //SmartDashboard autonomous selector
  private SendableChooser<Command> autoChooser;

  //Xbox Controller for command based 
  private final CommandXboxController driverController =
      new CommandXboxController(ControllerConstants.kDriverControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem = new TankDriveSubsystem();

    driveSubsystem.setDefaultCommand(
      new TankDriveCommand(driveSubsystem, () -> -driverController.getLeftY(), () -> -driverController.getRightX(), false));


    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Forward Auto", Autos.driveForwardAuto(driveSubsystem));
    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}

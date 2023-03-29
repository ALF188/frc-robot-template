// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class TankDriveSubsystem extends SubsystemBase {

  // Motors for wheels and tank drive
  CANSparkMax leftLeadMotor;
  CANSparkMax rightLeadMotor;

  DifferentialDrive roboDrive;

  // AHRS is a navx gyro (not necessary)
  private AHRS gyro;

  // PID for autos driving
  private PIDController drivePid;

  // Used for slow turning
  public double turnMultiplier;
  public double forwardMultiplier;

  public double currentHeading;

  /** Creates a new tank drive. */
  public TankDriveSubsystem() {
    // Initalize the wheel motors
    leftLeadMotor = new CANSparkMax(DriveConstants.leftLeadDeviceID, MotorType.kBrushless);
    rightLeadMotor = new CANSparkMax(DriveConstants.rightLeadDeviceID, MotorType.kBrushless);

    // Reset to default
    leftLeadMotor.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();

    // Set one inverted so they work together
    leftLeadMotor.setInverted(true);

    // Don't let the motors brownout easily
    leftLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);

    // Ramp rates
    leftLeadMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    rightLeadMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);

    // Initalize the tank drive using motors
    roboDrive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);
    
    // Reset the relative motor encoders
    leftLeadMotor.getEncoder().setPosition(0);
    rightLeadMotor.getEncoder().setPosition(0);

    // Sets the length per rotation
    leftLeadMotor.getEncoder().setPositionConversionFactor(1);
    rightLeadMotor.getEncoder().setPositionConversionFactor(1);

    // Initalize the gyro
    gyro = new AHRS();

    currentHeading = gyro.getYaw();

    // Sets the multipliers to 1
    turnMultiplier = DriveConstants.defaultSpeed;
    forwardMultiplier = DriveConstants.defaultSpeed;

    // PID used for autos driving
    drivePid = new PIDController(3, 0.5, 0.5);
  }

  /**
   * Sets motor speed from driver's stick input
   *
   * @param forward Input from left stick y axis.
   * @param rotation Input from right stick x axis.
   * @param slowTurn Drive forward at a slower set multiplier.
   */
  public void arcadeDrive(double forward, double rotation, Boolean slowTurn){
    if(slowTurn){
      turnMultiplier = DriveConstants.slowSpeed;
      forwardMultiplier = DriveConstants.slowForward;
    }

    roboDrive.arcadeDrive(forward * forwardMultiplier, rotation * turnMultiplier);

    turnMultiplier = DriveConstants.defaultSpeed;
    forwardMultiplier = DriveConstants.defaultSpeed;
  }

  /**
   * Drive forward a set distance
   * Used only in autos
   * 
   * @param length Distance to drive.
   * @param slowMode Boolean to drive and turn slower.
   */
  public boolean driveForward(double length, boolean slowMode){
    if(slowMode){
      forwardMultiplier = DriveConstants.slowForward;
    }

    // MathUtil.clamp limits the output to a slower speed per run time    
    roboDrive.arcadeDrive(MathUtil.clamp(drivePid.calculate(leftLeadMotor.getEncoder().getPosition(), length), -0.75, 0.75) * forwardMultiplier, 0);

    forwardMultiplier = DriveConstants.defaultSpeed;

    if(leftLeadMotor.getEncoder().getPosition() > length - AutoConstants.driveTolerance && leftLeadMotor.getEncoder().getPosition() < length + AutoConstants.driveTolerance){
      return true;
    }

    return false;
  }

  /**
   * Turn a set degree
   * Used only in autos
   * 
   * @param turnDegrees The degrees to turn
   * @param slowMode Boolean to drive and turn slower.
   */
  public boolean turn(double turnDegrees, boolean slowMode){
    if(slowMode){
      turnMultiplier = DriveConstants.slowForward;
    }

    double newHeading = currentHeading + turnDegrees;

    roboDrive.arcadeDrive(0, MathUtil.clamp(drivePid.calculate(gyro.getYaw(), newHeading), -.75, .75) * turnMultiplier);

    turnMultiplier = DriveConstants.defaultSpeed;

    if(gyro.getYaw() > newHeading - AutoConstants.turnToleranceForYaw && gyro.getYaw() < newHeading + AutoConstants.turnToleranceForYaw){
      return true;
    }
    
    return false;
    
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {   
    SmartDashboard.putNumber("Gyro Pitch: ", gyro.getPitch());
    SmartDashboard.putNumber("Gyro Roll: ", gyro.getRoll());
    SmartDashboard.putNumber("Gyro Yaw: ", gyro.getYaw());

    SmartDashboard.putNumber("Drive Encoder:" , leftLeadMotor.getEncoder().getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class TankDriveSubsystem extends SubsystemBase {

  CANSparkMax leftLeadMotor;
  CANSparkMax rightLeadMotor;

  DifferentialDrive roboDrive;

  private AHRS ahrs;

  private PIDController drivePid;

  public double turnMultiplier;
  public double forwardMultiplier;

  public double currentHeading;

  /** Creates a new ExampleSubsystem. */
  public TankDriveSubsystem() {
    leftLeadMotor = new CANSparkMax(DriveConstants.leftLeadDeviceID, MotorType.kBrushless);
    rightLeadMotor = new CANSparkMax(DriveConstants.rightLeadDeviceID, MotorType.kBrushless);

    leftLeadMotor.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();

    leftLeadMotor.setInverted(true);

    leftLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);

    leftLeadMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    rightLeadMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);

    roboDrive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

    leftLeadMotor.getEncoder().setPosition(0);
    rightLeadMotor.getEncoder().setPosition(0);

    //Sets the length per rotation
    leftLeadMotor.getEncoder().setPositionConversionFactor(1);
    rightLeadMotor.getEncoder().setPositionConversionFactor(1);

    ahrs = new AHRS();

    currentHeading = ahrs.getYaw();

    turnMultiplier = DriveConstants.defaultSpeed;
    forwardMultiplier = DriveConstants.defaultSpeed;
    
    drivePid = new PIDController(3, 0.5, 0.5);
  }

  /**
   * Sets motor speed from driver's stick input
   *
   * @param forward Input from left stick y axis
   * @param rotation Input from right stick x axis
   * @param slowTurn Drive forward at a slower set multiplier
   * @return null
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

  public boolean driveForward(double length, boolean slowMode){
    if(slowMode){
      forwardMultiplier = DriveConstants.slowForward;
    }

    SmartDashboard.putNumber("Left Encoder:" , leftLeadMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Length: ", length);
    
    roboDrive.arcadeDrive(MathUtil.clamp(drivePid.calculate(leftLeadMotor.getEncoder().getPosition(), length), -0.75, 0.75) * forwardMultiplier, 0);

    forwardMultiplier = DriveConstants.defaultSpeed;

    if(leftLeadMotor.getEncoder().getPosition() > length - AutoConstants.driveTolerance && leftLeadMotor.getEncoder().getPosition() < length + AutoConstants.driveTolerance){
      return true;
    }

    return false;
  }

  public boolean turn(double turnDegrees, boolean slowMode){
    if(slowMode){
      turnMultiplier = DriveConstants.slowForward;
    }

    double newHeading = currentHeading + turnDegrees;

    roboDrive.arcadeDrive(0, MathUtil.clamp(drivePid.calculate(ahrs.getYaw(), newHeading), -.75, .75) * turnMultiplier);

    turnMultiplier = DriveConstants.defaultSpeed;

    if(ahrs.getYaw() > newHeading - AutoConstants.turnToleranceForYaw && ahrs.getYaw() < newHeading + AutoConstants.turnToleranceForYaw){
      return true;
    }
    
    return false;
    
  }

  @Override
  public void periodic() {   
    SmartDashboard.putNumber("Gyro Pitch: ", ahrs.getPitch());
    SmartDashboard.putNumber("Gyro Roll: ", ahrs.getRoll());
    SmartDashboard.putNumber("Gyro Yaw: ", ahrs.getYaw());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
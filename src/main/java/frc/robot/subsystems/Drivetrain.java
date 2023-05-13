// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DeviceIds;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  
  private final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(
    new Translation2d(DrivetrainConstants.trackWidthLength / 2.0, DrivetrainConstants.wheelBaseLength / 2.0),
    new Translation2d(DrivetrainConstants.trackWidthLength / 2.0, -DrivetrainConstants.wheelBaseLength / 2.0),
    new Translation2d(-DrivetrainConstants.trackWidthLength / 2.0, DrivetrainConstants.wheelBaseLength / 2.0),
    new Translation2d(-DrivetrainConstants.trackWidthLength / 2.0, -DrivetrainConstants.wheelBaseLength / 2.0)
  );

  private final SwerveModule frontLeft = Mk4iSwerveModuleHelper.createNeo(
    Mk4iSwerveModuleHelper.GearRatio.L2, 
    DeviceIds.frontLeftDrive, 
    DeviceIds.frontLeftTurn, 
    DeviceIds.frontLeftEncoder, 
    DeviceIds.frontLeftEncoderOffset
  );

  private final SwerveModule frontRight = Mk4iSwerveModuleHelper.createNeo(
    Mk4iSwerveModuleHelper.GearRatio.L2, 
    DeviceIds.frontRightDrive, 
    DeviceIds.frontRightTurn, 
    DeviceIds.frontRightEncoder, 
    DeviceIds.frontRightEncoderOffset
  );

  private final SwerveModule backLeft = Mk4iSwerveModuleHelper.createNeo(
    Mk4iSwerveModuleHelper.GearRatio.L2, 
    DeviceIds.backLeftDrive, 
    DeviceIds.backLeftTurn, 
    DeviceIds.backLeftEncoder, 
    DeviceIds.backLeftEncoderOffset
  );

  private final SwerveModule backRight = Mk4iSwerveModuleHelper.createNeo(
    Mk4iSwerveModuleHelper.GearRatio.L2, 
    DeviceIds.backRightDrive, 
    DeviceIds.backRightTurn, 
    DeviceIds.backRightEncoder, 
    DeviceIds.backRightEncoderOffset
  );

  private final ADIS16448_IMU gyro = new ADIS16448_IMU();
  
  /**
   * Creates a new drivetrain object. Represents a four corner swerve drive 
   * with odometry using encoder/Limelight fused data.
   */
  public Drivetrain() {}

  @Override
  public void periodic() {}

  /**
   * Retrives rotational data from the gyro.
   * @return Gyro reading on the robot in a Rotation2d object
   */
  public Rotation2d getGyroRotation(){
    var rotation = Rotation2d.fromDegrees(this.gyro.getGyroAngleX());
    SmartDashboard.putNumber("Heading", rotation.getRadians());
    return rotation;
  }

  /**
   * Resets the gyro to a rotation of 0.
   */
  public void resetGyro(){
    this.gyro.reset();
  }

  /**
   * Sets the drivetrain to the desired kinematic state
   * @param movementVector Movement vector <xVel, yVel, rotVel>, defined using ChassisSpeeds object
   * @param fieldOriented Whether or not the movement vector is field oriented.
   */
  public void setDrivetrainOutput(ChassisSpeeds movementVector, boolean fieldOriented){
    SwerveModuleState[] swerveModuleStates = drivetrainKinematics.toSwerveModuleStates(
      fieldOriented ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(movementVector, this.getGyroRotation()) :
      movementVector
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.AbsoluteMaxWheelVelocity);
    
    this.frontLeft.set(
      swerveModuleStates[0].speedMetersPerSecond / DrivetrainConstants.AbsoluteMaxWheelVelocity * DrivetrainConstants.MaxVoltage, 
      swerveModuleStates[0].angle.getRadians()
    );
    this.frontRight.set(
      swerveModuleStates[1].speedMetersPerSecond / DrivetrainConstants.AbsoluteMaxWheelVelocity * DrivetrainConstants.MaxVoltage, 
      swerveModuleStates[1].angle.getRadians()
    );
    this.backLeft.set(
      swerveModuleStates[2].speedMetersPerSecond / DrivetrainConstants.AbsoluteMaxWheelVelocity * DrivetrainConstants.MaxVoltage, 
      swerveModuleStates[2].angle.getRadians()
    );
    this.backRight.set(
      swerveModuleStates[3].speedMetersPerSecond / DrivetrainConstants.AbsoluteMaxWheelVelocity * DrivetrainConstants.MaxVoltage, 
      swerveModuleStates[3].angle.getRadians()
    );
  }
}

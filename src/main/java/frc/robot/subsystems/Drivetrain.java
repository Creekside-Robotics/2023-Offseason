// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightResults;
import frc.robot.Constants.DeviceIds;

import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(
      new Translation2d(DrivetrainConstants.trackWidthLength / 2.0, DrivetrainConstants.wheelBaseLength / 2.0),
      new Translation2d(DrivetrainConstants.trackWidthLength / 2.0, -DrivetrainConstants.wheelBaseLength / 2.0),
      new Translation2d(-DrivetrainConstants.trackWidthLength / 2.0, DrivetrainConstants.wheelBaseLength / 2.0),
      new Translation2d(-DrivetrainConstants.trackWidthLength / 2.0, -DrivetrainConstants.wheelBaseLength / 2.0));

  private final SwerveModule frontLeft = new MkSwerveModuleBuilder(
      MkModuleConfiguration.getDefaultSteerNEO())
      .withDriveMotor(MotorType.NEO, DeviceIds.frontLeftDrive)
      .withGearRatio(SdsModuleConfigurations.MK4I_L2)
      .withSteerMotor(MotorType.NEO, DeviceIds.frontLeftTurn)
      .withSteerEncoderPort(DeviceIds.frontLeftEncoder)
      .withSteerOffset(DeviceIds.frontLeftEncoderOffset)
      .build();

  private final SwerveModule frontRight = new MkSwerveModuleBuilder(
      MkModuleConfiguration.getDefaultSteerNEO())
      .withDriveMotor(MotorType.NEO, DeviceIds.frontRightDrive)
      .withGearRatio(SdsModuleConfigurations.MK4I_L2)
      .withSteerMotor(MotorType.NEO, DeviceIds.frontRightTurn)
      .withSteerEncoderPort(DeviceIds.frontRightEncoder)
      .withSteerOffset(DeviceIds.frontRightEncoderOffset)
      .build();

  private final SwerveModule backLeft = new MkSwerveModuleBuilder(
      MkModuleConfiguration.getDefaultSteerNEO())
      .withDriveMotor(MotorType.NEO, DeviceIds.backLeftDrive)
      .withGearRatio(SdsModuleConfigurations.MK4I_L2)
      .withSteerMotor(MotorType.NEO, DeviceIds.backLeftTurn)
      .withSteerEncoderPort(DeviceIds.backLeftEncoder)
      .withSteerOffset(DeviceIds.backLeftEncoderOffset)
      .build();

  private final SwerveModule backRight = new MkSwerveModuleBuilder(
      MkModuleConfiguration.getDefaultSteerNEO())
      .withDriveMotor(MotorType.NEO, DeviceIds.backRightDrive)
      .withGearRatio(SdsModuleConfigurations.MK4I_L2)
      .withSteerMotor(MotorType.NEO, DeviceIds.backRightTurn)
      .withSteerEncoderPort(DeviceIds.backRightEncoder)
      .withSteerOffset(DeviceIds.backRightEncoderOffset)
      .build();
  

  private final ADIS16448_IMU gyro = new ADIS16448_IMU();

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  /**
   * Creates a new drivetrain object. Represents a four corner swerve drive
   * with odometry using encoder/Limelight fused data.
   */
  public Drivetrain() {
    this.poseEstimator = new SwerveDrivePoseEstimator(
        drivetrainKinematics,
        getGyroRotation(),
        getModulePositions(),
        new Pose2d());
    SmartDashboard.putData("Field Display", field2d);
  }

  @Override
  public void periodic() {
    this.poseEstimator.update(getGyroRotation(), getModulePositions());
    this.updatePoseWithLimelight();
    this.displayDrivetrainPose();
  }

  /**
   * Retrives rotational data from the gyro.
   * 
   * @return Gyro reading on the robot in a Rotation2d object
   */
  public Rotation2d getGyroRotation() {
    var rotation = Rotation2d.fromDegrees(this.gyro.getGyroAngleX());
    SmartDashboard.putNumber("Heading", rotation.getRadians());
    return rotation;
  }

  /**
   * Updates pose estimator with latency compensated vision estimate. Should be
   * called periodically.
   */
  public void updatePoseWithLimelight() {
    LimelightResults results = LimelightHelpers.getLatestResults("limelight");
    if (results.targetingResults.valid && results.targetingResults.getBotPose2d_wpiBlue().getX() != 0) {
      Pose2d pose = results.targetingResults.getBotPose2d_wpiBlue();
      SmartDashboard.putNumber("Pose X", pose.getX());
      double timestamp = Timer.getFPGATimestamp() - results.targetingResults.latency_capture/1000.0 - results.targetingResults.latency_pipeline/1000.0;
      SmartDashboard.putNumber("Timestamp", timestamp);
      this.poseEstimator.addVisionMeasurement(pose, timestamp);
    }
  }

  /**
   * Resets the gyro to a rotation of 0.
   */
  public void resetGyro() {
    this.gyro.reset();
  }

  /**
   * Sets the drivetrain to the desired kinematic state
   * 
   * @param movementVector Movement vector <xVel, yVel, rotVel>, defined using
   *                       ChassisSpeeds object
   * @param fieldOriented  Whether or not the movement vector is field oriented.
   */
  public void setDrivetrainOutput(ChassisSpeeds movementVector, boolean fieldOriented) {
    SwerveModuleState[] swerveModuleStates = drivetrainKinematics.toSwerveModuleStates(
        fieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(movementVector, this.getGyroRotation()) : movementVector);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.AbsoluteMaxWheelVelocity);

    this.frontLeft.set(
        swerveModuleStates[0].speedMetersPerSecond / DrivetrainConstants.AbsoluteMaxWheelVelocity
            * DrivetrainConstants.MaxVoltage,
        swerveModuleStates[0].angle.getRadians());
    this.frontRight.set(
        swerveModuleStates[1].speedMetersPerSecond / DrivetrainConstants.AbsoluteMaxWheelVelocity
            * DrivetrainConstants.MaxVoltage,
        swerveModuleStates[1].angle.getRadians());
    this.backLeft.set(
        swerveModuleStates[2].speedMetersPerSecond / DrivetrainConstants.AbsoluteMaxWheelVelocity
            * DrivetrainConstants.MaxVoltage,
        swerveModuleStates[2].angle.getRadians());
    this.backRight.set(
        swerveModuleStates[3].speedMetersPerSecond / DrivetrainConstants.AbsoluteMaxWheelVelocity
            * DrivetrainConstants.MaxVoltage,
        swerveModuleStates[3].angle.getRadians());
  }

  /**
   * @return Array with swerve module positions, {frontLeft, frontRight, backLeft,
   *         backRight}
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        this.frontLeft.getPosition(),
        this.frontRight.getPosition(),
        this.backLeft.getPosition(),
        this.backRight.getPosition()
    };
  }

  /**
   * Resets the drivetrain pose estimator to the given pose.
   * 
   * @param pose The Pose2d object that the drivetrain will be set to.
   */
  public void setDrivetrainPose(Pose2d pose) {
    this.poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
  }

  /**
   * Gets the estimated pose of the robot on the field.
   * 
   * @return Pose2d object representing the robot on the field.
   */
  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  /**
   * Updates the display with the current drivetrain pose.
   */
  private void displayDrivetrainPose() {
    this.field2d.setRobotPose(getPose());
  }

  /**
   * Gets the closest pose to the robot from a array of poses
   * 
   * @param testPoses Pose2d[] containing poses to be tested.
   * @return The Pose2d in the array that is closest to the robot.
   */
  public Pose2d getClosestPose(Pose2d[] testPoses) {
    Pose2d robotPose = getPose();
    double closestDistance = 1000.0;
    int closestIndex = -1;

    for (int i = 0; i < testPoses.length; i++) {
      double distance = robotPose.getTranslation().getDistance(testPoses[i].getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestIndex = i;
      }
    }

    return testPoses[closestIndex];
  }
}

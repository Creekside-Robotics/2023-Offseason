// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.PIDConstants;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 * 
 * <p>
 * All constants entered should use the SI unit system (AKA commie units).
 */
public final class Constants {

  /**
   * All Ids are stored in the ids class
   * 
   * <p>
   * Use the following format: public static {deviceName} = {IdNumber};
   */
  public static class DeviceIds {
    public static int frontLeftDrive = 1;
    public static int frontLeftTurn = 2;
    public static int frontRightDrive = 3;
    public static int frontRightTurn = 4;
    public static int backLeftDrive = 5;
    public static int backLeftTurn = 6;
    public static int backRightDrive = 7;
    public static int backRightTurn = 8;
    public static int lowerArmForward = 10;
    public static int lowerArmBackward = 9;
    public static int upperArm = 11;
    public static int intakeForwardMotor = 12;
    public static int intakeReverseMotor = 13;

    public static int frontLeftEncoder = 1;
    public static int frontRightEncoder = 2;
    public static int backLeftEncoder = 3;
    public static int backRightEncoder = 4;

    public static int lowerArmEncoder = 0;
    public static int upperArmEncoder = 1;

    public static int clawForward = 1;
    public static int clawReverse = 0;
    public static int intakePistonForward = 2;
    public static int intakePistonReverse = 3;

    public static int driverController = 0;
    public static int alternateController = 1;
  }

  public static class ControllerConstants {
    public static int autoPickup = 1;
    public static int intake = 2;

    public static int firstScore = 11;
    public static int secondScore = 9;
    public static int thirdScore = 7;

    public static int firstAuto = 12;
    public static int secondAuto = 10;
    public static int thirdAuto = 8;

    public static int openClaw = 7;
    public static int closeClaw = 8;
    public static int indexObject = 11;
    public static int retractArms = 12;
    public static int extendIntake = 9;
    public static int retractIntake = 10;
  }

  public static class DrivetrainConstants {
    public static double wheelBaseLength = 0.6985;
    public static double trackWidthLength = 0.6985;

    public static double MaxVoltage = 12.0;
    public static double AbsoluteMaxWheelVelocity = 5880.0 / 60.0 *
        SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
    public static double AbsoluteMaxAngularVelocity = AbsoluteMaxWheelVelocity /
        Math.hypot(wheelBaseLength / 2.0, wheelBaseLength / 2.0);

    public static double driveCurrentLimit = 20.0;

    public static double maxTranslationalSpeed = 3.0;
    public static double maxRotationalSpeed = Math.PI;

    public static double translationControllerPGain = 6.0;
    public static double translationControllerDGain = 0.5;
    public static double rotationControllerPGain = 10.0;
    public static double rotationControllerDGain = 0.5;

    public static double frontLeftEncoderOffset = Math.toRadians(-354.46) + Math.PI;
    public static double frontRightEncoderOffset = Math.toRadians(-69.87) + Math.PI;
    public static double backLeftEncoderOffset = Math.toRadians(-212.34);
    public static double backRightEncoderOffset = Math.toRadians(-312.53);

    public static Vector<N3> stateStandardDeviation = VecBuilder.fill(0.02, 0.02, Math.PI / 30.0);
    public static Vector<N3> visionStandardDeviation = VecBuilder.fill(0.8, 0.8, Math.PI / 2.0);

    public static double pushAgainstWallSpeed = 0.15;
  }

  public static class LowerArmConstants {
    public static double encoderMultiplier = 1.0;
    public static double encoderOffset = -0.5;

    public static double proportionalGain = 5.0;
    public static double derivativeGain = 0.0;

    public static double maxSpeed = 0.8;
    public static double maxAcceleration = 3.0;

    public static double tolerance = 0.01;
  }

  public static class UpperArmConstants {
    public static double encoderMultiplier = -1.0;
    public static double encoderOffset = -0.31;

    public static double proportionalGain = 5.0;
    public static double derivativeGain = 0.0;

    public static double maxSpeed = 0.8;
    public static double maxAcceleration = 3.0;

    public static double tolerance = 0.01;
  }

  public static class ArmPositions {
    public static double lowerThree = 0.255;
    public static double upperThree = -0.166;

    public static double lowerTwo = 0.062;
    public static double upperTwo = 0.065;

    public static double lowerOne = -0.1108;
    public static double upperOne = 0.1130;

    public static double lowerIntake = -0.170;
    public static double upperIntake = 0.020;

    public static double lowerStowed = -0.248;
    public static double upperStowed = 0.175;

    public static double lowerPickup = 0.055;
    public static double upperPickup = 0.060;

    public static double upperTransitional = 0.175;
  }

  public static class IntakeConstants {
    public static double intakeSpeed = 0.75;
    public static double climbSpeed = 0.75;
    public static double climbTime = 1;
    public static double dropSpeed = -0.75;
    public static double dropTime = 0.75;
  }

  public static class ClawConstants {
    public static double openingDelay = 0.25;
  }

  public static class AutoConstants {
    public static double maxVelocity = 2;
    public static double maxAcceleration = 1.5;

    public static PIDConstants translationConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static PIDConstants rotationConstants = new PIDConstants(5.0, 0.0, 0.0);
  }

  public static abstract class FieldConstants {
    public Pose2d[] gridPositions;
    public Pose2d[] substationPositions;
  }

  public static class BlueFieldConstants extends FieldConstants {
    public BlueFieldConstants() {
      this.gridPositions = new Pose2d[] {
          new Pose2d(1.82, 0.49, new Rotation2d(Math.PI)),
          new Pose2d(1.82, 1.08, new Rotation2d(Math.PI)),
          new Pose2d(1.82, 1.64, new Rotation2d(Math.PI)),
          new Pose2d(1.82, 2.20, new Rotation2d(Math.PI)),
          new Pose2d(1.82, 2.75, new Rotation2d(Math.PI)),
          new Pose2d(1.82, 3.32, new Rotation2d(Math.PI)),
          new Pose2d(1.82, 3.87, new Rotation2d(Math.PI)),
          new Pose2d(1.82, 4.43, new Rotation2d(Math.PI)),
          new Pose2d(1.82, 4.98, new Rotation2d(Math.PI))
      };

      this.substationPositions = new Pose2d[] {
          new Pose2d(0.00, 6.00, new Rotation2d()),
          new Pose2d(0.00, 7.34, new Rotation2d())
      };
    }
  }

  public static class RedFieldConstants extends FieldConstants {
    public RedFieldConstants() {
      this.gridPositions = new Pose2d[] {
          new Pose2d(14.70, 0.49, new Rotation2d()),
          new Pose2d(14.70, 1.08, new Rotation2d()),
          new Pose2d(14.70, 1.64, new Rotation2d()),
          new Pose2d(14.70, 2.20, new Rotation2d()),
          new Pose2d(14.70, 2.75, new Rotation2d()),
          new Pose2d(14.70, 3.32, new Rotation2d()),
          new Pose2d(14.70, 3.87, new Rotation2d()),
          new Pose2d(14.70, 4.43, new Rotation2d()),
          new Pose2d(14.70, 4.98, new Rotation2d())
      };

      this.substationPositions = new Pose2d[] {
          new Pose2d(0.00, 6.00, new Rotation2d(Math.PI)),
          new Pose2d(0.00, 7.34, new Rotation2d(Math.PI))
      };
    }
  }

  public static Map<DriverStation.Alliance, FieldConstants> fieldConstantsMap = Map.of(
      DriverStation.Alliance.Blue, new BlueFieldConstants(),
      DriverStation.Alliance.Red, new RedFieldConstants());
}

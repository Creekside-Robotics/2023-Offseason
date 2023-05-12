// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * <p>All constants entered should use the SI unit system (AKA commie units).
 */
public final class Constants {

    /**
     * All Ids are stored in the ids class
     * 
     * <p>Use the following format: public static {deviceName} = {IdNumber};
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

        public static int frontLeftEncoder = 1;
        public static int frontRightEncoder = 2;
        public static int backLeftEncoder = 3;
        public static int backRightEncoder = 4;

        public static double frontLeftEncoderOffset = Math.toRadians(-354.46) + Math.PI;
        public static double frontRightEncoderOffset = Math.toRadians(-69.87) + Math.PI;
        public static double backLeftEncoderOffset = Math.toRadians(-212.34);
        public static double backRightEncoderOffset = Math.toRadians(-312.53);

        public static int driverController = 0;
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

        public static double maxTranslationalSpeed = 3.0;
        public static double maxRotationalSpeed = Math.PI/2;
    }
}

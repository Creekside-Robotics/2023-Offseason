// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriverController;

public class DriveToNearestSubstationAxis extends DriveToPosePID {

    /**
     * Creates a new command which drives to the nearest grid position.
     * 
     * @param drivetrain
     * @param driverController
     */
    public DriveToNearestSubstationAxis(Drivetrain drivetrain, DriverController driverController) {
        super(
                drivetrain,
                driverController,
                () -> drivetrain.getClosestPose(
                        Constants.fieldConstantsMap.get(DriverStation.getAlliance()).substationPositions),
                new boolean[] { false, true, true },
                new Pose2d(0.01, 0.01, new Rotation2d(0.01)),
                true);
    }
}
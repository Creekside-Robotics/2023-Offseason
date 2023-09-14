// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.arms.MoveArmsToState;
import frc.robot.commands.drivetrain.SetDrivetrainOutput;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;

public class ThirdLevelScore extends ParallelDeadlineGroup {

    /**
     * Creates a commmand which scores on the third level.
     * 
     * @param drivetrain
     * @param lowerArm
     * @param upperArm
     */
    public ThirdLevelScore(Drivetrain drivetrain, LowerArm lowerArm, UpperArm upperArm) {
        super(
                new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerThree, ArmPositions.upperThree),
                new SetDrivetrainOutput(drivetrain, new ChassisSpeeds(DrivetrainConstants.pushAgainstWallSpeed, 0, 0), false));
    }
}

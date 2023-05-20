// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.composite;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.arms.MoveArmsToState;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.drivetrain.SetDrivetrainOutput;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThirdLevelScore extends ParallelDeadlineGroup {

    /**
     * Creates a commmand which scores on the third level.
     * 
     * @param drivetrain
     * @param lowerArm
     * @param upperArm
     * @param claw
     */
    public ThirdLevelScore(Drivetrain drivetrain, LowerArm lowerArm, UpperArm upperArm, Claw claw) {
        // Add the deadline command in the super() call. Add other commands using
        // addCommands().
        super(
                new SequentialCommandGroup(
                        new MoveArmsToState(lowerArm, upperArm, ArmPositions.lowerThree, ArmPositions.upperThree),
                        new OpenClaw(claw)),
                new SetDrivetrainOutput(drivetrain, new ChassisSpeeds(0.05, 0, 0), false));
    }
}

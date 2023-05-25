// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class RetractIntake extends SequentialCommandGroup {

  /**
   * Creates a new command to retract the intake on the robot.
   * 
   * @param intake Intake to retract.
   */
  public RetractIntake(Intake intake) {
    addCommands(
        new SetIntakeMode(intake, IntakeConstants.climbSpeed, false),
        new WaitCommand(IntakeConstants.climbTime),
        new SetIntakeMode(intake, 0, false));
  }
}

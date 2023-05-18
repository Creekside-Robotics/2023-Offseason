// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmsToState extends SequentialCommandGroup {
  /** Creates a new MoveArmToState. */
  public MoveArmsToState(Arm lower, Arm upper, double lowerPosition, double upperPosition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveToPosition(upper, ArmPositions.upperTransitional, true),
      new MoveToPosition(lower, lowerPosition, true),
      new MoveToPosition(upper, upperPosition, true)
    );
  }
}

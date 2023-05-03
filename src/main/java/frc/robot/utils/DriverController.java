// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainConstants;

/** 
 * DriveController serves as an extention to the typical xBoxController
 * with additional support for commonly used functions.
 */
public class DriverController extends XboxController {
    public Map<String, JoystickButton> buttons = new HashMap<String, JoystickButton>();

    /**
     * Creates a new drive controller, extends the XboxController class with additional functionality.
     * @param port Port number of the controller (start at 0)
     */
    public DriverController(int port) {
        super(port);
        this.generateButtons();
    }

    private void generateButtons(){
        buttons.put("A", new JoystickButton(this, 1));
        buttons.put("B", new JoystickButton(this, 2));
        buttons.put("X", new JoystickButton(this, 3));
        buttons.put("Y", new JoystickButton(this, 4));
        buttons.put("LB", new JoystickButton(this, 5));
        buttons.put("RB", new JoystickButton(this, 6));
        buttons.put("Back", new JoystickButton(this, 7));
        buttons.put("Start", new JoystickButton(this, 8));
    }

    /**
     * Quick way to get the drivetrain output determined by the controller
     * @return A 3D Pose vector <xVel, yVel, rotVel> in the format of the WPILIB ChassisSpeeds class
     */
    public ChassisSpeeds getDrivetrainOutput(){
        switch(DriverStation.getAlliance()){
            case Blue:
                return new ChassisSpeeds(
                    -this.getLeftY() * DrivetrainConstants.maxTranslationalSpeed,
                    -this.getLeftX() * DrivetrainConstants.maxTranslationalSpeed,
                    -this.getRightX() * DrivetrainConstants.maxRotationalSpeed
                );
            case Red:
                return new ChassisSpeeds(
                    this.getLeftY() * DrivetrainConstants.maxTranslationalSpeed,
                    this.getLeftX() * DrivetrainConstants.maxTranslationalSpeed,
                    -this.getRightX() * DrivetrainConstants.maxRotationalSpeed
                );
            default:
                return new ChassisSpeeds();
        }
    }
}
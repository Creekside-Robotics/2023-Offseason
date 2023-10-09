// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.arms.RetractArms;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.composite.FirstLevelScore;
import frc.robot.commands.composite.PickupObject;
import frc.robot.commands.composite.SecondLevelScore;
import frc.robot.commands.composite.StowObjectIntake;
import frc.robot.commands.composite.ThirdLevelScore;
import frc.robot.commands.drivetrain.DriveToNearestGridPosition;
import frc.robot.commands.drivetrain.DriveToNearestSubstationAxis;
import frc.robot.commands.drivetrain.ManualDrive;
import frc.robot.commands.intake.ExtendIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.UpperArm;
import frc.robot.utils.DriverController;

/**
 * This class is where the bulk of the robot is declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here.
    private final Drivetrain drivetrain = new Drivetrain();
    private final LowerArm lowerArm = new LowerArm();
    private final UpperArm upperArm = new UpperArm();
    private final Intake intake = new Intake();
    private final Claw claw = new Claw();

    private final DriverController mainController = new DriverController(Constants.DeviceIds.driverController);
    private final DriverController alternateController = new DriverController(Constants.DeviceIds.alternateController);

    private final SendableChooser<Command> commandChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        configureAutoCommands();
    }

    /**
     * Builds autoCommand based on the pathgroup provided.
     * 
     * @param pathName Name of pathplanner file
     * @return Full Auto command
     */
    public Command buildAutoCommand(String pathName) {
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner
                .loadPathGroup(pathName, new PathConstraints(AutoConstants.maxVelocity, AutoConstants.maxAcceleration));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Score", new ThirdLevelScore(drivetrain, lowerArm, upperArm));
        eventMap.put("Retract", new SequentialCommandGroup(
                new OpenClaw(claw),
                new RetractArms(lowerArm, upperArm)));
        eventMap.put("Intake", new ExtendIntake(intake));
        eventMap.put("Index", new SequentialCommandGroup(
                new RetractIntake(intake),
                new StowObjectIntake(lowerArm, upperArm, intake, claw)));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drivetrain::getPose,
                drivetrain::setDrivetrainPose,
                AutoConstants.translationConstants,
                AutoConstants.rotationConstants,
                drivetrain::setDrivetrainSpeedsAuto,
                eventMap,
                drivetrain);

        return autoBuilder.fullAuto(pathGroup);
    }

    /**
     * This command configures auto-commands and displays for driver to choose.
     */
    private void configureAutoCommands() {
        this.commandChooser.setDefaultOption("None", null);

        File[] autoRoutines = new File(Filesystem.getDeployDirectory()+"\\pathplanner").listFiles();
        for(File file : autoRoutines) {
                if (!file.isFile()) continue;
                String fileName = file.getName().replace(".path", "");
                this.commandChooser.addOption(fileName, buildAutoCommand(fileName));
        }

        SmartDashboard.putData(this.commandChooser);
    }

    /**
     * This command is used to configure command and button bindings.
     */
    private void configureButtonBindings() {
        this.drivetrain.setDefaultCommand(new ManualDrive(drivetrain, mainController));

        this.mainController.buttons.get(ControllerConstants.autoPickup).whileTrue(new ParallelCommandGroup(
                new PickupObject(lowerArm, upperArm, claw),
                new DriveToNearestSubstationAxis(drivetrain, mainController)));
        this.mainController.buttons.get(ControllerConstants.autoPickup).onFalse(new SequentialCommandGroup(
                new CloseClaw(claw),
                new RetractArms(lowerArm, upperArm)));

        this.mainController.buttons.get(ControllerConstants.intake)
                .whileTrue(new ExtendIntake(intake));
        this.mainController.buttons.get(ControllerConstants.intake)
                .onFalse(new SequentialCommandGroup(
                        new RetractIntake(intake),
                        new StowObjectIntake(lowerArm, upperArm, intake, claw)));

        this.mainController.buttons.get(ControllerConstants.thirdScore)
                .whileTrue(new ThirdLevelScore(drivetrain, lowerArm, upperArm));
        this.mainController.buttons.get(ControllerConstants.thirdScore)
                .onFalse(new SequentialCommandGroup(new OpenClaw(claw), new RetractArms(lowerArm, upperArm)));

        this.mainController.buttons.get(ControllerConstants.secondScore)
                .whileTrue(new SecondLevelScore(drivetrain, lowerArm, upperArm));
        this.mainController.buttons.get(ControllerConstants.secondScore)
                .onFalse(new SequentialCommandGroup(new OpenClaw(claw), new RetractArms(lowerArm, upperArm)));

        this.mainController.buttons.get(ControllerConstants.firstScore)
                .whileTrue(new FirstLevelScore(drivetrain, lowerArm, upperArm));
        this.mainController.buttons.get(ControllerConstants.firstScore)
                .onFalse(new SequentialCommandGroup(new OpenClaw(claw), new RetractArms(lowerArm, upperArm)));

        this.mainController.buttons.get(ControllerConstants.thirdAuto).whileTrue(new SequentialCommandGroup(
                new DriveToNearestGridPosition(drivetrain, mainController),
                new ThirdLevelScore(drivetrain, lowerArm, upperArm)));
        this.mainController.buttons.get(ControllerConstants.thirdAuto)
                .onFalse(new SequentialCommandGroup(new OpenClaw(claw), new RetractArms(lowerArm, upperArm)));

        this.mainController.buttons.get(ControllerConstants.secondAuto).whileTrue(new SequentialCommandGroup(
                new DriveToNearestGridPosition(drivetrain, mainController),
                new SecondLevelScore(drivetrain, lowerArm, upperArm)));
        this.mainController.buttons.get(ControllerConstants.secondAuto)
                .onFalse(new SequentialCommandGroup(new OpenClaw(claw), new RetractArms(lowerArm, upperArm)));

        this.mainController.buttons.get(ControllerConstants.firstAuto).whileTrue(new SequentialCommandGroup(
                new DriveToNearestGridPosition(drivetrain, mainController),
                new FirstLevelScore(drivetrain, lowerArm, upperArm)));
        this.mainController.buttons.get(ControllerConstants.firstAuto)
                .onFalse(new SequentialCommandGroup(new OpenClaw(claw), new RetractArms(lowerArm, upperArm)));

        this.alternateController.buttons.get(ControllerConstants.openClaw)
                .whileTrue(new OpenClaw(claw));
        this.alternateController.buttons.get(ControllerConstants.closeClaw)
                .whileTrue(new CloseClaw(claw));

        this.alternateController.buttons.get(ControllerConstants.retractArms)
                .whileTrue(new RetractArms(lowerArm, upperArm));

        this.alternateController.buttons.get(ControllerConstants.extendIntake)
                .whileTrue(new ExtendIntake(intake));
        this.alternateController.buttons.get(ControllerConstants.retractIntake)
                .whileTrue(new RetractIntake(intake));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return this.commandChooser.getSelected();
    }
}

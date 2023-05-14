// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriverController;

public class DriveToPosePID extends CommandBase {
  /** Creates a new DriveToPosePID. */
  private PIDController xController = new PIDController(2, 0, 0);
  private PIDController yController = new PIDController(2, 0, 0);
  private PIDController rotController = new PIDController(Math.PI/2, 0, 0);

  private Drivetrain drivetrain;
  private DriverController driverController;

  private Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose = new Pose2d();

  private boolean[] usePID;
  private boolean hold;
  
  /**
   * Creates a new command which moves the robot to the specified pose.
   * @param drivetrain Drivetrain object.
   * @param driverController Controller that will be used by the driver to control the robot.
   * @param pose Pose that the robot will move to.
   * @param usePID Boolean array {x, y, rot}, that determines whether or not to use the PID controller the specified axis. 
   * For example, using {false, false, true} will only control rotation of the robot.
   * @param tolerance Pose2d object representing the needed tolerance before finishing the command.
   * @param hold Whether or not to hold the robot at the specified position without finishing the command.
   */
  public DriveToPosePID(Drivetrain drivetrain, DriverController driverController, Pose2d pose, boolean[] usePID, Pose2d tolerance, boolean hold) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.poseSupplier = () -> pose;
    this.usePID = usePID;
    this.hold = hold;

    this.xController.setTolerance(tolerance.getX());
    this.yController.setTolerance(tolerance.getY());
    this.rotController.setTolerance(tolerance.getRotation().getRadians());

    addRequirements(this.drivetrain);
  }

  /**
   * Creates a new command which moves the robot to the specified pose.
   * @param drivetrain Drivetrain object.
   * @param driverController Controller that will be used by the driver to control the robot.
   * @param pose Supplier that provided pose that the robot will move to.
   * @param usePID Boolean array {x, y, rot}, that determines whether or not to use the PID controller the specified axis. 
   * For example, using {false, false, true} will only control rotation of the robot.
   * @param tolerance Pose2d object representing the needed tolerance before finishing the command.
   * @param hold Whether or not to hold the robot at the specified position without finishing the command.
   */
  public DriveToPosePID(Drivetrain drivetrain, DriverController driverController, Supplier<Pose2d> poseSupplier, boolean[] usePID, Pose2d tolerance, boolean hold) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.poseSupplier = poseSupplier;
    this.usePID = usePID;
    this.hold = hold;

    this.xController.setTolerance(tolerance.getX());
    this.yController.setTolerance(tolerance.getY());
    this.rotController.setTolerance(tolerance.getRotation().getRadians());

    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetPose = this.poseSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds pidOutput = getPIDChassisSpeeds();
    ChassisSpeeds driverOutput = this.driverController.getDrivetrainOutput();
    this.drivetrain.setDrivetrainOutput(fuseOutputSpeeds(pidOutput, driverOutput), true);
  }

  /**
   * Gets PID calculated output for drivetrain.
   * @return ChassisSpeed Object representing output.
   */
  private ChassisSpeeds getPIDChassisSpeeds(){
    return new ChassisSpeeds(
      MathUtil.clamp(xController.calculate(drivetrain.getPose().getX() - targetPose.getX()), -DrivetrainConstants.maxTranslationalSpeed, DrivetrainConstants.maxTranslationalSpeed),
      MathUtil.clamp(yController.calculate(drivetrain.getPose().getY() - targetPose.getY()), -DrivetrainConstants.maxTranslationalSpeed, DrivetrainConstants.maxTranslationalSpeed),
      MathUtil.clamp(rotController.calculate(drivetrain.getPose().getRotation().minus(targetPose.getRotation()).getRadians()), -DrivetrainConstants.maxRotationalSpeed, DrivetrainConstants.maxRotationalSpeed)
    );
  }

  /**
   * Fuses the PID calculated output and the controller output
   * @param PIDOutput ChassisSpeeds object representing output determined by PID controllers.
   * @param driverOutput ChassisSpeeds object representing output determined by driver controller.
   * @return Fused output as specified by usePID passed in consructor.
   */
  private ChassisSpeeds fuseOutputSpeeds(ChassisSpeeds PIDOutput, ChassisSpeeds driverOutput){
    ChassisSpeeds fusedOutput = driverOutput;
    if(this.usePID[0]){
      fusedOutput.vxMetersPerSecond = PIDOutput.vxMetersPerSecond;
    }
    if(this.usePID[1]){
      fusedOutput.vyMetersPerSecond = PIDOutput.vyMetersPerSecond;
    }
    if(this.usePID[2]){
      fusedOutput.omegaRadiansPerSecond = PIDOutput.omegaRadiansPerSecond;
    }
    return fusedOutput;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.setDrivetrainOutput(new ChassisSpeeds(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !hold && this.xController.atSetpoint() && this.yController.atSetpoint() && this.rotController.atSetpoint();
  }
}
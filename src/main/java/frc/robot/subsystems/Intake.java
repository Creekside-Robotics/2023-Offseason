// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;

/** Creates a new Intake. */
public class Intake extends SubsystemBase {
  private CANSparkMax forwardMotor;
  private CANSparkMax reverseMotor;
  private DoubleSolenoid actuator;

  private double motorVelocity = 0;

  /**
   * Creates a new subsystem representing an intake on the robot. The intake has a
   * pnuematic actuator for extending a retracting as as well as two rollers.
   */
  public Intake() {
    this.forwardMotor = new CANSparkMax(DeviceIds.intakeForwardMotor, MotorType.kBrushless);
    this.forwardMotor.setInverted(true);
    this.reverseMotor = new CANSparkMax(DeviceIds.intakeReverseMotor, MotorType.kBrushless);
    this.reverseMotor.setInverted(true);
    this.actuator = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        DeviceIds.intakePistonForward,
        DeviceIds.intakePistonReverse);
  }

  /**
   * Sets the state of the intake.
   * 
   * @param motorVelocity  Velocity to set intake rollers to. A double on the
   *                       range of -1 to 1.
   * @param intakeExtended Boolean representing whether or not the intake should
   *                       be extended.
   */
  public void setIntakeMode(double motorVelocity, boolean intakeExtended) {
    this.motorVelocity = motorVelocity;
    if (intakeExtended) {
      this.actuator.set(Value.kForward);
    } else {
      this.actuator.set(Value.kReverse);
    }
  };

  @Override
  public void periodic() {
    this.forwardMotor.set(motorVelocity);
    this.reverseMotor.set(motorVelocity);
  }
}

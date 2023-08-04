// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private DoubleSolenoid actuator;
  private ColorSensorV3 colorSensor;

  /**
   * Creates a new claw object with a double solenoid and color sensor.
   */
  public Claw() {
    this.actuator = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        DeviceIds.clawForward,
        DeviceIds.clawReverse);
    this.colorSensor = new ColorSensorV3(Port.kOnboard);
  }

  /**
   * Method used to set the mode of the claw.
   * 
   * @param mode ClawMode value representing the state of the claw.
   */
  public void setClawMode(ClawMode mode) {
    switch (mode) {
      case open:
        this.actuator.set(Value.kReverse);
        break;
      case closed:
        this.actuator.set(Value.kForward);
        break;
    }
  }

  /**
   * Method returns how far away from the claw an object is.
   * 
   * @return Distance away from the claw in centimeters. Between 1cm and 10cm.
   */
  public int getDistanceFromClaw() {
    return this.colorSensor.getProximity();
  }

  @Override
  public void periodic() {
  }

  /**
   * State of the claw object.
   */
  public static enum ClawMode {
    closed,
    open
  }
}

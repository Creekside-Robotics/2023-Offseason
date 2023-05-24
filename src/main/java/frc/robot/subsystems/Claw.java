// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
   * Returns whether or not a cube is detected in the claw.
   * 
   * @return Boolean representing the presence of a cube in the claw.
   */
  private boolean isCube() {
    RawColor color = this.colorSensor.getRawColor();
    return color.blue > 150 && color.green < 50 && color.red > 50;
  }

  /**
   * Returns whether or not a cone is detected in the claw.
   * 
   * @return Boolean representing the presence of a cone in the claw.
   */
  private boolean isCone() {
    RawColor color = this.colorSensor.getRawColor();
    return color.blue < 50 && color.green > 50 && color.red > 150;
  }

  /**
   * Returns whether or not an object is detected in the claw.
   * 
   * @return Boolean representing the presence of an object in the claw.
   */
  public boolean hasObject() {
    return isCone() || isCube();
  }

  /**
   * Method used to set the mode of the claw.
   * 
   * @param mode ClawMode value representing hte state of the claw.
   */
  public void setClawMode(ClawMode mode) {
    switch (mode) {
      case open:
        this.actuator.set(Value.kReverse);
        break;
      case closed:
        this.actuator.set(Value.kForward);
        break;
      case sensorOpen:
        this.actuator.set(Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    RawColor color = this.colorSensor.getRawColor();
    SmartDashboard.putNumber("Blue", color.blue);
    SmartDashboard.putNumber("Green", color.green);
    SmartDashboard.putNumber("Red", color.red);
  }

  /**
   * State of the claw object.
   */
  public static enum ClawMode {
    closed,
    open,
    sensorOpen
  }
}

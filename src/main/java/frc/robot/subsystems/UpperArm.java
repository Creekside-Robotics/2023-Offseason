// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.UpperArmConstants;

/** Add your docs here. */
public class UpperArm extends AbstractArm {
    private CANSparkMax motor;

    private DutyCycleEncoder armEncoder;

    private PIDController armController;

    private double goalPosition = 0.0;
    private double motorOutput = 0.0;
    private double latestSetTime = Timer.getFPGATimestamp();

    /**
     * Creates a new subsystem representing the upper arm the robot.
     */
    public UpperArm(){
        this.motor = new CANSparkMax(DeviceIds.upperArm, MotorType.kBrushless);
        this.motor.setInverted(false);
        this.armEncoder = new DutyCycleEncoder(DeviceIds.upperArmEncoder);

        this.armController = new PIDController(UpperArmConstants.proportionalGain, 0, UpperArmConstants.derivativeGain);
        this.armController.setTolerance(UpperArmConstants.tolerance);
    }

    @Override
    public void periodic(){
        updateMotorOutput();
        setOutput();
    }

    @Override
    public double getArmPosition() {
        var value = (this.armEncoder.get() * UpperArmConstants.encoderMultiplier + UpperArmConstants.encoderOffset) % 1;
        if (value < -0.5){
            value += 1.0;
        } 
        if (value > 0.5){
            value -= 1.0;
        }
        SmartDashboard.putNumber("Upper Arm Position", value);
        return value;
    }

    /**
     * Calculates the maximimum magnitude of changes in velocity for the mechanism.
     * @return Maximum value of change in velocity. 
     */
    private double getAccelerationTolerance(){
        return (Timer.getFPGATimestamp() - this.latestSetTime) * UpperArmConstants.maxAcceleration;
    }

    /**
     * Sets the motors to the updated output value.
     */
    private void setOutput(){
        this.motor.set(motorOutput);
        this.latestSetTime = Timer.getFPGATimestamp();
    }

    /**
     * Updates the motor output value. Uses a PID controller with limits on magnitude of velocity and
     * acceleration to protect mechanisms. Does not the motors to this output value.
     */
    private void updateMotorOutput(){
        double pidOutput = this.armController.calculate(getArmPosition(), this.goalPosition);
        double accelerationTolerance = getAccelerationTolerance();
        double accelerationConstrainedOutput = MathUtil.clamp(
            pidOutput, 
            this.motorOutput - accelerationTolerance, 
            this.motorOutput + accelerationTolerance
        );
        double velocityConstrainedOutput = MathUtil.clamp(
            accelerationConstrainedOutput, 
            -UpperArmConstants.maxSpeed,
            UpperArmConstants.maxSpeed
        );
        this.motorOutput = velocityConstrainedOutput;
        SmartDashboard.putNumber("Upper Arm Output", this.motorOutput);
    }

    @Override
    public void setArmPosition(double position) {
        this.goalPosition = position;
    }

    @Override
    public boolean atSetpoint() {
        return this.armController.atSetpoint();
    }
}

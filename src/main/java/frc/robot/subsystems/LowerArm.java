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
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.LowerArmConstants;

/** Add your docs here. */
public class LowerArm extends Arm {
    private CANSparkMax forwardMotor;
    private CANSparkMax reverseMotor;

    private DutyCycleEncoder armEncoder;

    private PIDController armController;

    private double goalPosition = ArmPositions.lowerStowed;
    private double motorOutput = 0.0;
    private double latestSetTime = Timer.getFPGATimestamp();

    /**
     * Creates a new subsystem representing the lower arm the robot.
     */
    public LowerArm() {
        this.forwardMotor = new CANSparkMax(DeviceIds.lowerArmForward, MotorType.kBrushless);
        this.forwardMotor.setInverted(false);
        this.reverseMotor = new CANSparkMax(DeviceIds.lowerArmBackward, MotorType.kBrushless);
        this.reverseMotor.setInverted(true);
        this.armEncoder = new DutyCycleEncoder(DeviceIds.lowerArmEncoder);

        this.armController = new PIDController(LowerArmConstants.proportionalGain, 0, LowerArmConstants.derivativeGain);
        this.armController.setTolerance(LowerArmConstants.tolerance);
    }

    @Override
    public void periodic() {
        updateMotorOutput();
        setOutput();
    }

    @Override
    public double getArmPosition() {
        var value = (this.armEncoder.get() * LowerArmConstants.encoderMultiplier + LowerArmConstants.encoderOffset) % 1;
        if (value < -0.5) {
            value += 1.0;
        }
        if (value > 0.5) {
            value -= 1.0;
        }
        SmartDashboard.putNumber("Lower Arm Position", value);
        return value;
    }

    /**
     * Calculates the maximimum magnitude of changes in velocity for the mechanism.
     * 
     * @return Maximum value of change in velocity.
     */
    private double getAccelerationTolerance() {
        return (Timer.getFPGATimestamp() - this.latestSetTime) * LowerArmConstants.maxAcceleration;
    }

    /**
     * Sets the motors to the updated output value.
     */
    private void setOutput() {
        this.forwardMotor.set(motorOutput);
        this.reverseMotor.set(motorOutput);
        this.latestSetTime = Timer.getFPGATimestamp();
    }

    /**
     * Updates the motor output value. Uses a PID controller with limits on
     * magnitude of velocity and
     * acceleration to protect mechanisms. Does not the motors to this output value.
     */
    private void updateMotorOutput() {
        double pidOutput = this.armController.calculate(getArmPosition(), this.goalPosition);
        double accelerationTolerance = getAccelerationTolerance();
        double accelerationConstrainedOutput = MathUtil.clamp(
                pidOutput,
                this.motorOutput - accelerationTolerance,
                this.motorOutput + accelerationTolerance);
        double velocityConstrainedOutput = MathUtil.clamp(
                accelerationConstrainedOutput,
                -LowerArmConstants.maxSpeed,
                LowerArmConstants.maxSpeed);
        this.motorOutput = velocityConstrainedOutput;
        SmartDashboard.putNumber("Lower Arm Output", this.motorOutput);
    }

    @Override
    public void setArmPosition(double position) {
        this.goalPosition = position;
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(getArmPosition() - this.goalPosition) < LowerArmConstants.tolerance;
    }
}

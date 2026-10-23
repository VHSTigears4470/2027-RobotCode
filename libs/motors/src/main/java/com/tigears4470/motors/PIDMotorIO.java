package com.tigears4470.motors;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ControlRequest;
import com.revrobotics.spark.SparkLowLevel;

public interface PIDMotorIO {
    @AutoLog
    public class PIDMotorIOInputs {
        public double velocity = 0.0;
        //add the rest
	}

	/** 
	 * Updates Logging for AdvantageKit with current velocity (i think?)
	*/
    default void updateInputs(PIDMotorIOInputsAutoLogged inputs){}

    default public void resetEncoder(){}

	/**
	 * Sets the setpoint for the motor controller with a given control type and feedforward value (REV Only)
	 * 
	 * @param setpoint The desired setpoint for the motor controller
	 * @param controlType The control type to use for the setpoint (e.g. position, velocity, etc.)
	 * @param FF The feedforward value to use for the setpoint
	*/
    default public void setSetpoint(double setpoint, SparkLowLevel.ControlType controlType, double FF){}
	
	/**
	 * Sets the setpoint for the motor controller with a given control type and feedforward value using a ControlRequest object (CTRE Only)
	 *
	 * @param request The ControlRequest object containing the desired setpoint, control type, and feedforward value
	*/
	default public void setSetpoint(ControlRequest request){}

	default public void setVelocity(double velocity, double FF){}

	/**
	 * Sets the setpoint for the motor controller with a given control type and feedforward value using a ControlRequest object (CTRE Only)
	 *
	 * @param request The ControlRequest object containing the desired setpoint, control type, and feedforward value
	*/
	default public void setVelocity(ControlRequest request){}

    default public void set(double speed){}

    default public void setVoltage(double voltage){}

    default public void setEncoder(double setpoint){}

    default public void stopMotors(){}

    default public double getEncoder(){return 0;}

    default public double getEncoderAbs(){return 0;}

    default public double getVelocity(){return 0;}

    default public double getAppliedOutput(){return 0;}

    default public double getBusVoltage(){return 0;}

    default public double getOutputCurrent(){return 0;}

    default public double getMotorTemperature(){return 0;}
}

package com.tigears4470.motors;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;

import com.ctre.phoenix6.controls.*;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.tigears4470.motors.ctre.PIDMotorTalonFX;
import com.tigears4470.motors.PIDMotorIOInputsAutoLogged;

public class PIDMotor extends SubsystemBase {
  private final double CTRE_FF_MULTI = 12.0;
  private PIDMotorIO io;
  private final PIDMotorIOInputsAutoLogged inputs = new PIDMotorIOInputsAutoLogged();
  private boolean isTalon;
  private double ff;
   

  /** 
   * Creates a new PIDMotor object
   * @param io {@link PIDMotorIO} one of the motors inside of com.tigears4470.motors.(ctre/rev)
   * @param FF {@link Double} Feed forward value for motor, typically in range of -1 < x < 1
   */
  public PIDMotor(PIDMotorIO io, double FF) {
    this.io = io;
	this.ff = FF;
    io.resetEncoder();
	this.isTalon = io instanceof PIDMotorTalonFX;
  }

  @Override public void periodic() {
    io.updateInputs(inputs);
  }

  /** 
   * Sets the setpoint of the motor controller, uses infered FF from initialization
   * @param setpoint {@link Double} value of the setpoint value
   * @param contrlType {@link com.revrobotics.spark.SparkLowLevel.ControlType} The requested motor controlType
   */
  public void setSetpoint(double setpoint, ControlType controlType) {
		this.setSetpoint(setpoint, controlType, this.ff);
  }

   /** 
   * Sets the setpoint of the motor controller, uses inferred FF from initialization, defaults to DutyCycle controlType
   * @param setpoint {@link Double} value of the setpoint value
   */
  public void setSetpoint(double setpoint) {
		this.setSetpoint(setpoint, ControlType.kDutyCycle);
  }

   /** 
   * Sets the setpoint of the motor controller, this function is NOT recommended for use. Please see {@link #setSetpoint(double, ControlType)} or {@link #setSetpoint(double)} for recommended usage.
   * @param setpoint {@link Double} value of the setpoint value
   * @param contrlType {@link com.revrobotics.spark.SparkLowLevel.ControlType} The requested motor controlType
   * @param FF {@link Double} Feed forward value for motor, typically in range of -1 < x < 1
   */
  public void setSetpoint(double setpoint, ControlType controlType, double FF) {
	Logger.recordOutput("PIDMotor/Setpoint", setpoint);
    if (isTalon) {
        switch (controlType) {
            case kDutyCycle:
                io.setSetpoint(new DutyCycleOut(setpoint));
                break;
                
            case kVelocity:
                io.setSetpoint(new VelocityVoltage(setpoint).withFeedForward(FF * CTRE_FF_MULTI));
                break;
                
            case kVoltage:
                io.setSetpoint(new VoltageOut(setpoint));
                break;
                
            case kPosition:
                io.setSetpoint(new PositionVoltage(setpoint).withFeedForward(FF * CTRE_FF_MULTI));
                break;
                
            case kCurrent:
                io.setSetpoint(new TorqueCurrentFOC(setpoint));
                break;
                
            case kMAXMotionPositionControl:
                io.setSetpoint(new MotionMagicVoltage(setpoint).withFeedForward(CTRE_FF_MULTI));
                break;
                
            case kMAXMotionVelocityControl:
                io.setSetpoint(new MotionMagicVelocityVoltage(setpoint).withFeedForward(FF * CTRE_FF_MULTI));
                break;
        }
		return;
	}
    io.setSetpoint(setpoint, controlType, FF);
  }

  /**
   * Sets the velocity of a motor, eequivalent to setSetpoint with ControlType.kVelocity, uses inferred FF from initialization
   * @param velocity {@link Double} value of the velocity setpoint
   */
  public void setVelocity(double velocity) {
		this.setVelocity(velocity, this.ff);
  }

   /**
   * Sets the velocity of a motor, eequivalent to setSetpoint with ControlType.kVelocity. Note: This function is NOT recommended for use. Please see {@link #setVelocity(double)} for recommended usage.
   * @param velocity {@link Double} value of the velocity setpoint
   * @param FF {@link Double} Feed forward value for motor, typically in range of -1 < x < 1
   */
  public void setVelocity(double velocity, double FF) {
	Logger.recordOutput("PIDMotor/SetVelocity", velocity);
   	if (isTalon) {
		setSetpoint(velocity, ControlType.kVelocity, FF);
	}
	io.setVelocity(velocity, FF);
  }

  /** 
   * Sets raw voltage of motor. Not recommended for use
   * @param voltage {@link Double} value of the voltage setpoint
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
    Logger.recordOutput("PIDMotor/SetVoltage", voltage);
  }
  /** 
   * Sets the raw motor speed
   * @param speed {@link Double} value of the speed setpoint
  */
  public void set(double speed) {
    io.set(speed);
    Logger.recordOutput("PIDMotor/Set", speed);
  }

  /** 
   * Sets the encoders possition, does not move motor, purely for logging
   * @paaram setpoint {@link Double} value of the encoder setpoint
   */
  public void setEncoder(double setpoint) {
    io.setEncoder(setpoint);
    Logger.recordOutput("PIDMotor/SetEncoder", setpoint);
  }

  /** 
   * Stops the motors
   */
  public void stopMotors() {
    io.stopMotors();
    Logger.recordOutput("PIDMotor/StopMotors", true);
  }
  /** 
   * Resets encoder position to 0
   */
  public void resetEncoder() {
    io.resetEncoder();
  }
  /** 
   * Gets the encoder position
   * @return {@link Double} value of the encoder position
   */
  public double getEncoder() {
    return io.getEncoder();
  }

  /** 
   * Gets the absolute encoder's position
   * @return {@link Double} value of the absolute encoder position
   */
  public double getEncoderAbs() {
    return io.getEncoderAbs();
  }
  /**
   * Gets the current velocity of the motor (aka RPM)
   * @return {@link Double} the current velocity of the motor
   */
  public double getVelocity() {
    return io.getVelocity();
  }

  /**
   * Gets the applied output of the motor
   * @return {@link Double} the applied output of the motor
   */
  public double getAppliedOutput(){
      return io.getAppliedOutput();
  }
  /** 
   * Gets the current bus voltage of the motor
   * @return {@link Double} the current bus voltage of the motor
   */
  public double getBusVoltage(){
      return io.getBusVoltage();
  }
  /**
   * Gets the current output current of the motor
   * @return {@link Double} the current output current of the motor
   */
  public double getOutputCurrent(){
      return io.getOutputCurrent();
  }
  /** 
   * Gets the current temperature of the motor
   * @return {@link Double} the current temperature of the motor in Celcius
   */
  public double getMotorTemperature() {
    return io.getMotorTemperature();
  }
}

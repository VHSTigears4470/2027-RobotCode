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
   
  public PIDMotor(PIDMotorIO io) {
    this.io = io;
    io.resetEncoder();
	this.isTalon = io instanceof PIDMotorTalonFX;
  }

  @Override public void periodic() {
    io.updateInputs(inputs);
  }

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

  public void setVelocity(double velocity, double FF) {
	Logger.recordOutput("PIDMotor/SetVelocity", velocity);
   	if (isTalon) {
		setSetpoint(velocity, ControlType.kVelocity, FF);
	}
	io.setVelocity(velocity, FF);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
    Logger.recordOutput("PIDMotor/SetVoltage", voltage);
  }

  public void set(double speed) {
    io.set(speed);
    Logger.recordOutput("PIDMotor/Set", speed);
  }

  public void setEncoder(double setpoint) {
    io.setEncoder(setpoint);
    Logger.recordOutput("PIDMotor/SetEncoder", setpoint);
  }

  public void stopMotors() {
    io.stopMotors();
    Logger.recordOutput("PIDMotor/StopMotors", true);
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public double getEncoder() {
    return io.getEncoder();
  }

  public double getEncoderAbs() {
    return io.getEncoderAbs();
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public double getAppliedOutput(){
      return io.getAppliedOutput();
  }

  public double getBusVoltage(){
      return io.getBusVoltage();
  }

  public double getOutputCurrent(){
      return io.getOutputCurrent();
  }

  public double getMotorTemperature() {
    return io.getMotorTemperature();
  }
}

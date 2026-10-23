package com.tigears4470.motors;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.tigears4470.motors.PIDMotorIOInputsAutoLogged;

public class PIDMotor extends SubsystemBase {

  private PIDMotorIO io;
  private final PIDMotorIOInputsAutoLogged inputs = new PIDMotorIOInputsAutoLogged();
   
  public PIDMotor(PIDMotorIO io) {
    this.io = io;
    io.resetEncoder();
  }

  @Override public void periodic() {
    io.updateInputs(inputs);
  }

  public void setSetpoint(double setpoint, ControlType controlType, double FF) {
    io.setSetpoint(setpoint, controlType, FF);
    Logger.recordOutput("PIDMotor/Setpoint", setpoint);
  }

  public void setVelocity(double velocity, double FF) {
    io.setVelocity(velocity, FF);
    Logger.recordOutput("PIDMotor/SetVelocity", velocity);
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

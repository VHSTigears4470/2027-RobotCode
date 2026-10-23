package com.tigears4470.rev;

import org.wpilib.hardware.bus.CANPort;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.tigears4470.motors.PIDMotorIO;
import com.tigears4470.motors.PIDMotorIOInputsAutoLogged;

public class PIDMotorSparkMax implements PIDMotorIO {
    private SparkMax motor = null;
    private RelativeEncoder encoder = null;
    private AbsoluteEncoder encoderAbs = null;
    private SparkClosedLoopController controller = null;
    private double encoderConversionFactor = 1;
    private double velocityConversionFactor = 1;


    public PIDMotorSparkMax(CANPort busID, int ID, SparkMaxConfig config, double encoderConversionFactor, double velocityConversionFactor) {
        motor = new SparkMax(busID, ID, MotorType.kBrushless);
        this.encoderConversionFactor = encoderConversionFactor;
        this.velocityConversionFactor = velocityConversionFactor;
        encoder = motor.getEncoder();
        encoderAbs = motor.getAbsoluteEncoder();
        controller = motor.getClosedLoopController();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

   @Override public void updateInputs(PIDMotorIOInputsAutoLogged inputs) {
        inputs.velocity = getVelocity();
    }

    @Override public void resetEncoder() {
        encoder.setPosition(0);
    }

   @Override public void setSetpoint(double setpoint, SparkLowLevel.ControlType controlType, double FF) {
        controller.setSetpoint(setpoint / encoderConversionFactor, controlType, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void setVelocity(double velocity, double FF) {
        controller.setSetpoint(velocity / velocityConversionFactor, SparkLowLevel.ControlType.kVelocity, ClosedLoopSlot.kSlot0, FF);
    }

    @Override public void set(double speed){
        motor.setThrottle(speed);
    }

    @Override public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override public void setEncoder(double position) {
        encoder.setPosition(position / encoderConversionFactor);
    }

    @Override public void stopMotors(){
        motor.stopMotor();
    }

    @Override public double getEncoder(){
        return encoder.getPosition().get() * encoderConversionFactor;
    }

    @Override public double getEncoderAbs(){
        return encoderAbs.getPosition().get() * encoderConversionFactor;
    }

    @Override public double getVelocity(){
        return encoder.getVelocity().get() * velocityConversionFactor;
    }

     @Override public double getAppliedOutput(){
        return motor.getAppliedOutput().get();
    }

    @Override public double getBusVoltage(){
        return motor.getBusVoltage().get();
    }

    @Override public double getOutputCurrent(){
        return motor.getOutputCurrent().get();
    }

    @Override public double getMotorTemperature() {
        return motor.getMotorTemperature().get();
    }
}

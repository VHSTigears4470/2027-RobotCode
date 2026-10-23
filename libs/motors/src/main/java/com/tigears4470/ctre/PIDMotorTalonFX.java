package com.tigears4470.ctre;

import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.tigears4470.motors.PIDMotorIO;
import com.tigears4470.motors.PIDMotorIOInputsAutoLogged;

public class PIDMotorTalonFX implements PIDMotorIO {
    private TalonFX motor = null;
	private CANcoder canCoder;

 	public PIDMotorTalonFX(CANPort busID, int ID, CANcoder canCoder, TalonFXConfiguration config) {
        motor = new TalonFX(ID, new CANBus(busID));
		this.canCoder = canCoder;
    	motor.getConfigurator().apply(config);
	}

	public PIDMotorTalonFX(CANPort busID, int ID, TalonFXConfiguration config) {
		motor = new TalonFX(ID, new CANBus(busID));
		motor.getConfigurator().apply(config);
		this.canCoder = null;
	}
	

   @Override public void updateInputs(PIDMotorIOInputsAutoLogged inputs) {
        inputs.velocity = getVelocity();
    }

    @Override public void resetEncoder() {
        motor.setPosition(0);
    }


	/* 
	 * @param double FF FeedForward in volts
	*/
   @Override public void setSetpoint(ControlRequest req) {
        motor.setControl(req);
    }

    @Override public void setVelocity(ControlRequest req) {
        motor.setControl(req);
    }

    @Override public void set(double speed){
        motor.setThrottle(speed);
    }

    @Override public void setVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override public void setEncoder(double position) {
        motor.setPosition(position);
    }

    @Override public void stopMotors(){
        motor.stopMotor();
    }

    @Override public double getEncoder(){
        return motor.getPosition().getValueAsDouble();
    }

    @Override public double getEncoderAbs(){
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override public double getVelocity(){
        return motor.getVelocity().getValueAsDouble();
    }

     @Override public double getAppliedOutput(){
        return motor.getDutyCycle().getValueAsDouble();
    }

    @Override public double getBusVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override public double getOutputCurrent(){
        return motor.getStatorCurrent().getValueAsDouble();
    }

    @Override public double getMotorTemperature() {
        return motor.getDeviceTemp().getValueAsDouble();
    }
}

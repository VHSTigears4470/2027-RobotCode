package first;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.tigears4470.motors.PIDMotor;
import com.tigears4470.motors.ctre.PIDMotorTalonFX;
import com.tigears4470.motors.rev.PIDMotorSparkFlex;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;

class MotorTests {
	private PIDMotor motor;
	
	private void callMotor(PIDMotor mtr) {
		mtr.setSetpoint(12, ControlType.kVelocity, 0.4);
		mtr.set(0.1);
	}

	@Test
	void motorTests() {
		SparkFlexConfig config = new SparkFlexConfig();

		config.apply(SparkFlexConfig.Presets.REV_Vortex);

		motor = new PIDMotor(new PIDMotorSparkFlex(
			CANPort.CAN_S0, 1, config, 1, 1
		));

		callMotor(motor);

		motor = new PIDMotor(new PIDMotorTalonFX(
			CANPort.CAN_S0, 1, new TalonFXConfiguration()
		));

		// if it crashes something is fundamentally wrong with the code
	}
}

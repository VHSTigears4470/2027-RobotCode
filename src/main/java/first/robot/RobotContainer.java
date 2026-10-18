// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;

import first.robot.Constants.IO;
import first.robot.Constants.Operating;
import first.robot.Constants.presets.Layout;
import first.robot.commands.Autos;
import first.robot.commands.ExampleCommand;
import first.robot.subsystems.ExampleSubsystem;
import first.robot.Constants.presets.James;

public class RobotContainer {
	private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

	private final CommandGamepad driverController = new CommandGamepad(IO.DRIVER_CONTROLLER_PORT);

	public RobotContainer() {
	configureBindings();
	}

	private void configureBindings() {
		

		Layout preset = Operating.LAYOUT;
	
		// driverController.button(preset.INTAKE()).onTrue(new IntakeRun(intakeSub));
	}

	public Command getAutonomousCommand() {
	// An example command will be run in autonomous
	return Autos.exampleAuto(exampleSubsystem);
	}
}

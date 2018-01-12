package org.usfirst.frc.team68.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team68.robot.Robot;


public class DriveWithXboxJoysticks extends Command {
	
	boolean isFinished = false;
	private double speedLeft;
	private double speedRight;
	
	
	public DriveWithXboxJoysticks() {
		
		requires(Robot.driveTrain);
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
   		Robot.driveTrain.tankDrive(Robot.oi.getLeftXboxJoystickValue(), Robot.oi.getRightXboxJoystickValue());
/*   		SmartDashboard.putNumber("Yaw: ", Robot.driveTrain.getGyroYaw());
   		SmartDashboard.putNumber("Pitch: ", Robot.driveTrain.getGyroPitch());
   		SmartDashboard.putNumber("Roll: ", Robot.driveTrain.getGyroRoll());
*/
   		
/*		speedLeft = Robot.driveTrain.getDriveLeftSpeed();
		SmartDashboard.putNumber("Drive Left RPM Chart: ", speedLeft);
		SmartDashboard.putNumber("Drive Left RPM: ", speedLeft);
		
		speedRight = Robot.driveTrain.getDriveRightSpeed();
		SmartDashboard.putNumber("Drive Right RPM Chart: ", speedRight);
		SmartDashboard.putNumber("Drive Right RPM: ", speedRight);
*/

	}

	@Override
	protected boolean isFinished() {
		return isFinished;
	}

	@Override
	protected void end() {
	}

	@Override
	protected void interrupted() {
	}
}


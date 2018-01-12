package org.usfirst.frc.team68.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team68.robot.RobotMap;
import org.usfirst.frc.team68.robot.commands.DriveWithXboxJoysticks;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.PigeonImu; 


public class DriveTrain extends Subsystem {
	
	private CANTalon leftFront;
	private CANTalon rightFront;
	private CANTalon leftRear;
	private CANTalon rightRear;
	private RobotDrive drive;
	private DoubleSolenoid driveShifter;
	private boolean reverseDrive;
	private Relay gearLights;
	private Relay intakeLights;
	private PigeonImu gyro; 
	private PigeonImu.GeneralStatus gyroStatus; 
	private double [] gyroYPR; 
	private double leftRearCruiseVelocity; 
	private double leftRearAcceleration; 
	private double rightRearCruiseVelocity; 
	private double rightRearAcceleration; 
	private double leftRearSetPoint; 
	private double rightRearSetPoint; 
	
	private double targetSpeedLeft;
	private double targetSpeedRight;
	StringBuilder reportPIDLeft = new StringBuilder();
	StringBuilder reportPIDRight = new StringBuilder();
	
	public static DriveTrain driveTrain;

	public static DriveTrain getDriveTrain() {
		if (driveTrain == null) {
			driveTrain = new DriveTrain();
		}
		return driveTrain;
	}

	private DriveTrain() {

		leftRear = new CANTalon(RobotMap.DRIVETRAIN_LEFT_REAR);
		rightRear = new CANTalon(RobotMap.DRIVETRAIN_RIGHT_REAR);
		
		leftRear.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		leftRear.reverseSensor(false); 
		leftRear.configNominalOutputVoltage(+0.0F,-0.0F); 
		leftRear.configPeakOutputVoltage(+12.0f, -12.0f); 
		leftRear.setProfile(0);  
		leftRear.setF(RobotMap.DRIVETRAIN_LEFT_PID_F); 
		leftRear.setP(RobotMap.DRIVETRAIN_LEFT_PID_P); 
		leftRear.setI(RobotMap.DRIVETRAIN_LEFT_PID_I); 
		leftRear.setD(RobotMap.DRIVETRAIN_LEFT_PID_D); 
		
		rightRear.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		rightRear.reverseSensor(false); 
		rightRear.configNominalOutputVoltage(+0.0F,-0.0F); 
		rightRear.configPeakOutputVoltage(+12.0f, -12.0f); 
		rightRear.setProfile(0); 
		rightRear.setF(RobotMap.DRIVETRAIN_RIGHT_PID_F); 
		rightRear.setP(RobotMap.DRIVETRAIN_RIGHT_PID_P); 
		rightRear.setI(RobotMap.DRIVETRAIN_RIGHT_PID_I); 
		rightRear.setD(RobotMap.DRIVETRAIN_RIGHT_PID_D); 
		
		leftFront = new CANTalon(RobotMap.DRIVETRAIN_LEFT_FRONT);
		leftFront.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftFront.set(leftRear.getDeviceID());

		rightFront = new CANTalon(RobotMap.DRIVETRAIN_RIGHT_FRONT);
		rightFront.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightFront.set(rightRear.getDeviceID());

		drive = new RobotDrive(leftRear, rightRear);
		
		// setting the setSafetyEnabled out because there is a known issue
		// with the Motion Magic where the setpoint can get lost
		// as it gets starved for input
		drive.setSafetyEnabled(false);
		
		leftRear.enableBrakeMode(false);
		leftFront.enableBrakeMode(false);
		rightRear.enableBrakeMode(false);
		rightFront.enableBrakeMode(false);

		// Initialize the drive orientation.  We start with the orientation of
		// robot front = gear. 
		reverseDrive = false;  // note that pushing forward on the joystick returns negative values
	
		driveShifter = new DoubleSolenoid(RobotMap.PCM_MAIN, RobotMap.DRIVETRAIN_SHIFT_HIGH, RobotMap.DRIVETRAIN_SHIFT_LOW);
		// Start in low gear
//		this.setShifterLow();
		
		gyro = new PigeonImu(leftFront);
		gyroStatus = new PigeonImu.GeneralStatus();
		gyroYPR = new double[3];
		
		gearLights = new Relay(0);
		intakeLights = new Relay(1);
	
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithXboxJoysticks());
	}
	
    public boolean getDriveOrientation() {
    	return reverseDrive;
    }
    
    public void setShifterHigh() {
    	driveShifter.set(Value.kForward);
    	//SmartDashboard.putBoolean("High Gear", true);
    }
    
    public void setShifterLow() {
    	driveShifter.set(Value.kReverse);
    	//SmartDashboard.putBoolean("High Gear", false);
    }
    
    public void setDriveOrientation() {
    	// Reverse the current drive orientation
    	reverseDrive = !reverseDrive;
    	// update the dashboard to reflect the current drive orientation
    	//SmartDashboard.putBoolean("Drive Orientation: ", reverseDrive);
    }
    
    public void zeroEncoders(){
    	leftRear.setPosition(0);
    	rightRear.setPosition(0);
    }

    public DoubleSolenoid.Value getShifter() {
    	return driveShifter.get();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
    	if(reverseDrive){
    		drive.tankDrive(-1*rightSpeed, -1*leftSpeed, true);
    	} else {
    		drive.tankDrive(leftSpeed, rightSpeed, true);
    	}
//    	SmartDashboard.putNumber("leftRear", leftSpeed);
//    	SmartDashboard.putNumber("rightRear", rightSpeed);
    }
   
    public void setLights() {
    	if(reverseDrive) {
    		intakeLights.set(Relay.Value.kForward);
    		gearLights.set(Relay.Value.kReverse);
    	} else {
    		gearLights.set(Relay.Value.kForward);
    		intakeLights.set(Relay.Value.kReverse);
    	}
    }
    
    public double [] getGyro() {
    	gyro.GetYawPitchRoll(gyroYPR);
    	return gyroYPR;
    }
    
    public double getGyroYaw() {
    	gyro.GetYawPitchRoll(gyroYPR);
    	return gyroYPR[0];
    }
    
    public double getGyroPitch() {
    	gyro.GetYawPitchRoll(gyroYPR);
    	return gyroYPR[1];
    }
    
    public double getGyroRoll() {
    	gyro.GetYawPitchRoll(gyroYPR);
    	return gyroYPR[2];
    }
    
    public void setModePercentVbus () {
    	leftRear.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	leftRear.set(0);
       	rightRear.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	rightRear.set(0);
    }
    
    public void setModeMotionMagic() {
    	leftRear.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
    	leftRear.set(0);
    	rightRear.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
    	rightRear.set(0);
    	this.zeroEncoders();
    }
    
    public void setMotionMagicProfile (double leftCruise, double leftAccel, double rightCruise, double rightAccel ) {
    	leftRear.setMotionMagicCruiseVelocity(leftCruise);
    	leftRear.setMotionMagicAcceleration(leftAccel);
    	rightRear.setMotionMagicCruiseVelocity(rightCruise);
    	rightRear.setMotionMagicAcceleration(rightAccel);
    }
    
    public void setMotionMagicPosition(double leftPos, double rightPos) {
//    	System.out.println("Setting Position to Left: " + leftPos +" Right: "+ rightPos);
    	leftRear.set(leftPos);
    	rightRear.set(rightPos);
    }
    
	public double getPositionLeft() {
		return leftRear.getPosition();
	}
	
	public double getPositionRight() {
		return rightRear.getPosition();
	}
	
	public double getClosedLoopErrorRight () {
		return rightRear.getClosedLoopError();
	}
	
	public double getClosedLoopErrorLeft () {
		return leftRear.getClosedLoopError();
	}
	  
    //**********************************************
    // For tuning drivetrain PID
    //**********************************************
    public void setDriveLeftSpeed(double speed) {
    	// set an instance variable to the incoming speed value.
    	// It is referenced by the getPIDReport() method in this class
    	//targetSpeed = speed;
    	
    	// Ensure that we are in speed mode.  We need to do this
    	// since we currently stop the shooter using percentVbus
       	leftRear.changeControlMode(TalonControlMode.Speed);

       	// Publish the target speed to the dashboard
       	targetSpeedLeft = speed;
    	//SmartDashboard.putNumber("Left Drive Target RPM Value: ", targetSpeedLeft);

    	leftRear.set(speed);
    }
    
    public void setDriveRightSpeed(double speed) {
    	// set an instance variable to the incoming speed value.
    	// It is referenced by the getPIDReport() method in this class
    	//targetSpeed = speed;
    	
    	// Ensure that we are in speed mode.  We need to do this
    	// since we currently stop the shooter using percentVbus
       	rightRear.changeControlMode(TalonControlMode.Speed);

       	// Publish the target speed to the dashboard
       	targetSpeedRight = speed;
    	//SmartDashboard.putNumber("Right Drive Target RPM Value: ", targetSpeedRight);

    	rightRear.set(speed);
    }
    
    public void setDriveLeftPID(double FL, double PL, double IL, double DL, double speed){
    	leftRear.setF(FL);
    	leftRear.setP(PL);
    	leftRear.setI(IL);
    	leftRear.setD(DL);
    	targetSpeedLeft = speed;
//    	this.setDriveLeftSpeed(SmartDashboard.getNumber("Drive Left Target Speed: ", targetSpeedLeft));
    }

    public void setDriveRightPID(double FR, double PR, double IR, double DR, double speed){
    	rightRear.setF(FR);
    	rightRear.setP(PR);
    	rightRear.setI(IR);
    	rightRear.setD(DR);
    	targetSpeedRight = speed;
    	//this.setDriveRightSpeed(SmartDashboard.getNumber("Drive Right Target RPM Value: ", targetSpeedRight));
    }
    
    public double getDriveLeftSpeed() {
    	return leftRear.getMotionMagicActTrajVelocity();
    }
    
    public double getDriveLeftSpeed2() {
    	return leftRear.getSpeed();
    }
    public double getDriveRightSpeed() {
    	return rightRear.getMotionMagicActTrajVelocity();
    }
 
    public String getPIDReportLeft() {
    	
    	double motorOutput = leftRear.getOutputVoltage() / leftRear.getBusVoltage();
    	
    	reportPIDLeft.setLength(0);
    	
    	reportPIDLeft.append("\tout:");
    	reportPIDLeft.append(motorOutput);
    	reportPIDLeft.append("\tspd:");
    	reportPIDLeft.append(leftRear.get() );
    	reportPIDLeft.append("\terr:");
    	reportPIDLeft.append(leftRear.getClosedLoopError());
    	reportPIDLeft.append("\ttrg:");
/*    	reportPIDLeft.append(leftRear.getMotionMagicCruiseVelocity());
    	reportPIDLeft.append("\tactvel:");
    	reportPIDLeft.append(leftRear.getMotionMagicActTrajVelocity());
    	reportPIDLeft.append("\n");
*/
    	return reportPIDLeft.toString();
    }
    
    public String getPIDReportRight() {
    	
    	double motorOutput = rightRear.getOutputVoltage() / rightRear.getBusVoltage();
    	
    	reportPIDRight.setLength(0);
    	
    	reportPIDRight.append("\tout:");
    	reportPIDRight.append(motorOutput);
    	reportPIDRight.append("\tspd:");
    	reportPIDRight.append(rightRear.getSpeed() );
    	reportPIDRight.append("\terr:");
    	reportPIDRight.append(rightRear.getClosedLoopError());
    	reportPIDRight.append("\ttrg:");
    	reportPIDRight.append(targetSpeedRight);

    	return reportPIDRight.toString();
    }

}

package org.usfirst.frc.team68.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Relay;
//RobotDrive -> DifferentialDrive
//import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team68.robot.RobotMap;
import org.usfirst.frc.team68.robot.commands.DriveWithXboxJoysticks;

//import com.ctre.CANTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.CANTalon.TalonControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;

//import com.ctre.PigeonImu; 
import com.ctre.phoenix.sensors.PigeonIMU;


public class DriveTrain extends Subsystem {
	
	/* private CANTalon leftFront;
	private CANTalon rightFront;
	private CANTalon leftRear;
	private CANTalon rightRear; */
	
	private WPI_TalonSRX leftFront;
	private WPI_TalonSRX rightFront;
	private WPI_TalonSRX leftRear;
	private WPI_TalonSRX rightRear;
	
	//private RobotDrive drive;
	private DifferentialDrive drive;
	private DoubleSolenoid driveShifter;
	private boolean reverseDrive;
	private Relay gearLights;
	private Relay intakeLights;
	private PigeonIMU gyro; 
	private PigeonIMU.GeneralStatus gyroStatus; 
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

	//Get DriveTrain
	public static DriveTrain getDriveTrain() {
		if (driveTrain == null) {
			driveTrain = new DriveTrain();
		}
		return driveTrain;
	}

	private DriveTrain() {
		
		
		//Create leftRear and rightRear
		
		/*leftRear = new CANTalon(RobotMap.DRIVETRAIN_LEFT_REAR);
		rightRear = new CANTalon(RobotMap.DRIVETRAIN_RIGHT_REAR); (deprecated) */
		
		leftRear = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_REAR);
		rightRear = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_REAR);
		
		
		// Setting up leftRear
		/* Check arguments later */

		//leftRear.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative); (deprecated)
		leftRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		//leftRear.reverseSensor(false); 
		leftRear.setInverted(false);
		/* forward or reverse? */
		//leftRear.configNominalOutputVoltage(+0.0F,-0.0F);
		leftRear.configNominalOutputForward(0,0);
		
		//leftRear.configPeakOutputVoltage(+12.0f, -12.0f);
		leftRear.configPeakOutputForward(12, -12);
		
		//leftRear.setProfile(0);  
		leftRear.selectProfileSlot(0,0);	
		/*
		leftRear.setF(RobotMap.DRIVETRAIN_LEFT_PID_F); 
		leftRear.setP(RobotMap.DRIVETRAIN_LEFT_PID_P); 
		leftRear.setI(RobotMap.DRIVETRAIN_LEFT_PID_I); 
		leftRear.setD(RobotMap.DRIVETRAIN_LEFT_PID_D); 
		*/
		leftRear.config_kF(0, RobotMap.DRIVETRAIN_LEFT_PID_F, 0);
		leftRear.config_kP(0, RobotMap.DRIVETRAIN_LEFT_PID_P, 0);
		leftRear.config_kI(0, RobotMap.DRIVETRAIN_LEFT_PID_I, 0);
		leftRear.config_kD(0, RobotMap.DRIVETRAIN_LEFT_PID_D, 0);

		
		
		
		//Setting up rightRear
		
		/*rightRear.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Relative);
		rightRear.reverseSensor(false); 
		rightRear.configNominalOutputForward(+0.0F,-0.0F); 
		rightRear.configPeakOutputVoltage(+12.0f, -12.0f); 
		rightRear.setProfile(0); 
		rightRear.setF(RobotMap.DRIVETRAIN_RIGHT_PID_F); 
		rightRear.setP(RobotMap.DRIVETRAIN_RIGHT_PID_P); 
		rightRear.setI(RobotMap.DRIVETRAIN_RIGHT_PID_I); 
		rightRear.setD(RobotMap.DRIVETRAIN_RIGHT_PID_D); */
		
		rightRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		rightRear.setInverted(false);
		rightRear.configNominalOutputForward(0,0);
		rightRear.configPeakOutputForward(12, -12);
		rightRear.selectProfileSlot(0,0);	
		rightRear.config_kF(0, RobotMap.DRIVETRAIN_RIGHT_PID_F, 0);
		rightRear.config_kP(0, RobotMap.DRIVETRAIN_RIGHT_PID_P, 0);
		rightRear.config_kI(0, RobotMap.DRIVETRAIN_RIGHT_PID_I, 0);
		rightRear.config_kD(0, RobotMap.DRIVETRAIN_RIGHT_PID_D, 0);
		
		
		//Setting up leftFront
		/*leftFront = new CANTalon(RobotMap.DRIVETRAIN_LEFT_FRONT);
		leftFront.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftFront.set(leftRear.getDeviceID());  */
		
		leftFront = new WPI_TalonSRX(RobotMap.DRIVETRAIN_LEFT_FRONT);
		leftFront.set(ControlMode.Follower, leftRear.getDeviceID()); //check second argument

		//Setting up rightFront
		/* rightFront = new CANTalon(RobotMap.DRIVETRAIN_RIGHT_FRONT);
		rightFront.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightFront.set(rightRear.getDeviceID()); */
		rightFront = new WPI_TalonSRX(RobotMap.DRIVETRAIN_RIGHT_FRONT);
		rightFront.set(ControlMode.Follower,rightRear.getDeviceID()); 
		

		//drive = new RobotDrive(leftRear, rightRear); (old)
		drive = new DifferentialDrive(leftRear, rightRear);
		
		// setting the setSafetyEnabled out because there is a known issue
		// with the Motion Magic where the setpoint can get lost
		// as it gets starved for input
		drive.setSafetyEnabled(false);
		
		/* leftRear.enableBrakeMode(false);
		leftFront.enableBrakeMode(false);
		rightRear.enableBrakeMode(false);
		rightFront.enableBrakeMode(false); */
		
		leftRear.setNeutralMode(NeutralMode.Coast);
		leftFront.setNeutralMode(NeutralMode.Coast);
		rightRear.setNeutralMode(NeutralMode.Coast);
		rightFront.setNeutralMode(NeutralMode.Coast);
		
		// Initialize the drive orientation.  We start with the orientation of
		// robot front = gear. 
		reverseDrive = false;  // note that pushing forward on the joystick returns negative values
	
		//driveShifter = new DoubleSolenoid(RobotMap.PCM_MAIN, RobotMap.DRIVETRAIN_SHIFT_HIGH, RobotMap.DRIVETRAIN_SHIFT_LOW);
		// Start in low gear
//		this.setShifterLow();
		
		gyro = new PigeonIMU(leftFront);
		gyroStatus = new PigeonIMU.GeneralStatus();
		gyroYPR = new double[3];
		
		gearLights = new Relay(0);
		intakeLights = new Relay(1);
	
	}
	
	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithXboxJoysticks());
	}
	
    /*public boolean getDriveOrientation() {
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
    */
    public void zeroEncoders(){
    	//leftRear.setPosition(0);
    	leftRear.setSelectedSensorPosition(0, 0, 0);
    	//rightRear.setPosition(0);
    	rightRear.setSelectedSensorPosition(0, 0, 0);

    }

    /*public DoubleSolenoid.Value getShifter() {
    	return driveShifter.get();
    }*/

    public void tankDrive(double leftSpeed, double rightSpeed) {
    	if(reverseDrive){
    		drive.tankDrive(-1*rightSpeed, -1*leftSpeed, true);
    	} else {
    		drive.tankDrive(leftSpeed, rightSpeed, true);
    	}
//    	SmartDashboard.putNumber("leftRear", leftSpeed);
//    	SmartDashboard.putNumber("rightRear", rightSpeed);
    }
   
    /*public void setLights() {
    	if(reverseDrive) {
    		intakeLights.set(Relay.Value.kForward);
    		gearLights.set(Relay.Value.kReverse);
    	} else {
    		gearLights.set(Relay.Value.kForward);
    		intakeLights.set(Relay.Value.kReverse);
    	}
    }
    */
    public double [] getGyro() {
    	gyro.getYawPitchRoll(gyroYPR);
    	return gyroYPR;
    }
    
    public double getGyroYaw() {
    	gyro.getYawPitchRoll(gyroYPR);
    	return gyroYPR[0];
    }
    
    public double getGyroPitch() {
    	gyro.getYawPitchRoll(gyroYPR);
    	return gyroYPR[1];
    }
    
    public double getGyroRoll() {
    	gyro.getYawPitchRoll(gyroYPR);
    	return gyroYPR[2];
    }
    
    public void setModePercentVbus () {
    	/*leftRear.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	leftRear.set(0); Check arguments*/
    	
    	//Set ControlMode
    	leftRear.set(ControlMode.PercentOutput, 0);
    	    	
       	/* rightRear.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	rightRear.set(0); */
    	
    	rightRear.set(ControlMode.PercentOutput, 0);


    }
    
    public void setModeMotionMagic() {
    	
    	/*leftRear.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
    	leftRear.set(0); */
    	
    	leftRear.set(ControlMode.MotionMagic, 0);
    	
    	/*rightRear.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
    	rightRear.set(0); */
    	
    	rightRear.set(ControlMode.MotionMagic, 0);
    	
    	this.zeroEncoders();
    }
    
    //public void setMotionMagicProfile (double leftCruise, double leftAccel, double rightCruise, double rightAccel ) {
    	/* leftRear.setMotionMagicCruiseVelocity(leftCruise);
    	leftRear.setMotionMagicAcceleration(leftAccel);
    	rightRear.setMotionMagicCruiseVelocity(rightCruise);
    	rightRear.setMotionMagicAcceleration(rightAccel); */
    
    
    //Check argument for sensor raw units
    public void setMotionMagicProfile(int leftCruise, int leftAccel, int rightCruise, int rightAccel) {
    	leftRear.configMotionCruiseVelocity(leftCruise, 0);
    	leftRear.configMotionAcceleration(leftAccel, 0);
    	rightRear.configMotionCruiseVelocity(rightCruise, 0);
    	rightRear.configMotionAcceleration(rightAccel, 0);
    }
    
    public void setMotionMagicPosition(double leftPos, double rightPos) {
//    	System.out.println("Setting Position to Left: " + leftPos +" Right: "+ rightPos);
    	leftRear.set(leftPos);
    	rightRear.set(rightPos);
    }
    
	public double getPositionLeft() {
		//return leftRear.getPosition();
		return leftRear.getSelectedSensorPosition(0);

	}
	
	public double getPositionRight() {
		//return rightRear.getPosition();
		return rightRear.getSelectedSensorPosition(0);

	}
	
	public double getClosedLoopErrorRight() {
		return rightRear.getClosedLoopError(0); //0 = primary closed loop, 1 = cascaded closed loop
	}
	
	public double getClosedLoopErrorLeft() {
		return leftRear.getClosedLoopError(0);
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
       	
    	//leftRear.changeControlMode(TalonControlMode.Speed);

       	// Publish the target speed to the dashboard
       	targetSpeedLeft = speed;
    	//SmartDashboard.putNumber("Left Drive Target RPM Value: ", targetSpeedLeft);

    	leftRear.set(ControlMode.Velocity,speed);
    }
    
    public void setDriveRightSpeed(double speed) {
    	// set an instance variable to the incoming speed value.
    	// It is referenced by the getPIDReport() method in this class
    	//targetSpeed = speed;
    	
    	// Ensure that we are in speed mode.  We need to do this
    	// since we currently stop the shooter using percentVbus
       
    	//rightRear.changeControlMode(TalonControlMode.Speed);

       	// Publish the target speed to the dashboard
       	targetSpeedRight = speed;
    	//SmartDashboard.putNumber("Right Drive Target RPM Value: ", targetSpeedRight);

    	rightRear.set(ControlMode.Velocity,speed);
    }
    
    public void setDriveLeftPID(double FL, double PL, double IL, double DL, double speed){
    	/* leftRear.setF(FL);
    	leftRear.setP(PL);
    	leftRear.setI(IL);
    	leftRear.setD(DL); */
    	
    	leftRear.config_kF(0, FL, 0);
    	leftRear.config_kP(0, PL, 0);
    	leftRear.config_kI(0, IL, 0);
    	leftRear.config_kD(0, DL, 0);
    	
    	targetSpeedLeft = speed;
//    	this.setDriveLeftSpeed(SmartDashboard.getNumber("Drive Left Target Speed: ", targetSpeedLeft));
    }

    public void setDriveRightPID(double FR, double PR, double IR, double DR, double speed){
    	/* rightRear.setF(FR);
    	rightRear.setP(PR);
    	rightRear.setI(IR);
    	rightRear.setD(DR); */
    	
    	rightRear.config_kF(0, FR, 0);
    	rightRear.config_kP(0, PR, 0);
    	rightRear.config_kI(0, IR, 0);
    	rightRear.config_kD(0, DR, 0);

    	
    	targetSpeedRight = speed;
    	//this.setDriveRightSpeed(SmartDashboard.getNumber("Drive Right Target RPM Value: ", targetSpeedRight));
    }
    
    public double getDriveLeftSpeed() {
    	//return leftRear.getMotionMagicActTrajVelocity();
    	return leftRear.getActiveTrajectoryVelocity();
    }
    
    public double getDriveLeftSpeed2() {
    	//return leftRear.getSpeed();
    	return leftRear.getSelectedSensorVelocity(0); // 0 = closed, 1 = cascaded
    }
    
    public double getDriveRightSpeed() {
    	//return rightRear.getMotionMagicActTrajVelocity();
    	return rightRear.getActiveTrajectoryVelocity();
    }
 
/*    public String getPIDReportLeft() {
    	
    	double motorOutput = leftRear.getMotorOutputVoltage() / leftRear.getBusVoltage();
    	
    	reportPIDLeft.setLength(0);
    	
    	reportPIDLeft.append("\tout:");
    	reportPIDLeft.append(motorOutput);
    	reportPIDLeft.append("\tspd:");
    	reportPIDLeft.append(leftRear.get() );
    	reportPIDLeft.append("\terr:");
    	reportPIDLeft.append(leftRear.getClosedLoopError(0));
    	reportPIDLeft.append("\ttrg:");
    	reportPIDLeft.append(leftRear.getMotionMagicCruiseVelocity());
    	reportPIDLeft.append("\tactvel:");
    	reportPIDLeft.append(leftRear.getMotionMagicActTrajVelocity());
    	reportPIDLeft.append("\n");

    	return reportPIDLeft.toString();
    }
    
    public String getPIDReportRight() {
    	
    	double motorOutput = rightRear.getMotorOutputVoltage() / rightRear.getBusVoltage();
    	
    	reportPIDRight.setLength(0);
    	
    	reportPIDRight.append("\tout:");
    	reportPIDRight.append(motorOutput);
    	reportPIDRight.append("\tspd:");
    	//reportPIDRight.append(rightRear.getSpeed() );
    	reportPIDRight.append(rightRear.getSelectedSensorVelocity(0));
    	reportPIDRight.append("\terr:");
    	reportPIDRight.append(rightRear.getClosedLoopError(0));
    	reportPIDRight.append("\ttrg:");
    	reportPIDRight.append(targetSpeedRight);

    	return reportPIDRight.toString();
    }*/

}

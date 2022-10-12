package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.PWM;

import edu.wpi.first.wpilibj.PowerDistribution;

import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.cscore.CvSink;
//import edu.wpi.cscore.CvSource;

//import edu.wpi.first.hal.sim.mockdata.DriverStationDataJNI;

public class Robot extends TimedRobot {

    Port port = Port.kOnboard;
    String print;

    private Joystick Pilot_Stick, Co_Pilot_Stick1, Co_Pilot_Stick2;

    //private pixy_Controller 


    private CANSparkMax leftMotor, leftFollowMotor, rightMotor, rightFollowMotor;
    private CANSparkMax pickupMotor, magazineMotor, turretMotor, shooterMotor;
    private CANSparkMax climber_Right_Motor, climber_Left_Motor;
    // private CANSparkMax sweeper_Motor_Left, sweeper_Motor_Right;

    private SparkMaxPIDController shooter_controller, turret_controller;
    private SparkMaxPIDController drive_pidController_left, drive_pidController_right;

    private RelativeEncoder shooter_encoder, turret_encoder;
    private RelativeEncoder LF_Motor_encoder, LR_Motor_encoder, RF_Motor_encoder, RR_Motor_encoder;
    private RelativeEncoder climber_Left_Motor_encoder, climber_Right_Motor_encoder;

    private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

    private static ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);

    private AnalogInput Pixy_Input;

    private PWM LightStrip;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, MaxRPM;

    private Double X, Y, invX, V, W, R, L, turret_speed, DriveAdjust, magazine_speed;

    private int Target_X_Avg, Target_Y_Avg, Turret_Start_Position;;

    private Timer timer;

    private PowerDistribution PDP = new PowerDistribution(1, ModuleType.kRev);

    private PneumaticHub pHub = new PneumaticHub(1);

    private DoubleSolenoid Gear_Shifter = pHub.makeDoubleSolenoid(1, 0);
    private DoubleSolenoid Pickup_Shifter = pHub.makeDoubleSolenoid(2, 3);

    @Override
    public void robotInit() {

	timer = new Timer(); 
	CameraServer.startAutomaticCapture();

	Pixy_Input = new AnalogInput(0);

	LightStrip = new PWM(0);

	//added climber booleans after Richmond

	DriveAdjust = 0.0;

	Pilot_Stick = new Joystick(Constants.USB_STICK_PILOT);
	Co_Pilot_Stick1 = new Joystick(Constants.USB_STICK_COPILOT1);
	Co_Pilot_Stick2 = new Joystick(Constants.USB_STICK_COPILOT2);

	CameraServer.startAutomaticCapture();

	/**********************************************************************************/
	m_gyro.calibrate();

	pHub.enableCompressorAnalog(110.0, 120.0);

	leftMotor = new CANSparkMax(Constants.CANIDs.DRIVE_LEFT_LEADER.getid(), MotorType.kBrushless);
	leftFollowMotor = new CANSparkMax(Constants.CANIDs.DRIVE_LEFT_FOLLOWER.getid(), MotorType.kBrushless);
	rightMotor = new CANSparkMax(Constants.CANIDs.DRIVE_RIGHT_LEADER.getid(), MotorType.kBrushless);
	rightFollowMotor = new CANSparkMax(Constants.CANIDs.DRIVE_RIGHT_FOLLOWER.getid(), MotorType.kBrushless);

	leftMotor.restoreFactoryDefaults();
	leftFollowMotor.restoreFactoryDefaults();
	rightMotor.restoreFactoryDefaults();
	rightFollowMotor.restoreFactoryDefaults();

	LF_Motor_encoder = leftMotor.getEncoder();
	LR_Motor_encoder = leftFollowMotor.getEncoder();
	RF_Motor_encoder = rightMotor.getEncoder();
	RR_Motor_encoder = rightFollowMotor.getEncoder();

	leftFollowMotor.follow(leftMotor);
	rightFollowMotor.follow(rightMotor);

	drive_pidController_left = leftMotor.getPIDController();
	drive_pidController_right = rightMotor.getPIDController();

	drive_pidController_left.setP(Constants.kP_Drive_Pixy);
	drive_pidController_left.setI(Constants.kI_Drive_Pixy);
	drive_pidController_left.setD(Constants.kD_Drive_Pixy);
	drive_pidController_left.setIZone(0.0);
	drive_pidController_left.setFF(0.0);
	drive_pidController_left.setOutputRange(-1.0, 1.0);

	drive_pidController_right.setP(Constants.kP_Drive_Pixy);
	drive_pidController_right.setI(Constants.kI_Drive_Pixy);
	drive_pidController_right.setD(Constants.kD_Drive_Pixy);
	drive_pidController_right.setIZone(0.0);
	drive_pidController_right.setFF(0.0);
	drive_pidController_right.setOutputRange(-1.0, 1.0);

	pickupMotor = new CANSparkMax(Constants.CANIDs.PICKUP.getid(), MotorType.kBrushless);
	pickupMotor.restoreFactoryDefaults();

	//sweeper_Motor_Left = new CANSparkMax(55, MotorType.kBrushless);
	//sweeper_Motor_Left.restoreFactoryDefaults();
	//sweeper_Motor_Left.setInverted(true);

	//sweeper_Motor_Right = new CANSparkMax(56, MotorType.kBrushless);
	//sweeper_Motor_Right.restoreFactoryDefaults();

	magazineMotor = new CANSparkMax(Constants.CANIDs.MAGAZINE.getid(), MotorType.kBrushless);
	magazineMotor.restoreFactoryDefaults();

	turretMotor = new CANSparkMax(Constants.CANIDs.TURRET_DIRECTION.getid(), MotorType.kBrushless);
	turretMotor.restoreFactoryDefaults();
	turretMotor.setIdleMode(IdleMode.kBrake); //added after Richmond


	turretMotor.setSoftLimit(SoftLimitDirection.kForward, 7.0f);   //was 340
	turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -310.0f); 
	turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
	turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true); 

	turret_controller = turretMotor.getPIDController();
	turret_controller.setP(Constants.kP_Turret_Position);
	turret_controller.setI(Constants.kI_Turret_Position);
	turret_controller.setD(Constants.kD_Turret_Position);
	turret_controller.setIZone(Constants.klz_Turret_Position);
	turret_controller.setFF(Constants.kFF_Turret_Position);
	turret_controller.setOutputRange(Constants.kMinOutput_Turret_Position, Constants.kMaxOutput_Turret_Position);

	turret_encoder = turretMotor.getEncoder();
	turret_encoder.setPosition(0.0);

	shooterMotor = new CANSparkMax(Constants.CANIDs.TURRET_SHOOTER.getid(), MotorType.kBrushless);
	shooterMotor.restoreFactoryDefaults();
	shooter_encoder = shooterMotor.getEncoder();

	// set PID coefficients
	shooter_controller = shooterMotor.getPIDController();     
	shooter_controller.setP(Constants.kP_Shooter);
	shooter_controller.setI(Constants.kI_Shooter);
	shooter_controller.setD(Constants.kD_Shooter);
	shooter_controller.setIZone(Constants.klz_Shooter);
	shooter_controller.setFF(Constants.kFF_Shooter);
	shooter_controller.setOutputRange(Constants.kMinOutput_Shooter, Constants.kMaxOutput_Shooter);

	climber_Left_Motor  = new CANSparkMax(Constants.CANIDs.CLIMBER_LEFT.getid(), MotorType.kBrushless);
	climber_Right_Motor = new CANSparkMax(Constants.CANIDs.CLIMBER_RIGHT.getid(), MotorType.kBrushless);
	climber_Left_Motor.restoreFactoryDefaults();
	climber_Left_Motor.setInverted(true);              ////////////////////////////////
	climber_Right_Motor.restoreFactoryDefaults();

	climber_Left_Motor_encoder = climber_Left_Motor.getEncoder();

	climber_Left_Motor.setSoftLimit(SoftLimitDirection.kForward, 666.0f);   
	climber_Left_Motor.setSoftLimit(SoftLimitDirection.kReverse, 0.0f); 
	climber_Left_Motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
	climber_Left_Motor.enableSoftLimit(SoftLimitDirection.kForward, true); 

	climber_Left_Motor_encoder.setPosition(0.0);

	climber_Right_Motor_encoder = climber_Right_Motor.getEncoder();

	climber_Right_Motor.setSoftLimit(SoftLimitDirection.kForward, 666.0f);   
	climber_Right_Motor.setSoftLimit(SoftLimitDirection.kReverse, 0.0f); 
	climber_Right_Motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
	climber_Right_Motor.enableSoftLimit(SoftLimitDirection.kForward, true); 

	climber_Right_Motor_encoder.setPosition(0.0);


    }

    @Override
    public void teleopPeriodic() {



	//  Drive Junk
	Drive();



	SmartDashboard.putNumber("RPM", shooter_encoder.getVelocity());

	// Gear Shifting           
	if (Pilot_Stick.getRawButton(Constants.PILOT_BUTTON_1)) 
	    Gear_Shifter.set(DoubleSolenoid.Value.kReverse);
	else 
	    Gear_Shifter.set(DoubleSolenoid.Value.kForward);

	rightMotor.set(R - DriveAdjust);
	leftMotor.set(L - DriveAdjust);

	// Pickup Commands
	if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_PICKUP)) {
	    pickupMotor.set(0.5);
	    Pickup_Shifter.set(DoubleSolenoid.Value.kForward);

	} else if (Co_Pilot_Stick2.getRawButton(Constants.COPILOT2_PICKUP_REV)) {
	    Pickup_Shifter.set(DoubleSolenoid.Value.kReverse);
	    pickupMotor.set(0.8);
	    //     sweeper_Motor_Left.set(1.0);
	    //   sweeper_Motor_Right.set(1.0);
	} else 
	{     pickupMotor.set(0.0);
	    //       sweeper_Motor_Right.set(0.0);
	    //     sweeper_Motor_Left.set(0.0);
	    Pickup_Shifter.set(DoubleSolenoid.Value.kForward);
	}
	//  Climber Commands

	if (Co_Pilot_Stick2.getRawButton(Constants.COPILOT2_ClIMBER_DEPLOY)) {
	    climber_Left_Motor.set(1.0); 
	    climber_Right_Motor.set(1.0);
	} else if (Co_Pilot_Stick2.getRawButton(Constants.COPILOT2_CLIMBER_RETRACT)) {
	    climber_Left_Motor.set(-1.0); 
	    climber_Right_Motor.set(-1.0);
	} else {
	    climber_Left_Motor.set(0.0); 
	    climber_Right_Motor.set(0.0);
	}

	//      Magazine Commands

	if ((Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_MAGAZINE_DOWN)) || (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_SHOOT)))
	    magazineMotor.set(0.5);
	else {
	    if (Co_Pilot_Stick2.getRawButton(Constants.COPILOT2_MAGAZINE_UP)) {
		magazineMotor.set(-0.3);
	    } else {
		magazineMotor.set(0.0);
	    }
	}


	//   if ((Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_MAGAZINE_DOWN)) || (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_SHOOT)))
	//       magazineMotor.set(0.5 + magazine_speed);
	//       else {
	//                    if (Co_Pilot_Stick2.getRawButton(Constants.COPILOT2_MAGAZINE_UP)) {
	//                          magazineMotor.set(-0.3 - magazine_speed);
	//                    } else {
	//                          magazineMotor.set(0.0);
	//                    }
	//              }

	// Turret Commands

	double turret_ratio;
	if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_FINE_ADJUST))
	{turret_ratio = .10;}
	else  {turret_ratio = 0.5;}

	if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_LEFT)) 
	    turretMotor.set(1.0 * turret_ratio); 
	else  if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_RIGHT)) 
	    turretMotor.set(-1.0 * turret_ratio);    
	else  if (!Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_LEFT) && !Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_RIGHT)) 
	    turretMotor.set(0.0);    

	// Shooter Commands
	// double Shooter_Speed = SHOOTER_SPEED_TARMAC;
	if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_SHOOT_TARMAC)) {
	    shooter_controller.setReference(Constants.SHOOTER_SPEED_TARMAC,CANSparkMax.ControlType.kVelocity);

	}
	else  if (Co_Pilot_Stick2.getRawButton(Constants.COPILOT2_SHOOT_CARGO))
	{shooter_controller.setReference(Constants.SHOOTER_SPEED_CARGO_RING,CANSparkMax.ControlType.kVelocity);
	    LightStrip.setPosition(1000);
	}
	else if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_SHOOT_PAD1)) {
	    shooter_controller.setReference(Constants.SHOOTER_SPEED_PAD1,CANSparkMax.ControlType.kVelocity);

	}
	else if (Co_Pilot_Stick2.getRawButton(Constants.COPILOT2_LOWER_HUB)) {
	    shooter_controller.setReference(Constants.SHOOTER_SPEED_LOWER_HUB,CANSparkMax.ControlType.kVelocity);

	}
	else if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_SHOOT_STOP)) {
	    shooter_controller.setReference(0, CANSparkMax.ControlType.kVelocity);

	}      

	if (Pilot_Stick.getRawButton(Constants.PILOT_BUTTON_2)) {
	    double Pixy_Avg_Volt = Pixy_Input.getAverageVoltage();
	    if ((Pixy_Avg_Volt > .15) && (Pixy_Avg_Volt < 3.0)) {
		DriveAdjust = -(Pixy_Avg_Volt - 1.65)/1.65 * 0.4;    /// start at 0.4
	    } else {
		DriveAdjust = 0.0;
	    }
	} else {
	    DriveAdjust = 0.0;
	}
    }

    public void PR_Dashboard() {
	SmartDashboard.putNumber("Pressure", pHub.getPressure(0));
	SmartDashboard.putNumber("Voltage", PDP.getVoltage());

	SmartDashboard.putNumber("FLMT", leftMotor.getMotorTemperature()*9/5+32);
	SmartDashboard.putNumber("RLMT", leftFollowMotor.getMotorTemperature()*9/5+32);

	SmartDashboard.putNumber("FRMT", rightMotor.getMotorTemperature()*9/5+32);
	SmartDashboard.putNumber("RRMT", rightFollowMotor.getMotorTemperature()*9/5+32);

	SmartDashboard.putNumber("PUMT", pickupMotor.getMotorTemperature()*9/5+32);

	//  SmartDashboard.putNumber("SMT", shooterMotor.getMotorTemperature());

	SmartDashboard.putNumber("RPM", shooter_encoder.getVelocity());
	SmartDashboard.putNumber("Drive Position", LF_Motor_encoder.getPosition());

	SmartDashboard.putNumber("Pixy Value", Pixy_Input.getAverageVoltage());
	SmartDashboard.putNumber("Drvie Adjust", DriveAdjust);

	SmartDashboard.putNumber("LC Pos", climber_Left_Motor_encoder.getPosition());

    }
    public void Drive() {

	// Y = Pilot_Stick.getX(GenericHID.Hand.kLeft);
	Y = Pilot_Stick.getX();
	// X = Pilot_Stick.getY(GenericHID.Hand.kLeft);
	X = Pilot_Stick.getY();

	if (Math.abs(X) < .15)
	    X = 0.0;
	if (Math.abs(Y) < .15)
	    Y = 0.0;

	// X = X;
	V = ((1.0 - Math.abs(X)) * Y) + Y;
	W = ((1.0 - Math.abs(Y)) * X) + X;
	R = (V + W) / 4; // / 4;
	L = (V - W) / 4; // / 4;

    }

    @Override
    public void  testInit() {

	turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
	turretMotor.enableSoftLimit(SoftLimitDirection.kForward, false); 

	climber_Right_Motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
	climber_Right_Motor.enableSoftLimit(SoftLimitDirection.kForward, false); 

	climber_Left_Motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
	climber_Left_Motor.enableSoftLimit(SoftLimitDirection.kForward, false); 

    }

    @Override
    public void testPeriodic() {

	double climber_up_speed = .3;
	double climber_down_speed = -.3;

	PR_Dashboard();

	////  This is used run left climber  Mag up and down buttons 
	if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_MAGAZINE_DOWN))
	    climber_Left_Motor.set(climber_down_speed); 
	else if (Co_Pilot_Stick2.getRawButton(Constants.COPILOT2_MAGAZINE_UP)) 
	    climber_Left_Motor.set(climber_up_speed); 
	else 
	    climber_Left_Motor.set(.0); 


	////  This is used run climber using the Tarmac and Cargo Ring buttons
	if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_SHOOT_TARMAC)) 
	    climber_Right_Motor.set(climber_down_speed);
	else  if (Co_Pilot_Stick2.getRawButton(Constants.COPILOT2_SHOOT_CARGO)) 
	    climber_Right_Motor.set(climber_up_speed);
	else 
	    climber_Right_Motor.set(.0);

	// Turret Commands

	double turret_ratio;
	if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_FINE_ADJUST))
	{turret_ratio = .10;}
	else  {turret_ratio = 0.5;}

	if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_LEFT)) 
	    turretMotor.set(1.0 * turret_ratio); 
	else  if (Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_RIGHT)) 
	    turretMotor.set(-1.0 * turret_ratio);    
	else  if (!Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_LEFT) && !Co_Pilot_Stick1.getRawButton(Constants.COPILOT1_TURRET_RIGHT)) 
	    turretMotor.set(0.0);    



    }           

    @Override

    public void autonomousInit() {

	timer.reset();
	timer.start();

	RF_Motor_encoder.setPosition(0);
	LF_Motor_encoder.setPosition(0);

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

	autonomousShootPickupBalls();
	//      autonomousTest();

    }

    private void autonomousShootPickupBalls() {
	/// Turns on Shooter for Tarmac shot
	if (timer.get() <= 1.5) {  

	    Pickup_Shifter.set(DoubleSolenoid.Value.kReverse);
	    shooter_controller.setReference(Constants.SHOOTER_SPEED_TARMAC,CANSparkMax.ControlType.kVelocity);

	} else if (timer.get() <= 10.0) {        
	    magazineMotor.set(0.8);            

	} else if (timer.get() <= 12.0) {

	    shooter_controller.setReference(Constants.SHOOTER_SPEED_CARGO_RING,CANSparkMax.ControlType.kVelocity);
	    pickupMotor.set(0.8);
	    drive_pidController_left.setReference(8,CANSparkMax.ControlType.kPosition);
	    drive_pidController_right.setReference(-8,CANSparkMax.ControlType.kPosition);

	} else  if (timer.get() <= 15.0) { 
	    magazineMotor.set(1.0);

	    Pickup_Shifter.set(DoubleSolenoid.Value.kForward);
	    pickupMotor.set(0.5);

	} else {  
	    pickupMotor.set(0.0);
	    magazineMotor.set(0.0); 
	    shooter_controller.setReference(0.0,CANSparkMax.ControlType.kVelocity);
	}


    }

    private void autonomousTest() {


    } 




}




//mport org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;

//import edu.wpi.first.cscore.CvSink;
//import edu.wpi.first.cscore.CvSource;



// // Get a CvSink. This will capture Mats from the camera
// CvSink cvSink = CameraServer.getVideo();

// // Setup a CvSource. This will send images back to the Dashboard
// CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

// Mat mat = new Mat();
// // add a vertical line at center and three horizontal lines for range finding
// Imgproc.line(mat, new Point(320,100), new Point(320,400), new Scalar(0,0,0),5);

// Imgproc.line(mat, new Point(320-200,      20),
//                   new Point(320+200,      20), 
//                   new Scalar(255,0,0), 5);
// Imgproc.line(mat, new Point(320-130,165),
//                   new Point(320+130,165), 
//                   new Scalar(255,125,0), 5);
// Imgproc.line(mat, new Point(320-100,220),
//                   new Point(320+100,220), 
//                   new Scalar(0,0,0), 5);
// outputStream.putFrame(mat);

package frc.team5406.subsystems;

import frc.team3256.warriorlib.subsystem.DriveTrainBase;
import frc.team3256.warriorlib.math.Rotation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team5406.robot.Constants;

public class Drive extends DriveTrainBase {
    private static Drive instance;
    private CANSparkMax leftDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax leftDriveSlave1 = new CANSparkMax(4, MotorType.kBrushless);
    private CANSparkMax rightDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax rightDriveSlave1 = new CANSparkMax(2, MotorType.kBrushless);
    private CANEncoder leftEncoder, rightEncoder;
    private CANPIDController leftDrivePID, rightDrivePID;  

    Solenoid shiftSolenoid = new Solenoid(Constants.SHIFT_SOLENOID);
    DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
  
    public boolean highGear = false;
  
    public boolean llHasValidTarget = false;
    public double llSteer = 0.0;
  
    public double llLastError = 0; 
    public double llTotalError = 0;
  
    
    public Drive() {
        leftEncoder=leftDriveMotor.getEncoder();
        rightEncoder=rightDriveMotor.getEncoder();
        leftDriveSlave1.follow(leftDriveMotor);
        rightDriveSlave1.follow(rightDriveMotor);
        rightDriveSlave1.setInverted(true);
        rightDriveMotor.setInverted(true);
        //leftDriveSlave1.setInverted(true);
        //leftDriveMotor.setInverted(true);
        leftDriveMotor.setSmartCurrentLimit(80);
        leftDriveSlave1.setSmartCurrentLimit(80);
        rightDriveMotor.setSmartCurrentLimit(80);
        rightDriveSlave1.setSmartCurrentLimit(80);

        leftDrivePID = leftDriveMotor.getPIDController();
        rightDrivePID = rightDriveMotor.getPIDController();
        drive.setSafetyEnabled(false);

        leftDrivePID.setP(3e-4, 0);
        leftDrivePID.setI(1e-6, 0);
        leftDrivePID.setD(0.002, 0);
        leftDrivePID.setIZone(0, 0);
        leftDrivePID.setFF(0.000156, 0);
        leftDrivePID.setOutputRange(-1, 1, 0);
    
        rightDrivePID.setP(3e-4, 0);
        rightDrivePID.setI(1e-6, 0);
        rightDrivePID.setD(0.002, 0);
        rightDrivePID.setIZone(0, 0);
        rightDrivePID.setFF(0.000156, 0);
        rightDrivePID.setOutputRange(-1, 1, 0);

        leftDriveMotor.setClosedLoopRampRate(0.05);
        rightDriveMotor.setClosedLoopRampRate(0.05);
        
        }
    
	public void init(double timestamp) {
    }
    
    public void update(double timestamp) {

	}

	public void end(double timestamp) {
	}

    public static Drive getInstance() {
		return instance == null ? instance = new Drive() : instance;
    }
    
	/**
	 * @return left encoder distance value
	 */
	public double getRightDistance(){
        return -1*leftEncoder.getPosition();

    }

	/**
	 * @return right encoder distance value
	 */
	public double getLeftDistance(){
        return -1*rightEncoder.getPosition();
    }

	/**
	 * NOTE: for typical implementation, angle should start at 90 degrees rather than zero. The idea is to go forward to increase y, and go right to increase x
	 * @return {@link frc.team3256.warriorlib.math.Rotation} that represents current gyro angle
	 */
	public Rotation getRotationAngle(){
        double angle = ((-1 * Constants.navX.getAngle()) + 90);
        return Rotation.fromDegrees(angle);
    }

	/**
	 * @return velocity of left side of robot
	 */
	public double getRightVelocity(){
        return -1*Constants.INCHES_PER_TICK*leftEncoder.getVelocity()/60;

    }

	/**
	 * @return velocity of right side of robot
	 */
	public double getLeftVelocity(){
        return -1*Constants.INCHES_PER_TICK*rightEncoder.getVelocity()/60;
    }

	/**
	 * Sets left and right velocities of the robot
	 * @param left left velocity
	 * @param right right velocity
	 */
	public void setVelocityClosedLoop(double left, double right){
       //System.out.println(60*left/Constants.INCHES_PER_TICK + ", " + 60*right/Constants.INCHES_PER_TICK);
        leftDrivePID.setReference(-60*right/Constants.INCHES_PER_TICK, ControlType.kVelocity, 0);
        rightDrivePID.setReference(-60*left/Constants.INCHES_PER_TICK, ControlType.kVelocity, 0);
    }

	/**
	 * Resets encoders to zero
	 */
	public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

	/**
	 * Resets gyro angle to zero
	 */
    public void resetGyro(){
        Constants.navX.zeroYaw();
    }
    
    public void outputToDashboard(){

    }

	public void zeroSensors(){
        resetGyro();
        resetEncoders();

    }

    public void shiftHigh() {
        shiftSolenoid.set(Constants.SHIFT_HIGH);
          highGear = true;
      }
      public void shiftLow() {
        shiftSolenoid.set(Constants.SHIFT_LOW);
          highGear = false;
      }
    
      public void arcadeDrive(double speed, double turn, boolean slow){
    
        double precisionDriveX;
        double precisionDriveY;
      
      if(slow){
            precisionDriveY = 1*0.5;
            precisionDriveX = 0.5;
          }else{
            precisionDriveY = 1;
            precisionDriveX = 1;
          }
    
      drive.arcadeDrive(precisionDriveX*turn, -1*precisionDriveY*speed);
      }
      
      public void cheesyDrive(double throttle, double turning, boolean slow){
    
        boolean quickTurn = false;
    
        //Driver Controls
        if(slow){
          throttle *= 0.5;
          turning *= 0.5;
        }
        
        if(Math.abs(turning) < 0.05){
          turning = 0;
        }
    
        if(Math.abs(throttle) < 0.03){
          quickTurn = true;
        }else{
          quickTurn = false;
        }
    
        drive.curvatureDrive(throttle, turning, quickTurn);
    
        }
    
        public void updateLimelightTracking()
        {
              double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
              double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
            // ts: close to 0 - left, close to 90 - right
            // tx: negative - left, positive - right
      
            // These numbers must be tuned for your Robot!  Be careful!
            final double STEER_KP = 0.08;                    // how hard to turn toward the target
            final double STEER_KD = 0.005;
            final double STEER_KI = 0.1;
            final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
      
              if (tv < 1.0)
              {
                llHasValidTarget = false;
                llSteer = 0.0;
                return;
              }
      
              llHasValidTarget = true;
              llTotalError += tx;
      
              // Start with proportional steering
              llSteer = tx * STEER_KP + STEER_KD * (tx - llLastError) / 0.02 + STEER_KI * llTotalError * 0.02;
      
              // try to drive forward until the target area reaches our desired area
              llLastError = tx;
              if (Math.abs(llSteer) > MAX_DRIVE)
              {
                llSteer = Math.signum(llSteer) * MAX_DRIVE;
              }
        }

          public double maxllArea(double angle){
   return -0.0054*angle*angle + 0.6546*angle - 12.084;
  }
      
}
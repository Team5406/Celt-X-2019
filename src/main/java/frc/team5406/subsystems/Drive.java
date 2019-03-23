package frc.team5406.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.*;


public class Drive extends Subsystems{

  CANSparkMax leftDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax leftDriveSlave1 = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax rightDriveSlave1 = new CANSparkMax(2, MotorType.kBrushless);

  Solenoid shiftSolenoid = new Solenoid(Constants.SHIFT_SOLENOID);
  DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

  public boolean highGear = false;

  public boolean llHasValidTarget = false;
  public double llSteer = 0.0;

  public double llLastError = 0; 
  public double llTotalError = 0;


public Drive(){

  leftDriveSlave1.follow(leftDriveMotor);
  rightDriveSlave1.follow(rightDriveMotor);
  leftDriveMotor.setSmartCurrentLimit(80);
  leftDriveSlave1.setSmartCurrentLimit(80);
  rightDriveMotor.setSmartCurrentLimit(80);
  rightDriveSlave1.setSmartCurrentLimit(80);

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

  drive.arcadeDrive(precisionDriveY*speed, precisionDriveX*turn);
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
        final double STEER_KP = 0.15;                    // how hard to turn toward the target
        final double STEER_KD = 0.003;
        final double STEER_KI = 0.08;
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
  
  }

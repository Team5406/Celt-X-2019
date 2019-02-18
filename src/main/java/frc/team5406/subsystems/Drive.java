package frc.team5406.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;

public class Drive extends Subsystems{

  CANSparkMax leftDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftDriveSlave1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax rightDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightDriveSlave1 = new CANSparkMax(4, MotorType.kBrushless);

  Solenoid shiftSolenoid = new Solenoid(Constants.SHIFT_SOLENOID);
  DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

  public boolean highGear = false;


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
  /* public void cheesyDrive(double acc, double dec, double turn, boolean slow){

   double throttle, turning;
    boolean quickTurn = false;

    //Driver Controls
    if(slow){
      throttle = (acc - dec)*0.5;
      turning = turn*0.5;
    }else{
      throttle = acc - dec;
      turning = turn;
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

    

    } */
}
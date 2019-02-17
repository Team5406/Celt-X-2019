package frc.team5406.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;

public class Drive extends Subsystems{

  WPI_TalonSRX leftDriveMotor = new WPI_TalonSRX(1);
  WPI_VictorSPX leftDriveSlave1 = new WPI_VictorSPX(2);
  WPI_VictorSPX leftDriveSlave2 = new WPI_VictorSPX(3);

  WPI_TalonSRX rightDriveMotor = new WPI_TalonSRX(4);
  WPI_VictorSPX rightDriveSlave1= new WPI_VictorSPX(5);
  WPI_VictorSPX rightDriveSlave2 = new WPI_VictorSPX(6);

  Solenoid shiftSolenoid = new Solenoid(Constants.SHIFT_SOLENOID);
  DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

  public boolean highGear = false;


public Drive(){
    leftDriveSlave1.follow(leftDriveMotor);
    leftDriveSlave2.follow(leftDriveMotor);
    rightDriveSlave1.follow(rightDriveMotor);
    rightDriveSlave2.follow(rightDriveMotor);
  
    leftDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
    leftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
  
    rightDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
    rightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
  
    leftDriveMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    rightDriveMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
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
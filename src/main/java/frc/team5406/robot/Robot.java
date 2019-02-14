package frc.team5406.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.team5406.util.XboxController;
import edu.wpi.first.wpilibj.Solenoid;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import frc.team5406.subsystems.Gamepieces;


public class Robot extends TimedRobot {
  private Gamepieces gamepieceHandler = new Gamepieces();

  public boolean highGear = false;
  Solenoid shiftSolenoid = new Solenoid(Constants.SHIFT_SOLENOID);
  
  public void shiftHigh() {
    shiftSolenoid.set(Constants.SHIFT_HIGH);
      highGear = true;
  }
  public void shiftLow() {
    shiftSolenoid.set(Constants.SHIFT_LOW);
      highGear = false;
  }

  int climbCount = 0;

  XboxController driverGamepad = new XboxController(1);
  XboxController operatorGamepad = new XboxController(0);

  WPI_TalonSRX leftDriveMotor = new WPI_TalonSRX(1);
  WPI_VictorSPX leftDriveSlave1 = new WPI_VictorSPX(2);
  WPI_VictorSPX leftDriveSlave2 = new WPI_VictorSPX(3);

  WPI_TalonSRX rightDriveMotor = new WPI_TalonSRX(4);
  WPI_VictorSPX rightDriveSlave1= new WPI_VictorSPX(5);
  WPI_VictorSPX rightDriveSlave2 = new WPI_VictorSPX(6);

  DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
 
  @Override
  public void robotInit() {

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

  @Override
  public void teleopInit(){
    climbCount = 0;
  }

  @Override
  public void teleopPeriodic() {
   // compressor.stop();
    double throttle, turning;
    boolean quickTurn = false;

    //Driver Controls
    if(driverGamepad.getButtonHeld(XboxController.A_BUTTON)){
      throttle = (driverGamepad.getRightTrigger() - driverGamepad.getLeftTrigger())*0.5;
      turning = (driverGamepad.getLeftX())*0.5;
    }else{
      throttle = driverGamepad.getRightTrigger() - driverGamepad.getLeftTrigger();
      turning = driverGamepad.getLeftX();
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

    if(driverGamepad.getButtonHeld(XboxController.B_BUTTON)){
      shiftHigh();
    }else{
      shiftLow();
    }
    if(operatorGamepad.getRightTriggerPressed()){
        gamepieceHandler.scoreCargo();
    }else if(driverGamepad.getButtonHeld(XboxController.X_BUTTON)){
        gamepieceHandler.intake();
    }else if(driverGamepad.getButtonHeld(XboxController.Y_BUTTON)){
        gamepieceHandler.reverseIntake();
    }else{
        gamepieceHandler.intakeDefault();
    }

    if(driverGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
      gamepieceHandler.armUp();
    }

    if(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
      gamepieceHandler.armIntake();
    }else if(Math.abs(driverGamepad.getRightY())>0.2 ) {
      gamepieceHandler.manualArm(driverGamepad.getRightY());
    }else{
      gamepieceHandler.armUp();
    }

    if(driverGamepad.getButtonHeld(XboxController.START_BUTTON) && driverGamepad.getButtonHeld(XboxController.BACK_BUTTON)){
      climbCount++;
      gamepieceHandler.climbRelease();
      gamepieceHandler.armClimb();
      if(climbCount > 50){
         gamepieceHandler.elevatorClimb();
      }
    }else{
      climbCount = 0;
    }

    //Operator Controls
    if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.ELEVATOR_LEVEL_1);
    }

    if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.ELEVATOR_LEVEL_2);
    }

    if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.ELEVATOR_CARGO_BOX);
    }

    if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.ELEVATOR_LEVEL_3);
    }
    if(operatorGamepad.getLeftTriggerPressed()){
      gamepieceHandler.hatchExtend();
    }else{
      gamepieceHandler.hatchRetract();
    }
    if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
      gamepieceHandler.hatchGrip();
    }else{
      gamepieceHandler.hatchRelease();
    }

  }

}
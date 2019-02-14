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
import frc.team5406.subsystems.Drive;

public class Robot extends TimedRobot {

  private Gamepieces gamepieceHandler = new Gamepieces();
  private Drive robotDrive = new Drive();

  int climbCount = 0;

  XboxController driverGamepad = new XboxController(1);
  XboxController operatorGamepad = new XboxController(0);
 
  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit(){
    climbCount = 0;
  }

  @Override
  public void teleopPeriodic() {
   // compressor.stop();
    
   robotDrive.cheesyDrive(driverGamepad.getRightTrigger(), driverGamepad.getLeftTrigger(), driverGamepad.getLeftX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));

    if(driverGamepad.getButtonHeld(XboxController.B_BUTTON)){
      robotDrive.shiftHigh();
    }else{
      robotDrive.shiftLow();
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
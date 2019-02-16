package frc.team5406.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team5406.util.XboxController;
import frc.team5406.robot.Constants;
import frc.team5406.subsystems.Gamepieces;
import frc.team5406.subsystems.Drive;

public class Robot extends TimedRobot {

  private Gamepieces gamepieceHandler = new Gamepieces();
  private Drive robotDrive = new Drive();

  int climbCount = 0;
  boolean climbTried = false;

  XboxController driverGamepad = new XboxController(1);
  XboxController operatorGamepad = new XboxController(0);
 
  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit(){
    gamepieceHandler.elevatorUp(Constants.ELEVATOR_START);
    climbCount = 0;
    
  }

  @Override
  public void teleopPeriodic() {

    
   robotDrive.cheesyDrive(driverGamepad.getRightTrigger(), driverGamepad.getLeftTrigger(), driverGamepad.getLeftX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));

    if(driverGamepad.getButtonHeld(XboxController.B_BUTTON)){
      robotDrive.shiftHigh();
    }else{
      robotDrive.shiftLow();
    }

    if(driverGamepad.getButtonHeld(XboxController.START_BUTTON) && driverGamepad.getButtonHeld(XboxController.BACK_BUTTON)){
      climbCount++;
      climbTried = true;
      gamepieceHandler.compressorDisabled();
      gamepieceHandler.armClimbStart();
      if(climbCount > 50){
        gamepieceHandler.climb();
      }
    }else if(climbTried && (driverGamepad.getButtonHeld(XboxController.START_BUTTON) || driverGamepad.getButtonHeld(XboxController.BACK_BUTTON))){
      gamepieceHandler.armClimbMid();
  }else{
    climbCount = 0;
    if (climbTried){
     climbTried = false;
     gamepieceHandler.elevatorUnClimb();
    }
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

    if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
      gamepieceHandler.armIntake();
    }else if(Math.abs(operatorGamepad.getRightY())>0.2 ) {
      gamepieceHandler.manualArm(operatorGamepad.getRightY());
    }else{
      gamepieceHandler.armUp();
    }

    if(operatorGamepad.getRightTriggerPressed()){
      gamepieceHandler.scoreCargo();
    }else if(operatorGamepad.getLeftY() > 0.3){
      gamepieceHandler.intake();
    }else if(operatorGamepad.getLeftY() < -0.3){
      gamepieceHandler.reverseIntake();
    }else{
      gamepieceHandler.intakeDefault();
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
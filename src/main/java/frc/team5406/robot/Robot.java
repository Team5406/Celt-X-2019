package frc.team5406.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team5406.util.XboxController;
import frc.team5406.robot.Constants;
import frc.team5406.subsystems.Gamepieces;
import frc.team5406.subsystems.Drive;
import edu.wpi.first.wpilibj.Notifier;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Robot extends TimedRobot {

  private Gamepieces gamepieceHandler = new Gamepieces();
  private Drive robotDrive = new Drive();

  int climbCount = 0;
  boolean climbTried = false;

  XboxController driverGamepad = new XboxController(1);
  XboxController operatorGamepad = new XboxController(0);


  class PeriodicRunnable implements java.lang.Runnable {
    public void run() { 

      int armPos;
  
        gamepieceHandler.armMotor.selectProfileSlot(0,0);
        armPos = gamepieceHandler.getElevatorPos();
        gamepieceHandler.armMotor.set(ControlMode.MotionMagic, gamepieceHandler.elevatorPosition(armPos));
        System.out.println(gamepieceHandler.elevatorPosition(armPos));
    }
  }
  Notifier notifier = new Notifier(new PeriodicRunnable());


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

    robotDrive.arcadeDrive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
   /*robotDrive.cheesyDrive(driverGamepad.getRightTrigger(), driverGamepad.getLeftTrigger(), driverGamepad.getLeftX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON)); */

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
        notifier.startPeriodic(0.005);
      }
    }else if(climbTried && (driverGamepad.getButtonHeld(XboxController.START_BUTTON) || driverGamepad.getButtonHeld(XboxController.BACK_BUTTON))){
      gamepieceHandler.armClimbMid();
      notifier.stop();
  }else{
    climbCount = 0;
    if (climbTried){
     climbTried = false;
     notifier.stop();
     gamepieceHandler.elevatorUnClimb();
    }
  }

  if(driverGamepad.getButtonHeld(XboxController.Y_BUTTON)){
    gamepieceHandler.armClimb();
    robotDrive.shiftLow();
  }else if(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){ 
    gamepieceHandler.armIntake();
  }else{
    gamepieceHandler.armUp();
  }

  if(Math.abs(driverGamepad.getRightY())>0.2 ) {
    gamepieceHandler.manualArm(driverGamepad.getRightY());
  }
  
    //Operator Controls
   
    if(operatorGamepad.getButtonHeld(XboxController.BACK_BUTTON) && operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.CARGO_LEVEL_1); 
    }else if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.HATCH_LEVEL_1);
    }

    if(operatorGamepad.getButtonHeld(XboxController.BACK_BUTTON) && operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.CARGO_LEVEL_2);
    }
    else if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.HATCH_LEVEL_2);
    }
    
   if(operatorGamepad.getButtonHeld(XboxController.BACK_BUTTON) && operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.CARGO_LEVEL_3);
    }
   else if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.HATCH_LEVEL_3);
    }
    
    if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.ELEVATOR_CARGO_BOX);
    }

    if(Math.abs(operatorGamepad.getRightY())>0.2 ) {
      gamepieceHandler.manualElevator(operatorGamepad.getRightY());
     }

   /* if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
      gamepieceHandler.armIntake();
    }else if(Math.abs(operatorGamepad.getRightY())>0.2 ) {
      gamepieceHandler.manualArm(operatorGamepad.getRightY());
    }else{
      gamepieceHandler.armUp();
    }*/

    if(operatorGamepad.getRightTriggerPressed()){
      gamepieceHandler.scoreCargo();
    }else if(driverGamepad.getRightTriggerPressed()){
      gamepieceHandler.intake();
    }else if(driverGamepad.getLeftTriggerPressed()){
      gamepieceHandler.reverseIntake();
    }else{
      gamepieceHandler.intakeDefault();
    }

    if(operatorGamepad.getLeftTriggerPressed()){
      gamepieceHandler.hatchExtend();
    }else{
      gamepieceHandler.hatchRetract();
    }

    if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
      gamepieceHandler.hatchGrip();
    }else{
      gamepieceHandler.hatchRelease();
    }

  }

}
package frc.team5406.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team5406.util.XboxController;
import frc.team5406.util.XboxController.DirectionPad;
import frc.team5406.robot.Constants;
import frc.team5406.subsystems.Gamepieces;
import frc.team5406.subsystems.Drive;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private Gamepieces gamepieceHandler = new Gamepieces();
  private Drive robotDrive = new Drive();

  int climbCount = 0;
  int climbDriveCount = 0;
  boolean climbTried = false;
  int climbDrive = 0;

  XboxController driverGamepad = new XboxController(1);
  XboxController operatorGamepad = new XboxController(0);


  class PeriodicRunnable implements java.lang.Runnable {
    public void run() { 
      gamepieceHandler.armClimbRunnable(gamepieceHandler.getElevatorPos());
      System.out.println(-1*gamepieceHandler.armPosition(gamepieceHandler.getElevatorPos()));
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
  public void disabledPeriodic(){
    SmartDashboard.putBoolean("hatchSensor", gamepieceHandler.haveHatch());    
  }

  @Override
  public void teleopPeriodic() {
    //System.out.println(gamepieceHandler.getElevatorPos());
    SmartDashboard.putBoolean("hatchSensor", gamepieceHandler.haveHatch());
    if( Math.abs(driverGamepad.getLeftY()) > 0.05 ||  !climbTried ){
    robotDrive.arcadeDrive(-1*driverGamepad.getLeftY(), driverGamepad.getRightX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
    //robotDrive.cheesyDrive(-1*driverGamepad.getLeftY(), driverGamepad.getRightX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
    //robotDrive.cheesyDrive(driverGamepad.getRightTrigger(), driverGamepad.getLeftTrigger(), driverGamepad.getLeftX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
    }
    if(driverGamepad.getButtonHeld(XboxController.B_BUTTON) || driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
      robotDrive.shiftHigh();
    }else{
      robotDrive.shiftLow();
    }
    System.out.println("climbTried: " + climbTried + ", climbCount: " + climbCount + ", climbDriveCount: " + climbDriveCount + ", climbDrive:" + climbDrive);
    if(operatorGamepad.getButtonHeld(XboxController.START_BUTTON) && operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
      climbCount++;
      climbTried = true;
      if (climbCount < 10){
        gamepieceHandler.elevatorUp(0);
        gamepieceHandler.armClimbStart();
        climbDriveCount = 0;
      }else if(climbCount > 50){
        gamepieceHandler.climb();
        notifier.startPeriodic(0.005);
        if(climbDriveCount > 150){
          System.out.println("ArmUp - both Buttons");
          gamepieceHandler.intakeDefault();
          robotDrive.arcadeDrive(0, 0, true);
        }else if(climbDriveCount > 40){
          robotDrive.arcadeDrive(-0.6, 0, false);
          System.out.println("ArmMid - both Buttons");
          gamepieceHandler.armClimbMid();
          robotDrive.arcadeDrive(-0.6, 0, false);
          notifier.stop();    
          climbDriveCount++;
        }else if(Math.abs(gamepieceHandler.getElevatorPos()) > Math.abs(0.7*Constants.ELEVATOR_CLIMB)){
          gamepieceHandler.intakeClimb();
          robotDrive.shiftLow();
          robotDrive.arcadeDrive(-0.6, 0, false);
          climbDriveCount++;
        }
      }
    }else if(climbTried && (operatorGamepad.getButtonHeld(XboxController.START_BUTTON) || operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER))){
      gamepieceHandler.armClimbMid();
      robotDrive.arcadeDrive(-0.40, 0, false);
      notifier.stop();
  }else{
    climbCount = 0;
    if (climbTried){
      climbDrive ++;
     //climbTried = false;
     //gamepieceHandler.intakeDefault();
     notifier.stop();
     gamepieceHandler.elevatorUnClimb();
     robotDrive.arcadeDrive(-0.40, 0, false);
    }
    if(climbTried && climbDrive > 200 ){
      climbTried = false;
      climbDrive = 0;
      robotDrive.arcadeDrive(0, 0, false);
    }
  }

  if(driverGamepad.getButtonHeld(XboxController.Y_BUTTON)){
    gamepieceHandler.armClimb();
    robotDrive.shiftLow();
  }else if (climbCount > 0){
  }else if (operatorGamepad.getButtonHeld(XboxController.START_BUTTON) && operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){ 
  }else if (operatorGamepad.getButtonHeld(XboxController.BACK_BUTTON) && operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
  }else if (driverGamepad.getRightTriggerPressed() || driverGamepad.getLeftTriggerPressed()){
  }else if (operatorGamepad.getDirectionPad() == DirectionPad.UP && operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
    gamepieceHandler.manualArm(0.3);
  }else if (operatorGamepad.getDirectionPad() == DirectionPad.DOWN && operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
    gamepieceHandler.manualArm(-0.3);
  }else{
    gamepieceHandler.armUp();
  }


  /*if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) && Math.abs(operatorGamepad.getRightY())>0.2 ) {
    gamepieceHandler.manualArm(operatorGamepad.getRightY());
  }*/

  
    //Operator Controls
   
    if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) && operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.CARGO_LEVEL_1); 
    }else if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.HATCH_LEVEL_1);
    }

    if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) && operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.CARGO_LEVEL_2);
    }
    else if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.HATCH_LEVEL_2);
    }
    
   if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) && operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.CARGO_LEVEL_3);
    }
   else if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.HATCH_LEVEL_3);
    }
    
    if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
      gamepieceHandler.elevatorUp(Constants.ELEVATOR_CARGO_BOX);
    }

    if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) && Math.abs(operatorGamepad.getRightY())>0.2 ) {
      gamepieceHandler.manualElevator(operatorGamepad.getRightY());
     }

   /* if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
      gamepieceHandler.armIntake();
    }else if(Math.abs(operatorGamepad.getRightY())>0.2 ) {
      gamepieceHandler.manualArm(operatorGamepad.getRightY());
    }else{
      gamepieceHandler.armUp();
    }*/

    if(driverGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
      gamepieceHandler.scoreCargo();
    }else if(driverGamepad.getRightTriggerPressed()){
      gamepieceHandler.intake();
      gamepieceHandler.armIntake();
    }else if(driverGamepad.getLeftTriggerPressed()){
      gamepieceHandler.reverseIntake();
      gamepieceHandler.armIntake();
    }else if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) && operatorGamepad.getButtonHeld(XboxController.BACK_BUTTON)  ) {
      gamepieceHandler.armClimbLevel2();
    }else if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) && Math.abs(operatorGamepad.getLeftY())>0.2 ) {
      if(climbCount > 0){
        gamepieceHandler.intakeClimb();
      }else{
        gamepieceHandler.intake();
      }
    }else if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) && Math.abs(operatorGamepad.getLeftY())>0.2 ) {
      gamepieceHandler.reverseIntake();
    }else if(operatorGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) && operatorGamepad.getButtonHeld(XboxController.START_BUTTON) ) {
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
package frc.team5406.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team5406.util.XboxController;
import frc.team5406.util.XboxController.DirectionPad;
import frc.team5406.robot.Constants;
import frc.team5406.subsystems.Gamepieces;
import frc.team5406.subsystems.Drive;
import frc.team3256.warriorlib.loop.Looper;
import frc.team3256.warriorlib.subsystem.DriveTrainBase;
import frc.team3256.warriorlib.auto.purepursuit.*;
import frc.team3256.warriorlib.auto.AutoModeExecuter;
import frc.team3256.warriorlib.math.*;
import frc.team3256.warriorlib.auto.purepursuit.PurePursuitTracker;
import frc.team3256.warriorlib.auto.purepursuit.PathGenerator;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.*;


public class Robot extends TimedRobot implements PIDOutput  {

  private Gamepieces gamepieceHandler = new Gamepieces();
  private Drive robotDrive;

  AutoModeExecuter autoModeExecuter;
  PoseEstimator poseEstimator;
  Looper autoLooper;
  PurePursuitTracker purePursuitTracker;
  int autoStep = 0;
  int autoCount = 0;
  PIDController turnController;
  PIDController rocketTurnController;
  double rotateToAngleRate;

  int climbCount = 0;
  int climbDriveCount = 0;
  boolean climbTried = false;
  int climbDrive = 0;
  double startTime = 0;

  double llSteer = 0.0;
  double llDrive = 0.0;
  double lastllDrive = 0;
  double llArea = 0;
  double llLastError = 0;
  double llTotalError = 0;
  boolean llHasValidTarget = false;
  boolean cancelAuto = false;
  boolean visionTurn = false;

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
    robotDrive = Drive.getInstance(); // singleton pattern is recommended
    DriveTrainBase.setDriveTrain(robotDrive);

    autoLooper = new Looper(1 / 50D); // recommended loop time
    poseEstimator = PoseEstimator.getInstance();
    autoLooper.addLoops(poseEstimator);
    robotDrive.resetGyro();

    PathGenerator pathGenerator = new PathGenerator(Constants.spacing, true);
    pathGenerator.addPoint(new Vector(0, 0));
    pathGenerator.addPoint(new Vector(0, 66));
    pathGenerator.addPoint(new Vector(51, 114));
    pathGenerator.addPoint(new Vector(70, 139));
    pathGenerator.setSmoothingParameters(Constants.a, Constants.b, Constants.tolerance);
    pathGenerator.setVelocities(Constants.maxVel, 40, Constants.maxVelk, 25);
    Path path = pathGenerator.generatePath();
    List<Path> paths = new ArrayList<Path>();
    paths.add(path);

    purePursuitTracker = PurePursuitTracker.getInstance();
    purePursuitTracker.setRobotTrack(Constants.robotTrack);
    purePursuitTracker.setFeedbackMultiplier(Constants.kP);
    purePursuitTracker.setPaths(paths, Constants.lookaheadDistance);
    purePursuitTracker.setPath(0);
    gamepieceHandler.compressorDisabled();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

  }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    robotDrive.resetGyro();
    robotDrive.resetEncoders();
    poseEstimator.reset();
    purePursuitTracker.reset();
    autoLooper.start();
    gamepieceHandler.elevatorUp(Constants.HATCH_LEVEL_1);

    autoModeExecuter = new AutoModeExecuter();
    autoModeExecuter.setAutoMode(new PurePursuitTestMode(0));
    autoModeExecuter.start();
    autoStep = 1;
    turnController = new PIDController(0.025, 0.0000, 0.05, 0.3, Constants.navX, this);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(5.0f);
    turnController.setContinuous(true);
    rocketTurnController = new PIDController(0.025, 0.0000, 0.05, 0.3, Constants.navX, this);
    rocketTurnController.setInputRange(-180.0f, 180.0f);
    rocketTurnController.setOutputRange(-1.0, 1.0);
    rocketTurnController.setAbsoluteTolerance(5.0f);
    rocketTurnController.setContinuous(true);
    gamepieceHandler.compressorDisabled();
    System.out.println("AutoInit: " +(Timer.getFPGATimestamp()-startTime));
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    cancelAuto = false;
    visionTurn = false;

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if(driverGamepad.getButtonHeld(XboxController.A_BUTTON)){
      if(!cancelAuto){
        autoLooper.stop();
        autoModeExecuter.stop();
        purePursuitTracker.reset();
        robotDrive.setVelocityClosedLoop(0, 0);
        cancelAuto = true;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
      }
    }else{
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);  
    }

    if(!cancelAuto){
    SmartDashboard.putNumber("X", poseEstimator.getPose().x);
    SmartDashboard.putNumber("Y", poseEstimator.getPose().y);
    SmartDashboard.putNumber("vLeft", robotDrive.getLeftVelocity());
    SmartDashboard.putNumber("vRight", robotDrive.getRightVelocity());
    SmartDashboard.putNumber("angle", robotDrive.getRotationAngle().degrees());

    switch (autoStep) {
    case 1:
    if (purePursuitTracker.isDone()) {
autoLimelightTracking();
        if (llHasValidTarget) {
          autoStep++;
        }else if(!visionTurn){
          visionTurn = true;
          double tx0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx0").getDouble(0);
          if (tx0 < 0){
            robotDrive.setVelocityClosedLoop(-10, 10); //counterclockwise
          }else if(tx0 > 0){
            robotDrive.setVelocityClosedLoop(10, -10); //clockwise
          }else if(Constants.navX.getAngle() - 90 > 0){
            robotDrive.setVelocityClosedLoop(10, -10); //clockwise
          }else{
            robotDrive.setVelocityClosedLoop(-10, 10); //counterclockwise
          }

        }
      }

        break;
        
    case 2:
      /*System.out.println("X: " + poseEstimator.getPose().x + ", Y: " + poseEstimator.getPose().y + ", vL: "
          + driveTrain.getLeftVelocity() + ", vR: " + driveTrain.getRightVelocity() + ", angle: "
          + driveTrain.getRotationAngle().degrees());
*/
      // At rocket, extend hatch mech, vision drive in (hatch fowards)
{
        System.out.println(autoStep + " - Drove to Rocket " +  (Timer.getFPGATimestamp()-startTime));
        gamepieceHandler.hatchExtend();
        autoLimelightTracking();
        if (llHasValidTarget) {
          System.out.println ("Limelight area: " + llArea);
          robotDrive.setVelocityClosedLoop(50*(llDrive + llDrive*llSteer*0.5), 50*(llDrive - llDrive*llSteer*0.5));
          if (llArea > robotDrive.maxllArea(robotDrive.getRotationAngle().degrees())) {
            robotDrive.setVelocityClosedLoop(20, 20);
            autoCount =0;
            autoStep++;
          }
        }
        //driveTrain.setVelocityClosedLoop(25, 25);
      }
      
      break;
    case 3:
        autoCount++;
        System.out.println(autoStep + " - Extra Driving" +  (Timer.getFPGATimestamp()-startTime));
        if(autoCount > 5){
          gamepieceHandler.hatchRelease();
        }        
        if(autoCount >= 26){
          robotDrive.setVelocityClosedLoop(0, 0);
          lastllDrive = 0;
          llLastError = 0;
          llTotalError = 0;
          llSteer = 0;
          llDrive = 0;
          autoStep++;

        }else{
          robotDrive.setVelocityClosedLoop(20-autoCount*0.8, 20-autoCount*0.8);
        }
        break;
      case 4:
      // Back away from rocket, (hatch backwards)
      {  System.out.println(autoStep + " - First Panel Delivered" +  (Timer.getFPGATimestamp()-startTime));

        PathGenerator pathGenerator = new PathGenerator(Constants.spacing, false);
        List<Path> paths = new ArrayList<Path>();
        pathGenerator.addPoint(new Vector(poseEstimator.getPose().x, poseEstimator.getPose().y));
        //pathGenerator.addPoint(new Vector(90, 60));
        //pathGenerator.addPoint(new Vector(90, 50));
        pathGenerator.addPoint(new Vector(83, 154)); //81, 172

        pathGenerator.setSmoothingParameters(Constants.a, Constants.b, Constants.tolerance);
        pathGenerator.setVelocities(Constants.maxVel, 50, Constants.maxVelk, -50);
        Path path2 = pathGenerator.generatePath();
        paths.add(path2);
        purePursuitTracker.setPaths(paths, Constants.lookaheadDistance);
        purePursuitTracker.setPath(0);
        autoCount++;
        purePursuitTracker.reset();
        autoModeExecuter = new AutoModeExecuter();
        autoModeExecuter.setAutoMode(new PurePursuitTestMode(0));
        autoModeExecuter.start();
        gamepieceHandler.hatchRetract();
        autoStep++;
      }
      break;
    case 5:
      // Drive towards feeder station (hatch fowards)
      if (purePursuitTracker.isDone()) {
        gamepieceHandler.hatchGrip();
        System.out.println(autoStep + " - turn around" +  (Timer.getFPGATimestamp()-startTime));
        turnController.setSetpoint(179.9);
        turnController.enable();
        SmartDashboard.putNumber("TurningAngle", Constants.navX.getAngle());
        SmartDashboard.putNumber("rotateToAngleRate", rotateToAngleRate);

        if (turnController.onTarget()) {
          System.out.println("Done Turning");
          turnController.disable();
          rotateToAngleRate = 0;
          turnController.reset();
          turnController.close();
          autoStep++;
          robotDrive.setVelocityClosedLoop(20, 20);
        } else {
          System.out.println(rotateToAngleRate);
          robotDrive.setVelocityClosedLoop(rotateToAngleRate * 50, -rotateToAngleRate * 50);
        }

      }

      break;
      case 6:
      {
      // Back away from rocket, (hatch backwards)
        System.out.println(autoStep + " - Drive towards feeder station" +  (Timer.getFPGATimestamp()-startTime));

        PathGenerator pathGenerator = new PathGenerator(Constants.spacing, true);
        List<Path> paths = new ArrayList<Path>();
        pathGenerator.addPoint(new Vector(poseEstimator.getPose().x, poseEstimator.getPose().y));
        pathGenerator.addPoint(new Vector(90, 80));
        pathGenerator.addPoint(new Vector(90, 60));

        pathGenerator.setSmoothingParameters(Constants.a, Constants.b, Constants.tolerance);
        pathGenerator.setVelocities(Constants.maxVel, 50, Constants.maxVelk, 35);
        Path path2 = pathGenerator.generatePath();
        paths.add(path2);
        purePursuitTracker.setPaths(paths, Constants.lookaheadDistance);
        purePursuitTracker.setPath(0);
        autoCount++;
        purePursuitTracker.reset();
        autoModeExecuter = new AutoModeExecuter();
        autoModeExecuter.setAutoMode(new PurePursuitTestMode(0));
        autoModeExecuter.start();
        gamepieceHandler.hatchRetract();
        visionTurn = false;
        autoStep++;
    }
      break;

      case 7:
      if (purePursuitTracker.isDone()) {
autoLimelightTracking();
        if (llHasValidTarget) {
          autoStep++;
        }else if(!visionTurn){
          visionTurn = true;
          double tx0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx0").getDouble(0);
          if (tx0 < 0){
            robotDrive.setVelocityClosedLoop(-10, 10);
          }else if(tx0 > 0){
            robotDrive.setVelocityClosedLoop(10, -10);
          }else if(Constants.navX.getAngle() - 90 > 0){
            robotDrive.setVelocityClosedLoop(10, -10);
          }else{
            robotDrive.setVelocityClosedLoop(-10, 10);
          }

        }
      }

        break;
      case 8:
      // Hatch extend, vision drive to feeder station (hatch forwards)
      
        System.out.println(autoStep + " - Near feeder station" +  (Timer.getFPGATimestamp()-startTime));
        autoLimelightTracking();
        gamepieceHandler.hatchExtend();
        if (llHasValidTarget) {
         // System.out.println ("tx " + tx +" ta: " + llArea + " dr: " + llDrive + ", st: " + llSteer);
          robotDrive.setVelocityClosedLoop(50*(llDrive + llDrive*llSteer*0.4), 50*(llDrive - llDrive*llSteer*0.4));
          if (llArea > Constants.LL_TARGET_AREA) {
            robotDrive.setVelocityClosedLoop(20, 20);
            autoCount =0;
            autoStep++;
          }
        }
     
      break;
    case 9:
        autoCount++;
        System.out.println(autoStep + " -Extra Driving" +  (Timer.getFPGATimestamp()-startTime));
        if(autoCount >= 11){
          robotDrive.setVelocityClosedLoop(0, 0);
          lastllDrive = 0;
          llLastError = 0;
          llTotalError = 0;
          autoStep++;
        }else{
          robotDrive.setVelocityClosedLoop(20-autoCount*2, 20-autoCount*2);
        }
        break;
    case 10:
      // Drive to cargo line (hatch backwards)
       { System.out.println(autoStep + " - At feeder station" +  (Timer.getFPGATimestamp()-startTime));
        PathGenerator pathGenerator = new PathGenerator(Constants.spacing, false);
        List<Path> paths = new ArrayList<Path>();
        pathGenerator.addPoint(new Vector(poseEstimator.getPose().x, poseEstimator.getPose().y));
        pathGenerator.addPoint(new Vector(70, 210));
        pathGenerator.addPoint(new Vector(80, 245));
        pathGenerator.addPoint(new Vector(80, 265));
        //pathGenerator.addPoint(new Vector(40, 265));

        pathGenerator.setSmoothingParameters(Constants.a, Constants.b, Constants.tolerance);
        pathGenerator.setVelocities(150, 50, Constants.maxVelk, -10);
        Path path2 = pathGenerator.generatePath();
        paths.add(path2);
        purePursuitTracker.setPaths(paths, 18);
        purePursuitTracker.setPath(0);
        purePursuitTracker.reset();
        autoModeExecuter = new AutoModeExecuter();
        autoModeExecuter.setAutoMode(new PurePursuitTestMode(0));
        autoModeExecuter.start();
        autoStep++;
        autoCount = 0;}
      break;
      case 11:
      // Drive to cargo line (hatch backwards)
      autoCount++;
      if(autoCount > 30){
        gamepieceHandler.hatchRetract();
      }
      System.out.println(autoStep + " - Finding Target " +  (Timer.getFPGATimestamp()-startTime));
       
      if (purePursuitTracker.isDone()) {
        gamepieceHandler.hatchGrip();
        autoLimelightTracking();

        if (llHasValidTarget) {

          System.out.println ("dr: " + llDrive + ", st: " + llSteer);
          robotDrive.setVelocityClosedLoop(-50*(llSteer), 50*(llSteer));
          if (llSteer <0.1) {
            robotDrive.setVelocityClosedLoop(0, 0);
            autoCount =0;
            autoStep++;
          }
        
        }else {
          rocketTurnController.setSetpoint(145);
          rocketTurnController.enable();
          System.out.println(Constants.navX.getAngle());
          SmartDashboard.putNumber("TurningAngle", Constants.navX.getAngle());
          SmartDashboard.putNumber("rotateToAngleRate", rotateToAngleRate);
  
          if (rocketTurnController.onTarget()) {
            System.out.println("Done Turning");
            rocketTurnController.disable();
            rotateToAngleRate = 0;
            rocketTurnController.reset();
            robotDrive.setVelocityClosedLoop(0, 0);
          } else {
            System.out.println(rotateToAngleRate);
            robotDrive.setVelocityClosedLoop(rotateToAngleRate * 50, -rotateToAngleRate * 50);
          }          }

      }
      break;

      
      case 12:
      // Hatch extend, vision drive to rocket (hatch forwards)
        gamepieceHandler.hatchExtend();
        System.out.println(autoStep + " - Near backside of rocket" +  (Timer.getFPGATimestamp()-startTime));
        autoLimelightTracking();
        if (llHasValidTarget) {
          if ((llDrive - lastllDrive) > 0.05){
            lastllDrive += 0.05;
            llDrive = lastllDrive;
          }
          lastllDrive = llDrive;
          System.out.println ("dr: " + llDrive + ", st: " + llSteer);
          robotDrive.setVelocityClosedLoop(50*(llDrive + llDrive*llSteer*0.3), 50*(llDrive - llDrive*llSteer*0.3));
          if (llArea > Constants.LL_TARGET_AREA-0.7) {
            robotDrive.setVelocityClosedLoop(20, 20);
            autoCount =0;
            autoStep++;
          }
        
      }
      break;
    case 13:
        autoCount++;
        System.out.println(autoStep + " - Extra Driving" +  (Timer.getFPGATimestamp()-startTime));
        if(autoCount > 5){
          gamepieceHandler.hatchRelease();
        }
        if(autoCount >= 11){
          robotDrive.setVelocityClosedLoop(0, 0);
          lastllDrive = 0;
          llLastError = 0;
          llTotalError = 0;
          autoStep++;
        }else{
          robotDrive.setVelocityClosedLoop(20-autoCount*2, 20-autoCount*2);
        }
        break;
    case 14:
      // drive to cargo ship
        {System.out.println(autoStep + " - 2nd panel delivered" +  (Timer.getFPGATimestamp()-startTime));
        PathGenerator pathGenerator = new PathGenerator(Constants.spacing, false);
        List<Path> paths = new ArrayList<Path>();
        pathGenerator.addPoint(new Vector(poseEstimator.getPose().x, poseEstimator.getPose().y));
        pathGenerator.addPoint(new Vector(50, 260));
        //pathGenerator.addPoint(new Vector(30, 260));
        pathGenerator.setSmoothingParameters(Constants.a, Constants.b, Constants.tolerance);
        pathGenerator.setVelocities(Constants.maxVel, Constants.maxAccel, Constants.maxVelk, 0);
        Path path2 = pathGenerator.generatePath();
        paths.add(path2);
        purePursuitTracker.setPaths(paths, Constants.lookaheadDistance);
        purePursuitTracker.setPath(0);
        autoCount++;
        purePursuitTracker.reset();
        autoModeExecuter = new AutoModeExecuter();
        autoModeExecuter.setAutoMode(new PurePursuitTestMode(0));
        autoModeExecuter.start();
        autoStep++;}
      
    case 15:
      // drive to cargo ship
      if (purePursuitTracker.isDone()) {
        System.out.println(autoStep + " - Auto done. Waiting for cargo near cargo ship" +  (Timer.getFPGATimestamp()-startTime));
        autoStep++;
      }

    }
  }else{

   /***************************************************
   **********manual controls in auto*****************
   **************************************************/


    robotDrive.updateLimelightTracking();

    //System.out.println(gamepieceHandler.getElevatorPos());
    gamepieceHandler.hatchNotify();
    if (driverGamepad.getButtonHeld(XboxController.Y_BUTTON) && !gamepieceHandler.haveHatch())
    {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

      if (robotDrive.llHasValidTarget)
      {
        double driveSpeed = driverGamepad.getLeftY();
        robotDrive.shiftLow();
        if (Math.abs(driveSpeed) > 0.7){
          driveSpeed = 0.7*Math.signum(driveSpeed);
        }
            robotDrive.arcadeDrive(-1*driveSpeed, robotDrive.llSteer, false);
      }
      else 
      {
            robotDrive.arcadeDrive(0.0, 0.0, false);
      }
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);

      if( Math.abs(driverGamepad.getLeftY()) > 0.05 ||  !climbTried ){
        robotDrive.llLastError = 0;
        robotDrive.llTotalError = 0;
        robotDrive.arcadeDrive(-1*driverGamepad.getLeftY(), driverGamepad.getRightX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
      //robotDrive.cheesyDrive(-1*driverGamepad.getLeftY(), driverGamepad.getRightX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
      //robotDrive.cheesyDrive(driverGamepad.getRightTrigger(), driverGamepad.getLeftTrigger(), driverGamepad.getLeftX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
       }
       
       if(driverGamepad.getButtonHeld(XboxController.B_BUTTON) || driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
        robotDrive.shiftHigh();
      }else{
        robotDrive.shiftLow();
      }
    
    }

    if(driverGamepad.getButtonHeld(XboxController.Y_BUTTON)){
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

    if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
      gamepieceHandler.hatchExtend();
    }else{
      gamepieceHandler.hatchRetract();
    }

    if(operatorGamepad.getLeftTriggerPressed()){
      gamepieceHandler.hatchRelease();
    }else{
      gamepieceHandler.hatchGrip();
    }


  }
  }

  @Override
  public void disabledPeriodic(){
    gamepieceHandler.hatchNotify();
    SmartDashboard.putNumber("X", poseEstimator.getPose().x);
    SmartDashboard.putNumber("Y", poseEstimator.getPose().y);
    SmartDashboard.putNumber("dLeft", robotDrive.getLeftDistance());
    SmartDashboard.putNumber("dRight", robotDrive.getRightDistance());
    SmartDashboard.putNumber("vLeft", robotDrive.getLeftVelocity());
    SmartDashboard.putNumber("vRight", robotDrive.getRightVelocity());
    SmartDashboard.putNumber("angle", robotDrive.getRotationAngle().degrees());
  
  }

  @Override
  public void teleopInit(){
    gamepieceHandler.elevatorUp(Constants.ELEVATOR_START);
    climbCount = 0;
    autoLooper.stop();
    purePursuitTracker.reset();
    robotDrive.setVelocityClosedLoop(0, 0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    gamepieceHandler.compressorEnabled();

  }

  @Override
  public void teleopPeriodic() {
    robotDrive.updateLimelightTracking();

    //System.out.println(gamepieceHandler.getElevatorPos());
    gamepieceHandler.hatchNotify();
    if (driverGamepad.getButtonHeld(XboxController.Y_BUTTON) && !gamepieceHandler.haveHatch())
    {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

      if (robotDrive.llHasValidTarget)
      {
        double driveSpeed = driverGamepad.getLeftY();
        robotDrive.shiftLow();
        if (Math.abs(driveSpeed) > 0.7){
          driveSpeed = 0.7*Math.signum(driveSpeed);
        }
            robotDrive.arcadeDrive(-1*driveSpeed, robotDrive.llSteer, false);
      }
      else 
      {
            robotDrive.arcadeDrive(0.0, 0.0, false);
      }
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
      if( Math.abs(driverGamepad.getLeftY()) > 0.05 ||  !climbTried ){
        robotDrive.llLastError = 0;
        robotDrive.llTotalError = 0;
        robotDrive.arcadeDrive(-1*driverGamepad.getLeftY(), driverGamepad.getRightX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
      //robotDrive.cheesyDrive(-1*driverGamepad.getLeftY(), driverGamepad.getRightX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
      //robotDrive.cheesyDrive(driverGamepad.getRightTrigger(), driverGamepad.getLeftTrigger(), driverGamepad.getLeftX(), driverGamepad.getButtonHeld(XboxController.A_BUTTON));
       }
       
       if(driverGamepad.getButtonHeld(XboxController.B_BUTTON) || driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
        robotDrive.shiftHigh();
      }else{
        robotDrive.shiftLow();
      }
    
    }

   // System.out.println("climbTried: " + climbTried + ", climbCount: " + climbCount + ", climbDriveCount: " + climbDriveCount + ", climbDrive:" + climbDrive);
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

if (climbCount > 0){
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

    if(operatorGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
      gamepieceHandler.hatchExtend();
    }else{
      gamepieceHandler.hatchRetract();
    }

    if(operatorGamepad.getLeftTriggerPressed()){
      gamepieceHandler.hatchRelease();
    }else{
      gamepieceHandler.hatchGrip();
    }

  }
  

  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }



  public void autoLimelightTracking() {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    // ts: close to 0 - left, close to 90 - right
    // tx: negative - left, positive - right

    // These numbers must be tuned for your Robot! Be careful!
    final double DRIVE_KP = 0.2; // how hard to turn toward the target
    final double STEER_KP = 0.08; // how hard to turn toward the target
    final double STEER_KD = 0;//0.005;
    final double STEER_KI = 0;//0.1;
    final double MAX_DRIVE = 0.8; // Simple speed limit so we don't drive too fast

    if (tv < 1.0) {
      llHasValidTarget = false;
      llSteer = 0.0;
      llArea = 0;
      llDrive = 0;
      return;
    }

    llHasValidTarget = true;
    llTotalError += tx;

    // Start with proportional steering
    llSteer = tx * STEER_KP + STEER_KD * (tx - llLastError) / 0.02 + STEER_KI * llTotalError * 0.02;

    // try to drive forward until the target area reaches our desired area
    llLastError = tx;
    if (Math.abs(llSteer) > MAX_DRIVE) {
      llSteer = Math.signum(llSteer) * MAX_DRIVE;
    }

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (Constants.LL_TARGET_AREA - ta) * DRIVE_KP;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    if (drive_cmd < 0.4){
      drive_cmd = 0.4;
    }
    llDrive = drive_cmd;
    llArea = ta;
  }

  @Override
  public void testPeriodic() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

  }
}

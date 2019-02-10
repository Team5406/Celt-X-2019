package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.util.XboxController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;


public class Robot extends TimedRobot {
  Compressor compressor = new Compressor();
  public boolean highGear = false;
  Solenoid shiftSolenoid = new Solenoid(0);
  Solenoid hatchDeploy = new Solenoid(1);
  Solenoid hatchGrip = new Solenoid(2);
  Solenoid cargoDeploy = new Solenoid(3);
  public static boolean SHIFT_HIGH = true;
  public static boolean SHIFT_LOW = false;
  public static AHRS navX = new AHRS(SPI.Port.kMXP);

  public void shiftHigh() {
    shiftSolenoid.set(SHIFT_HIGH);
      highGear = true;
  }
  public void shiftLow() {
    shiftSolenoid.set(SHIFT_LOW);
      highGear = false;
  }

  XboxController driverGamepad = new XboxController(1);
  XboxController operatorGamepad = new XboxController(0);

  WPI_TalonSRX leftDriveMotor = new WPI_TalonSRX(1);
  WPI_VictorSPX leftDriveSlave1 = new WPI_VictorSPX(2);
  WPI_VictorSPX leftDriveSlave2 = new WPI_VictorSPX(3);

  WPI_TalonSRX rightDriveMotor = new WPI_TalonSRX(4);
  WPI_VictorSPX rightDriveSlave1= new WPI_VictorSPX(5);
  WPI_VictorSPX rightDriveSlave2 = new WPI_VictorSPX(6);

  WPI_TalonSRX armMotor = new WPI_TalonSRX(7);
  WPI_VictorSPX armSlave = new WPI_VictorSPX(8);

  WPI_TalonSRX intakeMotor = new WPI_TalonSRX(9);
  WPI_TalonSRX conveyorMotor = new WPI_TalonSRX(10);

  WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(11);
  WPI_VictorSPX elevatorMotorSlave1 = new WPI_VictorSPX(12);
  WPI_VictorSPX elevatorMotorSlave2 = new WPI_VictorSPX(13);
  WPI_VictorSPX elevatorMotorSlave3 = new WPI_VictorSPX(14);

  WPI_TalonSRX boxMotor = new WPI_TalonSRX(15);

  DifferentialDrive drive = new DifferentialDrive(leftDriveMotor, rightDriveMotor);
  
  public void armClimb() {
    armMotor.selectProfileSlot(1,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_CLIMB);    
  }
  public void armIntake() {
    armMotor.selectProfileSlot(1,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_INTAKE);    
  }

  public void armUp() {
    armMotor.selectProfileSlot(0,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_UP);    
  }

  public void elevatorUp(int pos) {
    elevatorMotor.selectProfileSlot(0,0);
    elevatorMotor.set(ControlMode.MotionMagic, pos);
  }

  public void elevatorClimb() {
    elevatorMotor.selectProfileSlot(0,0);
    elevatorMotor.set(ControlMode.MotionMagic, Constants.ELEVATOR_CLIMB);
  }

 
  @Override
  public void robotInit() {
    elevatorMotorSlave2.setInverted(true);
    elevatorMotorSlave3.setInverted(true);
    armSlave.setInverted(true);

    leftDriveSlave1.follow(leftDriveMotor);
    leftDriveSlave2.follow(leftDriveMotor);
    rightDriveSlave1.follow(rightDriveMotor);
    rightDriveSlave2.follow(rightDriveMotor);
    armSlave.follow(armMotor);
    elevatorMotorSlave1.follow(elevatorMotor);
    elevatorMotorSlave2.follow(elevatorMotor);
    elevatorMotorSlave3.follow(elevatorMotor);
    
      //Elevator Up
    elevatorMotor.selectProfileSlot(0,0);
    elevatorMotor.config_kF(0, 0.0525, Constants.kTimeoutMs);
    elevatorMotor.config_kP(0, 0.1, Constants.kTimeoutMs);
    elevatorMotor.config_kI(0, 0, Constants.kTimeoutMs);
    elevatorMotor.config_kD(0, 0, Constants.kTimeoutMs);

  //Elevator Down
    elevatorMotor.selectProfileSlot(1,0);
    elevatorMotor.config_kF(1, 0.0525, Constants.kTimeoutMs);
    elevatorMotor.config_kP(1, 0.08, Constants.kTimeoutMs);
    elevatorMotor.config_kI(1, 0, Constants.kTimeoutMs);
    elevatorMotor.config_kD(1, 0, Constants.kTimeoutMs);
  /* set acceleration and vcruise velocity - see documentation */
    elevatorMotor.configMotionCruiseVelocity(60000, Constants.kTimeoutMs);
    elevatorMotor.configMotionAcceleration(200000, Constants.kTimeoutMs);

    armMotor.selectProfileSlot(0,0);
    armMotor.config_kF(0, 1.2, Constants.kTimeoutMs);
    armMotor.config_kP(0, 0.01, Constants.kTimeoutMs);
    armMotor.config_kI(0, 0, Constants.kTimeoutMs);
    armMotor.config_kD(0, 0, Constants.kTimeoutMs);
    
    armMotor.selectProfileSlot(1,0);
    armMotor.config_kF(1, 1.2, Constants.kTimeoutMs);
    armMotor.config_kP(1, 0.01, Constants.kTimeoutMs);
    armMotor.config_kI(1, 0, Constants.kTimeoutMs);
    armMotor.config_kD(1, 0, Constants.kTimeoutMs);
    armMotor.configMotionCruiseVelocity(500, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(500, Constants.kTimeoutMs);

    intakeMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    intakeMotor.configPeakCurrentLimit(15, Constants.kTimeoutMs);
    intakeMotor.configPeakCurrentDuration(50, Constants.kTimeoutMs);
    intakeMotor.enableCurrentLimit(true);

    conveyorMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    conveyorMotor.configPeakCurrentLimit(15, Constants.kTimeoutMs);
    conveyorMotor.configPeakCurrentDuration(50, Constants.kTimeoutMs);
    conveyorMotor.enableCurrentLimit(true);

    armMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    armMotor.configPeakCurrentLimit(15, Constants.kTimeoutMs);
    armMotor.configPeakCurrentDuration(50, Constants.kTimeoutMs);
    armMotor.enableCurrentLimit(true);

    elevatorMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    elevatorMotor.configPeakCurrentLimit(50, Constants.kTimeoutMs);
    elevatorMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    elevatorMotor.enableCurrentLimit(true);

    boxMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    boxMotor.configPeakCurrentLimit(15, Constants.kTimeoutMs);
    boxMotor.configPeakCurrentDuration(50, Constants.kTimeoutMs);
    boxMotor.enableCurrentLimit(true);

    leftDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
    leftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);

    rightDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
    rightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);

    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);

    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);

    armMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    elevatorMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    leftDriveMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    rightDriveMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);

  }

  @Override
  public void teleopPeriodic() {
    compressor.stop();
    double throttle, turning;
    boolean quickTurn = false;

    boxMotor.set(ControlMode.PercentOutput,0.3);

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

    if(driverGamepad.getButtonHeld(XboxController.X_BUTTON)){
        intakeMotor.set(ControlMode.PercentOutput,0.4);
        conveyorMotor.set(ControlMode.PercentOutput,0.4);
    }else if(driverGamepad.getButtonHeld(XboxController.Y_BUTTON)){
        intakeMotor.set(ControlMode.PercentOutput,-0.25);
        conveyorMotor.set(ControlMode.PercentOutput,-0.25);
    }else{
        intakeMotor.set(ControlMode.PercentOutput,0.0);
        conveyorMotor.set(ControlMode.PercentOutput,0.0);
    }

    if(driverGamepad.getButtonHeld(XboxController.LEFT_BUMPER)){
      armUp();
    }

    if(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER)){
      armIntake();
    }

    if(driverGamepad.getButtonHeld(XboxController.START_BUTTON) && driverGamepad.getButtonHeld(XboxController.BACK_BUTTON)){
      armClimb();
    }

    if(Math.abs(driverGamepad.getRightY())>0.2 ) {
      armMotor.set(ControlMode.PercentOutput, driverGamepad.getRightY());
    }else{
      armMotor.set(ControlMode.PercentOutput, 0);

    }

    //Operator Controls
    if(operatorGamepad.getButtonHeld(XboxController.A_BUTTON)){
      elevatorUp(Constants.ELEVATOR_LEVEL_1);
    }

    if(operatorGamepad.getButtonHeld(XboxController.B_BUTTON)){
      elevatorUp(Constants.ELEVATOR_LEVEL_2);
    }

    if(operatorGamepad.getButtonHeld(XboxController.X_BUTTON)){
      elevatorUp(Constants.ELEVATOR_CARGO_BOX);
    }

    if(operatorGamepad.getButtonHeld(XboxController.Y_BUTTON)){
      elevatorUp(Constants.ELEVATOR_LEVEL_3);
    }

    if(operatorGamepad.getRightTriggerPressed()){
      //Score Cargo
    }else{
      //Stop Score Cargo
    }

  }

}
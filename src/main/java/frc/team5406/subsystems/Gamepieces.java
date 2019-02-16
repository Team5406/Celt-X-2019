package frc.team5406.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.Solenoid;
import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;

public class Gamepieces extends Subsystems{

    WPI_TalonSRX armMotor = new WPI_TalonSRX(7);
    WPI_VictorSPX armSlave = new WPI_VictorSPX(8);
  
    WPI_TalonSRX intakeMotor = new WPI_TalonSRX(9);
    WPI_TalonSRX conveyorMotor = new WPI_TalonSRX(10);
  
    WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(11);
    WPI_VictorSPX elevatorMotorSlave1 = new WPI_VictorSPX(12);
    WPI_VictorSPX elevatorMotorSlave2 = new WPI_VictorSPX(13);
    WPI_VictorSPX elevatorMotorSlave3 = new WPI_VictorSPX(14);
  
    WPI_TalonSRX boxMotor = new WPI_TalonSRX(15);

    Compressor compressor = new Compressor();

    Solenoid cargoDeploySolenoid = new Solenoid(Constants.CARGO_DEPLOY_SOLENOID);
    Solenoid hatchGripSolenoid = new Solenoid(Constants.HATCH_GRIP_SOLENOID);
    Solenoid hatchExtendSolenoid = new Solenoid(Constants.HATCH_EXTEND_SOLENOID);
    Solenoid climbReleaseSolenoid = new Solenoid(Constants.CLIMB_RELEASE_SOLENOID);

    public Gamepieces(){
    elevatorMotorSlave2.setInverted(true);
    elevatorMotorSlave3.setInverted(true);
    armSlave.setInverted(true);

    armSlave.follow(armMotor);
    elevatorMotorSlave1.follow(elevatorMotor);
    elevatorMotorSlave2.follow(elevatorMotor);
    elevatorMotorSlave3.follow(elevatorMotor);
    
      //Elevator Up
    elevatorMotor.selectProfileSlot(0,0);
    elevatorMotor.config_kF(0, 0.03, Constants.kTimeoutMs);
    elevatorMotor.config_kP(0, 0.4, Constants.kTimeoutMs);
    elevatorMotor.config_kI(0, 0, Constants.kTimeoutMs);
    elevatorMotor.config_kD(0, 0, Constants.kTimeoutMs);
    elevatorMotor.configAllowableClosedloopError(0, 100, Constants.kTimeoutMs);

  //Elevator Climb
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
    armMotor.config_kP(0, 0.5, Constants.kTimeoutMs);
    armMotor.config_kI(0, 0, Constants.kTimeoutMs);
    armMotor.config_kD(0, 0, Constants.kTimeoutMs);
    armMotor.configAllowableClosedloopError(0, 100, Constants.kTimeoutMs);

    armMotor.selectProfileSlot(1,0);
    armMotor.config_kF(1, 0.5, Constants.kTimeoutMs);
    armMotor.config_kP(1, 0.6, Constants.kTimeoutMs);
    armMotor.config_kI(1, 0, Constants.kTimeoutMs);
    armMotor.config_kD(1, 0, Constants.kTimeoutMs);
    armMotor.configMotionCruiseVelocity(7000, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(15000, Constants.kTimeoutMs);

    intakeMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    intakeMotor.configPeakCurrentLimit(45, Constants.kTimeoutMs);
    intakeMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    intakeMotor.enableCurrentLimit(true);

    conveyorMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    conveyorMotor.configPeakCurrentLimit(45, Constants.kTimeoutMs);
    conveyorMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    conveyorMotor.enableCurrentLimit(true);

    armMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    armMotor.configPeakCurrentLimit(45, Constants.kTimeoutMs);
    armMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    armMotor.enableCurrentLimit(true);

    elevatorMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    elevatorMotor.configPeakCurrentLimit(50, Constants.kTimeoutMs);
    elevatorMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    elevatorMotor.enableCurrentLimit(true);

    boxMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    boxMotor.configPeakCurrentLimit(45, Constants.kTimeoutMs);
    boxMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    boxMotor.enableCurrentLimit(true);

    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);

    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, Constants.kTimeoutMs);
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);

    armMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    elevatorMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);

    armMotor.setSensorPhase(true);

 }

  public void armClimb() {
    armMotor.configMotionCruiseVelocity(170, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(200, Constants.kTimeoutMs);
    armMotor.selectProfileSlot(1,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_CLIMB_END);    
  }
  public void armClimbStart() {
    armMotor.configMotionCruiseVelocity(7000, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(15000, Constants.kTimeoutMs);
    armMotor.selectProfileSlot(0,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_CLIMB_START);    
  }

  public void armClimbMid() {
    armMotor.configMotionCruiseVelocity(170, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(200, Constants.kTimeoutMs);
    armMotor.selectProfileSlot(1,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_CLIMB_MID);    
  }
  public void armIntake() {
    armMotor.configMotionCruiseVelocity(7000, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(15000, Constants.kTimeoutMs);
    armMotor.selectProfileSlot(0,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_INTAKE);    
  }

  public void armUp() {
    armMotor.configMotionCruiseVelocity(7000, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(15000, Constants.kTimeoutMs);
    armMotor.selectProfileSlot(0,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_UP);    
  }

  public void elevatorUp(int pos) {
    elevatorMotor.configMotionCruiseVelocity(50000, Constants.kTimeoutMs);
    elevatorMotor.configMotionAcceleration(200000, Constants.kTimeoutMs);
    elevatorMotor.selectProfileSlot(0,0);
    elevatorMotor.set(ControlMode.MotionMagic, pos);
  }

  public void elevatorClimb() {
    elevatorMotor.configMotionCruiseVelocity(1700, Constants.kTimeoutMs);
    elevatorMotor.configMotionAcceleration(1700, Constants.kTimeoutMs);  
    elevatorMotor.selectProfileSlot(1,0);
    elevatorMotor.set(ControlMode.MotionMagic, Constants.ELEVATOR_CLIMB);
  }
  public void elevatorUnClimb() {
    elevatorMotor.configMotionCruiseVelocity(1000, Constants.kTimeoutMs);
    elevatorMotor.configMotionAcceleration(1000, Constants.kTimeoutMs);  
    elevatorMotor.selectProfileSlot(1,0);
    elevatorMotor.set(ControlMode.MotionMagic, 0);
  }

  public void scoreCargo(){
    boxMotor.set(ControlMode.PercentOutput,0.8);
    intakeMotor.set(ControlMode.PercentOutput,0.0);
    conveyorMotor.set(ControlMode.PercentOutput,0.0);
    cargoDeploySolenoid.set(true);
  }
  public void intake(){
    elevatorUp(Constants.ELEVATOR_START);
    intakeMotor.set(ControlMode.PercentOutput,1.0);
    conveyorMotor.set(ControlMode.PercentOutput,1.0);
    boxMotor.set(ControlMode.PercentOutput,0.8);
    cargoDeploySolenoid.set(false);
  }
  public void reverseIntake(){
    elevatorUp(Constants.ELEVATOR_START);
    intakeMotor.set(ControlMode.PercentOutput,-1.0);
    conveyorMotor.set(ControlMode.PercentOutput,-1.0);
    boxMotor.set(ControlMode.PercentOutput,-0.5);
    cargoDeploySolenoid.set(false);
  }
  public void intakeDefault(){
    intakeMotor.set(ControlMode.PercentOutput,0.0);
    conveyorMotor.set(ControlMode.PercentOutput,0.0);
    boxMotor.set(ControlMode.PercentOutput,0.0);
    cargoDeploySolenoid.set(false);
  }
  public void manualArm(double joystickY){
    armMotor.set(ControlMode.PercentOutput, joystickY);
  }
  public void climbRelease() {
    climbReleaseSolenoid.set(true);
  }
  public void climb() {
    intakeMotor.set(ControlMode.PercentOutput,1.0);
    armClimb();
    elevatorClimb();
  }

  public void hatchExtend() {
    hatchExtendSolenoid.set(true);
  }
  public void hatchRetract() {
    hatchExtendSolenoid.set(false);
  }

  public void hatchGrip() {
    hatchGripSolenoid.set(true);
  }
  public void hatchRelease() {
    hatchGripSolenoid.set(false);
  }
  public void compressorEnabled() {
    compressor.start();
  }
  public void compressorDisabled() {
    compressor.stop();
  }
  public int armPosition(int elevatorPos) {
    return (int)Math.round((Math.PI / 180) * Math.asin((((Constants.CLIMB_HEIGHT * elevatorPos)/ Constants.ELEVATOR_CLIMB) - Constants.ARM_ORIGIN) / Constants.ARM_LENGTH) * (4096 / 120));
  }
  public int elevatorPosition(int armPos) {
    return (int)Math.round((Constants.ELEVATOR_CLIMB * (Math.sin((180 / Math.PI) * ((((120 * armPos) / 4096) * Constants.ARM_LENGTH) + Constants.ARM_ORIGIN))) / Constants.CLIMB_HEIGHT));
  }
}
package frc.team5406.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Gamepieces extends Subsystems{

    WPI_TalonSRX armMotor = new WPI_TalonSRX(7);
    WPI_VictorSPX armSlave = new WPI_VictorSPX(8);
  
    WPI_TalonSRX intakeMotor = new WPI_TalonSRX(9);
    WPI_TalonSRX intakeMotorSlave = new WPI_TalonSRX(13);
    WPI_TalonSRX conveyorMotor = new WPI_TalonSRX(10);
  
    CANSparkMax elevatorMotor = new CANSparkMax(11, MotorType.kBrushless);
    CANSparkMax elevatorMotorSlave1 = new CANSparkMax(12, MotorType.kBrushless);
  
    WPI_TalonSRX boxMotor = new WPI_TalonSRX(15);

    DigitalInput hatchSensor = new DigitalInput(Constants.HATCH_SENSOR);
    Compressor compressor = new Compressor();

    CANEncoder elevatorEncoder;

    CANPIDController elevatorPID;
    double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    
    Solenoid cargoDeploySolenoid = new Solenoid(Constants.CARGO_DEPLOY_SOLENOID);
    Solenoid hatchGripSolenoid = new Solenoid(Constants.HATCH_GRIP_SOLENOID);
    Solenoid hatchExtendSolenoid = new Solenoid(Constants.HATCH_EXTEND_SOLENOID);
    Solenoid climbReleaseSolenoid = new Solenoid(Constants.CLIMB_RELEASE_SOLENOID);

    public Gamepieces(){


    armSlave.setInverted(true);

    armSlave.follow(armMotor);
    elevatorMotorSlave1.follow(elevatorMotor, true);


    elevatorPID = elevatorMotor.getPIDController();
    elevatorEncoder = elevatorMotor.getEncoder();

    intakeMotorSlave.setInverted(true);
    intakeMotorSlave.follow(intakeMotor);

    // set PID coefficients
    elevatorPID.setP(2e-5, 0);
    elevatorPID.setI(1e-6, 0);
    elevatorPID.setD(0.001, 0);
    elevatorPID.setIZone(0, 0);
    elevatorPID.setFF(0.000156, 0);
    elevatorPID.setOutputRange(-1, 1, 0);


    elevatorPID.setSmartMotionMaxVelocity(20000, 0); //2500
    elevatorPID.setSmartMotionMinOutputVelocity(0, 0);
    elevatorPID.setSmartMotionMaxAccel(10000, 0);
    elevatorPID.setSmartMotionAllowedClosedLoopError(0.2, 0);

    // set PID coefficients
    elevatorPID.setP(6e-5, 1);
    elevatorPID.setI(1e-6, 1);
    elevatorPID.setD(0.004, 1);
    elevatorPID.setIZone(0, 1);
    elevatorPID.setFF(0.000156, 1);
    elevatorPID.setOutputRange(-1, 1, 1);
    

    elevatorPID.setSmartMotionMaxVelocity(7000, 1);
    elevatorPID.setSmartMotionMinOutputVelocity(0, 1);
    elevatorPID.setSmartMotionMaxAccel(5000, 1);
    elevatorPID.setSmartMotionAllowedClosedLoopError(0.1, 1);

    armMotor.selectProfileSlot(0,0);
    armMotor.config_kF(0, 1.2, Constants.kTimeoutMs);
    armMotor.config_kP(0, 0.6, Constants.kTimeoutMs);
    armMotor.config_kI(0, 0, Constants.kTimeoutMs);
    armMotor.config_kD(0, 0.0001, Constants.kTimeoutMs);
    armMotor.configAllowableClosedloopError(0, 50, Constants.kTimeoutMs);
    armMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
    armMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    armMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
  
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
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);


    conveyorMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    conveyorMotor.configPeakCurrentLimit(45, Constants.kTimeoutMs);
    conveyorMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    conveyorMotor.enableCurrentLimit(true);
    conveyorMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
    conveyorMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    conveyorMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);


    armMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    armMotor.configPeakCurrentLimit(45, Constants.kTimeoutMs);
    armMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    armMotor.enableCurrentLimit(true);

    elevatorMotor.setSmartCurrentLimit(80, 50);
    elevatorMotorSlave1.setSmartCurrentLimit(80, 50);

    boxMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    boxMotor.configPeakCurrentLimit(45, Constants.kTimeoutMs);
    boxMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    boxMotor.enableCurrentLimit(true);
    boxMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
    boxMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
    boxMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);


    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, Constants.kTimeoutMs);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);


    armMotor.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
    elevatorEncoder.setPosition(0);

    armMotor.setSensorPhase(true);

 }

  public void armClimbRunnable(double elevPos) {
    armMotor.selectProfileSlot(1,0);
    armMotor.set(ControlMode.MotionMagic, armPosition(elevPos));
  } 
  public void armClimb() {
    armMotor.config_kF(1, 0.5, Constants.kTimeoutMs);
    armMotor.config_kP(1, 0.6, Constants.kTimeoutMs);
    armMotor.config_kI(1, 0, Constants.kTimeoutMs);
    armMotor.config_kD(1, 0, Constants.kTimeoutMs);
    armMotor.configMotionCruiseVelocity(170, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(200, Constants.kTimeoutMs);
    armMotor.selectProfileSlot(1,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_CLIMB_END);    
  }
  public void armClimbLevel2() {
    armMotor.config_kF(1, 0.5, Constants.kTimeoutMs);
    armMotor.config_kP(1, 0.6, Constants.kTimeoutMs);
    armMotor.config_kI(1, 0, Constants.kTimeoutMs);
    armMotor.config_kD(1, 0, Constants.kTimeoutMs);
    armMotor.configMotionCruiseVelocity(1700, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(2000, Constants.kTimeoutMs);
    armMotor.selectProfileSlot(1,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_L2_CLIMB_END);    
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
    armMotor.configMotionCruiseVelocity(12000, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(35000, Constants.kTimeoutMs);
    armMotor.selectProfileSlot(0,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_INTAKE);    
  }

  public void armUp() {
    armMotor.configMotionCruiseVelocity(12000, Constants.kTimeoutMs);
    armMotor.configMotionAcceleration(35000, Constants.kTimeoutMs);
    armMotor.selectProfileSlot(0,0);
    armMotor.set(ControlMode.MotionMagic, Constants.ARM_UP);    
  }

  public void elevatorUp(double pos) {
    elevatorPID.setReference(pos, ControlType.kSmartMotion, 0);
  }




  public void elevatorClimb() {
    elevatorPID.setReference(Constants.ELEVATOR_CLIMB, ControlType.kSmartMotion, 1);
  }
  public void elevatorUnClimb() {
    elevatorPID.setReference(Constants.ELEVATOR_START, ControlType.kSmartMotion, 1);
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
  public void intakeClimb(){
    intakeMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    intakeMotor.configPeakCurrentLimit(15, Constants.kTimeoutMs);
    intakeMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    intakeMotor.enableCurrentLimit(true);

    intakeMotor.set(ControlMode.PercentOutput,1.0);
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
    boxMotor.set(ControlMode.PercentOutput,0.32);
    cargoDeploySolenoid.set(false);
  }
  public void manualArm(double joystickY){
    armMotor.set(ControlMode.PercentOutput, joystickY);
  }

  public void manualElevator(double joystickY){
    double pos = elevatorEncoder.getPosition();
    pos += Constants.ELEV_UP*joystickY*0.5;
    elevatorUp(pos);
  }


  public void climbRelease() {
    climbReleaseSolenoid.set(true);
  }
  public void climb() {
    //intakeMotor.set(ControlMode.PercentOutput,1.0);
    intakeMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    intakeMotor.configPeakCurrentLimit(25, Constants.kTimeoutMs);
    intakeMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);
    elevatorMotor.setSmartCurrentLimit(50, 70);
    armMotor.configContinuousCurrentLimit(9, Constants.kTimeoutMs);
    armMotor.configPeakCurrentLimit(40, Constants.kTimeoutMs);
    armMotor.configPeakCurrentDuration(100, Constants.kTimeoutMs);

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
  public int armPosition(double elevatorPos) {
    return (int)Math.round(1.2*((180/Math.PI)*(Math.asin(((Constants.CLIMB_HEIGHT*(elevatorPos)/Constants.ELEVATOR_CLIMB)+Constants.ARM_CLIMB_START_HEIGHT-Constants.ARM_ORIGIN)/Constants.ARM_LENGTH))-Constants.ARM_CLIMB_START_ANGLE)*(4096/120));
    }
  public int elevatorPosition(int armPos) {
    return (int)Math.round((Constants.ELEVATOR_CLIMB * (Math.sin((180 / Math.PI) * ((((120 * armPos) / 4096) * Constants.ARM_LENGTH) + Constants.ARM_ORIGIN * (getArmPos() > -2815 ? +1 : -1)))) / Constants.CLIMB_HEIGHT));
  }
  public double getElevatorPos(){
    return elevatorEncoder.getPosition();
  }
  public int getArmPos(){
    return armMotor.getSelectedSensorPosition(0);
  }

  public boolean haveHatch(){
    return !hatchSensor.get();
  }

  
}

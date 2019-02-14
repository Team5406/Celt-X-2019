package frc.team5406.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public final class Constants{

    public int elev1, level2, level3, cargoBoxLevel; //Encoder values

    public static int kTimeoutMs = 15;
    public static int ARM_UP = -50;
    public static int ARM_INTAKE = -2400;
    public static int ARM_CLIMB = -3800;
    public static int ELEVATOR_LEVEL_1 = -3000;
    public static int ELEVATOR_LEVEL_2 = -27655;
    public static int ELEVATOR_LEVEL_3 = -54810;
    public static int ELEVATOR_CARGO_BOX = -12455;
    public static int ELEVATOR_CLIMB = 12000;
    public static int ELEVATOR_START = 0;

    // PCM Ports
    public static int SHIFT_SOLENOID = 0;
    public static int CARGO_DEPLOY_SOLENOID = 1;
    public static int CLIMB_RELEASE_SOLENOID = 2;
    public static int HATCH_GRIP_SOLENOID = 3;
    public static int HATCH_EXTEND_SOLENOID = 4;
    

    public static AHRS navX = new AHRS(SPI.Port.kMXP);
    public static double xboxControllerDeadband = 0.2;

    public boolean equalsDeadband(double value){
        return ((-1*xboxControllerDeadband) < value && value < xboxControllerDeadband);
        }

    public static boolean SHIFT_HIGH = true;
    public static boolean SHIFT_LOW = false;
    
    public Constants() {
    }
}
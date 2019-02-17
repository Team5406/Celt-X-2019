package frc.team5406.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public final class Constants{

    public int elev1, level2, level3, cargoBoxLevel; //Encoder values

    public static int kTimeoutMs = 15;
    public static int ARM_UP = -50;
    public static int ARM_INTAKE = -2400;
    public static int ARM_CLIMB_START = -1400;
    public static double ARM_CLIMB_START_HEIGHT = 20.5;
    public static double ARM_CLIMB_START_ANGLE = 83;

    public static int ARM_CLIMB_END = -4530;
    public static int ARM_CLIMB_MID = -3000;
    public static int CARGO_LEVEL_1 = -1800;
    public static int CARGO_LEVEL_2 = -15634;
    public static int CARGO_LEVEL_3 = -29500;
    public static int HATCH_LEVEL_1 = -3000;
    public static int HATCH_LEVEL_2 = -17700;
    public static int HATCH_LEVEL_3 = -31300;
    public static int ELEVATOR_CARGO_BOX = -8000;
    public static int ELEVATOR_CLIMB = 22000;
    public static int ELEVATOR_START = -1400;
    public static double CLIMB_HEIGHT = -19;
    public static double ARM_LENGTH = 16.5;
    public static double ARM_ORIGIN = 9.5;

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
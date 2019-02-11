package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public final class Constants{

    public int elev1, level2, level3, cargoBoxLevel; //Encoder values

    public static int kTimeoutMs = 15;
    public static int ARM_UP = 50;
    public static int ARM_INTAKE = 2340;
    public static int ARM_CLIMB = 3800;
    public static int ELEVATOR_LEVEL_1 = -1258;
    public static int ELEVATOR_LEVEL_2 = -25655;
    public static int ELEVATOR_LEVEL_3 = -52810;
    public static int ELEVATOR_CARGO_BOX = -25655;
    public static int ELEVATOR_CLIMB = 12000;
    public static int ELEVATOR_START = 0;

    

    public static AHRS navX = new AHRS(SPI.Port.kMXP);
    public static double xboxControllerDeadband = 0.2;

    public boolean equalsDeadband(double value){
        return ((-1*xboxControllerDeadband) < value && value < xboxControllerDeadband);
        }

    public Constants() {
    }
}
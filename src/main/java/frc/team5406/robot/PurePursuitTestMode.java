package frc.team5406.robot;

import frc.team3256.warriorlib.auto.AutoModeBase;
import frc.team3256.warriorlib.auto.AutoModeEndedException;

import frc.team3256.warriorlib.auto.purepursuit.PurePursuitAction;

public class PurePursuitTestMode extends AutoModeBase {
    int pathIndex = 0;
    public PurePursuitTestMode (int path){
        pathIndex = path;
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new PurePursuitAction(pathIndex));
    }
}
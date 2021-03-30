package frc.robot.auto.modes;

import com.team254.lib.autos.actions.WaitAction;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("DON'T MOVE!!!!!");
    }

}
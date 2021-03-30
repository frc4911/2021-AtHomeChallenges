package frc.robot.auto.actions;

import java.util.Optional;

import com.team254.lib.autos.actions.RunOnceAction;
import com.team254.lib.autos.actions.Action;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.auto.SmartDashboardInteractions;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Limelights.CollectwardsLimelight;
import frc.robot.subsystems.Limelights.Limelight.WantedState;

import com.team254.lib.geometry.Translation2d;

public class BallSearchAction extends RunOnceAction {
    private CollectwardsLimelight collectwardsLimelight = CollectwardsLimelight.getInstance("BallSearch");
    private RobotState RS = RobotState.getInstance("BallSearch");
    private Optional<AimingParameters> PC;
    public enum PathSelection{
        NOTHING, RED_A, RED_B, BLUE_A, BLUE_B
    }
    private PathSelection selection;

    @Override
	public synchronized void runOnce(){
        PC = RS.getPowerCell();
        if (PC.isPresent() && collectwardsLimelight.seesTarget()) {
            Translation2d ballPos = PC.get().getRobotToGoal().getTranslation();
            SmartDashboard.putString("Powercell", ballPos.toString());
            if (ballPos.x() < 100){ //red
                if (ballPos.y() > 0){ //a 
                    selection = PathSelection.RED_A;
                } else {
                    selection = PathSelection.RED_B;
                }
            } else { //blue
                if (ballPos.y() > 16){ //a 
                    selection = PathSelection.BLUE_A;
                } else {
                    selection = PathSelection.BLUE_B;
                }
            }            
        } else {
            selection = PathSelection.NOTHING;
        }

        SmartDashboard.putString("Path Selection", selection.toString());
    }
    
    public PathSelection getPathSelection() {
        return selection;
    }
}
    
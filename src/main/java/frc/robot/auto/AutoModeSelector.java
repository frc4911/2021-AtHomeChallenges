package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.DoNothingMode;
import frc.robot.auto.modes.ShootMode;
import frc.robot.auto.modes.TestMode;
import frc.robot.auto.modes.BarrelMode;
import frc.robot.auto.modes.SlalomMode;
import frc.robot.auto.modes.BounceMode;
import frc.robot.auto.modes.SearchMode;
import frc.robot.auto.modes.AccuracyMode;
import frc.robot.auto.modes.ShootMode;

public class AutoModeSelector {

    public enum AutoModeChoice {
        TESTMODE,
        AUTONAV_BARREL,
        AUTONAV_SLALOM,
        AUTONAV_BOUNCE,
        GALACTIC_SEARCH,
        INTERSTELLAR__ACCURACY,
        POWERPORT_SHOOT,
        NONE
    }

    private SendableChooser<AutoModeChoice> mAutoModeChooser;

    public AutoModeSelector() {
        mAutoModeChooser = new SendableChooser<AutoModeChoice>();
        mAutoModeChooser.addOption("Test Mode", AutoModeChoice.TESTMODE);
        mAutoModeChooser.addOption("AutoNav Barrel Mode", AutoModeChoice.AUTONAV_BARREL);
        mAutoModeChooser.addOption("AutoNav Slalom Mode", AutoModeChoice.AUTONAV_SLALOM);
        mAutoModeChooser.addOption("AutoNav Bounce Mode", AutoModeChoice.AUTONAV_BOUNCE);
        mAutoModeChooser.addOption("Galactic Search Mode", AutoModeChoice.GALACTIC_SEARCH);
        mAutoModeChooser.addOption("InterStellar Accuracy Mode", AutoModeChoice.INTERSTELLAR__ACCURACY);
        mAutoModeChooser.addOption("PowerPort Shoot Mode", AutoModeChoice.POWERPORT_SHOOT);
        mAutoModeChooser.setDefaultOption("None", AutoModeChoice.NONE);
        SmartDashboard.putData("Auto Mode", mAutoModeChooser);
    }

    public AutoModeBase getSelectedAutoMode() {
        AutoModeChoice choice = mAutoModeChooser.getSelected();
        SmartDashboard.putString("Selected Auto Mode", choice.toString());
        switch (choice) {
            case TESTMODE:
                return new TestMode();
            case AUTONAV_BARREL:
                return new BarrelMode();
            case AUTONAV_SLALOM:
                return new SlalomMode();
            case AUTONAV_BOUNCE:
                return new BounceMode();
            case GALACTIC_SEARCH:
                return new SearchMode();
            case INTERSTELLAR__ACCURACY:
                return new AccuracyMode();
            case POWERPORT_SHOOT:
                return new ShootMode();
            default:
                return new DoNothingMode();
        }
    }

}
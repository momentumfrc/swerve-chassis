package frc.robot;

import java.io.File;

import com.momentum4999.utils.PIDTuner.PIDTunerSettings;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final PIDTunerSettings TUNER_SETTINGS = new PIDTunerSettings();

    static {
        if(RobotBase.isReal()) {
            TUNER_SETTINGS.saveValuesLocation = new File("/home/lvuser/pid_constants.ini");
        }
    }

}

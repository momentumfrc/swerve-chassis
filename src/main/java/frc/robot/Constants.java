package frc.robot;

import java.io.File;

import com.momentum4999.utils.PIDTuner.PIDTunerSettings;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final PIDTunerSettings TUNER_SETTINGS = new PIDTunerSettings();

    public static final int FL_DRIVE = 4;
    public static final int FL_TURN = 5;

    public static final int FR_DRIVE = 2;
    public static final int FR_TURN = 12;

    public static final int RL_DRIVE = 3;
    public static final int RL_TURN = 13;

    public static final int RR_DRIVE = 1;
    public static final int RR_TURN = 11;

    static {
        if(RobotBase.isReal()) {
            TUNER_SETTINGS.saveValuesLocation = new File("/home/lvuser/pid_constants.ini");
        }
    }

}

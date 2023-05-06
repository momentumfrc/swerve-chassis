package frc.robot.utils;

import com.momentum4999.utils.PIDTuner;
import com.momentum4999.utils.PIDTunerBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class TunerUtils {
    private static PIDTuner tryBuild(PIDTunerBuilder builder) {
        try {
            return builder.build();
        } catch(IllegalArgumentException e) {
            DriverStation.reportError("Error building tuner: " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    public static PIDTuner forSparkMax(MoSparkMaxPID sparkMax, String controllerName) {
        return forSparkMax(sparkMax, controllerName, false);
    }

    public static PIDTuner forSparkMax(MoSparkMaxPID sparkMax, String controllerName, boolean hide) {
        PIDTunerBuilder builder = new PIDTunerBuilder(controllerName)
            .withTunerSettings(Constants.TUNER_SETTINGS.withShowOnShuffleboard(!hide))
            .withP(sparkMax::setP)
            .withI(sparkMax::setI)
            .withD(sparkMax::setD)
            .withFF(sparkMax::setFF)
            .withIZone(sparkMax::setIZone)
            .withGraphValues(sparkMax);

        return tryBuild(builder);
    }

    public static PIDTuner forTalonFx(MoTalonFxPID talon, String controllerName) {
        return forTalonFx(talon, controllerName, false);
    }

    public static PIDTuner forTalonFx(MoTalonFxPID talon, String controllerName, boolean hide) {
        PIDTunerBuilder builder = new PIDTunerBuilder(controllerName)
            .withTunerSettings(Constants.TUNER_SETTINGS.withShowOnShuffleboard(!hide))
            .withP(talon::setP)
            .withI(talon::setI)
            .withD(talon::setD)
            .withFF(talon::setFF)
            .withIZone(talon::setIZone)
            .withGraphValues(talon);

        return tryBuild(builder);

    }
}

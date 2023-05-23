package frc.robot.utils;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.momentum4999.utils.PIDTuner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.utils.MoPrefs.Pref;

public class SwerveModule {
    private final String key;
    public final CANSparkMax turnMotor;
    public final WPI_TalonFX driveMotor;

    // Note: the absolute encoder returns rotations, in the range [0, 1)
    public final AnalogEncoder absoluteEncoder;

    private final MoSparkMaxPID turnPID;
    private final MoTalonFxPID drivePID;

    private PIDTuner turnTuner;
    private PIDTuner driveTuner;

    // Note: the relative encoder is scaled to return radians
    private final RelativeEncoder relativeEncoder;

    private final Pref<Double> encoderZero;
    private final Pref<Double> encoderScale;
    private final Pref<Double> driveMtrScale;

    public SwerveModule(String key, CANSparkMax turnMotor, WPI_TalonFX driveMotor, AnalogEncoder absoluteEncoder, Pref<Double> encoderZero, Pref<Double> encoderScale, Pref<Double> driveMtrScale, boolean showPidTuners) {
        this.key = key;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.encoderZero = encoderZero;
        this.encoderScale = encoderScale;
        this.driveMtrScale = driveMtrScale;

        this.turnPID = new MoSparkMaxPID(MoSparkMaxPID.Type.POSITION, turnMotor, 0);
        this.drivePID = new MoTalonFxPID(MoTalonFxPID.Type.VELOCITY, driveMotor);

        var turnSparkMaxPID = turnPID.getPID();
        turnSparkMaxPID.setPositionPIDWrappingMinInput(-Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingMaxInput(Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingEnabled(true);

        turnTuner = TunerUtils.forSparkMax(turnPID, key + "_turn", !showPidTuners);
        driveTuner = TunerUtils.forTalonFx(drivePID, key + "_drive", !showPidTuners);

        relativeEncoder = turnMotor.getEncoder();

        encoderZero.subscribe(zero -> this.setupRelativeEncoder(absoluteEncoder.get(), zero, encoderScale.get()), false);
        encoderScale.subscribe(scale -> this.setupRelativeEncoder(absoluteEncoder.get(), encoderZero.get(), scale), false);
        setupRelativeEncoder();

        var layout = Shuffleboard.getTab("match").getLayout(key, BuiltInLayouts.kList)
            .withSize(2, 1)
            .withProperties(Map.of("Label position", "LEFT"));
        layout.addDouble("Relative", () -> (AngleMath.radToRot(relativeEncoder.getPosition())));
        layout.addDouble("Absolute", absoluteEncoder::get);
    }

    public void setupRelativeEncoder() {
        setupRelativeEncoder(absoluteEncoder.get(), encoderZero.get(), encoderScale.get());
    }

    private void setupRelativeEncoder(double absPos, double absZero, double scale) {
        relativeEncoder.setPositionConversionFactor(scale);
        relativeEncoder.setVelocityConversionFactor(scale);

        double rots = absPos;
        rots = (rots + 1 - absZero) % 1;
        relativeEncoder.setPosition(AngleMath.rotToRad(rots));
    }

    public void drive(SwerveModuleState state) {
        var optimized = SwerveModuleState.optimize(state, Rotation2d.fromRadians(relativeEncoder.getPosition()));
        turnPID.setReference(MathUtil.angleModulus(optimized.angle.getRadians()));
        drivePID.setReference(optimized.speedMetersPerSecond * driveMtrScale.get());
    }

    public void directDrive(double turnSpeed, double driveSpeed) {
        turnMotor.set(turnSpeed);
        driveMotor.set(ControlMode.PercentOutput, driveSpeed);
    }

    @Override
    public String toString() {
        return String.format("SwerveModule(key=\"%s\")", this.key);
    }
}

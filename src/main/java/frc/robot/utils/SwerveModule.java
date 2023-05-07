package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.momentum4999.utils.PIDTuner;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.utils.MoPrefs.Pref;

public class SwerveModule {
    private final String key;
    public final CANSparkMax turnMotor;
    public final WPI_TalonFX driveMotor;
    public final AnalogEncoder absoluteEncoder;

    private final MoSparkMaxPID turnPID;
    private final MoTalonFxPID drivePID;

    private PIDTuner turnTuner;
    private PIDTuner driveTuner;

    private final RelativeEncoder relativeEncoder;

    private final Pref<Double> encoderZero;
    private final Pref<Double> encoderScale;

    public SwerveModule(String key, CANSparkMax turnMotor, WPI_TalonFX driveMotor, AnalogEncoder absoluteEncoder, Pref<Double> encoderZero, Pref<Double> encoderScale) {
        this.key = key;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.absoluteEncoder = absoluteEncoder;
        this.encoderZero = encoderZero;
        this.encoderScale = encoderScale;

        this.turnPID = new MoSparkMaxPID(MoSparkMaxPID.Type.POSITION, turnMotor, 0);
        this.drivePID = new MoTalonFxPID(MoTalonFxPID.Type.POSITION, driveMotor);

        var turnSparkMaxPID = turnPID.getPID();
        turnSparkMaxPID.setPositionPIDWrappingMinInput(-Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingMaxInput(Math.PI);
        turnSparkMaxPID.setPositionPIDWrappingEnabled(true);

        turnTuner = TunerUtils.forSparkMax(turnPID, key + "_turn");
        driveTuner = TunerUtils.forTalonFx(drivePID, key + "_drive");

        relativeEncoder = turnMotor.getEncoder();

        encoderZero.subscribe(zero -> this.setupRelativeEncoder(absoluteEncoder.get(), zero, encoderScale.get()), false);
        encoderScale.subscribe(scale -> this.setupRelativeEncoder(absoluteEncoder.get(), encoderZero.get(), scale), false);
        setupRelativeEncoder();
    }

    public void setupRelativeEncoder() {
        setupRelativeEncoder(absoluteEncoder.get(), encoderZero.get(), encoderScale.get());
    }

    private void setupRelativeEncoder(double absPos, double absZero, double scale) {
        relativeEncoder.setPositionConversionFactor(scale);
        relativeEncoder.setVelocityConversionFactor(scale);

        double pos = absPos;
        pos = (pos + 1 - absZero) % 1;
        relativeEncoder.setPosition(pos);
    }

    public void drive(SwerveModuleState state) {
        turnPID.setReference(MathUtil.angleModulus(state.angle.getRadians()));
        drivePID.setReference(state.speedMetersPerSecond);
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

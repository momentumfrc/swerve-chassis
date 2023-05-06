package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.SwerveModule;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule rearLeft;
    private final SwerveModule rearRight;

    private final SwerveDriveKinematics kinematics;

    public DriveSubsystem() {
        frontLeft = new SwerveModule(
            "FL",
            new CANSparkMax(Constants.FL_TURN, MotorType.kBrushless),
            new WPI_TalonFX(Constants.FL_DRIVE),
            new AnalogEncoder(Constants.FL_ENCODER),
            MoPrefs.fl_zero,
            MoPrefs.fl_scale
        );

        frontRight = new SwerveModule(
            "FR",
            new CANSparkMax(Constants.FR_TURN, MotorType.kBrushless),
            new WPI_TalonFX(Constants.FR_DRIVE),
            new AnalogEncoder(Constants.FR_ENCODER),
            MoPrefs.fr_zero,
            MoPrefs.fr_scale
        );

        rearLeft = new SwerveModule(
            "RL",
            new CANSparkMax(Constants.RL_TURN, MotorType.kBrushless),
            new WPI_TalonFX(Constants.RL_DRIVE),
            new AnalogEncoder(Constants.RL_ENCODER),
            MoPrefs.rl_zero,
            MoPrefs.rl_scale
        );

        rearRight = new SwerveModule(
            "RR",
            new CANSparkMax(Constants.RR_TURN, MotorType.kBrushless),
            new WPI_TalonFX(Constants.RR_DRIVE),
            new AnalogEncoder(Constants.RR_ENCODER),
            MoPrefs.rr_zero,
            MoPrefs.rr_scale
        );

        this.kinematics = getKinematics();
    }

    private SwerveDriveKinematics getKinematics() {
        double xoff = MoPrefs.chassis_size_x.get() / 2;
        double yoff = MoPrefs.chassis_size_y.get() / 2;

        Translation2d fl = new Translation2d(xoff, yoff);
        Translation2d fr = new Translation2d(xoff, -yoff);
        Translation2d rl = new Translation2d(-xoff, yoff);
        Translation2d rr = new Translation2d(-xoff, -yoff);
        return new SwerveDriveKinematics(fl, fr, rl, rr);
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeft.drive(states[0]);
        frontRight.drive(states[1]);
        rearLeft.drive(states[2]);
        rearRight.drive(states[3]);
    }

}

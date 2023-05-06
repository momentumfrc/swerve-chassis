package frc.robot.commands;

import javax.xml.crypto.dsig.Transform;

import com.momentum4999.utils.Utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoPrefs.Pref;

public class TeleopDriveCommand extends CommandBase {
    private static final double RAMP_TIME = 0.25;
    private static final double DEADZONE = 0.1;
    private static final double CURVE = 2;

    private DriveSubsystem drive;
    private XboxController controller;

    private SlewRateLimiter fwdLimiter;
    private SlewRateLimiter leftLimiter;
    private SlewRateLimiter turnLimiter;

    public TeleopDriveCommand(DriveSubsystem drive, XboxController controller) {
        this.drive = drive;
        this.controller = controller;

        double slewRate = 1.0 / RAMP_TIME;
        fwdLimiter = new SlewRateLimiter(slewRate);
        leftLimiter = new SlewRateLimiter(slewRate);
        turnLimiter = new SlewRateLimiter(slewRate);

        addRequirements(drive);
    }

    private double transform(double value) {
        return Utils.curve(Utils.deadzone(value, DEADZONE), CURVE);
    }

    @Override
    public void execute() {
        double fwdRequest = controller.getLeftY();
        double leftRequest = -1 * controller.getLeftX();
        double turnRequest = -1 * controller.getRightX();

        fwdRequest = transform(fwdRequest);
        leftRequest = transform(leftRequest);
        turnRequest = transform(turnRequest);

        fwdRequest = fwdLimiter.calculate(fwdRequest);
        leftRequest = leftLimiter.calculate(leftRequest);
        turnRequest = turnLimiter.calculate(turnRequest);

        double maxLinearSpeed = MoPrefs.maxLinearSpeed.get();
        double maxAngularSpeed = MoPrefs.maxAngularSpeed.get();
        ChassisSpeeds speeds = new ChassisSpeeds(
            fwdRequest * maxLinearSpeed,
            leftRequest * maxLinearSpeed,
            turnRequest * maxAngularSpeed
        );

        drive.drive(speeds);
    }
}

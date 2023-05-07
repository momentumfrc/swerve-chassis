package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MoPrefs;

public class CalibrateEncodersCommand extends CommandBase {
    private static class Calibrator {
        private static final double CALIBRATE_END = 0.8;

        private static class DataPoint {
            public final double abs;
            public final double rel;

            public DataPoint(double abs, double rel) {
                this.abs = abs;
                this.rel = rel;
            }
        }

        private ArrayList<DataPoint> data = new ArrayList<>();

        private DoubleSupplier absEncoder;
        private DoubleSupplier relEncoder;

        double absZero;
        double relZero;

        public Calibrator(DoubleSupplier absEncoder, DoubleSupplier relEncoder) {
            this.absEncoder = absEncoder;
            this.relEncoder = relEncoder;
        }

        public void start() {
            data.clear();
            absZero = absEncoder.getAsDouble();
            relZero = relEncoder.getAsDouble();
        }

        private double getAbs() {
            return (absEncoder.getAsDouble() + 1 - absZero) % 1;
        }

        private double getRel() {
            return (relEncoder.getAsDouble() + 1 - relZero) % 1;
        }

        public boolean isFinished() {
            return getAbs() > CALIBRATE_END;
        }

        public void recordDataPoint() {
            DataPoint datum = new DataPoint(getAbs(), getRel());
            if(datum.abs > CALIBRATE_END) {
                return;
            }
            data.add(datum);
        }

        /**
         * Uses least-squares regression to calculate by what factor the current relative encoder
         * scale should be adjusted by to minimize the error between the relative and absolute
         * encoders.
         * <p>
         * Let R be the relative encoder count, and A be the absolute position (as given by the
         * absolute encoder). Also let B be the current encoder scale and let F be the error factor
         * such that R*B*F = A. It follows that B = A / R*B.
         * Find the line of best fit where x = A and y = R*B. The slope of this line, m, estimates
         * R*B / A. Thus, B = 1/m, and so the correction factor is the reciprocal of the slope of
         * the line of best fit where the absolute position is the x-axis and the current estimated
         * position is the y-axis.
         *
         * @return The factor the current relative encoder scale should be adjusted by to minimize
         * the error between the relative and absolute encoders.
         */
        public double calculateCorrectionFactor() {
            double sumx = 0;
            double sumy = 0;
            double sumx2 = 0;
            double sumxy = 0;
            double n = data.size();
            for(DataPoint point : data) {
                sumx += point.abs;
                sumy += point.rel;
                sumx2 += point.abs * point.abs;
                sumxy += point.abs * point.rel;
            }

            double lsrl_slope = ( (n * sumxy) - (sumx * sumy) ) / ( (n * sumx2) - (sumx * sumx) );

            return 1 / lsrl_slope;
        }
    }

    private static final double CALIBRATE_SPEED = 0.1;

    private final DriveSubsystem drive;
    private Calibrator frontLeft;
    private Calibrator frontRight;
    private Calibrator rearLeft;
    private Calibrator rearRight;

    public CalibrateEncodersCommand(DriveSubsystem drive) {
        this.drive = drive;

        frontLeft = new Calibrator(drive.frontLeft.absoluteEncoder::get, drive.frontLeft.turnMotor.getEncoder()::getPosition);
        frontRight = new Calibrator(drive.frontRight.absoluteEncoder::get, drive.frontRight.turnMotor.getEncoder()::getPosition);
        rearLeft = new Calibrator(drive.rearLeft.absoluteEncoder::get, drive.rearLeft.turnMotor.getEncoder()::getPosition);
        rearRight = new Calibrator(drive.rearRight.absoluteEncoder::get, drive.rearRight.turnMotor.getEncoder()::getPosition);
    }

    @Override
    public void initialize() {
        frontLeft.start();
        rearLeft.start();
        frontRight.start();
        rearRight.start();
    }

    @Override
    public void execute() {
        drive.frontLeft.directDrive(frontLeft.isFinished() ? 0 : CALIBRATE_SPEED, 0);
        drive.frontRight.directDrive(frontRight.isFinished() ? 0 : CALIBRATE_SPEED, 0);
        drive.rearLeft.directDrive(rearLeft.isFinished() ? 0 : CALIBRATE_SPEED, 0);
        drive.rearRight.directDrive(rearRight.isFinished() ? 0 : CALIBRATE_SPEED, 0);

        frontLeft.recordDataPoint();
        frontRight.recordDataPoint();
        rearLeft.recordDataPoint();
        rearRight.recordDataPoint();
    }

    @Override
    public boolean isFinished() {
        return frontLeft.isFinished() && frontRight.isFinished() && rearLeft.isFinished() && rearRight.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            return;
        }
        MoPrefs.fl_scale.set(MoPrefs.fl_scale.get() * frontLeft.calculateCorrectionFactor());
        MoPrefs.fr_scale.set(MoPrefs.fr_scale.get() * frontRight.calculateCorrectionFactor());
        MoPrefs.rl_scale.set(MoPrefs.rl_scale.get() * rearLeft.calculateCorrectionFactor());
        MoPrefs.rr_scale.set(MoPrefs.rr_scale.get() * rearRight.calculateCorrectionFactor());
    }
}
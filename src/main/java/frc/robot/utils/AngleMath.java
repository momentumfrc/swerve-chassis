package frc.robot.utils;

public class AngleMath {
    public static double rotToRad(double rot) {
        return 2 * Math.PI * (rot - 0.5);
    }

    public static double radToRot(double rad) {
        return (rad / (2 * Math.PI)) + 0.5;
    }
}

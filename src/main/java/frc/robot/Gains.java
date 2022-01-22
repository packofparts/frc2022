package frc.robot;

public class Gains {
	public double kP;
	public double kI;
	public double kD;
	public final double kIz;
	public final double kFF;
	public final double kMinOutput;
	public final double kMaxOutput;
	public final int kSlot;

	public Gains(double _kP, double _kI, double _kD, double _kIz, double _kFF, double _kMinOutput, double _kMaxOutput, int _kSlot){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kIz = _kIz;
		kFF = _kFF;
		kMinOutput = _kMinOutput;
		kMaxOutput = _kMaxOutput;
		kSlot = _kSlot;
	}
}
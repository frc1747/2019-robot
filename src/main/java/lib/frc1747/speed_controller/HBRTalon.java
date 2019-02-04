package lib.frc1747.speed_controller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class HBRTalon extends TalonSRX {
	private static final double READ_TIME = 0.1; // Seconds
	private double scaling = 1;
	private boolean sensorPhase = false;

	public HBRTalon(int deviceNumber) {
		super(deviceNumber);
	}

	

	public String getSmartDashboardType() {
		return "HBRTalon";
	}

	@Override
	public void setSensorPhase(boolean PhaseSensor) {
		sensorPhase = PhaseSensor;
		super.setSensorPhase(PhaseSensor);
	}
	
	/**
	 * Get the speed of the device (units per second)
	 * If scaling is used, this method will return the scaled units per second.
	 * @return scaled units per second
	 */
	
	public double getSpeed(int pidIdx) {
		double speed = super.getSelectedSensorVelocity(pidIdx) / (READ_TIME * scaling);
		return speed;
	}
	
	public double getPosition(int pidIdx){
		return super.getSelectedSensorPosition(pidIdx) / (scaling);
	}
	/**
	 * Refer to CTRE javadocs if used with non-SPEED and non-POSITION modes.
	 * In position mode, set the param as the desired scaled units.
	 * In speed mode, set the param as the scaled units per SECOND.
	 * @param outputValue the desired output of the Talon (depends on the current Talon Control Mode)
	 */
	@Override
	public void set(ControlMode mode, double outputValue) {
		if(super.getControlMode() == ControlMode.Velocity) {
			outputValue *= (scaling * READ_TIME);
		}
		else if(super.getControlMode() == ControlMode.Position) {
			outputValue *= scaling; // fix this
		}
		super.set(mode, outputValue);
	}
	
	/**
	 * Sets the maximum error where Integral Accumulation will occur during a closed-loop mode.
	 * This just calls the other setIZone method that takes a double param
	 * @param izone an integer for the maximum error allowed
	 */
	
	
	/**
	 * Sets the maximum error where Integral Accumulation will occur during a closed-loop mode.
	 * @param izone a double for the maximum error allowed
	 */
	public void setIZone(int slotIdx, double izone, int timeoutMs) {
		izone *= (scaling * READ_TIME);
		izone = Math.round(izone);
		super.config_IntegralZone(slotIdx, (int) izone, timeoutMs);
	}

	/**
	   * Scales from native units to desired units, for example:
	   * <p> getPosition returns super.getPosition() / (scaling = ENCODER_CYCLES_PER_REVOLUTION * 4), where 4 is the number of edges on the encoder.
	   * Example2: If the encoder has 100 edges for every 1 foot of travel, scaling would equal 1/100.
	   * @param scaling the value to be used when scaling the Talon's native units.
	   */
	public void setScaling(double scaling) {
		this.scaling = scaling;
	}
	
	/**
	 * Gets the currently set scaling value. If scaling is not currently set, a 1 will be returned.
	 * @return scaling the value used for scaling the Talon's native units
	 */
	public double getScaling() {
		return scaling;
	}

	/**
	 * Returns whether or not the sensor is reversed.
	 * @return sensorReversed True if the sensor is reversed. False if the sensor is not reversed.
	 */
	public boolean getSensorPhase() {
		return sensorPhase;
	}
	
	/**
	 * Sets PIDF constants on the Talon.
	 * @param kP Proportion constant
	 * @param kI Integral constant
	 * @param kD Derivative constant
	 * @param kF Feed Forward constant
	 */
	public void setPIDF(double kP, double kI, double kD, double kF, int timeoutMs, int slotIdx){
		super.config_kP(slotIdx, kP, timeoutMs);
		super.config_kI(slotIdx, kI, timeoutMs);
		super.config_kD(slotIdx, kD, timeoutMs);
		super.config_kF(slotIdx, kF, timeoutMs);
	}
}
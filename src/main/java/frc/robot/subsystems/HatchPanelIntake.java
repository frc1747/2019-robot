/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap;
import lib.frc1747.subsytems.HBRSubsystem;

/**
 * Add your docs here.
 */
public class HatchPanelIntake extends HBRSubsystem<HatchPanelIntake.Follower> {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static HatchPanelIntake hatch;
  TalonSRX wrist;
  TalonSRX roller;
  AnalogInput sensorLeft;
  AnalogInput sensorRight;
  AnalogInput encoder;
  double wristOffset;
  double setPoint;
  public static double[] positions = {3.7, Math.PI - 0.1, Math.PI / 2};

  public HatchPanelIntake(){
    //This subsystem is not configured for proper PID usage, I just made the basic parts
    wrist = new TalonSRX(RobotMap.WRIST_TALON);
    wrist.setInverted(RobotMap.WRIST_INVERTED);
    roller = new TalonSRX(RobotMap.ROLLER_TALON);
    sensorLeft = new AnalogInput(RobotMap.LEFT_IR_SENSOR_PORT);
    sensorRight = new AnalogInput(RobotMap.RIGHT_IR_SENSOR_PORT);
    encoder = new AnalogInput(RobotMap.HATCH_INTAKE_ENCODER_PORT);
    wristOffset = RobotMap.WRIST_OFFSET;
  }



  public enum Follower{
    WRIST, NOTHING;
  }

  public enum Positions{
    STOWED, HANDOFF, GROUND;
  }

  public void tellSetpoint(double setpoint) {
    setPoint = setpoint;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setWristPower(double power) {
    wrist.set(ControlMode.PercentOutput, power);
  }

  public void setRollerPower(double power) {
    roller.set(ControlMode.PercentOutput, power);
  }

  public double getWristPosition() {
    if (RobotMap.WRIST_ENCODER_GEAR * (-encoder.getVoltage()) * 2 * Math.PI/5 - wristOffset >= 0) {
			return RobotMap.WRIST_ENCODER_GEAR * (-encoder.getVoltage()) * 2 * Math.PI/5 - wristOffset;
		} else {
			return RobotMap.WRIST_ENCODER_GEAR * (5 + -encoder.getVoltage()) * 2 * Math.PI/5 - wristOffset;
		}
  }

  public double getRollerSpeed() {
    return roller.getMotorOutputPercent();
  }

  public Boolean leftSensorActivated() {
    return (sensorLeft.getVoltage() > RobotMap.SENSOR_THRESHOLD);
  }

  public Boolean rightSensorActivated() {
    return (sensorRight.getVoltage() > RobotMap.SENSOR_THRESHOLD);
  }

  public Boolean sensorsActivated() {
    return (leftSensorActivated() && rightSensorActivated());
  }

  public static HatchPanelIntake getInstance(){
    if(hatch == null){
      hatch = new HatchPanelIntake();
    }
    return hatch;
  }
  public void pidWrite(double[] power) {
    if (setPoint == positions[2] && Math.abs(getWristPosition() - setPoint) < Math.PI / 12) {
      wrist.set(ControlMode.PercentOutput, 0);
      GambeziDashboard.set_double("Wrist Power", 0);
    }
    else if (setPoint == positions[0] && Math.abs(getWristPosition() - setPoint) < Math.PI / 12) {
      wrist.set(ControlMode.PercentOutput, 0);
      GambeziDashboard.set_double("Wrist Power", 0);
    }
    // else if (setPoint == positions[1]) {
    //   wrist.set(ControlMode.PercentOutput, power[0] + -Math.sin(getWristPosition()) * 0.07);
    //   GambeziDashboard.set_double("Wrist Power", power[0] + -Math.sin(getWristPosition()) * 0.07);
    // }
    else {
      wrist.set(ControlMode.PercentOutput, power[0] + Math.sin(getWristPosition()) * 0.14);
      GambeziDashboard.set_double("Wrist Power", power[0] + Math.sin(getWristPosition()) * 0.14);
    }

    
    // SmartDashboard.putNumber("distance setpoint", power[0]);
    // SmartDashboard.putNumber("angle setpoint", power[1]);
  }

  public void errorsWrite(double[] errors) {

  }

  public double[][] pidRead() {
    double[][] input = new double[2][2];
    input[0][0] = getWristPosition();
    input[1][0] = 0;
    input[0][1] = 0;
    input[1][1] = 0;
    return input;
  }

  public void internalVariablesWrite(double[] vars) {
    GambeziDashboard.set_double("Wrist Setpoint", vars[0]);
    GambeziDashboard.set_double("Wrist Supposed Setpoint", setPoint);
  }
}

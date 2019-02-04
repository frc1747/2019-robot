/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

  public HatchPanelIntake(){
    //This subsystem is not configured for proper PID usage, I just made the basic parts
    wrist = new TalonSRX(RobotMap.WRIST_TALON);
    roller = new TalonSRX(RobotMap.ROLLER_TALON);
    sensorLeft = new AnalogInput(RobotMap.LEFT_IR_SENSOR_PORT);
    sensorRight = new AnalogInput(RobotMap.RIGHT_IR_SENSOR_PORT);
    encoder = new AnalogInput(RobotMap.CARGO_INTAKE_ENCODER_PORT);
  }

  

  public enum Follower{
    WRIST;
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
    return encoder.getVoltage();
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
    wrist.set(ControlMode.PercentOutput, power[0]);
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
  }
}

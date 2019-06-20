/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotMap;
import lib.frc1747.subsytems.HBRSubsystem;

public class Climber extends HBRSubsystem<Climber.Follower> {

  static Climber climb;
  public VictorSPX liftMotor;
  private Encoder encoder;
  public static double[] positions = {0.5, 27};

  public Climber(){
    liftMotor = new VictorSPX(RobotMap.LIFT_MOTOR_PORT);
    // encoder = new Encoder(RobotMap.LIFT_ENCODER_PORT_1, RobotMap.LIFT_ENCODER_PORT_2);
  }

  public void setPower(double power){
    liftMotor.set(ControlMode.PercentOutput, -power);
  }

  public enum Follower{
    LIFT, NOTHING
  }

  public enum LiftHeights{
    RETRACTED, HAB3
  }

  public void pidWrite(double[] power) {
    setPower(power[0]);
  }

  public void errorsWrite(double[] errors) {

  }

  public double[][] pidRead() {
    double[][] input = new double[2][2];
    input[0][0] = getCurrentHeight();
    input[1][0] = getCurrentSpeed();
    input[0][1] = 0;
    input[1][1] = 0;
    return input;
  }
  
  public void internalVariablesWrite(double[] vars) {
    GambeziDashboard.set_double("Climber/setpoint", vars[0]);
    GambeziDashboard.set_double("Climber/actualposition", vars[1]);
  }

  public static Climber getInstance(){
    if(climb == null){
      climb = new Climber();
    }
    return climb;
  }

  public double getVoltage(){
    return liftMotor.getMotorOutputVoltage();
  }

  public double getCurrent(){
    return 1;
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public double getCurrentHeight(){
    return encoder.get() * RobotMap.LIFT_ENCODER_SCALING;
  }

  public double getCurrentSpeed() {
    return encoder.getRate() / RobotMap.LIFT_ENCODER_SCALING;
  }

  @Override
  public void initDefaultCommand() {
  }
}

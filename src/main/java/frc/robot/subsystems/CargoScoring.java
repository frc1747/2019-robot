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
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class CargoScoring extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static CargoScoring cargo;
  TalonSRX motor;
  AnalogInput sensor;

  public CargoScoring() {
    motor = new TalonSRX(RobotMap.CARGO_SCORING_MOTOR_PORT);
    sensor = new AnalogInput(RobotMap.CARGO_SENSOR_PORT);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  public Boolean sensorActivated() {
    return (sensor.getVoltage() > RobotMap.SENSOR_THRESHOLD);
  }
  public static CargoScoring getInstance(){
    if(cargo == null){
      cargo = new CargoScoring();
    }
    return cargo;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class CargoIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  static CargoIntake intake;
  TalonSRX motor;
  Solenoid solenoid;

  public CargoIntake() {
    motor = new TalonSRX(RobotMap.CARGO_INTAKE_MOTOR_PORT);
    solenoid = new Solenoid(RobotMap.CARGO_SOLENOID);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  public void setExtended(Boolean bool) {
    solenoid.set(bool);
  }

  public Boolean getPosition() {
    return solenoid.get();
  }
  public double getCurrent(){
    return motor.getOutputCurrent();
  }
  public static CargoIntake getInstance(){
    if(intake == null){
      intake = new CargoIntake();
    }
    return intake;
  }
}

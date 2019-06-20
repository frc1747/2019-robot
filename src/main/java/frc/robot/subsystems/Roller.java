/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Roller extends Subsystem {
  TalonSRX roller;
  static Roller rollersub;
  public Roller(){
    roller = new TalonSRX(RobotMap.ROLLER_TALON);
    roller.setInverted(true);
  }

  public void setRollerPower(double power) {
    roller.set(ControlMode.PercentOutput, power);
  }

  public double getRollerSpeed() {
    return roller.getMotorOutputPercent();
  }

  @Override
  public void initDefaultCommand() {
  }
  
  public static Roller getInstance(){
    if(rollersub == null){
      rollersub = new Roller();
      return (rollersub);
    }else{
      return rollersub;
    }
  }
}
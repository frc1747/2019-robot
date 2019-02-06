/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.SetWristPower;
import frc.robot.commands.Elevator.ElevatorProfiles;
import frc.robot.commands.Extension.Extend;
import frc.robot.commands.HatchPanelIntake.WristProfile;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchPanelIntake;
import lib.frc1747.controller.Logitech;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Logitech driver;
  Logitech operator;
  static OI oi;

  public OI() {
    driver = new Logitech(RobotMap.DRIVER_PORT);
    operator = new Logitech(RobotMap.OPERATOR_PORT);
    driver();
  }

  public void driver() {
    driver.getButton(Logitech.A).whenPressed(new ElevatorProfiles(Elevator.ElevatorPositions.C1));
    driver.getButton(Logitech.B).whenPressed(new ElevatorProfiles(Elevator.ElevatorPositions.HP1));
    driver.getButton(Logitech.X).whenPressed(new ElevatorProfiles(Elevator.ElevatorPositions.C2));
    driver.getButton(Logitech.Y).whenPressed(new ElevatorProfiles(Elevator.ElevatorPositions.CFLOOR));
    driver.getButton(Logitech.RB).whenPressed(new ElevatorProfiles(Elevator.ElevatorPositions.CSHIP));
    driver.getButton(Logitech.LB).whenPressed(new ElevatorProfiles(Elevator.ElevatorPositions.GROUND));
    driver.getButton(Logitech.RT).whenPressed(new ElevatorProfiles(Elevator.ElevatorPositions.HP2));
    driver.getButton(Logitech.LT).whenPressed(new Extend());
    driver.getDPad(Logitech.UP).whenPressed(new WristProfile(HatchPanelIntake.Positions.STOWED));
    driver.getDPad(Logitech.RIGHT).whenPressed(new WristProfile(HatchPanelIntake.Positions.HANDOFF));
    driver.getDPad(Logitech.DOWN).whenPressed(new WristProfile(HatchPanelIntake.Positions.GROUND));
    driver.getDPad(Logitech.LEFT).whenPressed(new SetWristPower());

    // driver.getButton(Logitech.Y).whenPressed(new FakeAuton());
    // driver.getButton(Logitech.RB).whenPressed(new Stop());
    // driver.getButton(Logitech.LT).whenPressed(new DriveSetSpeed());
    // driver.getDPad(Logitech.UP).whenPressed(new DriveProfiles("/home/lvuser/platform_to_rocket_far_fwd_nor.csv"));
    // driver.getDPad(Logitech.DOWN).whenPressed(new DriveProfiles("/home/lvuser/big_s_curve_fwd_nor.csv"));
    // driver.getDPad(Logitech.RIGHT).whenPressed(new DriveProfiles("/home/lvuser/swerve_fwd_nor.csv"));

    // driver.getButton(Logitech.B).whenPressed(new DriveDistance(-5));
    // driver.getButton(Logitech.Y).whenPressed(new TakeOut(-0.4));
    // driver.getButton(Logitech.X).whenPressed(new TakeIn(0.4));
    // GROUND, CFLOOR, HP1, C1, CSHIP, HP2, C2;
  }

  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }

  public Logitech getDriver() {
    return driver;
  }

  public Logitech getOperator(){
    return operator;
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.CargoIntake.Intake;
import frc.robot.commands.CargoScoring.Outtake;
import frc.robot.commands.Drivetrain.DriveProfiles;
import frc.robot.commands.Elevator.ElevatorProfiles;
import frc.robot.commands.Elevator.HatchFinal;
import frc.robot.commands.Elevator.ZeroElevator;
import frc.robot.commands.HatchPanelIntake.GroundIntake2;
import frc.robot.commands.HatchPanelIntake.Handoff;
import frc.robot.commands.HatchPanelScoring.Beak;
import frc.robot.commands.HatchPanelScoring.HPScore;
// import frc.robot.commands.HatchPanelScoring.WeakAuton;
import frc.robot.subsystems.Elevator;
import lib.frc1747.controller.Xbox;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Xbox driver;
  Xbox operator;
  static OI oi;

  public OI() {
    driver = new Xbox(RobotMap.DRIVER_PORT);
    operator = new Xbox(RobotMap.OPERATOR_PORT);
    driver();
    operator();
  }
  public void operator(){
    operator.getButton(Xbox.Y).whenPressed(new ElevatorProfiles(3));
    operator.getButton(Xbox.B).whenPressed(new ElevatorProfiles(1));
    operator.getButton(Xbox.A).whenPressed(new ElevatorProfiles(4));
    operator.getButton(Xbox.X).whenPressed(new ElevatorProfiles(2));
    operator.getDPad(Xbox.UP).whenPressed(new Intake(0, 0.7, false));
    operator.getDPad(Xbox.UP).whenReleased(new Intake(0, 0, false));

    operator.getButton(Xbox.START).whenPressed(new ZeroElevator());
    // operator.getButton(Xbox.BACK).whenPressed(new ManualWrist());
  }
  public void driver() {
    driver.getButton(Xbox.A).whenPressed(new HPScore());
    // driver.getButton(Xbox.A).whenPressed(new SetWristPower());

    driver.getButton(Xbox.X).whenPressed(new ElevatorProfiles(Elevator.ElevatorPositions.HP1));
    // driver.getButton(Xbox.X).whenPressed(new LiftElevator());
    // driver.getButton(Xbox.Y).whenPressed(new WeakAuton());
    // driver.getButton(Xbox.B).whenPressed(new DriveProfiles("/home/lvuser/center_to_pickup_2_fwd_nor.csv"));
    // driver.getButton(Xbox.B).whenPressed(new DriveProfiles("/home/lvuser/staight_fwd_nor.csv"));



    driver.getRTButton().whenPressed(new Intake(0.7, 0.7, true));
    driver.getRTButton().whenReleased(new Intake(0, 0, false));

    // driver.getRTButton().whenPressed(new Outtake(1));
    driver.getButton(Xbox.RB).whenPressed(new Outtake(-1));
    driver.getLTButton().whenPressed(new GroundIntake2());//
    driver.getButton(Xbox.LB).whenPressed(new Beak());

    driver.getDPad(Xbox.UP).whenPressed(new Handoff());
    driver.getDPad(Xbox.RIGHT).whenPressed(new HatchFinal());




  }

  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }

  public Xbox getDriver() {
    return driver;
  }

  public Xbox getOperator(){
    return operator;
  }
}

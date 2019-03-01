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

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotMap;
import frc.robot.commands.Elevator.ManualElevator;
import lib.frc1747.subsytems.HBRSubsystem;

/**
 * Add your docs here.
 */
public class Elevator extends HBRSubsystem <Elevator.Follower> {
  
  private TalonSRX leftmotor;
  private TalonSRX rightmotor;
  private Encoder encoder;
  double setPoint;
  public static double[] positions = {0, 16.5, 45-11, 47-19+3, 55.5-11, 75-19+4, 87.5-11};
  static Elevator elevator;

    public Elevator(){
      super(RobotMap.DT);
      leftmotor = new TalonSRX(RobotMap.LEFT_ELEVATOR_PORT);
      rightmotor = new TalonSRX(RobotMap.RIGHT_ELEVATOR_PORT);
      leftmotor.setInverted(RobotMap.LEFT_ELEVATOR_INVERTED);
      rightmotor.setInverted(RobotMap.RIGHT_ELEVATOR_INVERTED);
      encoder = new Encoder(RobotMap.ELEVATOR_ENCODER_PORT_2, RobotMap.ELEVATOR_ENCODER_PORT);
      //put in 10 turn potentiometer
    } 

    public enum Follower{
      ELEVATOR, NOTHING;
    }
    
    public enum ElevatorPositions{
      HP1, C1, CSHIP, HP2, C2, HP3, C3;
    }
    public double getVoltage(){
      return leftmotor.getMotorOutputVoltage();
    }
    public double getCurrent(){
      return leftmotor.getOutputCurrent();
    }
    public void resetEncoder() {
      encoder.reset();
    }
    
    public double getElevatorSpeed() {
      return encoder.getRate() / RobotMap.ELEVATOR_SCALING;
    }
    public void tellSetpoint(double setPoint){
      this.setPoint = setPoint;
    }
    public double getSetPoint(){
      return setPoint;
    }
    
    public void pidWrite(double[] power) {
      if(getDistance() <= 5 && setPoint < 5){
        setPower(0);
      }else{
        setPower(power[0] + 0.3);
        GambeziDashboard.set_double("elev voltage", elevator.getVoltage());
        GambeziDashboard.set_double("elev voltage", elevator.getCurrent());
      }
      GambeziDashboard.set_double("Elevator/PIDWrite", power[0]);
    }
    
    public void errorsWrite(double[] errors) {

    }

    public double[][] pidRead() {
      double[][] input = new double[2][2];
      input[0][0] = getDistance();
      input[1][0] = getElevatorSpeed();
      input[0][1] = getDistance();
      input[1][1] = getElevatorSpeed();
      return input;
    }
    
    public void internalVariablesWrite(double[] vars) {
      GambeziDashboard.set_double("Elevator/setpoint", vars[0]);
      GambeziDashboard.set_double("Elevator/actualposition", vars[1]);
    }
    
    public void setPower(double power){
      leftmotor.set(ControlMode.PercentOutput, power);
      rightmotor.set(ControlMode.PercentOutput, power);
    }
    
    public double getElevatorPosition(){
      return encoder.get();
    }
    
    public double getDistance(){
      return encoder.get()/RobotMap.ELEVATOR_SCALING;
    }
    
    public static Elevator getInstance(){
      if(elevator == null){
        elevator = new Elevator();
      }
      return elevator;
    }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualElevator());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

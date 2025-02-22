// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LED.LEDStates;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  private boolean teleop, deathToMarkFlag, deathToJackFlag = false;

  private PowerDistribution pdh;

  private GenericEntry canUtil = Shuffleboard.getTab("System").add("CAN", 0).withPosition(0, 0).getEntry();
  private GenericEntry currentScoringLocation = Shuffleboard.getTab("System").add("Scoring Location", "").withPosition(1, 0).getEntry();
  private GenericEntry currentPickUpLocation = Shuffleboard.getTab("System").add("PickUp Location", "").withPosition(2, 0).getEntry();

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject");
    if(isReal()){
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      pdh = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);
      pdh.setSwitchableChannel(true);
      pdh.clearStickyFaults();
      ComplexWidget pdhWidget = Shuffleboard.getTab("System").add("PDH", pdh).withPosition(3, 0);
      pdhWidget.hashCode();
    }
    
    if(Constants.DEBUG){
      Logger.start();
    }

    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    currentScoringLocation.setString(OperatorController.getScoringLocation().get().name());
    currentPickUpLocation.setString(OperatorController.getPickUpLocation().get().name());
    
    canUtil.setDouble(RobotController.getCANStatus().percentBusUtilization * 100);
    Logger.recordOutput("CAN", RobotController.getCANStatus().percentBusUtilization * 100);

    if(DriverStation.isEStopped() && !deathToMarkFlag){
      deathToMarkFlag = true;
      Shuffleboard.getTab("Death").add("Death", "We have failed!!! :(").withPosition(0, 0);
      Shuffleboard.getTab("Death").add("Death2", "MARKKKKKKKK!!!!!!").withPosition(0, 1);
      Shuffleboard.selectTab("Death");
    }

    if(DriverStation.isAutonomous() && DriverStation.isFMSAttached() && DriverStation.getMatchTime() < 10 && DriverStation.isDisabled() && !deathToJackFlag) {
      deathToJackFlag = true;
      Shuffleboard.getTab("Death").add("Death", "We have failed!!! :(").withPosition(1, 0);
      Shuffleboard.getTab("Death").add("Death2", "JACKKKKKKKK!!!!!!").withPosition(1, 1);
      Shuffleboard.selectTab("Death");
    }

    if(DriverStation.isFMSAttached() && DriverStation.isTeleopEnabled()){
      teleop = true;
    }
  }

  @Override
  public void disabledInit() {
    if(DriverStation.isEStopped() || (DriverStation.isFMSAttached() && DriverStation.getMatchTime() < 3 && teleop)){
      robotContainer.getMusicCommand().ignoringDisable(true).schedule();
    }
  }

  @Override
  public void disabledPeriodic() {
    robotContainer.getLEDSystem().setState(LEDStates.DISABLED);
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if(autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    robotContainer.getLEDSystem().setState(LEDStates.AUTONOMOUS);
  }

  @Override
  public void teleopInit() {
    if(autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.getInitialProtectCommand().schedule();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}

































































































































































































































/*
 * Beavers to inches = 36 / number of beaver
 */




// Luke is Coach's Teddy Bear 





//smidgen = 0.1
/*if(DriverStation.isDisabled() && flag && DriverStation.getMatchTime() > 130){
  //FIX ME Get RoboRandy and Greyson to Approve!
  flag = false;
  Shuffleboard.getTab("GREYSON").add("GREYSON", "STRONGER CORE!!  MORE SIX-PACK!! BEST LIFT!!").withPosition(0, 0);
  Shuffleboard.selectTab("GREYSON");
}
if(DriverStation.isEStopped() && flag1){
  //FIX ME Get RoboRandy to Approve!
  flag1 = false;
  Shuffleboard.getTab("Death").add("Death", "We have failed!!! :(").withPosition(0, 0);
  Shuffleboard.getTab("Death").add("Death2", "MARKKKKKKKK!!!!!!").withPosition(0, 1);
  Shuffleboard.selectTab("Death");
}*/
//This is the life of a programmer. We are under valued and over worked. Everything we get breaks. There are so many mechanical issues with the robot that I have lost count. This will probably never get seen again, but if it does, good luck in your future in Java. I hope that you don't have to use Shuffleboard.
//Sincerely, 
//  Jack Left
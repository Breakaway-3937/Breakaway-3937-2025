// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbAvator.ClimbAvator;
import frc.robot.subsystems.Soda.DrPepper;
import frc.robot.subsystems.Soda.MrPibb;
import frc.robot.subsystems.Swerve.Swerve;

public class Music extends Command {
  private final Orchestra orchestra;
  private final Swerve s_Swerve;
  private final MrPibb s_MrPibb;
  private final DrPepper s_DrPepper;
  private final ClimbAvator s_ClimbAvator;
  private int count;
  private double time;
  /** Creates a new Music. */
  public Music(Swerve s_Swerve, MrPibb s_MrPibb, DrPepper s_DrPepper, ClimbAvator s_ClimbAvator) {
    orchestra = new Orchestra();
    this.s_Swerve = s_Swerve;
    this.s_MrPibb = s_MrPibb;
    this.s_DrPepper = s_DrPepper;
    this.s_ClimbAvator = s_ClimbAvator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve, s_MrPibb, s_DrPepper, s_ClimbAvator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = (int) (Math.random() * 12);
    for(int i = 0; i < 4; i++){
      orchestra.addInstrument(s_Swerve.getModule(i).getDriveMotor());
      orchestra.addInstrument(s_Swerve.getModule(i).getSteerMotor());
    }

    orchestra.addInstrument(s_MrPibb.getTurretMotor());
    orchestra.addInstrument(s_MrPibb.getWristMotor());
    orchestra.addInstrument(s_DrPepper.getLoaderMotor());
    orchestra.addInstrument(s_DrPepper.getThumbMotor());
    orchestra.addInstrument(s_ClimbAvator.getShoulderMotor());
    orchestra.addInstrument(s_ClimbAvator.getBoulderMotor());
    orchestra.addInstrument(s_ClimbAvator.getElevatorMotor());
    orchestra.addInstrument(s_ClimbAvator.getDetonatorMotor());
    orchestra.addInstrument(s_ClimbAvator.getBilboBagginsTheBackMotor());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(orchestra.getCurrentTime() >= time){
      if(count == 0){
        orchestra.loadMusic("GodBless.chrp");
        count++;
        time = 99;
        orchestra.play();
      }
      else if(count == 1){
        orchestra.loadMusic("Champions.chrp");
        count++;
        time = 180;
        orchestra.play();
      }
      else if(count == 2){
        orchestra.loadMusic("Giorno.chrp");
        count++;
        time = 285;
        orchestra.play();
      }
      else if(count == 3){
        orchestra.loadMusic("Duel.chrp");
        count++;
        time = 250;
        orchestra.play();
      }
      else if(count == 4){
        orchestra.loadMusic("Imperial.chrp");
        count++;
        time = 106;
        orchestra.play();
      }
      else if(count == 5){
        orchestra.loadMusic("Megalovania.chrp");
        count++;
        time = 38;
        orchestra.play();
      }
      else if(count == 6){
        orchestra.loadMusic("Mountain.chrp");
        count++;
        time = 151;
        orchestra.play();
      }
      else if(count == 7){
        orchestra.loadMusic("Pomp.chrp");
        count++;
        time = 107;
        orchestra.play();
      }
      else if(count == 8){
        orchestra.loadMusic("US.chrp");
        count++;
        time = 219;
        orchestra.play();
      }
      else if(count == 9){
        orchestra.loadMusic("76.chrp");
        count++;
        time = 200;
        orchestra.play();
      }
      if(count == 10){
        orchestra.loadMusic("Elves.chrp");
        count++;
        time = 42;
        orchestra.play();
      }
      if(count == 11){
        //By Request of RoboRandy
        orchestra.loadMusic("RockyTop.chrp");
        time = 86;
        orchestra.play();
      }
      if(count == 11){
        count = 0;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

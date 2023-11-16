// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  ArmSubsystem arm;
  int setTopSetpoints;
  int setBottomSetPoint;
 
  public SetArm(ArmSubsystem m_arm,int setTopSetpoints, int setBottomSetPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = m_arm;
    this.setTopSetpoints =setTopSetpoints;
    this.setBottomSetPoint=setBottomSetPoint;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.




  @Override
  public void execute() {
    arm.reachSetpoint();
    arm.setvoltage(setTopSetpoints, setBottomSetPoint);
    arm.reachSetpoint();
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

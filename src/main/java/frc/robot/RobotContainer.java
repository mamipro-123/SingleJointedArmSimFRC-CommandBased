
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.setPoint;
import frc.commands.SetArm;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
  ArmSubsystem arm = new ArmSubsystem();
  public final Joystick m_joystick = new Joystick(0);

  public RobotContainer() {
    configureBindings();
    arm.setDefaultCommand(new SetArm(arm,setPoint.defaultTopPosition,setPoint.defaultBottomPosition));
  }

  private void configureBindings() {
    arm.reachSetpoint();
    new JoystickButton(m_joystick, 1).onTrue(new SetArm(arm, setPoint.positionOneTop, setPoint.positionOneBottom));
    new JoystickButton(m_joystick,2).onTrue(new SetArm(arm,  setPoint.positionTwoTop,  setPoint.positionTwoBottom));
    new JoystickButton(m_joystick, 3).onTrue(new SetArm(arm,  setPoint.positionThreeTop,  setPoint.positionThreeBottom));
    new JoystickButton(m_joystick, 4).onTrue(new SetArm(arm,  setPoint.positionFourTop,  setPoint.positionFourBottom));
    arm.reachSetpoint();

  }

  public Command getAutonomousCommand() {
    return null;
  }

  
}
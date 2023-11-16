// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  private final DCMotor mArmFalcon = DCMotor.getFalcon500(2);

  public final ProfiledPIDController topController = new ProfiledPIDController(ArmConstants.kArmKp, ArmConstants.kArmKi,
      0,
      new TrapezoidProfile.Constraints(2, 5));
  public final ProfiledPIDController bottomController = new ProfiledPIDController(ArmConstants.kArmKp,
      ArmConstants.kArmKi, 0,
      new TrapezoidProfile.Constraints(2, 5));
  private final Encoder topEncoder = new Encoder(ArmConstants.kEncoderAChannel, ArmConstants.kEncoderBChannel);
  private final Encoder bottomEncoder = new Encoder(ArmConstants.kEncoderAChannel + 2,
      ArmConstants.kEncoderBChannel + 2);

  private final WPI_TalonFX topMotor = new WPI_TalonFX(ArmConstants.kMotorPort);
  private final WPI_TalonFX bottomMotor = new WPI_TalonFX(ArmConstants.kMotorPort + 1);

  private final EncoderSim m_topEncoderSim = new EncoderSim(topEncoder);
  private final EncoderSim m_bottomEncoderSim = new EncoderSim(bottomEncoder);

  private final Mechanism2d m_mech2d = new Mechanism2d(90, 90,new Color8Bit(Color.kWhite));

  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 55, 21.75);

  public ArmSubsystem() {
    topEncoder.setDistancePerPulse(ArmConstants.kArmEncoderDistPerPulse);
    bottomEncoder.setDistancePerPulse(ArmConstants.kArmEncoderDistPerPulse);
  }

  public double pidoutputTop(int topSetpoint, int bottomSetpoint) {
    double pidOutputTop = topController.calculate(topEncoder.getDistance(),
        Units.degreesToRadians(topSetpoint - bottomSetpoint));
    return pidOutputTop;
  }

  public void setvoltage(int topSetpoint, int bottomSetpoint) {
    double pidOutputTop = topController.calculate(topEncoder.getDistance(),
        Units.degreesToRadians(topSetpoint - bottomSetpoint));
    topMotor.setVoltage(pidOutputTop);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", bottomSetpoint);
    SmartDashboard.putNumber("Setpoint top (degrees)", topSetpoint);
    double pidOutputBottom = bottomController.calculate(bottomEncoder.getDistance(),
        Units.degreesToRadians(bottomSetpoint));
    bottomMotor.setVoltage(pidOutputBottom);
  }

  public double pidoutputBottom(int topSetpoint, int bottomSetpoint) {
    double pidOutputBottom = bottomController.calculate(bottomEncoder.getDistance(),
        Units.degreesToRadians(bottomSetpoint));
    return pidOutputBottom;
  }

  /**
   * Run the control loop to reach and maintain the setpoint from the preferences.
   */

  private final SingleJointedArmSim m_arm_topSim = new SingleJointedArmSim(
      mArmFalcon,
      ArmConstants.m_armGravity,
      SingleJointedArmSim.estimateMOI(ArmConstants.m_arm_topLength, ArmConstants.m_arm_topMass),
      ArmConstants.m_arm_topLength, // The length of the arm.
      Units.degreesToRadians(ArmConstants.m_arm_top_min_angle), // The minimum angle that the arm is capable of.
      Units.degreesToRadians(ArmConstants.m_arm_top_max_angle), // The maximum angle that the arm is capable of.
      false, // Whether gravity should be simulated or not.
      VecBuilder.fill(ArmConstants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );

  private final SingleJointedArmSim m_arm_bottomSim = new SingleJointedArmSim(
      mArmFalcon,
      ArmConstants.m_armGravity,
      SingleJointedArmSim.estimateMOI(ArmConstants.m_arm_bottomLength, ArmConstants.m_arm_bottomMass),
      ArmConstants.m_arm_bottomLength,
      Units.degreesToRadians(ArmConstants.m_arm_bottom_min_angle),
      Units.degreesToRadians(ArmConstants.m_arm_bottom_max_angle),
      false,
      VecBuilder.fill(ArmConstants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );

  private final MechanismLigament2d m_arm_bottom = m_armPivot.append(
      new MechanismLigament2d(
          "Arm Bottom",
          20,
          -90,
          10,
          new Color8Bit(Color.kDarkTurquoise)));
  public final MechanismLigament2d m_arm_tower = m_armPivot
      .append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kDarkGray)));

  private final MechanismLigament2d m_arm_top = m_arm_bottom.append(
      new MechanismLigament2d(
          "Arm Top",
          28.5,
          Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
          10,
          new Color8Bit(Color.kDarkSeaGreen)));

  public void reachSetpoint() {
    double pidOutputTop = topController.calculate(topEncoder.getDistance(), Units.degreesToRadians(MathUtil.clamp(
        SmartDashboard.getNumber("Setpoint top (degrees)", 0) - MathUtil.clamp(
            SmartDashboard.getNumber("Setpoint bottom (degrees)", 150), ArmConstants.m_arm_bottom_min_angle,
            ArmConstants.m_arm_bottom_max_angle),
        ArmConstants.m_arm_top_min_angle, ArmConstants.m_arm_top_max_angle)));
    topMotor.setVoltage(pidOutputTop);
    double pidOutputBottom = bottomController.calculate(bottomEncoder.getDistance(),
        Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0),
            ArmConstants.m_arm_bottom_min_angle, ArmConstants.m_arm_bottom_max_angle)));
    bottomMotor.setVoltage(pidOutputBottom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Setpoint top (degrees)", 90);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", 90);

    SmartDashboard.putData("Arm Sim", m_mech2d);
  }

  public void simulationPeriodic() {
    m_arm_topSim.setInput(topMotor.get() * RobotController.getBatteryVoltage());
    m_arm_bottomSim.setInput(bottomMotor.get() * RobotController.getBatteryVoltage());

    m_arm_topSim.update(0.020);
    m_arm_bottomSim.update(0.020);

    
    m_topEncoderSim.setDistance(m_arm_topSim.getAngleRads());
    m_bottomEncoderSim.setDistance(m_arm_bottomSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_arm_topSim.getCurrentDrawAmps() + m_arm_bottomSim.getCurrentDrawAmps()));

    m_arm_top.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));
    m_arm_bottom.setAngle(Units.radiansToDegrees(m_arm_bottomSim.getAngleRads()));
  }

}
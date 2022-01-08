// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  MotorControllerGroup m_left;
  MotorControllerGroup m_right;

  DifferentialDrive m_drive;

  AHRS m_gyro;
  PIDController m_turnController;

  SimpleMotorFeedforward m_feedForward;

  // Constants for PID Gains. These will require tuning. Use the Ziegler-Nichols rule.
  static final double kP = 0;
  static final double kI = 0;
  static final double kD = 0;

  // Constants for Feedforward Gains
  static final double kS = 0;
  static final double kV = 0;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_left = new MotorControllerGroup(
      new WPI_VictorSPX(Constants.leftFront),
      new WPI_VictorSPX(Constants.leftMiddle),
      new WPI_VictorSPX(Constants.leftBack)
    );

    m_right = new MotorControllerGroup(
      new WPI_VictorSPX(Constants.rightFront),
      new WPI_VictorSPX(Constants.rightMiddle),
      new WPI_VictorSPX(Constants.rightBack)
    );

    m_drive = new DifferentialDrive(m_left, m_right);

    m_gyro = new AHRS(SPI.Port.kMXP);

    m_turnController = new PIDController(kP, kI, kD);
    m_turnController.enableContinuousInput(-180.0f, 180.0f);

    m_feedForward = new SimpleMotorFeedforward(kS, kV);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveManually(Joystick joystick, double speed) {
    m_drive.arcadeDrive(-joystick.getRawAxis(Constants.y_axis) * speed, joystick.getRawAxis(Constants.z_axis) * speed);
  }

  public void driveForward(double speed) {
    m_drive.tankDrive(speed, speed);
  }

  public void turnToAngle(double speed, double angleSetpoint) {
    m_left.setVoltage(m_feedForward.calculate(speed) + m_turnController.calculate(m_gyro.getAngle(), angleSetpoint));
    m_right.setVoltage(m_feedForward.calculate(speed) + m_turnController.calculate(m_gyro.getAngle(), angleSetpoint));
  }

  public void stop() {
    m_drive.stopMotor();
  }
}

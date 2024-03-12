// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 1.5;

  private final CANSparkMax wheel1 = new CANSparkMax(17, MotorType.kBrushless);
  private final CANSparkMax wheel2 = new CANSparkMax(18, MotorType.kBrushless);
  private final CANSparkMax ramp = new CANSparkMax(19, MotorType.kBrushless);
  private final RelativeEncoder encoder = wheel1.getEncoder();
  private final SparkPIDController pid = wheel1.getPIDController();

  public IntakeIOSparkMax() {
    wheel1.restoreFactoryDefaults();
    wheel2.restoreFactoryDefaults();
    ramp.restoreFactoryDefaults();

    wheel1.setCANTimeout(250);
    wheel2.setCANTimeout(250);
    ramp.setCANTimeout(250);

    wheel1.setInverted(false);
    wheel2.setInverted(true);
    wheel2.follow(wheel1, false);
    ramp.setInverted(false);

    wheel1.enableVoltageCompensation(12.0);
    wheel1.setSmartCurrentLimit(30);
    ramp.enableVoltageCompensation(12.0);
    ramp.setSmartCurrentLimit(30);

    wheel1.burnFlash();
    wheel2.burnFlash();
    ramp.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = wheel1.getAppliedOutput() * wheel1.getBusVoltage();
    inputs.currentAmps =
        new double[] {
          wheel1.getOutputCurrent(), wheel2.getOutputCurrent(), ramp.getOutputCurrent()
        };
  }

  @Override
  public void setVoltage(double volts) {
    wheel1.setVoltage(volts);
    ramp.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    wheel1.stopMotor();
    ramp.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}

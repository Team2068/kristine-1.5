// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.IdManager;

/** Add your docs here. */
public class Shooter {
    private IdManager idManager;

    public CANSparkMax helperMotor = new CANSparkMax(idManager.shooterHelperId, MotorType.kBrushless);
    public TalonFX flywheelMotor = new TalonFX(idManager.shooterId, "rio");
    public TalonFX pivotMotor = new TalonFX(idManager.shooterPivotId, "rio");
    public DutyCycleEncoder encoder = new DutyCycleEncoder(idManager.shooterEncoder);

    public Shooter(){
        helperMotor.setSmartCurrentLimit(20);
        helperMotor.setIdleMode(IdleMode.kCoast);
        helperMotor.setInverted(true);
        TalonFXConfiguration configs = new TalonFXConfiguration();
        flywheelMotor.getConfigurator().apply(configs);
        flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void helperVoltage(double voltage){
        helperMotor.setVoltage(voltage);
    }

    public void flywheelSpeed(double speed){
        flywheelMotor.set(speed);
    }

    public void pivotVoltage(double voltage){
        pivotMotor.setVoltage(voltage);
    }

    public void setRPM(double RPM) {
        flywheelMotor.set(Math.max(Math.min(RPM, 6000), 0) / 6000 / 6000);
    }

    public double getAngle() {
        return encoder.getAbsolutePosition() * 360;
    }
}

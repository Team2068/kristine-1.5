package frc.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

public class KrakenSwerveModule {
    public final TalonFX driveMotor;
    public final CANSparkMax steerMotor;
    public final Canandcoder steerEncoder;
    
    double desiredAngle;

    final double PI2 = 2.0 * Math.PI;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double DRIVE_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;
    public double positional_error = 0;
    public double in_volts = 0;

    public KrakenSwerveModule(ShuffleboardLayout tab, int driveID, int steerID, int steerCANID) {
        driveMotor = new TalonFX(driveID, "rio");
        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
        steerEncoder = new Canandcoder(steerCANID);

        steerMotor.restoreFactoryDefaults();

        Canandcoder.Settings settings = new Canandcoder.Settings();
        settings.setInvertDirection(true);
        steerEncoder.clearStickyFaults();
        steerEncoder.resetFactoryDefaults(false);
        steerEncoder.setSettings(settings);

        TalonFXConfiguration config = new TalonFXConfiguration();

        steerMotor.setSmartCurrentLimit(20);
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        steerMotor.setIdleMode(IdleMode.kBrake);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        steerMotor.getEncoder().setPositionConversionFactor(Math.PI * STEER_REDUCTION);
        steerMotor.getEncoder().setVelocityConversionFactor(Math.PI * STEER_REDUCTION / 60);
        steerMotor.getEncoder().setPosition(angle());

        steerMotor.getPIDController().setPositionPIDWrappingEnabled(true);
        steerMotor.getPIDController().setPositionPIDWrappingMaxInput(PI2);
        steerMotor.getPIDController().setSmartMotionAllowedClosedLoopError(0.001, 0);

        driveMotor.setInverted(true);
        steerMotor.setInverted(false);

        driveMotor.getConfigurator().apply(config);

        steerMotor.getPIDController().setP(0.2);
        steerMotor.getPIDController().setI(0.0);
        steerMotor.getPIDController().setD(0.0);


        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

        steerMotor.getPIDController().setSmartMotionAllowedClosedLoopError(0.0, 0);

        tab.addDouble("Absolute Angle", () -> Math.toDegrees(angle()));
        tab.addDouble("Current Angle", () -> Math.toDegrees(steerMotor.getEncoder().getPosition()));
        tab.addDouble("Target Angle", () -> Math.toDegrees(desiredAngle));
        tab.addDouble("Drive Volts", () -> in_volts);
        tab.addDouble("Positional Error", () -> positional_error);
        tab.addBoolean("Active", steerEncoder::isConnected);
    }

    public void resetDrivePosition() {
        driveMotor.setPosition(0.0);
    }

    public void resetSteerPosition() {
        steerMotor.getEncoder().setPosition(angle());
    }

    public void resetAbsolute() {
        steerEncoder.setAbsPosition(0, 250);
    }

    public double drivePosition() {
        return driveMotor.getPosition().getValue() * .502 * WHEEL_DIAMETER;
    }

    public double velocity() {
        return driveMotor.getRotorVelocity().getValueAsDouble() * PI2 * .502 * Units.inchesToMeters(3);
    }

    public double angle() {
        return (steerEncoder.getAbsPosition() * PI2) % PI2;
    }

    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public void set(double driveVolts, double targetAngle) {
        resetSteerPosition();

        targetAngle %= PI2;
        targetAngle += (targetAngle < 0.0) ? PI2 : 0.0;

        desiredAngle = targetAngle;

        double diff = targetAngle - angle();

        if (diff > (Math.PI / 2.0) || diff < -(Math.PI / 2.0)) {
            targetAngle = (targetAngle + Math.PI) % PI2;
            driveVolts *= -1.0;
        }

        positional_error = steerMotor.getPIDController().getSmartMotionAllowedClosedLoopError(0);

        in_volts = driveVolts;
        driveMotor.set(driveVolts);
        steerMotor.getPIDController().setReference(targetAngle, ControlType.kPosition);
    }

}
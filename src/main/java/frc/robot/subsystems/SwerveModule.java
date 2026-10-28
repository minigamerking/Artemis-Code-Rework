package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;

    private SwerveModuleState state;

    private final int swerveID;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
            boolean turnMotorReversed, int absoluteEncoderId, boolean absoluteEncoderReversed, int swerveID) {

        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Preferences.getDouble("Module_" + swerveID + "_Angle", 0);
        absoluteEncoder.getConfigurator().apply(config);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.configure(getDriveConfig(driveMotorReversed), null, PersistMode.kPersistParameters);
        turningMotor.configure(getTurningConfig(turnMotorReversed), null, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        turningEncoder.setPosition(getAbsoluteEncoderRad());

        turningPidController = new PIDController(0.25, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        turningPidController.setTolerance(0.2);

        //this.state = new SwerveModuleState(0, Rotation2d.fromRadians(getAbsoluteEncoderRad()));

        this.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(getAbsoluteEncoderRad())));

        this.swerveID = swerveID;
    }

    private SparkMaxConfig getDriveConfig(boolean reversed) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder
            .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec)
            .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        
        config.signals
            .analogVelocityAlwaysOn(true)
            .analogVoltageAlwaysOn(true)
            .analogPositionAlwaysOn(true);

        config.inverted(reversed);

        return config;
    }

    private SparkMaxConfig getTurningConfig(boolean reversed) {
        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder
            .positionConversionFactor((2 * Math.PI) * ModuleConstants.kTurningMotorGearRatio);

        config.signals
            .analogVelocityAlwaysOn(true)
            .analogVoltageAlwaysOn(true)
            .analogPositionAlwaysOn(true);

        config.inverted(reversed);

        return config;
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(rotations);

        if (absoluteEncoderReversed) {
            angle = angle.unaryMinus();
        }

        return getTurningPosition();

        //return angle.getRadians();
    }

    public void resetEncoders() {
        // System.out.println("1 " + this.swerveID + " " + getAbsoluteEncoderRad() + "\n");
        driveEncoder.setPosition(0);
        CANcoderConfiguration setZero = new CANcoderConfiguration();
        setZero.MagnetSensor.MagnetOffset = 0;
        absoluteEncoder.getConfigurator().apply(setZero);
        double offset = (Rotation2d.kZero.minus(Rotation2d.fromRadians(getAbsoluteEncoderRad()))).getRotations();
        Preferences.setDouble("Module_" + swerveID + "_Angle", offset);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        absoluteEncoder.getConfigurator().apply(config);
        turningEncoder.setPosition(0);
        // System.out.println("2" + this.swerveID + " " + getAbsoluteEncoderRad() + "\n");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }

    public void runVolts(double volts) {
        this.driveMotor.setVoltage(volts);
    }

    public void periodic() {
        //System.out.println(this.swerveID + " Desired Angle: " + this.state.angle.getRadians());
        driveMotor.set(this.state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad()));
        SmartDashboard.putNumber(this.swerveID + " Module Angle", Rotation2d.fromRadians(getAbsoluteEncoderRad()).getDegrees());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            desiredState.speedMetersPerSecond = 0;
        }
        
        this.state = desiredState;
        this.state.optimize(Rotation2d.fromRadians(getAbsoluteEncoderRad()));
        turningPidController.setSetpoint(this.state.angle.getRadians());
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition position = new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
        return position;
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public int getSwerveID() {
        return this.swerveID;
    }

    public PIDController getController() {
        return this.turningPidController;
    }

    public void setControllerP(double p) {
        Preferences.setDouble("Module_" + swerveID + "_P", p);
        this.turningPidController.setP(p);
    }

    public void setControllerD(double d) {
        Preferences.setDouble("Module_" + swerveID + "_D", d);
        this.turningPidController.setD(d);
    }
}
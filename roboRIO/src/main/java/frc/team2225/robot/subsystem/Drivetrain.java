package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2225.robot.ScaleInputs;
import frc.team2225.robot.Vector2D;
import frc.team2225.robot.command.Teleop;

import static frc.team2225.robot.subsystem.Drivetrain.Position.*;

public class Drivetrain extends Subsystem {
    public enum Position {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT;
    }
    public static final Vector2D frontLeftVec = new Vector2D(Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public static final Vector2D frontRightVec = new Vector2D(-Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public static final Vector2D backLeftVec = new Vector2D(-Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public static final Vector2D backRightVec = new Vector2D(Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public TalonSRX[] motors;
    public ADXRS450_Gyro gyro;

    public Drivetrain(int frontLeft, int frontRight, int backLeft, int backRight, SPI.Port gyro) {
        motors = new TalonSRX[4];
        motors[FRONT_LEFT.ordinal()] = new TalonSRX(frontLeft);
        motors[FRONT_RIGHT.ordinal()] = new TalonSRX(frontRight);
        motors[BACK_LEFT.ordinal()] = new TalonSRX(backLeft);
        motors[BACK_RIGHT.ordinal()] = new TalonSRX(backRight);
        this.gyro = new ADXRS450_Gyro(gyro);

        for(TalonSRX motor : motors) {
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
            motor.configNominalOutputForward(0.0, 0);
            motor.configNominalOutputReverse(0.0, 0);
            motor.configPeakOutputForward(1, 0);
            motor.configPeakOutputReverse(-1, 0);
            motor.configClosedloopRamp(0.4, 0);
            motor.config_kP(0, 0.5, 0);
            motor.config_kI(0, 0, 0);
            motor.config_kD(0, 2, 0);
            motor.config_IntegralZone(0, 0, 0);
        }
    }

    public TalonSRX motorOf(Position position) {
        return motors[position.ordinal()];
    }

    public void omniDrive(Vector2D translate, double rotateIn) {
        DriverStation.reportWarning("Omni Drive", false);
        final double p = 1.0/100.0;
        final double d = 1.0/400.0;
        translate.mapSquareToDiamond().divide(Math.sqrt(2) / 2);
        double fr, fl, br, bl;
        fl = translate.dot(frontLeftVec);
        fr = translate.dot(frontRightVec);
        bl = translate.dot(backLeftVec);
        br = translate.dot(backRightVec);

        fl = ScaleInputs.padMinValue(rotateIn, fl, false) + rotateIn;
        fr = ScaleInputs.padMinValue(rotateIn, fr, false) + rotateIn;
        bl = ScaleInputs.padMinValue(rotateIn, bl, false) - rotateIn;
        br = ScaleInputs.padMinValue(rotateIn, br, false) - rotateIn;

        setMotorVoltage(fl, fr, bl, br);
    }
    
    public void translate(Vector2D v){
        motorOf(FRONT_LEFT).set(ControlMode.Position, v.y + v.x);
        motorOf(FRONT_RIGHT).set(ControlMode.Position, v.y - v.x);
        motorOf(BACK_LEFT).set(ControlMode.Position, v.y - v.x);
        motorOf(BACK_RIGHT).set(ControlMode.Position, v.y + v.x);
    }

    public void setMotorVoltage(double fl, double fr, double bl, double br) {
        motorOf(FRONT_LEFT).set(ControlMode.PercentOutput, fl);
        motorOf(FRONT_RIGHT).set(ControlMode.PercentOutput, fr);
        motorOf(BACK_LEFT).set(ControlMode.PercentOutput, bl);
        motorOf(BACK_LEFT).set(ControlMode.PercentOutput, br);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Teleop());
    }
}

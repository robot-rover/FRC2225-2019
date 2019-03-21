package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.team2225.robot.Robot;
import frc.team2225.robot.ScaleInputs;
import frc.team2225.robot.Vector2D;
import frc.team2225.robot.command.Teleop;

import java.util.function.BiConsumer;

import static frc.team2225.robot.subsystem.Drivetrain.Position.*;

public class Drivetrain extends Subsystem {

    public enum Position {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
    }
    public static final Vector2D frontLeftVec = new Vector2D(Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public static final Vector2D frontRightVec = new Vector2D(-Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public static final Vector2D backLeftVec = new Vector2D(-Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public static final Vector2D backRightVec = new Vector2D(Math.sqrt(2) / 2, Math.sqrt(2) / 2);

    public static final double _wheelCircumferenceCm = 6 * 2.54 * Math.PI;
    public static final double _motorRotsPerWheelRot = 16;
    public static final int _countsPerMotorRot = 40;

    public static final int deadZone = 100;

    int resetTargetRot;
    double targetRot;

    PIDCallback pidWrite = new PIDCallback();

    // Units: counts / 100ms
    public static final int maxVelocity = 100;

    public static final double fGain = 1023.0 / maxVelocity;

    // Cruise Velocity = Max Velocity * 30%
    public static final int cruiseVelocity = 30;

    //Accelerate to cruise in 1 second
    public static final int acceleration = cruiseVelocity;

    private ShuffleboardLayout drivePid = Robot.debugTab.getLayout("Drivetrain PID", BuiltInLayouts.kList.getLayoutName());
    private ShuffleboardLayout gyroPid = Robot.debugTab.getLayout("Gyro PID", BuiltInLayouts.kList.getLayoutName());
    private ShuffleboardLayout drivetrainOutputs = Robot.debugTab.getLayout("Drivetrain Outputs", BuiltInLayouts.kGrid.getLayoutName());

    private NetworkTableEntry[] motorOutputs = new NetworkTableEntry[4];

    private NetworkTableEntry pChooser = drivePid.add("kP", 0).getEntry();
    private NetworkTableEntry iChooser = drivePid.add("kI", 0).getEntry();
    private NetworkTableEntry dChooser = drivePid.add("kD", 0).getEntry();
    private NetworkTableEntry fChooser = drivePid.add("kF", fGain).getEntry();
    private NetworkTableEntry izChooser = drivePid.add("iZone", 0).getEntry();

    private NetworkTableEntry gpChooser = gyroPid.add("kP", 0).getEntry();
    private NetworkTableEntry giChooser = gyroPid.add("kI", 0).getEntry();
    private NetworkTableEntry gdChooser = gyroPid.add("kD", 0).getEntry();
    private NetworkTableEntry gyroPos = gyroPid.add("Position", 0).getEntry();

    @Override
    public void periodic() {

    }

    private void setMotorParam(NetworkTableEntry slot, BiConsumer<TalonSRX, Double> method) {
        slot.addListener(change -> {
            for (TalonSRX motor : motors) {
                method.accept(motor, change.value.getDouble());
            }
        }, Robot.updateFlags);
    }

    public TalonSRX[] motors;
    public ADXRS450_Gyro gyro;
    final PIDController turnController;
    public Drivetrain(int frontLeft, int frontRight, int backLeft, int backRight, SPI.Port gyro) {
        motors = new TalonSRX[4];
        motors[FRONT_LEFT.ordinal()] = new TalonSRX(frontLeft);
        motors[FRONT_RIGHT.ordinal()] = new TalonSRX(frontRight);
        motors[BACK_LEFT.ordinal()] = new TalonSRX(backLeft);
        motors[BACK_RIGHT.ordinal()] = new TalonSRX(backRight);
        for (Position position : Position.values()) {
            motorOutputs[position.ordinal()] = drivetrainOutputs.add(position.name() + " Output", 0).getEntry();
        }
        this.gyro = new ADXRS450_Gyro(gyro);
        Robot.debugTab.add("Gyro", this.gyro);
        DriverStation.reportWarning("Gyro: " + this.gyro.isConnected() + ", " + this.gyro.getAngle(), false);
        setMotorParam(pChooser, (m, p) -> m.config_kP(0, p));
        setMotorParam(iChooser, (m, i) -> m.config_kI(0, i));
        setMotorParam(dChooser, (m, d) -> m.config_kD(0, d));
        setMotorParam(fChooser, (m, f) -> m.config_kF(0, f));
        setMotorParam(izChooser, (m, iz) -> m.config_IntegralZone(0, iz.intValue()));
        turnController = new PIDController(0, 0, 0, this.gyro, pidWrite);
        turnController.setOutputRange(-1, 1);
        gpChooser.addListener(v -> {
            turnController.setP(v.value.getDouble());
            DriverStation.reportWarning("P Updated " + v.value.getDouble(), false);
        }, Robot.updateFlags);
        giChooser.addListener(v -> turnController.setI(v.value.getDouble()), Robot.updateFlags);
        gdChooser.addListener(v -> turnController.setD(v.value.getDouble()), Robot.updateFlags);

        for (TalonSRX motor : motors) {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
            motor.configNominalOutputForward(0.0);
            motor.configNominalOutputReverse(0.0);
            motor.configPeakOutputForward(1);
            motor.configPeakOutputReverse(-1);

            motor.selectProfileSlot(0, 0);
            motor.config_IntegralZone(0, 0);
            motor.config_kF(0, fGain);

            motor.configMotionCruiseVelocity(cruiseVelocity);
            motor.configMotionAcceleration(acceleration);
        }

        targetRot = 0;
        motorOf(FRONT_RIGHT).setInverted(false);
        motorOf(BACK_RIGHT).setInverted(false);
        motorOf(FRONT_LEFT).setInverted(false);
        motorOf(BACK_LEFT).setInverted(true);
    }

    public TalonSRX motorOf(Position position) {
        return motors[position.ordinal()];
    }

    /**
     * Drive the robot by setting the output voltage
     *
     * @param translate A vector which describes the desired movement direction. Units are percent output [0, 1]
     * @param rotateIn  The desired rotation amount (positive is clockwise)
     */
    public void omniDrive(Vector2D translate, double rotateIn) {
        // TODO: test and use rotate instead of rotateIn
        translate.mapSquareToDiamond().divide(Math.sqrt(2) / 2);
        double fr, fl, br, bl;
        double rotate = 0;
        if(rotateIn != 0) {
            resetTargetRot = 10;
            rotate = rotateIn;
        }
        if(resetTargetRot > 0) {
            targetRot = gyro.getAngle();
            resetTargetRot--;
        }
        if(rotateIn == 0 && resetTargetRot <= 0) {
            rotate = Math.max(-1, Math.min(pidWrite.output ,1));
        }
        fl = translate.dot(frontLeftVec);
        fr = translate.dot(frontRightVec);
        bl = translate.dot(backLeftVec);
        br = translate.dot(backRightVec);


        fl = ScaleInputs.padMinValue(rotateIn, fl, false) + rotate;
        fr = ScaleInputs.padMinValue(rotateIn, fr, false) - rotate;
        bl = ScaleInputs.padMinValue(rotateIn, bl, false) + rotate;
        br = ScaleInputs.padMinValue(rotateIn, br, false) - rotate;

        setMotorVoltage(fl, fr, bl, br);
    }

    private int cmToCounts(double cm) {
        return (int)(cm / _wheelCircumferenceCm * _motorRotsPerWheelRot * _countsPerMotorRot);
    }

    /**
     * Used to drive the robot autonomously a certain distance
     * @param v The desired distance of translation (units are centimeters)
     * @return A CheckPosition object that can be used to see if the movement has completed
     */
    public CheckPosition translate(Vector2D v){
        int fl = motorOf(FRONT_LEFT).getSelectedSensorPosition() + cmToCounts(v.dot(frontLeftVec));
        int fr = motorOf(FRONT_RIGHT).getSelectedSensorPosition() + cmToCounts(v.dot(frontRightVec));
        int bl = motorOf(BACK_LEFT).getSelectedSensorPosition() + cmToCounts(v.dot(backLeftVec));
        int br = motorOf(BACK_RIGHT).getSelectedSensorPosition() + cmToCounts(v.dot(backRightVec));
        motorOf(FRONT_LEFT).set(ControlMode.Position, fl);
        motorOf(FRONT_RIGHT).set(ControlMode.Position, fr);
        motorOf(BACK_LEFT).set(ControlMode.Position, bl);
        motorOf(BACK_RIGHT).set(ControlMode.Position, br);
        return new CheckPosition(fl, fr, bl, br, deadZone);
    }

    public void setMotorVoltage(double fl, double fr, double bl, double br) {
        motorOf(FRONT_LEFT).set(ControlMode.PercentOutput, fl);
        motorOutputs[FRONT_LEFT.ordinal()].setDouble(fl);
        motorOf(FRONT_RIGHT).set(ControlMode.PercentOutput, fr);
        motorOutputs[FRONT_RIGHT.ordinal()].setDouble(fr);
        motorOf(BACK_LEFT).set(ControlMode.PercentOutput, bl);
        motorOutputs[BACK_LEFT.ordinal()].setDouble(bl);
        motorOf(BACK_RIGHT).set(ControlMode.PercentOutput, br);
        motorOutputs[BACK_RIGHT.ordinal()].setDouble(br);
    }

    /**
     * Stop the robot
     */
    public void stop() {
        omniDrive(Vector2D.zero(), 0);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Teleop());
    }

    public class CheckPosition{

        private final int deadZone;
        int fl, fr, bl, br;

        CheckPosition(int fl, int fr, int bl, int br, int deadZone){
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
            this.deadZone = deadZone;
        }

        public boolean isDone(){
            if(Math.abs(fl - motorOf(FRONT_LEFT).getSelectedSensorPosition()) <= deadZone){
                if(Math.abs(fr - motorOf(FRONT_RIGHT).getSelectedSensorPosition()) <= deadZone){
                    if(Math.abs(bl - motorOf(BACK_LEFT).getSelectedSensorPosition()) <= deadZone){
                        return Math.abs(br - motorOf(BACK_RIGHT).getSelectedSensorPosition()) <= deadZone;
                    }
                }
            }
            return false;
        }

        @Override
        public String toString() {
            return "fl: " + (fl - motorOf(FRONT_LEFT).getSelectedSensorPosition()) +
                    ", fr: " + (fr - motorOf(FRONT_RIGHT).getSelectedSensorPosition()) +
                    ", bl: " + (bl - motorOf(BACK_LEFT).getSelectedSensorPosition()) +
                    ", br: " + (br - motorOf(BACK_RIGHT).getSelectedSensorPosition());
        }
    }

    public class PIDCallback implements PIDOutput {
        public double output;
        @Override
        public void pidWrite(double output) {
            this.output = output;
            DriverStation.reportWarning("PID Updated: " + output, false);
        }
    }

}

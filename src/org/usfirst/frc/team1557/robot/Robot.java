
package org.usfirst.frc.team1557.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Encoder leftEncoder = new Encoder(0, 1);
	Encoder rightEncoder = new Encoder(2, 3);
	PIDController leftEncoderPID;
	PIDController rightEncoderPID;
	PIDController rotationPID;
	double leftEncoderOutput;
	double rightEncoderOutput;
	double rotationOutput;
	CANTalon motorOne = new CANTalon(0);
	CANTalon motorTwo = new CANTalon(1);
	CANTalon motorThree = new CANTalon(2);
	CANTalon motorFour = new CANTalon(3);
	double[] encoderPosisiton = { 0, 0 };
	double encoderRotation = 0;
	Joystick leftJoy = new Joystick(0);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		leftEncoder.setDistancePerPulse(33d / 250d);
		rightEncoder.setDistancePerPulse(33d / 250d);
		leftEncoderPID = new PIDController(0.05, 0_0, 0, leftEncoder, new PIDOutput() {
			@Override
			public void pidWrite(double output) {
				leftEncoderOutput = output;
			}
		});
		leftEncoderPID.enable();
		leftEncoderPID.setAbsoluteTolerance(1);
		SmartDashboard.putData("LEFT PID", leftEncoderPID);
		rightEncoderPID = new PIDController(0.05, 0_0, 0, rightEncoder, new PIDOutput() {
			@Override
			public void pidWrite(double output) {
				rightEncoderOutput = output;
			}
		});
		rotationPID = new PIDController(0.05, 0_0, 0_0, new PIDSource() {

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				// TODO Auto-generated method stub

			}

			@Override
			public double pidGet() {
				// TODO Auto-generated method stub
				return encoderRotation;
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				// TODO Auto-generated method stub
				return PIDSourceType.kDisplacement;
			}
		}, new PIDOutput() {

			@Override
			public void pidWrite(double output) {
				rotationOutput = output;
			}
		});
		rightEncoderPID.enable();
		rightEncoderPID.setAbsoluteTolerance(1);
		SmartDashboard.putNumber("PPR_LEFT", 28);
		// Right ppr ~= 230
		SmartDashboard.putNumber("PPR_RIGHT", 230);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		// TODO Auto-generated method stub
		super.teleopInit();

		leftEncoder.reset();
		rightEncoder.reset();
		leftEncoderPID.setSetpoint(33);
		rightEncoderPID.setSetpoint(33);
		leftEncoderPID.enable();
		rightEncoderPID.enable();
	}

	double lastLeftEncCount = 0, lastRightEncCount = 0;
	boolean isTurning = false;

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		double PPR_LEFT = SmartDashboard.getNumber("PPR_LEFT", 28);
		double PPR_RIGHT = SmartDashboard.getNumber("PPR_RIGHT", 35);
		leftEncoder.setDistancePerPulse(33d / PPR_LEFT);
		rightEncoder.setDistancePerPulse(33d / PPR_RIGHT);

		// DISTANCE
		double rightDistancePerCount = Math.PI * 10 / PPR_LEFT;
		double leftDistancePerCount = Math.PI * 10 / PPR_RIGHT;
		double lec = leftEncoder.get() - lastLeftEncCount, rec = rightEncoder.get() - lastRightEncCount;
		double deltaDistance = ((rec * rightDistancePerCount) + (lec * leftDistancePerCount)) / 2;
		// DISTANCE
		// ROTATION
		double deltaHeading = (rec / rightDistancePerCount - lec / leftDistancePerCount) * (Math.PI * (10 / 20));
		encoderRotation += deltaHeading;
		// ROTATION
		// CONVERTING
		double deltaX = deltaDistance * Math.cos(encoderRotation);
		double deltaY = deltaDistance * Math.sin(encoderRotation);
		encoderPosisiton[0] += deltaX;
		encoderPosisiton[1] += deltaY;
		rightEncoder.get();
		// CONVERTING

		if (leftJoy.getRawButton(3)) {
			isTurning = false;
			leftEncoderPID.setSetpoint(leftEncoder.getDistance() + 33);
			rightEncoderPID.setSetpoint(rightEncoder.getDistance() + 33);
		} else if (leftJoy.getRawButton(4)) {
			isTurning = true;
			// Rotation is still in radians. Blegh.
			rotationPID.setSetpoint(encoderRotation + 90);
		}

		if (isTurning) {
			motorOne.set(-1 * -rotationOutput * 0.55);
			motorTwo.set(-1 * rotationOutput * 0.55);
			motorThree.set(-1 * rotationOutput * 0.55);
			motorFour.set(-1 * -rotationOutput * 0.55);
		} else {
			motorOne.set(-1 * -rightEncoderOutput * 0.55);
			motorTwo.set(leftEncoderOutput * 0.55);
			motorThree.set(-1 * rightEncoderOutput * 0.55);
			motorFour.set(-leftEncoderOutput * 0.55);
		}
		lastLeftEncCount = leftEncoder.get();
		lastRightEncCount = rightEncoder.get();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {

	}

}

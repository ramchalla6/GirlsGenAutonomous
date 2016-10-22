package util;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * PID Controller Object
 * <p>
 * Starts a runnable class PIDCompute that executes the PID algorithm
 * <p>
 * Method Signature: public PIDMain(Object pidsource,int setpoint, int
 * sampleTime, double kp, double ki, double kd)
 * 
 * @author NeilHazra
 */

// The heart of PID
public class PIDMain {
	private boolean enabled = true; // allows the pid algorithm to stop
									// computing
	private Timer pidtimer; // PID runs concurrently, timer class executes the
							// algorithm uniformly
	private int sampleTime; // how quickly to sample and calculate
	private double outputMax = 1;
	private double outputMin = -1;
	private PIDSource pidsource;
	private double output; // value to send to the motor
	/** The process error */
	private double error;
	/** The input of the PIDController */
	private double input; // what the value actually is
	private double setpoint; // desired target value
	private double prevInput;
	private double proportional; // P term
	private double integral; // I term
	private double derivative;// D term

	private double kp, ki, kd; // tuning parameters, the hardest part of PID

	/**
	 * @param pidsource
	 *            Object implementing PIDSource, contains method returning input
	 * @param setpoint
	 *            target value for PID controller
	 * @param sampleTime
	 *            time between successive calculations
	 * @param kp
	 *            proportional gain
	 * @param ki
	 *            integral gain
	 * @param kd
	 *            derivative gain
	 */
	public PIDMain(Object pidsource, int setpoint, int sampleTime, double kp, double ki, double kd) {
		this.pidsource = (PIDSource) pidsource;
		this.setSampleTime(sampleTime);
		this.setpoint = setpoint;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		pidtimer = new Timer();// create the timer
		pidtimer.scheduleAtFixedRate(new PIDCompute(), 0, sampleTime);// set the
																		// timer
	}

	public void isEnabled(boolean enabled) {
		this.enabled = enabled;
	}

	public double getOutput() {
		return output;
	}

	public static double map(double oldValue, double oldMin, double oldMax, double newMin, double newMax) {
		return (newMax - newMin) * (oldValue - oldMin) / (oldMax - oldMin) + newMin;
	}

	// For debug and tuning
	public double getInput() {
		return input;
	}

	public double getError() {
		return error;
	}

	/**
	 * @return the desired target value
	 */
	public double getSetpoint() {
		return setpoint;
	}

	/**
	 * Sets the desired target value
	 * 
	 * @param setpoint
	 *            desired target value
	 */
	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
		resetPID();
	}

	public void setOutputLimits(double min, double max) {
		outputMax = max;
		outputMin = min;
	}

	/*
	 * TODO: No code to make use of sample time changes yet
	 */
	/**
	 * @return how quickly to sample and calculate
	 */
	public int getSampleTime() {
		return sampleTime;
	}

	/**
	 * @param sampleTime
	 *            how quickly to sample and calculate
	 */
	public void setSampleTime(int sampleTime) {
		this.sampleTime = sampleTime;
	}

	/**
	 * PID Algorithm calculates in this TimerTask created by PIDMain
	 * 
	 * @author NeilHazra
	 *
	 */
	public void resetPID() {
		proportional = 0;
		integral = 0;
		derivative = 0;
	}

	int i = 0;

	private class PIDCompute extends TimerTask {
		public void run() {
			if (!enabled) {
				output = 0;
				SmartDashboard.putNumber("@PIDCOMPUTE EnabledCheck", (int) i);
				i++;
				return;
			}

			input = pidsource.getInput();
			error = input - setpoint;
			proportional = kp * error;

			integral += ki * error;
			// constrains integral in between outputMin and outputMax
			if (integral > outputMax) {
				integral = outputMax;
			}
			if (integral < outputMin) {
				integral = outputMin;
			}

			derivative = kd * (input - prevInput);

			output = proportional + integral + derivative;
			// constrains output in between outputMin and outputMax
			if (output > outputMax) {
				output = outputMax;
			}
			if (output < outputMin) {
				output = outputMin;
			}
			prevInput = input;

		}
	}
}

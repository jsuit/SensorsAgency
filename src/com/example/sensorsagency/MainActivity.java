package com.example.sensorsagency;

import java.util.Timer;
import java.util.TimerTask;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.view.Menu;
import android.widget.EditText;
import android.widget.TextView;

public class MainActivity extends Activity {
	// see: http://www.thousand-thoughts.com/2012/03/android-sensor-fusion-tutorial/2/
	private static final float NS2S = 1.0f / 1000000000.0f;
	private EditText acc;
	private EditText gyro;
	private EditText compass;
	private SensorManager mSensorManager;
	private Sensor a;
	private Sensor g;
	private Sensor c;
	private SensorEventListener acc_listener;
	private SensorEventListener gyro_listener;
	private SensorEventListener c_listener;
	private float[] gravity;
	private float[] linear_acceleration;
	private long timestamp;
	private float[] compass_value;

	private TextView timestamp1;
	private TextView timestamp2;
	private TextView timestamp3;
	private float[] rotationMatrix = new float[9];
	private float[] accMagOrientation;
	private float [] gyroArray = new float[3];
	private float[] accel = new float[3];
	private boolean initState = true;
	private float[] gyroMatrix = new float[9];
	public static final float EPSILON = 0.000000001f;
	private float[] gyroOrientation = new float[3];
	// final orientation angles from sensor fusion
    private float[] fusedOrientation = new float[3];
    public static final int TIME_CONSTANT = 30;
	public static final float FILTER_COEFFICIENT = 0.98f;
	private Timer fuseTimer;
	public Handler mHandler;
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		acc = (EditText) findViewById(R.id.display_accel);
		gyro = (EditText) findViewById(R.id.displayGyro);
		compass = (EditText) findViewById(R.id.displayCompass);
		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		a = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		g = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		c = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
		timestamp1 = (TextView) findViewById(R.id.TimeStamp);
		timestamp2 = (TextView) findViewById(R.id.TimeStamp2);
		timestamp3 = (TextView) findViewById(R.id.timeStamp3);
		gravity = new float[3];
		linear_acceleration = new float[3];
		compass_value = new float[3];
		accMagOrientation = new float[3];
		
		// initialise gyroMatrix with identity matrix
		gyroMatrix[0] = 1.0f;
		gyroMatrix[1] = 0.0f;
		gyroMatrix[2] = 0.0f;
		gyroMatrix[3] = 0.0f;
		gyroMatrix[4] = 1.0f;
		gyroMatrix[5] = 0.0f;
		gyroMatrix[6] = 0.0f;
		gyroMatrix[7] = 0.0f;
		gyroMatrix[8] = 1.0f;
		gyroOrientation[0] = 0.0f;
        gyroOrientation[1] = 0.0f;
        gyroOrientation[2] = 0.0f;
		setupListeners();
		setupSensors();
		fuseTimer = new Timer();
		fuseTimer.scheduleAtFixedRate(new calculateFusedOrientationTask(),
                1000, TIME_CONSTANT);
		 mHandler = new Handler();

	}

	private void setupListeners() {
		// TODO Auto-generated method stub
		acc_listener = new SensorEventListener() {
			long timestamp;
			long timestampPrev;

			@Override
			public void onSensorChanged(SensorEvent event) {
				// TODO Auto-generated method stub
				final float alpha = 0.7f;
				
				
					gravity[0] = (alpha * gravity[0] + (1 - alpha)
							* event.values[0]);
					gravity[1] = (float) (alpha * gravity[1] + (1 - alpha)
							* event.values[1]);
					gravity[2] = (float) (alpha * gravity[2] + (1 - alpha)
							* event.values[2]);

					linear_acceleration[0] = event.values[0] - gravity[0];
					linear_acceleration[1] = event.values[1] - gravity[1];
					linear_acceleration[2] = event.values[2] - gravity[2];
					accel[0] = linear_acceleration[0];
					accel[1] = linear_acceleration[1];
					accel[2] = linear_acceleration[2];
					String ac = new String(linear_acceleration[0] + ","
							+ linear_acceleration[1] + ","
							+ linear_acceleration[2]);
		
					calculateAccMagOrientation();
					//Log.d("event values[0]", "" + event.values[0]);
					//Log.d("acceleration", ac);


			}

			private void calculateAccMagOrientation() {
				if (SensorManager.getRotationMatrix(rotationMatrix, null,
						accel, compass_value)) {
					SensorManager.getOrientation(rotationMatrix,
							accMagOrientation);
				}
			}

			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {
				// TODO Auto-generated method stub

			}
		};

		gyro_listener = new SensorEventListener() {
			@Override
			public void onSensorChanged(SensorEvent event) {

				// Axis of the rotation sample.
				float axisX = event.values[0];
				float axisY = event.values[1];
				float axisZ = event.values[2];
				gyrofunction(event);
				//gyro.setText(new String(axisX + ", " + axisY + ", " + axisZ));
			}

			private void gyrofunction(SensorEvent event) {
				if (accMagOrientation == null)
					return;

				if (initState) {
					float[] initMatrix = new float[9];
					initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
					float[] test = new float[3];
					SensorManager.getOrientation(initMatrix, test);
					gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
					initState = false;
				}
				// copy the new gyro values into the gyro array
				// convert the raw gyro data into a rotation vector
				float[] deltaVector = new float[4];
				if (timestamp != 0) {
					final float dT = (event.timestamp - timestamp) * NS2S;
					System.arraycopy(event.values, 0, gyroArray, 0, 3);
					getRotationVectorFromGyro(gyroArray, deltaVector, dT / 2.0f);
				}

				 // measurement done, save current time for next interval
		        timestamp = event.timestamp;
		        
		        // convert rotation vector into rotation matrix
		        float[] deltaMatrix = new float[9];
		        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

		     // apply the new rotation interval on the gyroscope based rotation matrix
		        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);
		        
		     // get the gyroscope based orientation from the rotation matrix
		        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
			}

			private void getRotationVectorFromGyro(float[] gyroValues,
					float[] deltaRotationVector, float timeFactor) {
				float[] normValues = new float[3];

				// Calculate the angular speed of the sample
				float omegaMagnitude = (float) Math.sqrt(gyroValues[0]
						* gyroValues[0] + gyroValues[1] * gyroValues[1]
						+ gyroValues[2] * gyroValues[2]);

				// Normalize the rotation vector if it's big enough to get the
				// axis
				if (omegaMagnitude > EPSILON) {
					normValues[0] = gyroValues[0] / omegaMagnitude;
					normValues[1] = gyroValues[1] / omegaMagnitude;
					normValues[2] = gyroValues[2] / omegaMagnitude;
				}

				// Integrate around this axis with the angular speed by the
				// timestep
				// in order to get a delta rotation from this sample over the
				// timestep
				// We will convert this axis-angle representation of the delta
				// rotation
				// into a quaternion before turning it into the rotation matrix.
				float thetaOverTwo = omegaMagnitude * timeFactor;
				float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
				float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
				deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
				deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
				deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
				deltaRotationVector[3] = cosThetaOverTwo;

			}

			

		

			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {
				// TODO Auto-generated method stub

			}
		};

		c_listener = new SensorEventListener() {

			@Override
			public void onSensorChanged(SensorEvent event) {
				// TODO Auto-generated method stub
				// angle between the magnetic north directio
				// 0=North, 90=East, 180=South, 270=West
	
				compass_value[0] = event.values[0];
				compass_value[1] = event.values[1];
				compass_value[2] = event.values[2];
				timestamp1.setText("" + event.timestamp);

				compass.setText(" " + compass_value[0] + ", "
						+ compass_value[1] + " , " + compass_value[2]);
			}

			@Override
			public void onAccuracyChanged(Sensor sensor, int accuracy) {
				// TODO Auto-generated method stub

			}
		};
	}

	private void setupSensors() {
		// TODO Auto-generated method stub
		if (a != null) {
			mSensorManager.registerListener(acc_listener, a,
					SensorManager.SENSOR_DELAY_FASTEST);
		}
		if (g != null) {
			mSensorManager.registerListener(gyro_listener, g,
					SensorManager.SENSOR_DELAY_FASTEST);
		}
		if (c != null) {
			mSensorManager.registerListener(c_listener, c,
					SensorManager.SENSOR_DELAY_FASTEST);
		}
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

	public float[] getAcceleration() {
		return linear_acceleration;
	}

	public float[] gyroReadings() {
		return gyroReadings();
	}

	public float[] getCompassReadings() {
		return compass_value;
	}
	
	
	class calculateFusedOrientationTask extends TimerTask {
        public void run() {
            float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;
            
            /*
             * Fix for 179° <--> -179° transition problem:
             * Check whether one of the two orientation angles (gyro or accMag) is negative while the other one is positive.
             * If so, add 360° (2 * math.PI) to the negative value, perform the sensor fusion, and remove the 360° from the result
             * if it is greater than 180°. This stabilizes the output in positive-to-negative-transition cases.
             */
            
            // azimuth
            if (gyroOrientation[0] < -0.5 * Math.PI && accMagOrientation[0] > 0.0) {
            	fusedOrientation[0] = (float) (FILTER_COEFFICIENT * (gyroOrientation[0] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[0]);
        		fusedOrientation[0] -= (fusedOrientation[0] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[0] < -0.5 * Math.PI && gyroOrientation[0] > 0.0) {
            	fusedOrientation[0] = (float) (FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * (accMagOrientation[0] + 2.0 * Math.PI));
            	fusedOrientation[0] -= (fusedOrientation[0] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
            	fusedOrientation[0] = FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * accMagOrientation[0];
            }
            
            // pitch
            if (gyroOrientation[1] < -0.5 * Math.PI && accMagOrientation[1] > 0.0) {
            	fusedOrientation[1] = (float) (FILTER_COEFFICIENT * (gyroOrientation[1] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[1]);
        		fusedOrientation[1] -= (fusedOrientation[1] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[1] < -0.5 * Math.PI && gyroOrientation[1] > 0.0) {
            	fusedOrientation[1] = (float) (FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * (accMagOrientation[1] + 2.0 * Math.PI));
            	fusedOrientation[1] -= (fusedOrientation[1] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
            	fusedOrientation[1] = FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * accMagOrientation[1];
            }
            
            // roll
            if (gyroOrientation[2] < -0.5 * Math.PI && accMagOrientation[2] > 0.0) {
            	fusedOrientation[2] = (float) (FILTER_COEFFICIENT * (gyroOrientation[2] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[2]);
        		fusedOrientation[2] -= (fusedOrientation[2] > Math.PI) ? 2.0 * Math.PI : 0;
            }
            else if (accMagOrientation[2] < -0.5 * Math.PI && gyroOrientation[2] > 0.0) {
            	fusedOrientation[2] = (float) (FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * (accMagOrientation[2] + 2.0 * Math.PI));
            	fusedOrientation[2] -= (fusedOrientation[2] > Math.PI)? 2.0 * Math.PI : 0;
            }
            else {
            	fusedOrientation[2] = FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * accMagOrientation[2];
            }
     
            // overwrite gyro matrix and orientation with fused orientation
            // to comensate gyro drift
            gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
            System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);
            
			mHandler.post(updateOreintationDisplayTask);
        
        }
    }


	public float[] getRotationMatrixFromOrientation(float[] o) {
		float[] xM = new float[9];
		float[] yM = new float[9];
		float[] zM = new float[9];

		float sinX = (float) Math.sin(o[1]);
		float cosX = (float) Math.cos(o[1]);
		float sinY = (float) Math.sin(o[2]);
		float cosY = (float) Math.cos(o[2]);
		float sinZ = (float) Math.sin(o[0]);
		float cosZ = (float) Math.cos(o[0]);

		// rotation about x-axis (pitch)
		xM[0] = 1.0f;
		xM[1] = 0.0f;
		xM[2] = 0.0f;
		xM[3] = 0.0f;
		xM[4] = cosX;
		xM[5] = sinX;
		xM[6] = 0.0f;
		xM[7] = -sinX;
		xM[8] = cosX;

		// rotation about y-axis (roll)
		yM[0] = cosY;
		yM[1] = 0.0f;
		yM[2] = sinY;
		yM[3] = 0.0f;
		yM[4] = 1.0f;
		yM[5] = 0.0f;
		yM[6] = -sinY;
		yM[7] = 0.0f;
		yM[8] = cosY;

		// rotation about z-axis (azimuth)
		zM[0] = cosZ;
		zM[1] = sinZ;
		zM[2] = 0.0f;
		zM[3] = -sinZ;
		zM[4] = cosZ;
		zM[5] = 0.0f;
		zM[6] = 0.0f;
		zM[7] = 0.0f;
		zM[8] = 1.0f;

		// rotation order is y, x, z (roll, pitch, azimuth)
		float[] resultMatrix = matrixMultiplication(xM, yM);
		resultMatrix = matrixMultiplication(zM, resultMatrix);
		return resultMatrix;

	}

	public float[] matrixMultiplication(float[] A, float[] B) {
		// TODO Auto-generated method stub
		float[] result = new float[9];

		result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
		result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
		result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

		result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
		result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
		result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

		result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
		result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
		result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

		return result;

	}
	
	private Runnable updateOreintationDisplayTask = new Runnable() {
		public void run() {
			updateOreintationDisplay();
		}

		private void updateOreintationDisplay() {
			// TODO Auto-generated method stub
			gyro.setText("Azimuth: " + fusedOrientation[0] * 180/Math.PI + " Pitch: " + fusedOrientation[1] * 180/Math.PI + "Roll: " + fusedOrientation[2] * 180/Math.PI);
			acc.setText("X: " + linear_acceleration[0] + " Y: "+ linear_acceleration[1] + " Z: " + linear_acceleration[2]);
		}
	};
}

/*
		Project:		GpsMotorCycle
		Module:			GpsMotorCycle.java
		Description:	The android activity base for GpsMotorCycle app
		Author:			Martin Gäckler
		Address:		Hofmannsthalweg 14, A-4030 Linz
		Web:			https://www.gaeckler.at/

		Copyright:		(c) 2013-2026 Martin Gäckler

		This program is free software: you can redistribute it and/or modify
		it under the terms of the GNU General Public License as published by
		the Free Software Foundation, version 3.

		You should have received a copy of the GNU General Public License
		along with this program. If not, see <http://www.gnu.org/licenses/>.

		THIS SOFTWARE IS PROVIDED BY Martin Gäckler, Linz, Austria ``AS IS''
		AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
		TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
		PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR
		CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
		SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
		LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
		USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
		ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
		OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
		OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
		SUCH DAMAGE.
*/
package at.gaeckler.GpxMotorCycle;

import java.io.IOException;
import java.text.DecimalFormat;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.annotation.SuppressLint;
import android.content.Context;
import android.content.SharedPreferences;
import android.location.GnssStatus;
import android.location.Location;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.TextView;

import androidx.annotation.NonNull;

import at.gaeckler.gps.GpsActivity;
import at.gaeckler.gps.GpsProcessor;
import at.gaeckler.gps.GpsService;
import at.gaeckler.gps.GpsUtils;

public class GpxMotorCycleActivity extends GpsActivity implements SensorEventListener
{
	private static final DecimalFormat s_altitudeFormat = new DecimalFormat( "0.0 m" );
	private static final DecimalFormat s_speedFormat = new DecimalFormat( "0.0 km/h" );

	private static final String CONFIGURATION_FILE = "GpxMotorCycle.cfg";
	private static final String	GPS_SPEED_KEY = "gpsInterval";
	private static final String	START_TIME_KEY = "startTime";
	private static final String	BREAK_TIME_KEY = "breakTime";
	private static final String	MAX_SPEED_KEY = "maxSpeed";
	private static final String	MIN_ACCEL_KEY = "minAccel";
	private static final String	MAX_ACCEL_KEY = "maxAccel";
	
	private static final String	DISTANCE_KEY = "distance";
	private static final String	UP_KEY = "upMeter";
	private static final String	DOWN_KEY = "downMeter";
	private static final String	LOCATION_KEY = "distanceLocation";
	
	private static final String	CALIBRATION_KEY = "calibrationMode";
	private static final String	FIX_COUNT_KEY = "fixCount";
	private static final String	SUM_LONGITUDE_KEY = "sumLongitude";
	private static final String	SUM_LATITUDE_KEY = "sumLatitude";
	private static final String	SUM_ALTITUDE_KEY = "sumAltitude";

	private static final DecimalFormat	s_accuracyFormat = new DecimalFormat( "Genauigkeit: 0.000m" );
	private static final DecimalFormat	s_orientFormat = new DecimalFormat( "0.000" );
	
	private TextView	m_statusView = null;
	private TextView	m_speedView = null;
	private TextView	m_maxSpeedView = null;
	private TextView	m_minAccelView = null;
	private TextView	m_curAccelView = null;
	private TextView	m_maxAccelView = null;
	
	private TextView	m_distanceView = null;
	private TextView	m_timeView = null;
	private TextView	m_breakView = null;
	
	private TextView	m_lonView = null;
	private TextView	m_latView = null;
	private TextView	m_altitudeView = null;
	private TextView	m_upView = null;
	private TextView	m_downView = null;
	private TextView	m_combinedOrientStatusLabel = null;
	private TextView	m_rotationStatusLabel = null;
	private TextView	m_gameStatusLabel = null;

	private String		m_myStatus = "Willkommen";

	private long		m_maxSpeed = 0;
	private double		m_minAccel = +100.0;
	private double		m_maxAccel = -100.0;
	private Location	m_distanceLocation = null;
	private double		m_distance = 0.0;
	private double		m_distanceIncrement = 0.0;
	private long		m_startTime = 0;
	private double		m_upMeter = 0.0;
	private double		m_downMeter = 0.0;
	
	private boolean		m_calibration = false;
	private double		m_sumLongitude = 0;
	private double		m_sumLatitude = 0;
	private double		m_sumAltitude = 0;
	private long		m_locationFixCount = 0;

	private SensorManager	m_sensorManager;
	private Sensor			m_gameRotationMeter;
	private Sensor			m_rotationVector;
	private Sensor			m_accelerometer;
	private Sensor			m_magneticField;
	
	private float[]			m_gameRotationMeterReading = new float[3];
	private float[]			m_rotationVectorReading = new float[3];
	private float[]			m_accelerometerReading = new float[3];
	private float[]			m_magneticFieldReading = new float[3];

	private final float[]	m_rotationMatrix = new float[9];
	private final float[]	m_orientationAngles = new float[3];


	@SuppressLint("DefaultLocale")
	private String fmtElapsed( long elapsedTime )
	{
		elapsedTime = elapsedTime / 1000;
		int sec = (int) (elapsedTime % 60);
		elapsedTime = elapsedTime / 60;
		int min = (int) (elapsedTime % 60);
		elapsedTime = elapsedTime / 60;
		int hour = (int) (elapsedTime);

		return String.format("%02d:%02d:%02d", hour, min, sec);
	}


	public void showMessage( String title, String message, final boolean terminate )
	{
		showMessage(R.drawable.icon, title, message, terminate, null);
	}

	private void startListening()
	{
		m_sensorManager.registerListener(this, m_rotationVector, SensorManager.SENSOR_DELAY_NORMAL);
		m_sensorManager.registerListener(this, m_gameRotationMeter, SensorManager.SENSOR_DELAY_NORMAL);
		m_sensorManager.registerListener(this, m_accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
		m_sensorManager.registerListener(this, m_magneticField, SensorManager.SENSOR_DELAY_NORMAL);
	}

	private void stopListening()
	{
		if( m_sensorManager != null )
			m_sensorManager.unregisterListener(this);
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		int sensorType = event.sensor.getType();

		switch (sensorType) {
			case Sensor.TYPE_GAME_ROTATION_VECTOR:
				m_gameRotationMeterReading = event.values.clone();
				break;
			case Sensor.TYPE_ROTATION_VECTOR:
				m_rotationVectorReading = event.values.clone();
				break;
			case Sensor.TYPE_ACCELEROMETER:
				m_accelerometerReading = event.values.clone();
				break;
			case Sensor.TYPE_MAGNETIC_FIELD:
				m_magneticFieldReading = event.values.clone();
				break;
			default:
				return;
		}

		SensorManager.getRotationMatrix(m_rotationMatrix, null, m_accelerometerReading, m_magneticFieldReading);
		SensorManager.getOrientation(m_rotationMatrix, m_orientationAngles);
		// Orientation angles are in radians. Convert to degrees if needed.
		double xAxis = Math.toDegrees(m_orientationAngles[0]);
		double yAxis = Math.toDegrees(m_orientationAngles[1]);
		double zAxis = Math.toDegrees(m_orientationAngles[2]);
		m_combinedOrientStatusLabel.setText(
			"C " +
			s_orientFormat.format(xAxis) + "x " +
			s_orientFormat.format(yAxis) + "y " +
			s_orientFormat.format(zAxis) + "z"
		);

		SensorManager.getRotationMatrixFromVector(m_rotationMatrix, m_rotationVectorReading);
		SensorManager.getOrientation(m_rotationMatrix, m_orientationAngles);
		// Orientation angles are in radians. Convert to degrees if needed.
		xAxis = Math.toDegrees(m_orientationAngles[0]);
		yAxis = Math.toDegrees(m_orientationAngles[1]);
		zAxis = Math.toDegrees(m_orientationAngles[2]);
		m_rotationStatusLabel.setText(
			"R " +
			s_orientFormat.format(xAxis) + "x " +
			s_orientFormat.format(yAxis) + "y " +
			s_orientFormat.format(zAxis) + "z"
		);

		SensorManager.getRotationMatrixFromVector(m_rotationMatrix, m_gameRotationMeterReading);
		SensorManager.getOrientation(m_rotationMatrix, m_orientationAngles);
		xAxis = Math.toDegrees(m_orientationAngles[0]);
		yAxis = Math.toDegrees(m_orientationAngles[1]);
		zAxis = Math.toDegrees(m_orientationAngles[2]);
		m_gameStatusLabel.setText(
			"G " +
			s_orientFormat.format(xAxis) + "x " +
			s_orientFormat.format(yAxis) + "y " +
			s_orientFormat.format(zAxis) + "z"
		);

		// Do something with the orientation angles (azimuth, pitch, roll)
		// For example, update a UI element or trigger an action based on orientation
		
	}

	@Override
	public void onAccuracyChanged(Sensor arg0, int arg1) {
		// TODO Auto-generated method stub
		
	}

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState)
	{
		super.onCreate(savedInstanceState);
		if( !checkLocationPermission() )
		{
			return;
		}

		if( requestStoragePermission(R.drawable.icon, "GPX-Motorcycle") == RequestCode.DENIED )
		{
			return;
		}

		if( savedInstanceState != null )
		{
			m_locationFixCount = savedInstanceState.getLong(FIX_COUNT_KEY,0);
			m_calibration = savedInstanceState.getBoolean(CALIBRATION_KEY,false);
			m_sumLongitude = savedInstanceState.getDouble(SUM_LONGITUDE_KEY,0);
			m_sumLatitude = savedInstanceState.getDouble(SUM_LATITUDE_KEY,0);
			m_sumAltitude = savedInstanceState.getDouble(SUM_ALTITUDE_KEY,0);

			m_distanceLocation = GpsUtils.locationString(savedInstanceState.getString(LOCATION_KEY));
			m_distance = savedInstanceState.getDouble(DISTANCE_KEY);
			m_upMeter = savedInstanceState.getDouble(UP_KEY);
			m_downMeter = savedInstanceState.getDouble(DOWN_KEY);
			m_maxSpeed = savedInstanceState.getLong(MAX_SPEED_KEY);
			m_minAccel = savedInstanceState.getDouble(MIN_ACCEL_KEY);
			m_maxAccel = savedInstanceState.getDouble(MAX_ACCEL_KEY);

			m_startTime = savedInstanceState.getLong(START_TIME_KEY);
			setBrakeTime(savedInstanceState.getLong(BREAK_TIME_KEY));
		}

		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		setContentView(R.layout.main);

		m_statusView = findViewById( R.id.statusView );
		setStatus( m_myStatus );
		m_speedView = findViewById( R.id.speedView );
		m_maxSpeedView = findViewById( R.id.maxDpeedView );

		m_minAccelView = findViewById( R.id.minAccelView );
		m_curAccelView = findViewById( R.id.curAccelView );
		m_maxAccelView = findViewById( R.id.maxAccelView );

		m_distanceView = findViewById( R.id.distanceView );
		m_timeView = findViewById( R.id.timeView );
		m_breakView = findViewById( R.id.breakView );

		m_lonView = findViewById( R.id.lonView );
		m_latView = findViewById( R.id.latView );
		m_altitudeView = findViewById( R.id.altitudeView );
		m_upView = findViewById( R.id.upView );
		m_downView = findViewById( R.id.downView );

		m_combinedOrientStatusLabel = findViewById( R.id.combinedStatus );
		m_rotationStatusLabel = findViewById( R.id.rotationStatus );
		m_gameStatusLabel = findViewById( R.id.gameStatus );

		m_sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		m_gameRotationMeter = m_sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
		m_rotationVector = m_sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
		m_accelerometer = m_sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		m_magneticField = m_sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
		startListening();

		setIgnoreAccuracy(true);
		//simulateLocationFix(m_home);
	}

	@Override
	protected void onConfigureService()
	{
		SharedPreferences settings = getSharedPreferences(CONFIGURATION_FILE, Context.MODE_PRIVATE);
		int gpsInterval = settings.getInt(GPS_SPEED_KEY,0);
		getService().createGpsTimer(gpsInterval);

		if(checkReadStoragePermission())
		{
			getService().getGpsLogger().readTrackPoints();
		}

		super.onConfigureService();
	}

	@Override
	public boolean onCreateOptionsMenu( android.view.Menu menu )
	{
		MenuInflater inflater = getMenuInflater();
		inflater.inflate(R.menu.gmc_menu, menu);

		return super.onCreateOptionsMenu(menu);
	}
	
	@Override
	public boolean onPrepareOptionsMenu(Menu menu)
	{
		menu.findItem(R.id.calibration).setChecked(m_calibration);

		int gpsInterval = getInterval();
		menu.findItem(R.id.autoGps).setChecked(gpsInterval== GpsService.AUTO_GPS);
		menu.findItem(R.id.fastGps).setChecked(gpsInterval==GpsService.FAST_GPS);
		menu.findItem(R.id.normalGps).setChecked(gpsInterval==GpsService.NORMAL_GPS);
		menu.findItem(R.id.slowGps).setChecked(gpsInterval==GpsService.SLOW_GPS);
		
		return super.onPrepareOptionsMenu(menu);
	}

	private void showAbout()
	{
		String name = getString(R.string.app_name);
		String version = getString(R.string.app_version);
		String copyright = getString(R.string.app_copyright);
		String url = getString(R.string.app_url);
		showMessage(
				name,
				name + " "+version+"\n"+copyright+"\n"+url,
				false
		);
	}
	@Override
	public boolean onOptionsItemSelected( MenuItem item )
	{
		int	itemID = item.getItemId();
		if( itemID == R.id.calibration )
		{
			if (!m_calibration)
			{
				m_calibration = true;
				m_sumLongitude = 0;
				m_sumLatitude = 0;
				m_sumAltitude = 0;
				m_locationFixCount = 0;
			}
			else
			{
				m_calibration = false;
			}
		}
		else if( itemID ==  R.id.autoGps )
		{
			getService().removeGpsTimer();
		}
		else if( itemID ==  R.id.fastGps )
		{
			getService().createGpsTimer(GpsService.FAST_GPS);
		}
		else if( itemID ==  R.id.normalGps )
		{
			getService().createGpsTimer(GpsService.NORMAL_GPS);
		}
		else if( itemID ==  R.id.slowGps )
		{
			getService().createGpsTimer(GpsService.SLOW_GPS);
		}
		else if( itemID ==  R.id.exit )
		{
			stopListening();
			try
			{
				getService().getGpsLogger().createGpxTrack();
			}
			catch (IOException e)
			{
				// TODO Auto-generated catch block
				Log.e(getLocalClassName(), "createGpxTrack failed", e);
			}
			finish();
		}
		else if( itemID ==  R.id.about )
		{
			showAbout();
		}

		return super.onOptionsItemSelected(item);
	}

	@Override
	public void onOptionsMenuClosed(Menu menu)
	{
		super.onOptionsMenuClosed(menu);
		// Workaround for https://issuetracker.google.com/issues/315761686
		invalidateOptionsMenu();
	}

	private void saveSharedPreferences()
	{
		SharedPreferences settings = getSharedPreferences(CONFIGURATION_FILE, Context.MODE_PRIVATE);
		SharedPreferences.Editor editor = settings.edit();

		editor.putInt(GPS_SPEED_KEY, getInterval() );

		// Commit the edits!
		editor.apply();
	}

	@Override
	protected void onPause()
	{
		saveSharedPreferences();
		super.onPause();
	}

	@Override
	protected void onDestroy()
	{
		stopListening();

		saveSharedPreferences();

		super.onDestroy();
	}
	
	@Override
	protected void  onSaveInstanceState (@NonNull Bundle outState)
	{
		super.onSaveInstanceState(outState);
		outState.putDouble(DISTANCE_KEY, m_distance );
		if( m_distanceLocation != null )
		{
			outState.putString(LOCATION_KEY, GpsUtils.locationString(m_distanceLocation));
		}
		outState.putDouble(UP_KEY, m_upMeter );
		outState.putDouble(DOWN_KEY, m_downMeter );
		outState.putLong(MAX_SPEED_KEY, m_maxSpeed);
		outState.putDouble(MIN_ACCEL_KEY, m_minAccel);
		outState.putDouble(MAX_ACCEL_KEY, m_maxAccel);

		outState.putLong(START_TIME_KEY, m_startTime);
		outState.putLong(BREAK_TIME_KEY, getBrakeTime());
		
		outState.putLong(FIX_COUNT_KEY, m_locationFixCount);
		outState.putBoolean(CALIBRATION_KEY, m_calibration);
		outState.putDouble(SUM_LONGITUDE_KEY, m_sumLongitude);
		outState.putDouble(SUM_LATITUDE_KEY, m_sumLatitude);
		outState.putDouble(SUM_ALTITUDE_KEY, m_sumAltitude);
	}
	
	private void updateDisplay( Location newLocation )
	{
		long speed = GpsProcessor.speedToKmh(getSpeed());
		m_speedView.setText(
			s_speedFormat.format(speed)
		);

		if( speed > m_maxSpeed )
		{
			m_maxSpeed = speed;
			m_maxSpeedView.setText(
				s_speedFormat.format(m_maxSpeed)
			);
		}

		double accel = getAccel();
		/// TODO remove
		String accelStr = getAccelStr();
		// String accelStr = Double.toString(accel);
		//String accelStr = s_accelFormat.format(accel);
		if( accel < m_minAccel )
		{
			m_minAccel = accel;
			m_minAccelView.setText(accelStr);
		}
		if( accel > m_maxAccel )
		{
			m_maxAccel = accel;
			m_maxAccelView.setText(accelStr);
		}
		m_curAccelView.setText(accelStr);
		double displayedDay = m_distance > 1000 ? m_distance/1000.0 : m_distance; 
		m_distanceView.setText(
			displayedDay+" "+m_distanceIncrement
		);
		m_timeView.setText(
			fmtElapsed(newLocation.getTime()-m_startTime)
		);
		m_breakView.setText(
			fmtElapsed(getBrakeTime())
		);
		int snapedAltitude = GpsUtils.getCorrectedAltitude(newLocation);
		double longitude, latitude, altitude;

		if(m_calibration)
		{
			longitude = m_sumLongitude/m_locationFixCount;
			latitude = m_sumLatitude/m_locationFixCount;
			altitude = m_sumAltitude/m_locationFixCount;
		}
		else
		{
			longitude = newLocation.getLongitude();
			latitude = newLocation.getLatitude();
			altitude = (int)newLocation.getAltitude();
		}
		
		m_lonView.setText(
				(m_calibration ? "*" : " ") +
				longitude
			);
		m_latView.setText(
			(m_calibration ? "*" : " ") +
			latitude
		);
		m_altitudeView.setText(
				(m_calibration ? "*" : " ") +
				s_altitudeFormat.format(snapedAltitude) + " (" + (int)(altitude+0.5) + ")"
			);
		m_upView.setText( s_altitudeFormat.format(m_upMeter) + "↑" );
		m_downView.setText( s_altitudeFormat.format(m_downMeter) + "↓" );
	}
	
	void setStatus( String text )
	{
		m_myStatus = text;
		m_statusView.setText(
			text + ' ' + 
			s_accuracyFormat.format(getAccuracy()) + ' ' + 
			m_locationFixCount + '/' +
			getNumLocations()
		);
	}

	@Override
	public void onLocationEnabled()
	{
		setStatus( "GPS ist eingeschaltet");
	}

	@Override
	public void onLocationDisabled()
	{
		setStatus( "GPS ist abgeschaltet");
	}
	
	@Override
	public void onGnssStatusChanged2(int event, GnssStatus status)
	{
		if( event == GPS_EVENT_STARTED )
			setStatus( "GPS gestartet");
		else if( event == GPS_EVENT_STOPPED )
			setStatus( "GPS gestoppt");
		else if( event == GPS_EVENT_FIRST_FIX )
			setStatus( "GPS erster Fix");
		else if( event == GPS_EVENT_SATELLITE_STATUS  )
		{
			int Satellites = status.getSatelliteCount();
			int SatellitesInFix = 0;

			for (int i = 0; i < Satellites; i++)
			{
				if(status.usedInFix(i))
				{
					SatellitesInFix++;
				}
			}

			setStatus( "GPS Satelliten: " + SatellitesInFix + "/" + Satellites );
		}
	}

	@Override
	public void onLocationChanged( Location newLocation )
	{
		if( m_startTime == 0 )
		{
			m_startTime = newLocation.getTime();
		}
		if( m_distanceLocation != null )
		{
			double	distance = m_distanceLocation.distanceTo(newLocation);
			m_distance += distance;
			m_distanceIncrement = distance;
			double heightChange = newLocation.getAltitude() - m_distanceLocation.getAltitude();
			if( heightChange > 0 )
			{
				m_upMeter += heightChange; 
			}
			else
			{
				m_downMeter -= heightChange;
			}
		}
		m_distanceLocation = newLocation;

		++m_locationFixCount;
		if( m_calibration )
		{
			m_sumLongitude += newLocation.getLongitude();
			m_sumLatitude += newLocation.getLatitude();
			m_sumAltitude += newLocation.getAltitude();
		}

		setStatus( m_myStatus );

		updateDisplay(newLocation);
	}
}
package com.example.edurond.myapplication;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.IBinder;
import java.util.Date;

/**
 * Created by android on 5/29/2016.
 */
public class CustomSensorService extends Service implements SensorEventListener {

    static SensorManager sensorManager;
    static Sensor mAccelerometer;
    private Sensor mMagnetometer;
    private Sensor mLinearAccelertion;

    //static Context mContext;

    private static float[] AccelerometerValue;
    private static float[] MagnetometerValue;

    public static  Float currentAcceleration = 0.0F;
    public static  Float  currentDirection = 0.0F;
    public static Float CurrentSpeed = 0.0F;
    public static Float CurrentDistanceTravelled = 0.0F;
    /*---------------------------------------------*/
    float[] prevValues,speed;
    float[] currentValues;
    float prevTime, currentTime, changeTime,distanceY,distanceX,distanceZ;
    float[] currentVelocity;
    public CalculatePosition CalcPosition;
/*-----FILTER VARIABLES-------------------------*/



    public static Float prevAcceleration = 0.0F;
    public static Float prevSpeed = 0.0F;
    public static Float prevDistance = 0.0F;

    public static Float totalDistance;
    public static Context theContext;
    Boolean First,FirstSensor = true;


    public CustomSensorService(Context theContext, double xInitial, double yInitial){
        super();
        this.theContext=theContext;

        CalcPosition =  new CalculatePosition(xInitial, yInitial);
        First = FirstSensor = true;
        currentValues = new float[3];
        prevValues = new float[3];
        currentVelocity = new float[3];
        speed = new float[3];
        totalDistance = 0.0F;

        sensorManager = (SensorManager) theContext.getSystemService(SENSOR_SERVICE);

        mAccelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMagnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        //mGyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mLinearAccelertion = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        sensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, mMagnetometer, SensorManager.SENSOR_DELAY_FASTEST);
        //sensorManager.registerListener(this, mGyro, SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, mLinearAccelertion, SensorManager.SENSOR_DELAY_FASTEST);

    }

    @Override
    public void onDestroy(){
        sensorManager.unregisterListener(this);
        sensorManager = null;
        super.onDestroy();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // TODO Auto-generated method stub
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        float[] values = event.values;
        Sensor mSensor = event.sensor;

        if(mSensor.getType() == Sensor.TYPE_ACCELEROMETER){
            AccelerometerValue = values;
        }

        if(mSensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION){
            if(First){
                prevValues = values;
                prevTime = event.timestamp / 1000000000;
                First = false;
                currentVelocity[0] = currentVelocity[1] = currentVelocity[2] = 0;
                distanceX = distanceY= distanceZ = 0;
            }
            else{
                currentTime = event.timestamp / 1000000000.0f;
                changeTime = currentTime - prevTime;
                prevTime = currentTime;

                calculateDistance(event.values, changeTime);

                currentAcceleration =  (float) Math.sqrt(event.values[0] * event.values[0] + event.values[1] * event.values[1] + event.values[2] * event.values[2]);
                CurrentSpeed = (float) Math.sqrt(speed[0] * speed[0] + speed[1] * speed[1] + speed[2] * speed[2]);
                CurrentDistanceTravelled = (float) Math.sqrt(distanceX *  distanceX + distanceY * distanceY +  distanceZ * distanceZ);
                CurrentDistanceTravelled = CurrentDistanceTravelled / 1000;

                if(FirstSensor){
                    prevAcceleration = currentAcceleration;
                    prevDistance = CurrentDistanceTravelled;
                    prevSpeed = CurrentSpeed;
                    FirstSensor = false;
                }
                prevValues = values;
            }
        }

        if(mSensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
            MagnetometerValue = values;
        }

        if(currentAcceleration != prevAcceleration || CurrentSpeed != prevSpeed || prevDistance != CurrentDistanceTravelled){

            if(!FirstSensor)
                totalDistance = totalDistance + CurrentDistanceTravelled * 1000;
            if (AccelerometerValue != null && MagnetometerValue != null && currentAcceleration != null) {
                //Direction
                float RT[] = new float[9];
                float I[] = new float[9];
                boolean success = SensorManager.getRotationMatrix(RT, I, AccelerometerValue,
                        MagnetometerValue);
                if (success) {
                    float orientation[] = new float[3];
                    SensorManager.getOrientation(RT, orientation);
                    float azimut = (float) Math.round(Math.toDegrees(orientation[0]));
                    currentDirection =(azimut+ 360) % 360;
                    if( CurrentSpeed > 0.2){
                        CalculatePosition.calculateNewPosition(currentAcceleration,CurrentSpeed,CurrentDistanceTravelled,currentDirection,totalDistance);
                    }
                }
                prevAcceleration = currentAcceleration;
                prevSpeed = CurrentSpeed;
                prevDistance = CurrentDistanceTravelled;
            }
        }
    }
    @Override
    public IBinder onBind(Intent arg0) {
        // TODO Auto-generated method stub
        return null;
    }
    public void calculateDistance (float[] acceleration, float deltaTime) {
        float[] distance = new float[acceleration.length];

        for (int i = 0; i < acceleration.length; i++) {
            speed[i] = acceleration[i] * deltaTime;
            distance[i] = speed[i] * deltaTime + acceleration[i] * deltaTime * deltaTime / 2;
        }
        distanceX = distance[0];
        distanceY = distance[1];
        distanceZ = distance[2];
    }
}



class CalculatePosition {

    static Double earthRadius = 6378D;
    static Boolean IsFirst = true;
    static Double sensorLatitude, sensorLongitude;

    static Date CollaborationWithGPSTime;

    public static float[] results;

    //Initial position of the player
    public static double xInitial, yInitial;

    public static Double
            oldLatitude,
            oldLongitude;
    //Add by abd
    public static float
            currentAcceleration,
            currentSpeed,
            currentDistanceTravelled,
            currentDirection,
            TotalDistance;

    public CalculatePosition(double xInitial, double yInitial){
        this.xInitial=xInitial;
        this.yInitial=yInitial;
    }
    public static void calculateNewPosition(
            Float currentAcceleration, Float currentSpeed,
            Float currentDistanceTravelled, Float currentDirection, Float TotalDistance) {


        results = new float[3];
        if(IsFirst){
            CollaborationWithGPSTime = new Date();

            //Saving data 1 of 3
            CalculatePosition.currentAcceleration=currentAcceleration;
            CalculatePosition.currentSpeed=currentSpeed;
            CalculatePosition.currentDistanceTravelled=currentDistanceTravelled;
            CalculatePosition.currentDirection=currentDirection;
            CalculatePosition.TotalDistance=TotalDistance;
            //oldLatitude = CustomLocationListener.mLatitude; //GPS reads
            //oldLongitude = CustomLocationListener.mLongitude; //GPS reads
            oldLatitude = xInitial; // initial position of the player
            oldLongitude = yInitial; // initial position of the player

            sensorLatitude = oldLatitude;
            sensorLongitude = oldLongitude;

            IsFirst  = false;
            return;
        }

        Date CurrentDateTime = new Date();

        if(CurrentDateTime.getTime() - CollaborationWithGPSTime.getTime() > 900000){
            //This IF Statement is to Collaborate with GPS position --> For accuracy --> 900,000 == 15 minutes

            //Saving data 2 of 3
            CalculatePosition.currentAcceleration=currentAcceleration;
            CalculatePosition.currentSpeed=currentSpeed;
            CalculatePosition.currentDistanceTravelled=currentDistanceTravelled;
            CalculatePosition.currentDirection=currentDirection;
            CalculatePosition.TotalDistance=TotalDistance;
            //CalculatePosition.oldLatitude = CustomLocationListener.mLatitude; //GPS reads
            //CalculatePosition.oldLongitude = CustomLocationListener.mLongitude; //GPS reads
            CalculatePosition.oldLatitude = xInitial; // initial position of the player
            CalculatePosition.oldLongitude = yInitial; // initial position of the player

            return;
        }

        if(CalculatePosition.oldLatitude==null)
            CalculatePosition.oldLatitude=0d;
        if(CalculatePosition.oldLongitude==null)
            CalculatePosition.oldLongitude=0d;
        //Convert Variables to Radian for the Formula
        CalculatePosition.oldLatitude = Math.PI * oldLatitude / 180;
        CalculatePosition.oldLongitude = Math.PI * oldLongitude / 180;
        currentDirection = (float) (Math.PI * currentDirection / 180.0);

        //Formulae to Calculate the NewLAtitude and NewLongtiude
        Double newLatitude = Math.asin(Math.sin(oldLatitude) * Math.cos(currentDistanceTravelled / earthRadius) +
                Math.cos(oldLatitude) * Math.sin(currentDistanceTravelled / earthRadius) * Math.cos(currentDirection));
        Double newLongitude = oldLongitude + Math.atan2(Math.sin(currentDirection) * Math.sin(currentDistanceTravelled / earthRadius)
                * Math.cos(oldLatitude), Math.cos(currentDistanceTravelled / earthRadius)
                - Math.sin(oldLatitude) * Math.sin(newLatitude));

        //Convert Back from radians
        newLatitude = 180 * newLatitude / Math.PI;
        newLongitude = 180 * newLongitude / Math.PI;
        currentDirection = (float) (180 * currentDirection / Math.PI);


        IsFirst = false;
        //Plot Position on Map


        ////Saving data 3 of 3
        CalculatePosition.currentAcceleration=currentAcceleration;
        CalculatePosition.currentSpeed=currentSpeed;
        CalculatePosition.currentDistanceTravelled=currentDistanceTravelled;
        CalculatePosition.currentDirection=currentDirection;
        CalculatePosition.TotalDistance=TotalDistance;
        CalculatePosition.oldLatitude = newLatitude;
        CalculatePosition.oldLongitude = newLongitude;

        sensorLatitude = oldLatitude;
        sensorLongitude = oldLongitude;
    }
}
/*class CustomLocationListener implements LocationListener {
    public static Double mLatitude,mLongitude;
    @Override
    public void onLocationChanged(Location location) {
        mLatitude = location.getLatitude();
        mLongitude = location.getLongitude();
    }
    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {

    }
    @Override
    public void onProviderEnabled(String provider) {

    }
    @Override
    public void onProviderDisabled(String provider) {

    }
}*/
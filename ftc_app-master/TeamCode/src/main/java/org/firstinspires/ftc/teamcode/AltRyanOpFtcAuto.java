package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import ftclib.FtcAccelerometer;
import ftclib.FtcAndroidAccel;
import ftclib.FtcAndroidGyro;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcAccelerometer;
import trclib.TrcRobot;
import trclib.TrcSensor;

/**
 * Created by notryan on 2/11/2017.
 */


@Autonomous(name = "Alt Ryan Auto", group = "notryan")
public class AltRyanOpFtcAuto extends FtcOpMode {


    private int numBeacons = 2;

    private Robot robot;
    private TrcRobot.RobotCommand autoCommand = null;
    private Alliance alliance = Alliance.RED_ALLIANCE;
    private double delay = 0.0;
    private int numParticles = 2;
    private ParkOption parkOption = ParkOption.PARK_CORNER;
    private Strategy strategy = Strategy.AUTO_100_1;
    private int beaconButtons = 2;
    private double driveDistance = 0.0;
    private double driveTime = 0.0;
    private double drivePower = 0.0;

    enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    enum ParkOption
    {
        DO_NOTHING,
        PARK_CORNER,
        PARK_CENTER
    }   //enum ParkOption

    private enum Strategy
    {
        AUTO_100_1,
        AUTO_100_2,
        AUTO_40_NEAR,
        AUTO_40_FAR,
        AUTO_40D_FAR,
        DISTANCE_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //enum Strategy






    private void shoot_particles() {

    }

    private void press_button(){

    }

    private void angle_to_wall() {

    }

    private  void turn(int degrees) {

    }


    FtcAndroidGyro gyro;
    FtcAndroidAccel accel;


    @Override
    public void initRobot() {


        robot = new Robot(TrcRobot.RunMode.AUTO_MODE);

        autoCommand = new Cmd100(robot, alliance, delay, numParticles, parkOption, beaconButtons);

    }

    @Override
    public void startMode() {
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setEnabled(true);
        robot.dashboard.clearDisplay();
    }

    @Override
    public void stopMode() {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setEnabled(false);
    }

    @Override
    public void runContinuous(double elapsedTime) {

//        TrcSensor.SensorData<Double> distance = accel.getRawXData(TrcAccelerometer.DataType.ACCELERATION);
//
//        dashboard.displayPrintf(1, "Distance x: %f", distance.value);
//
//
//        TrcSensor.SensorData<Double> gyrox = gyro.getXHeading();
//        TrcSensor.SensorData<Double> gyroy = gyro.getYHeading();
//        TrcSensor.SensorData<Double> gyroz = gyro.getZHeading();
//
//        dashboard.displayPrintf(2, "Heading x: %f", gyrox.value);
//        dashboard.displayPrintf(3, "Heading y: %f", gyroy.value);
//        dashboard.displayPrintf(4, "Heading z: %f", gyroz.value);


        if(autoCommand!=null){
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }

    @Override
    public void runPeriodic(double elapsedTime) {
        //dashboard.displayText(0, "Running..");


    }



}

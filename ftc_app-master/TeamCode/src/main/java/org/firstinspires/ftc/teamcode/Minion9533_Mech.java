
        package org.firstinspires.ftc.teamcode;

        import android.media.MediaPlayer;

        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcontroller.Util.Global;

        import hallib.HalDashboard;

        import static java.lang.Math.abs;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tele: Mech Op (USE THIS)", group="9533")
public class Minion9533_Mech extends MMOpMode_Linear implements FtcGamePad.ButtonHandler {



    private static final boolean USE_GYRO = false;
    private static final int tickInterval = 20;


    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    ElapsedTime ballStopTimer = new ElapsedTime();

    MinionsGyro gyro = null;

    //private HalDashboard dashboard;

    FtcGamePad driverGamepad;
    private FtcGamePad operatorGamepad;


    private boolean capballLiftReleased = false;

    private boolean ballStopOpen = false;



    @Override
    public void runOpMode() throws InterruptedException{

        super.runOpMode();


        driverGamepad = new FtcGamePad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamePad("OperatorGamepad", gamepad2, this);


        mechDrive.setGamepad(gamepad1);
        mechDrive.setHalBoard(robot.dashboard);


        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setMaxSpeed(3000);


        if(USE_GYRO) {
            gyro = new MinionsGyro(robot, "gyro");

        }
        robot.dashboard.displayPrintf(1, "Hello Driver");

        // Wait for the game to start (driver presses PLAY)


        robot.dashboard.displayPrintf(2, "Waiting for start..");
        waitForStart();

        robot.buttonPusher.setPosition(0.5);

        robot.dashboard.clearDisplay();
        // run until the end of the match (driver presses STOP)

        timer.reset();


        //MediaPlayer mediaPlayer = MediaPlayer.create(Global.context, com.qualcomm.ftcrobotcontroller.R.raw.banana);
        //mediaPlayer.start();

        while (opModeIsActive()) {


            if(ballStopOpen && ballStopTimer.seconds() >= 1){

                ballStopOpen = false;
                robot.ballStop.setPosition(0);
            }


            robot.dashboard.displayPrintf(1, "Drive Mode: %s", robot.invertedDrive ? "Reverse":"Normal");

            driverGamepad.update();
            operatorGamepad.update();

            mechDrive.Drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

//            robot.dashboard.displayText(2, "backLeft: " + robot.backLeftMotor.getPosition() );
//            robot.dashboard.displayText(3, "backRight: " + robot.backRightMotor.getPosition());
//            robot.dashboard.displayText(4, "frontLeft: " + robot.leftMotor.getPosition());
//            robot.dashboard.displayText(5, "frontRight: " + robot.rightMotor.getPosition());

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(tickInterval);
        }

    }


    @Override
    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed)
    {
        robot.dashboard.displayPrintf(7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamePad.GAMEPAD_A:
//                    if (pressed) {
//                        driveMode = DriveMode.MECANUM_MODE;
//                    }
                    break;

                case FtcGamePad.GAMEPAD_B:
//                    if (pressed) {
//                        driveMode = DriveMode.TANK_MODE;
//                    }
                    break;

                case FtcGamePad.GAMEPAD_X:
                    break;

                case FtcGamePad.GAMEPAD_Y:
                    break;

                case FtcGamePad.GAMEPAD_LBUMPER:
                    //drivePowerScale = pressed? 0.5: 1.0;
                    break;

                case FtcGamePad.GAMEPAD_RBUMPER:
                    robot.invertedDrive = pressed;
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamePad.GAMEPAD_A:

                    if(pressed && !ballStopOpen) {
                        ballStopOpen = true;
                        robot.ballStop.setPosition(1);
                        ballStopTimer.reset();
                    }
                    //
                    // Load particle.
                    //
//                    robot.shooter.setBallGatePosition(
//                            pressed? RobotInfo.BALLGATE_UP_POSITION : RobotInfo.BALLGATE_DOWN_POSITION);
                    break;

                case FtcGamePad.GAMEPAD_B:
                    //
                    // Manual firing.
                    //

                    robot.shooterMotor.setPower(pressed ? 1 : 0);

                    break;

                case FtcGamePad.GAMEPAD_X:
                    //
                    // Load particle, arm and fire the particle.
                    // This is basically combining button A and B.
                    //
//                    if (pressed)
//                    {
//                        robot.shooter.loadAndFireOneShot();
//                    }
                    break;

                case FtcGamePad.GAMEPAD_Y:

                    break;

                case FtcGamePad.GAMEPAD_LBUMPER:
                    //intake, drop balls out front
                    robot.elevator.setPower(pressed ? -1 : 0);
                    break;

                case FtcGamePad.GAMEPAD_RBUMPER:
                    //intake, lift balls up
                    robot.elevator.setPower(pressed ? 1 : 0);
                    break;

                case FtcGamePad.GAMEPAD_BACK:

                    capballLiftReleased = true;

                    robot.leftHold.setPosition(1);
                    robot.rightHold.setPosition(1);

                    break;

                case FtcGamePad.GAMEPAD_START:
                    break;

                case FtcGamePad.GAMEPAD_DPAD_UP:
                    if(pressed && capballLiftReleased) {
                        robot.liftMotor.setPower(1);
                    } else {
                        robot.liftMotor.setPower(0);
                    }

                    break;

                case FtcGamePad.GAMEPAD_DPAD_DOWN:
                    if(pressed && capballLiftReleased) {
                        robot.liftMotor.setPower(-0.3);
                    } else {
                        robot.liftMotor.setPower(0);
                    }
                    break;

                case FtcGamePad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamePad.GAMEPAD_DPAD_RIGHT:
                    /*
                    if (pressed)
                    {
                        robot.shooter.fireContinuous(true);
                    }
                    */
                    break;
            }
        }
    }   //gamepadButtonEvent
}


package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

class RobotInfo
{
    //
    // DriveBase subsystem.
    //
    static final float MM_PER_INCH                      = 25.4f;

    static final int DRIVE_MAX_SPEED                    = 4000;     //encoder counts per second
    //static final DcMotor.RunMode DRIVE_MOTOR_MODE       = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final DcMotor.RunMode DRIVE_MOTOR_MODE       = DcMotor.RunMode.RUN_USING_ENCODER;
    static final double TURN_POWER_LIMIT                = 0.5;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    //counts per inch
    //114.59493872353971032946044883018

    //inches per count
    //0.008726388889 - y
    //0.007817390046 - x


    //0.01667


    //(1440 * IpC) / 3.1415 == wheel diameter

    //
    // INCHES_PER_COUNT: 0.0132166817227156
    // 12/06/2016: 0.2, 0.0, 0.0 [< 8] 0.2, 0.0, 0.0 (No speed control)
    //
    static final double ENCODER_X_KP                    = 0.2;
    static final double ENCODER_X_KI                    = 0.0;
    static final double ENCODER_X_KD                    = 0.0;
    static final double ENCODER_X_KF                    = 0.0;
    static final double ENCODER_X_TOLERANCE             = 2.0;
    static final double ENCODER_X_SETTLING              = 0.2;
    static final double ENCODER_X_INCHES_PER_COUNT      = 0.0132166817227156;

    static final double SMALL_X_THRESHOLD               = 8.0;
    static final double ENCODER_SMALL_X_KP              = 0.2;
    static final double ENCODER_SMALL_X_KI              = 0.0;
    static final double ENCODER_SMALL_X_KD              = 0.0;
    //
    // INCHES_PER_COUNT: 0.01667
    // 12/06/2016: 0.045, 0.0, 0.0045 (No speed control)
    // 12/22/2016: 0.046, 0.0, 0.0046 (No speed control)
    // 01/09/2017: 0.02, 0.0, 0.0022 [< 8] 0.045, 0.0, 0.001 (No speed control)
    //
    static final double ENCODER_Y_KP                    = 0.020;
    static final double ENCODER_Y_KI                    = 0.0;
    static final double ENCODER_Y_KD                    = 0.0022;
    static final double ENCODER_Y_KF                    = 0.0;
    static final double ENCODER_Y_TOLERANCE             = 2.0;
    static final double ENCODER_Y_SETTLING              = 0.2;
    static final double ENCODER_Y_INCHES_PER_COUNT      = 1/COUNTS_PER_INCH; //0.01667

    static final double SMALL_Y_THRESHOLD               = 8.0;
    static final double ENCODER_SMALL_Y_KP              = 0.045;
    static final double ENCODER_SMALL_Y_KI              = 0.0;
    static final double ENCODER_SMALL_Y_KD              = 0.001;
    //
    // Accurate PID but slow (with speed control ON): 0.02, 0.0, 0.0028
    // Faster PID but may oscillate (with speed control ON): 0.022, 0.0, 0.0046 (limit to half power for mid-range)
    // 12/06/2016: 0.021, 0.0, 0.0021; [< 15] 0.024, 0.0, 0.002;    (No speed control, 0.75 power limit)
    // 12/07/2016: 0.021, 0.0, 0.0021; [< 15] 0.023, 0.0, 0.0023;   (No speed control, 0.75 power limit)
    // 12/07/2016: 0.02,  0.0, 0.0022; [< 15] 0.024, 0.0, 0.0024;   (No speed control, 0.75 power limit)
    // 12/08/2016: 0.02,  0.0, 0.0022; [< 15] 0.055, 0.0, 0.011;    (No speed control, 0.75 power limit)
    // 12/10/2016: 0.022, 0.0, 0.0024; [< 15] 0.028, 0.0, 0.0;      (No speed control, 0.75 power limit)
    // 01/09/2017: 0.024, 0.0, 0.024; [< 15] 0.03, 0.0, 0.001;        (No speed control, 0.75 power limit)
    //
    //static final double ANALOG_GYRO_SCALE               = 1.0136;
    static final double ANALOG_GYRO_SCALE               = -56.7286;
    static final double ANALOG_GYRO_VOLT_PER_DEG_PER_SEC= 0.007;
    static final double GYRO_KP                         = 0.024;
    static final double GYRO_KI                         = 0.0;
    static final double GYRO_KD                         = 0.024;
    static final double GYRO_KF                         = 0.0;
    static final double GYRO_TOLERANCE                  = 2.0;
    static final double GYRO_SETTLING                   = 0.2;

    static final double SMALL_TURN_THRESHOLD            = 15.0;
    static final double GYRO_SMALL_TURN_KP              = 0.03;
    static final double GYRO_SMALL_TURN_KI              = 0.0;
    static final double GYRO_SMALL_TURN_KD              = 0.001;

    //
    // 12/07/2016: 0.4, 0.0, 0.04 (No speed control)
    //
    static final double RANGE_KP                        = 0.4;
    static final double RANGE_KI                        = 0.0;
    static final double RANGE_KD                        = 0.04;
    static final double RANGE_KF                        = 0.0;
    static final double RANGE_TOLERANCE                 = 0.5;
    static final double RANGE_SETTLING                  = 0.2;

    static final double PIDDRIVE_STALL_TIMEOUT          = 0.25;     //in msec.

    //
    // Line detection.
    //
//    static final double LINE_DARK_LEVEL                 = 2.0;      //for color sensor
//    static final double LINE_WHITE_LEVEL                = 21.0;     //for color sensor
    static final double LINE_DARK_LEVEL                 = 0.3;      //for ODS
    static final double LINE_WHITE_LEVEL                = 2.3;      //for ODS

    //
    // Shooter subsystem.
    //
    static final double SHOOTER_KP                      = 0.02;
    static final double SHOOTER_KI                      = 0.0;
    static final double SHOOTER_KD                      = 0.0;
    static final double SHOOTER_KF                      = 0.0;
    static final double SHOOTER_TOLERANCE               = 2.0;
    static final double SHOOTER_SETTLING                = 0.2;
    static final double SHOOTER_POWER                   = 1.0;
    static final double SHOOTER_BALLGATE_OPEN_TIME      = 0.7;
    static final double SHOOTER_PAUSE_TIME              = 0.1;
    static final double SHOOTER_DEGREES_PER_COUNT       = (360.0/1680.0);
    static final double SHOOTER_PULLBACK_TARGET         = 180.0;    //in degrees

    //
    // Ball gate subsystem.
    //
    static final double BALLGATE_DOWN_POSITION          = 0.72;
    static final double BALLGATE_UP_POSITION            = 0.56;

    //
    // Button pusher subsystem.
    //
    static final double BUTTON_PUSHER_RETRACT_POSITION  = (45.0/255.0);
    static final double BUTTON_PUSHER_EXTEND_POSITION   = (200.0/255.0);

    //
    // Ball pickup subsystem.
    //
    static final double BALL_PICKUP_MOTOR_POWER         = 1.0;

    //
    // Conveyor subsystem.
    //
    static final double CONVEYOR_MOTOR_POWER            = 1.0;

}   //class RobotInfo

//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.Util.Helpers;
//
///**
// * Created by Kerfuffle on 2/12/2017.
// */
//
//@Autonomous(name = "ODS", group = "Test")
//public class TestODS extends MMOpMode_Linear {
//
//    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    public void runOpMode() throws InterruptedException
//    {
//        super.runOpMode();
//
////        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////
////        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        waitForStart();
//
//        if (opModeIsActive())
//        {
//            // initial diagonal off wall
//            driveTo(67, -0.4, -0.4, 0);
//
//            //move to right a lil
//            driveTo(16, 0, -0.9, 0);
//
//        }
//
//
//        /*while (opModeIsActive())
//        {
//            robot.dashboard.displayText(0, "Raw: " + robot.ods.getRawLightDetected());
//            robot.dashboard.displayText(1, "Reg: " + robot.ods.getLightDetected());
//        }*/
//    }
//
//
//    private void driveTo(double dist, double h, double v, double r)
//    {
//        if (h != 0 && v == 0 && r == 0)
//        {
//            //dist+=5;
//        }
//
//        if (opModeIsActive())
//        {
//            int backLeftTarget = (int)robot.backLeftMotor.getPosition() + (int)(dist*COUNTS_PER_INCH);
//            int frontLeftTarget = (int)robot.leftMotor.getPosition() + (int)(dist*COUNTS_PER_INCH);
//            int backRightTarget = (int)robot.backRightMotor.getPosition() + (int)(dist*COUNTS_PER_INCH);
//            int frontRightTarget = (int)robot.rightMotor.getPosition() + (int)(dist*COUNTS_PER_INCH);
//
//
//
//           /* robot.backLeftMotor.setTargetPosition(backLeftTarget);
//            robot.backRightMotor.setTargetPosition(backRightTarget);
//            robot.leftMotor.setTargetPosition(frontLeftTarget);
//            robot.rightMotor.setTargetPosition(frontRightTarget);
//
//            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
//
//
//
//            mechDrive.Drive(h, v, r, false);
//            do{
//                robot.dashboard.displayText(2, "backLeft: " + robot.backLeftMotor.getCurrentPosition() + " target: " + backLeftTarget);
//                robot.dashboard.displayText(3, "backRight: " + robot.backRightMotor.getCurrentPosition() + " target: " + backRightTarget);
//                robot.dashboard.displayText(4, "frontLeft: " + robot.rightMotor.getCurrentPosition() + " target: " + frontLeftTarget);
//                robot.dashboard.displayText(5, "frontRight: " + robot.leftMotor.getCurrentPosition() + " target: " + frontRightTarget);
//
//                /*if ((Math.abs(robot.backLeftMotor.getCurrentPosition()) >= backLeftTarget))
//                {
//                    robot.backLeftMotor.setPower(0);
//                }
//                if ((Math.abs(robot.backRightMotor.getCurrentPosition()) >= backRightTarget))
//                {
//                    robot.backRightMotor.setPower(0);
//                }
//                if ((Math.abs(robot.rightMotor.getCurrentPosition()) >= frontRightTarget))
//                {
//                    robot.rightMotor.setPower(0);
//                }
//                if ((Math.abs(robot.leftMotor.getCurrentPosition()) >= frontLeftTarget))
//                {
//                    robot.leftMotor.setPower(0);
//                }*/
//            }
//            while ((Math.abs(robot.backLeftMotor.getCurrentPosition()) < backLeftTarget) && (Math.abs(robot.backRightMotor.getCurrentPosition()) < backRightTarget) && (Math.abs(robot.rightMotor.getCurrentPosition()) < frontRightTarget) && (Math.abs(robot.leftMotor.getCurrentPosition()) < frontLeftTarget));
//            mechDrive.Stop();
//
//
//
//            /*while (robot.backLeftMotor.isBusy())
//            {
//                robot.dashboard.displayText(2, "backLeft: " + robot.backLeftMotor.getCurrentPosition());
//                robot.dashboard.displayText(3, "backRight: " + robot.backRightMotor.getCurrentPosition());
//                robot.dashboard.displayText(4, "frontLeft: " + robot.rightMotor.getCurrentPosition());
//                robot.dashboard.displayText(5, "frontRight: " + robot.leftMotor.getCurrentPosition());
//                robot.dashboard.displayText(0, "should be moving");
//            }*/
//
//
//
//            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        }
//
//    }
//
//}

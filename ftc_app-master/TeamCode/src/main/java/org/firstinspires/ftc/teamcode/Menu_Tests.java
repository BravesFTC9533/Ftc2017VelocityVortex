//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import static org.firstinspires.ftc.robotcontroller.Util.Global.tests;
//
///**
// * Created by Kerfuffle on 1/27/2017.
// */
//
//@TeleOp(name = "(MENU) Tests", group = "Menu")
//public class Menu_Tests extends LinearOpMode {
//
//    public void runOpMode()
//    {
//
//        tests.setGamepad(gamepad1);
//        tests.setTelemetry(telemetry);
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            tests.displayMenu();
//
//            Minion9533_Mech.shootPower = Double.parseDouble(tests.getCurrentChoiceOf("Shoot Power"));
//            Hardware9533.liftDropPower = Double.parseDouble(tests.getCurrentChoiceOf("Elevator Down Power"));
//        }
//    }
//
//}

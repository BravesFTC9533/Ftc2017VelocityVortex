/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Test mode", group = "Concept")
public class TestMenuOp extends MMOpMode_Linear  implements FtcMenu.MenuButtons {


  private enum Test
  {
    SENSORS_TEST,
    MOTORS_TEST,
    TIMED_DRIVE,
    DISTANCE_DRIVE,
    DEGREES_TURN,
    LINE_FOLLOW
  }   //enum Test

  private enum Alliance
  {
    RED_ALLIANCE,
    BLUE_ALLIANCE
  }   //enum Alliance

  private enum State
  {
    START,
    TURN_TO_LINE,
    FOLLOW_LINE,
    DONE
  }   //enum State





  private ElapsedTime runtime = new ElapsedTime();
  private HalDashboard dashboard;
  //
  // Menu choices.
  //
  private Test test = Test.SENSORS_TEST;
  private double driveTime = 0.0;
  private double driveDistance = 0.0;
  private double turnDegrees = 0.0;
  private Alliance alliance = Alliance.RED_ALLIANCE;
  private double wallDistance = 0.0;

  @Override
  public void runOpMode() {


    super.runOpMode();



    dashboard = getDashboard();

    dashboard.clearDisplay();
    doMenus();

    waitForStart();

    while(opModeIsActive()){
      switch (test)
      {
//        case SENSORS_TEST:
//          doSensorsTest();
//          break;

        case MOTORS_TEST:
          doMotorsTest();
          break;

        case TIMED_DRIVE:
          doTimedDrive(driveTime);
          break;

        case DISTANCE_DRIVE:
          doDistanceDrive(driveDistance);
          break;

        case DEGREES_TURN:
          doDegreesTurn(turnDegrees);
          break;

//        case LINE_FOLLOW:
//          doLineFollow(alliance, wallDistance);
//          break;
      }
    }

  }

  private void doMenus()
  {
    FtcChoiceMenu testMenu = new FtcChoiceMenu("Tests:", null, this);
    FtcValueMenu driveTimeMenu = new FtcValueMenu("Drive time:", testMenu, this,
            1.0, 10.0, 1.0, 8.0, " %.0f sec");
    FtcValueMenu driveDistanceMenu = new FtcValueMenu("Drive distance:", testMenu, this,
            1.0, 10.0, 1.0, 8.0, " %.0f ft");
    FtcValueMenu turnDegreesMenu =
            new FtcValueMenu("Turn degrees:", testMenu, this,
                    -360.0, 360.0, 90.0, 360.0, " %.0f deg");
    FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", testMenu, this);
    FtcValueMenu wallDistanceMenu = new FtcValueMenu("Wall distance:", allianceMenu, this,
            2.0, 12.0, 2.0, 2.0, " %.0f in");


    testMenu.addChoice("Sensors test", Test.SENSORS_TEST);
    testMenu.addChoice("Motors test", Test.MOTORS_TEST);
    testMenu.addChoice("Timed drive", Test.TIMED_DRIVE, driveTimeMenu);
    testMenu.addChoice("Distance drive", Test.DISTANCE_DRIVE, driveDistanceMenu);
    testMenu.addChoice("Degrees turn", Test.DEGREES_TURN, turnDegreesMenu);
    testMenu.addChoice("Line follow", Test.LINE_FOLLOW, allianceMenu);

    allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, wallDistanceMenu);
    allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, wallDistanceMenu);

    FtcMenu.walkMenuTree(testMenu);

    test = (Test)testMenu.getCurrentChoiceObject();
    driveTime = driveTimeMenu.getCurrentValue();
    driveDistance = driveDistanceMenu.getCurrentValue();
    turnDegrees = turnDegreesMenu.getCurrentValue();
    alliance = (Alliance)allianceMenu.getCurrentChoiceObject();
    wallDistance = wallDistanceMenu.getCurrentValue();

    dashboard.displayPrintf(0, "Test: %s", testMenu.getCurrentChoiceText());
  }   //doMenus



  private void doMotorsTest()
  {
      //dashboard.displayPrintf(9, "Motors Test: index=%d", motorIndex);
//    dashboard.displayPrintf(10, "Enc: lf=%.0f, rf=%.0f",
//            robot.leftFrontWheel.getPosition(),
//            robot.rightFrontWheel.getPosition());
//    dashboard.displayPrintf(11, "Enc: lr=%.0f, rr=%.0f",
//            robot.leftRearWheel.getPosition(),
//            robot.rightRearWheel.getPosition());

//    if (sm.isReady())
//    {
//      State state = (State)sm.getState();
//      switch (state)
//      {
//        case START:
//          //
//          // Spin a wheel for 5 seconds.
//          //
//          switch (motorIndex)
//          {
//            case 0:
//              robot.leftFrontWheel.setPower(0.5);
//              robot.rightFrontWheel.setPower(0.0);
//              robot.leftRearWheel.setPower(0.0);
//              robot.rightRearWheel.setPower(0.0);
//              break;
//
//            case 1:
//              robot.leftFrontWheel.setPower(0.0);
//              robot.rightFrontWheel.setPower(0.5);
//              robot.leftRearWheel.setPower(0.0);
//              robot.rightRearWheel.setPower(0.0);
//              break;
//
//            case 2:
//              robot.leftFrontWheel.setPower(0.0);
//              robot.rightFrontWheel.setPower(0.0);
//              robot.leftRearWheel.setPower(0.5);
//              robot.rightRearWheel.setPower(0.0);
//              break;
//
//            case 3:
//              robot.leftFrontWheel.setPower(0.0);
//              robot.rightFrontWheel.setPower(0.0);
//              robot.leftRearWheel.setPower(0.0);
//              robot.rightRearWheel.setPower(0.5);
//              break;
//          }
//          motorIndex = motorIndex + 1;
//          timer.set(5.0, event);
//          sm.addEvent(event);
//          sm.waitForEvents(motorIndex < 4? State.START: State.DONE);
//          break;
//
//        case DONE:
//        default:
//          //
//          // We are done.
//          //
//          robot.leftFrontWheel.setPower(0.0);
//          robot.rightFrontWheel.setPower(0.0);
//          robot.leftRearWheel.setPower(0.0);
//          robot.rightRearWheel.setPower(0.0);
//          sm.stop();
//          break;
//      }
//    }
  }   //doMotorsTest

  private void doTimedDrive(double time)
  {
//    double lfEnc = robot.leftFrontWheel.getPosition();
//    double rfEnc = robot.rightFrontWheel.getPosition();
//    double lrEnc = robot.leftRearWheel.getPosition();
//    double rrEnc = robot.rightRearWheel.getPosition();
//    dashboard.displayPrintf(9, "Timed Drive: %.0f sec", time);
//    dashboard.displayPrintf(10, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
//    dashboard.displayPrintf(11, "Enc:lr=%.0f,rr=%.0f", lrEnc, rrEnc);
//    dashboard.displayPrintf(12, "average=%f", (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
//    dashboard.displayPrintf(13, "xPos=%.1f,yPos=%.1f,heading=%.1f",
//            robot.driveBase.getXPosition(),
//            robot.driveBase.getYPosition(),
//            robot.driveBase.getHeading());
//
//    if (sm.isReady())
//    {
//      State state = (State)sm.getState();
//      switch (state)
//      {
//        case START:
//          //
//          // Drive the robot forward and set a timer for the given time.
//          //
//          robot.driveBase.tankDrive(0.2, 0.2);
//          timer.set(time, event);
//          sm.addEvent(event);
//          sm.waitForEvents(State.DONE);
//          break;
//
//        case DONE:
//        default:
//          //
//          // We are done, stop the robot.
//          //
//          robot.driveBase.stop();
//          sm.stop();
//          break;
//      }
//    }
  }   //doTimedDrive

  private void doDistanceDrive(double distance)
  {
//    dashboard.displayPrintf(9, "Distance Drive: %.1f ft", distance);
//    dashboard.displayPrintf(10, "xPos=%.1f,yPos=%.1f,heading=%.1f",
//            robot.driveBase.getXPosition(),
//            robot.driveBase.getYPosition(),
//            robot.driveBase.getHeading());
//    robot.encoderPidCtrl.displayPidInfo(11);
//    robot.gyroPidCtrl.displayPidInfo(13);
//
//    if (sm.isReady())
//    {
//      State state = (State)sm.getState();
//      switch (state)
//      {
//        case START:
//          //
//          // Drive the given distance.
//          //
//          robot.pidDrive.setTarget(distance*12.0, 0.0, false, event);
//          sm.addEvent(event);
//          sm.waitForEvents(State.DONE);
//          break;
//
//        case DONE:
//        default:
//          //
//          // We are done.
//          //
//          sm.stop();
//          break;
//      }
//    }
  }   //doDistanceDrive

  private void doDegreesTurn(double degrees)
  {
//    dashboard.displayPrintf(9, "Degrees Turn: %.1f", degrees);
//    dashboard.displayPrintf(10, "xPos=%.1f,yPos=%.1f,heading=%.1f",
//            robot.driveBase.getXPosition(),
//            robot.driveBase.getYPosition(),
//            robot.driveBase.getHeading());
//    robot.encoderPidCtrl.displayPidInfo(11);
//    robot.gyroPidCtrl.displayPidInfo(13);
//
//    if (sm.isReady())
//    {
//      State state = (State)sm.getState();
//      switch (state)
//      {
//        case START:
//          //
//          // Turn the given degrees.
//          //
//          robot.pidDrive.setTarget(0.0, degrees, false, event);
//          sm.addEvent(event);
//          sm.waitForEvents(State.DONE);
//          break;
//
//        case DONE:
//        default:
//          //
//          // We are done.
//          //
//          sm.stop();
//          break;
//      }
//    }
  }   //doDegreesTurn






  @Override
  public boolean isMenuUpButton()
  {
    return gamepad1.dpad_up;
  }   //isMenuUpButton

  @Override
  public boolean isMenuDownButton()
  {
    return gamepad1.dpad_down;
  }   //isMenuDownButton

  @Override
  public boolean isMenuEnterButton()
  {
    return gamepad1.a;
  }   //isMenuEnterButton

  @Override
  public boolean isMenuBackButton()
  {
    return gamepad1.dpad_left;
  }   //isMenuBackButton
}

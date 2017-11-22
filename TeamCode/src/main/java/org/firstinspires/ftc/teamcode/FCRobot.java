/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name = "FullControl", group = "TeleOp")
public class FCRobot extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo servo1 = null;
    private int servo1position = 0;
    private int loops = 0;
    private int motorTickPerRevolution = (1440/86)*100;
    private double wheelDiameter = 2 * 5.1 * 3.14;
    private double distanceBetween = 39;
    private double placeSpinDiameter = 2 * distanceBetween * 3.14;
    @Deprecated
    private double spinPerSecond = 1.85;
    @Deprecated
    private double distancePerSecond = spinPerSecond * wheelDiameter;
    @Deprecated
    private double distancePerDegree = placeSpinDiameter / 360;
    private double timePerCentimeter = 1 / wheelDiameter * distancePerSecond;
    private double timePerDegree = timePerCentimeter * distancePerDegree;
    private double tickPerCentimeter = (motorTickPerRevolution ) / (wheelDiameter);
    private double tickPerDegree = ((placeSpinDiameter * tickPerCentimeter / 360)/3.6)*3.90;
    private ArrayList<RobotAction> actions = new ArrayList<>();
    private RobotAction[] autonomous=new RobotAction[]{
            new RobotAction((int)getTime() + 1000, new RobotAction.Execute() {
                @Override
                public void onExecute() {
                    autonomousTurnRight(50);
                }

                @Override
                public boolean loopExecute() {
                    return false;
                }
            }),
            new RobotAction(0,null)
    };
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leftDrive = hardwareMap.get(DcMotor.class, "l");
        rightDrive = hardwareMap.get(DcMotor.class, "r");
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servo1 = hardwareMap.get(Servo.class, "s1");
        servo1.setDirection(Servo.Direction.FORWARD);
        servo1.setPosition(5);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Info", "Values: ");
        double turn = getTurnPower();
        if (turn != 0) {
            if (turn < 0) {
                turnLeft(getDrivePower());
            } else {
                turnRight(getDrivePower());
            }
        } else {
            if (actions.size() == 0) {
                leftDrive.setPower(getDrivePower());
                rightDrive.setPower(getDrivePower());
            }
        }
        if (gamepad1.b) {
            autonomousDrive(100);
            //                meterTest();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
            }
        } else if (gamepad1.y) {
            autonomousTurnLeft(180);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
            }
        }
        if (gamepad1.x) {
            emergencyStop();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
            }
        }
        for (int a = 0; a < actions.size(); a++) {
            RobotAction act = actions.get(a);
            if(act.getTime() != -1) {
                if (act.getTime() == (int) runtime.milliseconds()) {
                    act.getExecutable().onExecute();
                    actions.remove(a);
                } else if (act.getTime() < (int) runtime.milliseconds()) {
                    actions.remove(a);
                }
            }else{
                boolean remove=act.getExecutable().loopExecute();
                if(remove){
                    actions.remove(a);
                }
            }
        }
        loops++;
        telemetry.addData("Actions Left", actions.size());
        if (actions.size() > 0) {
            telemetry.addData("Next Action In", (actions.get(0).time - runtime.milliseconds()) / 1000);
        }
        telemetry.addData("Seconds", (int) runtime.seconds());
        telemetry.addData("Loops", loops);
        telemetry.addData("Motor", leftDrive.getPower() + " " + rightDrive.getPower());
    }
    void resetRobot(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void emergencyStop() {
        resetRobot();
        actions.clear();
        telemetry.addData("EMERGENCY", "Emergency Stop!");
    }

    int getPosition(int target) {
        return 0;
    }

    double getTimeForCM(int cm) {
        return cm / distancePerSecond;
    }

    double getTime() {
        return runtime.milliseconds();
    }

    double fullTimeForCM(int cm) {
        return getTime() + getTimeForCM(cm) * 1000;
    }

//    void meterTest() {
//        leftDrive.setPower(1);
//        rightDrive.setPower(1);
//        actions.add(new RobotAction((int) fullTimeForCM(100), new RobotAction.Execute() {
//            @Override
//            public void onExecute() {
//                leftDrive.setPower(0);
//                rightDrive.setPower(0);
//            }
//        }));
//    }

    void autonomousDrive(int centimeters) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        actions.add(new RobotAction(-1, new RobotAction.Execute() {
            @Override
            public void onExecute() {
            }

            @Override
            public boolean loopExecute() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    telemetry.addData("Action", "DONE");
                    return true;
                } else {
                    telemetry.addData("Action", "NOT_DONE");
                    return false;
                }
            }
        }));
    }

    void autonomousTurnLeft(final int degree) {
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
        rightDrive.setPower(1);
        actions.add(new RobotAction(-1, new RobotAction.Execute() {
            @Override
            public void onExecute() {
            }

            @Override
            public boolean loopExecute() {
                if (rightDrive.isBusy() && leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        }));
    }

    void autonomousTurnRight(final int degree) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
        leftDrive.setPower(1);
        actions.add(new RobotAction(-1, new RobotAction.Execute() {
            @Override
            public void onExecute() {
                if (rightDrive.isBusy() && leftDrive.isBusy()) {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            @Override
            public boolean loopExecute() {
                return true;
            }
        }));
    }

    void turnLeft(double forwardPower) {
        if (forwardPower == 0) {
            leftDrive.setPower(-1.0);
            rightDrive.setPower(1.0);
        } else {
            if (forwardPower > 0) {
                leftDrive.setPower(forwardPower / 8);
                rightDrive.setPower(forwardPower);
            } else {
                rightDrive.setPower(forwardPower);
                leftDrive.setPower(forwardPower / 8);
            }
        }
    }

    void turnRight(double forwardPower) {
        if (forwardPower == 0) {
            leftDrive.setPower(1.0);
            rightDrive.setPower(-1.0);
        } else {
            if (forwardPower > 0) {
                leftDrive.setPower(forwardPower);
                //                rightDrive.setPower( forwardPower / 8);
                rightDrive.setPower(forwardPower / 8);
            } else {
                rightDrive.setPower(forwardPower / 8);
                leftDrive.setPower(forwardPower);
            }
        }
    }

    double getDrivePower() {
        double power = 0;
        if (gamepad1.right_trigger != 0) {
            power += gamepad1.right_trigger;
        }
        if (gamepad1.left_trigger != 0) {
            power -= gamepad1.left_trigger;
        }
        return power;
    }

    double getTurnPower() {
        if (gamepad1.dpad_left) {
            return 1;
        } else if (gamepad1.dpad_right) {
            return -1;
        } else {
            return 0;
        }
    }

    @Override
    public void stop() {
    }
}

class RobotAction {
    Execute e;
    int time;

    RobotAction(int time, @Nullable Execute execute) {
        this.time = time;
        this.e = execute;
    }

    Execute getExecutable() {
        return e;
    }

    int getTime() {
        return time;
    }

    interface Execute {
        void onExecute();
        boolean loopExecute();
    }
}
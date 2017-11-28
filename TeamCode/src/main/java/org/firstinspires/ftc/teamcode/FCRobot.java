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

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "FullControl", group = "TeleOp")
public class FCRobot extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private int armInitial=0;
    private int armFinal=0;
    private int MULTI = 1;
    private int LINEBYLINE = 2;
    private int MODE = MULTI;
    private int motorTickPerRevolution = (1440 / 86) * 100;
    private double wheelDiameter = 2 * 5.1 * 3.14;
    private double distanceBetween = 39;
    private double placeSpinDiameter = 2 * distanceBetween * 3.14;
    private double tickPerCentimeter = (motorTickPerRevolution) / (wheelDiameter);
    private double tickPerDegree = ((placeSpinDiameter * tickPerCentimeter / 360) /231)*245;
    private double armToGear=6.5/3;
    private ArrayList<Action> actions = new ArrayList<>();
    private ArrayList<String> permanent_messages =new ArrayList<>();
    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "l");
        rightDrive = hardwareMap.get(DcMotor.class, "r");
        arm = hardwareMap.get(DcMotor.class, "arm");

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawLeft = hardwareMap.get(Servo.class, "s1");
        clawLeft.setDirection(Servo.Direction.REVERSE);
        clawRight = hardwareMap.get(Servo.class, "s2");
        clawRight.setDirection(Servo.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);
        armInitial=arm.getCurrentPosition();
        armFinal=armInitial+0;
        collapseClaw();

        //        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void init_loop() {
    }
    @Override
    public void start() {
        runtime.reset();
        clawOpen();
    }
    @Override
    public void loop() {
        checkEmergency();
        handleGamepad1();
        handleGamepad2();
        handleActions();
        showMessages();
    }
    @Override
    public void stop() {
        collapseClaw();
        resetRobot();
    }
    private Action[] getAutonomous() {
        Action[] autonomousTest = new Action[]{autonomousDrive(50), autonomousClawOpen(),autonomousDrive(20),autonomousClawClose(),autonomousArmMove(10),new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                MODE=MULTI;
            }

            @Override
            public boolean onLoop() {
                return true;
            }
        })};
        return autonomousTest;
    }
    double getDrivePower() {
        double power = 0;
        if (gamepad1.right_trigger != 0) {
            power += gamepad1.right_trigger;
        }
        if (gamepad1.left_trigger != 0) {
            power -= gamepad1.left_trigger/3;
        }
        return power;
    }
    double getTurnPower() {
        if (gamepad1.dpad_left) {
            return -1;
        } else if (gamepad1.dpad_right) {
            return 1;
        } else {
            return 0;
        }
    }
    double getArmSpeed(){
        double speed = 0;
        if (gamepad2.right_trigger != 0) {
            speed += gamepad2.right_trigger;
        }
        if (gamepad2.left_trigger != 0) {
            speed -= gamepad2.left_trigger/4;
        }
        return speed;
    }
    boolean isClawOpen(){
        if (clawLeft.getPosition()<0.45&&clawRight.getPosition()<0.45) {
            return false;
        }
        return true;
    }
    boolean isArmUp(){
        if(arm.getCurrentPosition()>armInitial){
            return true;
        }
        return false;
    }
    Action autonomousTurnRight(final int degree) {
        Action a = new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                leftDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
        return a;
    }
    Action autonomousTurnLeft(final int degree) {
        Action a = new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                rightDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
        return a;
    }
    Action autonomousArmMove(final int cm) {
        Action a = new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition((int) ((tickPerCentimeter/armToGear)*cm));
                arm.setPower(-0.7);
            }

            @Override
            public boolean onLoop() {
                if (!arm.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
        return a;
    }
    Action autonomousClawOpen() {
        Action a = new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                clawLeft.setPosition(0.4);
                clawRight.setPosition(0.4);
            }

            @Override
            public boolean onLoop() {
                return isClawOpen();
            }
        });
        return a;
    }
    Action autonomousClawClose() {
        Action a = new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                clawLeft.setPosition(0.5);
                clawRight.setPosition(0.5);
            }

            @Override
            public boolean onLoop() {
                return !isClawOpen();
            }
        });
        return a;
    }
    Action autonomousDrive(final int centimeters) {
        Action a = new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
        return a;
    }
    void autoTurnLeft(final int degree) {
        actions.add(new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                rightDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        }));
    }
    void autoTurnRight(final int degree) {
        actions.add(new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                leftDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        }));
    }
    void handleActions(){
        if (MODE == LINEBYLINE) {
            if (actions.size() != 0) {
                if (actions.get(0).isAlive()) {
                    if (!actions.get(0).isSetup()) {
                        actions.get(0).setup();
                    } else {
                        actions.get(0).loop();
                    }
                } else {
                    actions.remove(0);
                }
            }
        } else if (MODE == MULTI) {
            for (int a = 0; a < actions.size(); a++) {
                Action act = actions.get(a);
                if (act.isAlive()) {
                    if (!act.isSetup()) {
                        act.setup();
                    } else {
                        act.loop();
                    }
                }
            }
        }
    }
    void showMessages(){
        for(int s = 0; s< permanent_messages.size(); s++){
            telemetry.addLine(permanent_messages.get(s));
        }
        showStats();
    }
    void showStats(){
        telemetry.addData("Actions:", actions.size());
        if(isClawOpen()) {
            telemetry.addData("Claw State:", "Opened");
        }else{
            telemetry.addData("Claw State:", "Closed");
        }
        //        telemetry.addData("Seconds", (int) runtime.seconds());
        //        telemetry.addData("Motor", leftDrive.getPower() + " " + rightDrive.getPower());
    }
    void handleGamepad1(){
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
            autoDrive(100);
            buttonSleep();
        } else if (gamepad1.y) {
            fullAuto();
            buttonSleep();
        } else if (gamepad1.a) {
        }

    }
    void handleGamepad2(){
        armMove(getArmSpeed());
        if(gamepad2.dpad_right){
            clawOpen();
        }else if(gamepad2.dpad_left){
            clawClose();
        }else if(gamepad2.a){

        }else if(gamepad2.b){

        }else if(gamepad2.y){
            if(isArmUp()){
                autoArm(armInitial);
            }else{
                autoArm(armFinal);
            }
        }
    }
    void checkEmergency(){
        if (gamepad1.x||gamepad2.x) {
            emergencyStop();
            buttonSleep();
        }
    }
    void buttonSleep(){
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    void resetRobot() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void emergencyStop() {
        resetRobot();
        actions.clear();
        telemetry.addLine("Emergency Stop!");
    }
    void clawOpen(){
        clawLeft.setPosition(0.4);
        clawRight.setPosition(0.4);
    }
    void clawClose(){
        clawLeft.setPosition(0.55);
        clawRight.setPosition(0.55);
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
    void armMove(double speed){
        arm.setPower(speed);
    }
    void fullAuto() {
        resetRobot();
        actions.clear();
        MODE = LINEBYLINE;
        actions.addAll(Arrays.asList(getAutonomous()));
    }
    void autoArm(int toPosition){

    }
    void autoDrive(final int centimeters) {
        actions.add(new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        }));
    }
    void collapseClaw(){
        clawLeft.setPosition(0);
        clawRight.setPosition(0);
        buttonSleep();
        buttonSleep();
        buttonSleep();
        clawLeft.setPosition(1);
        buttonSleep();
        buttonSleep();
        buttonSleep();
        clawRight.setPosition(0.8);
    }
}

class Action {
    @Deprecated
    static class TimeableAction {
        Execute e;
        int time;

        TimeableAction(int time, @Nullable Execute execute) {
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
        }
    }

    Execute e;

    Action(@NonNull Execute execute) {
        e = execute;
    }

    private boolean alive = true;
    private boolean setup = false;

    boolean isAlive() {
        return alive;
    }

    boolean isSetup() {
        return setup;
    }

    void setup() {
        e.onSetup();
        setup = true;
    }

    void loop() {
        alive = !e.onLoop();
    }

    interface Execute {
        void onSetup();

        boolean onLoop();
    }
}

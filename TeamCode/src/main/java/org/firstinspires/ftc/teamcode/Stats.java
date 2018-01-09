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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Stats.LEFT;
import static org.firstinspires.ftc.teamcode.Stats.RED_TEAM;
import static org.firstinspires.ftc.teamcode.Stats.RIGHT;

public class Stats {
    static final int MULTI = 1;
    static final int LINEBYLINE = 2;
    static final int RIGHT = 1;
    static final int LEFT = -1;
    static final int BLUE_TEAM = 1;
    static final int RED_TEAM = 2;
    static final int westRightForwardCm = 30 + 39;
    static final int westLeftForwardCm = westRightForwardCm + 19 + 19;
    static final int westCenterForwardCm = westRightForwardCm + 19;
    static final int eastLeftCm = 54;
    static final int eastCenterCm = 36;
    static final int eastRightCm = 19;
    static final int motorTickPerRevolution = (1440 / 86) * 100;//1674.41
    static final int armMotorTickPerRevolution = (1440);
    static final double PI = 3.14;
    static final double wheelDiameter = 2 * 5.1 * PI;
    static final double distanceBetween = 39;
    static final double maxArmCentimeter = 65.3;
    static final double placeSpinDiameter = 2 * distanceBetween * PI;
    static final double placeOutSpinDiameter = distanceBetween * PI;//122.46
    static final double tickPerCentimeter = (motorTickPerRevolution) / (wheelDiameter);//52.27
    static final double armToGear = 3 / 6.5;
    static final double armLength = 28.8;
    static final double armCmPerMotorRevolution = (2 * armLength * PI);
    static final double tickPerArmCentimeter = (armMotorTickPerRevolution / armToGear) / armCmPerMotorRevolution;
    static final double tickPerDegree = (((placeSpinDiameter * tickPerCentimeter) / 360) / 231) * 260;
    static final double tickPerDegreeAtPlace = (((placeOutSpinDiameter * tickPerCentimeter) / 360) / 231) * 245;//18.69023569
    static final double fullDown = 0.54;
    static final double readDown = 0.48;
    static final double julZero = 0.02;
    static final double clawOpen = 0.3;
    static final double clawClose = 0.55;
    static final int armUp = 60;

    static class Vuforia {
        static final String key = "AcJU0s7/////AAAAmYAF+R25rU5elxvWG+jzOfkipjv/EqlAWEJ12K0WFESUlxRj3trJYggUrl1cGvVLLpEbh56nvKa7zL9mYbG3P6lcCeUUNtXMKwg7QzPfJBG8HRas8zkQPFahOEgvii83GQCKLihiRGi2UFmlK3RkzVi6NU2d/9v8q1lrFfAT2lJQsqyThtcaYKHbDt9DFQzSTwcQLz7Pblr1cM57P7H3T/XovWdMOZBKeXzT8G00c5J4rRtWmq+Pvk83YFxfAiA22sFPepjkt2eke/QeMiCFE6hwRoq/WlGFsKDhMnGDAy16UlexgrjEQn85vclQxkJh6/Rd7IJ6FF0aCp8ewg29ZV14bWxdr1GEyWwh1BJ50YFQ";
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

class Robot {
    private HardwareMap hardwareMap;
    private ElapsedTime runtime;
    private Telemetry telemetry;
    private DcMotor leftDrive, rightDrive, arm;
    private Servo clawRight, clawLeft, julie;
    private ColorSensor julieSensor;
    private ArrayList<Action> actions = new ArrayList<>();
    private ArrayList<String> permanent_messages = new ArrayList<>();
    private int MODE = Stats.MULTI;

    public Robot(HardwareMap hardwareMap, ElapsedTime runtime, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.telemetry = telemetry;
    }

    public void init() {
        hardwareInit();
        collapse();
    }

    public void glue(String message) {
        permanent_messages.add(message);
    }

    public void peel() {
        permanent_messages.clear();
    }

    public void emergency() {
        emergencyStop();
    }

    public void loop() {
        handleActions();
        showMessages();
    }

    public void addAction(Action a) {
    }

    public void addActions(Action[] a) {
    }

    public void addActions(ArrayList<Action> a) {
    }

    public void prepareForAutonomous() {
        resetRobot();
        actions.clear();
        MODE = Stats.LINEBYLINE;
    }

    private void handleActions() {
        if (MODE == Stats.LINEBYLINE) {
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
        } else if (MODE == Stats.MULTI) {
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

    private void showMessages() {
        for (int s = 0; s < permanent_messages.size(); s++) {
            telemetry.addLine(permanent_messages.get(s));
        }
        showStats();
    }

    private void showStats() {
        telemetry.addData("Actions:", actions.size());
        if (isClawOpen()) {
            telemetry.addData("Claw State:", "Opened");
        } else {
            telemetry.addData("Claw State:", "Closed");
        }
        if (MODE == Stats.MULTI) {
            telemetry.addData("Mode", "MultiAction");
        } else {
            telemetry.addData("Mode", "ActionByAction");
        }
        //        telemetry.addData("Seconds", (int) runtime.seconds());
        //        telemetry.addData("Motor", leftDrive.getPower() + " " + rightDrive.getPower());
    }

    private void resetRobot() {
        //        MODE=MULTI;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetArm();
    }

    private void resetArm() {
        arm.setPower(0);
        //        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void emergencyStop() {
        resetRobot();
        actions.clear();
        telemetry.addLine("Emergency Stop!");
    }

    private void hardwareInit() {
        initServos();
        initMotors();
        initColor();
    }

    private void collapse() {
        collapseClaw();
        collapseJulie();
    }

    private void collapseClaw() {
        clawLeft.setPosition(0.6);
        clawRight.setPosition(0.6);
    }

    private void collapseJulie() {
        julie.setPosition(0);
    }

    private void initServos() {
        initClaw();
        initJulie();
    }

    private void initClaw() {
        clawLeft = hardwareMap.get(Servo.class, "s1");
        clawRight = hardwareMap.get(Servo.class, "s2");
        clawLeft.setDirection(Servo.Direction.REVERSE);
        clawRight.setDirection(Servo.Direction.FORWARD);
    }

    private void initJulie() {
        julie = hardwareMap.get(Servo.class, "jul");
        julie.setDirection(Servo.Direction.FORWARD);
    }

    private void initMotors() {
        initLeftDrive();
        initRightDrive();
        initArm();
    }

    private void initLeftDrive() {
        leftDrive = hardwareMap.get(DcMotor.class, "l");
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initRightDrive() {
        rightDrive = hardwareMap.get(DcMotor.class, "r");
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    private void initArm() {
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initColor() {
        julieSensor = hardwareMap.get(ColorSensor.class, "color");
        julieSensor.enableLed(false);
    }

    public void julieUp() {
        julie.setPosition(Stats.julZero);
    }

    public void julieDown() {
        julie.setPosition(Stats.fullDown);
    }

    public void clawOpen() {
        clawLeft.setPosition(Stats.clawOpen);
        clawRight.setPosition(Stats.clawOpen);
    }

    public void clawClose() {
        clawLeft.setPosition(Stats.clawClose);
        clawRight.setPosition(Stats.clawClose);
    }

    public void drive(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void turn(double power, double turn) {
        if (power == 0) {
            leftDrive.setPower(turn);
            rightDrive.setPower(-turn);
        } else {
            if (turn < 0) {
                leftDrive.setPower(power / 8);
                rightDrive.setPower(power);
            } else {
                rightDrive.setPower(power / 8);
                leftDrive.setPower(power);
            }
        }
    }

    public void arm(double speed) {
        arm.setPower(speed);
    }

    public boolean isRobotBusy() {
        return (actions.size() != 0);
    }

    private boolean isClawOpen() {
        return clawLeft.getPosition() < 0.5 && clawRight.getPosition() < 0.5;
    }

    private boolean isJulieDown() {
        return (julie.getPosition() > Stats.julZero + 0.05);
    }

    public Action autonomousCircle(final int direction, final int degree, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + direction * ((int) (Stats.tickPerDegreeAtPlace * degree)));
                leftDrive.setPower(power);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - direction * ((int) (Stats.tickPerDegreeAtPlace * degree)));
                rightDrive.setPower(power);
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
    }

    public Action autonomousTurn(final int direction, final int degree, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                if (direction == RIGHT) {
                    leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (Stats.tickPerDegree * degree));
                    leftDrive.setPower(power);
                } else {
                    rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (Stats.tickPerDegree * degree));
                    rightDrive.setPower(power);
                }
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
    }

    public Action autonomousArmMove(final int cm, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition((int) (arm.getCurrentPosition() + (Stats.tickPerArmCentimeter * cm)));
                arm.setPower(power);
            }

            @Override
            public boolean onLoop() {
                if (!arm.isBusy()) {
                    resetArm();
                    return true;
                }
                return false;
            }
        });
    }

    public Action autonomousJulieUp() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julie.setPosition(Stats.julZero);
            }

            @Override
            public boolean onLoop() {
                return !isJulieDown();
            }
        });
    }

    public Action autonomousJulieDown() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julie.setPosition(Stats.fullDown);
            }

            @Override
            public boolean onLoop() {
                return isJulieDown();
            }
        });
    }

    public Action autonomousJulieScanPosition() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julie.setPosition(Stats.readDown);
            }

            @Override
            public boolean onLoop() {
                return isJulieDown();
            }
        });
    }

    public Action autonomousJulieColorScan(final int team) {
        return new Action(new Action.Execute() {
            ArrayList<Action> miniaction = new ArrayList<>();

            @Override
            public void onSetup() {
                miniaction.add(autonomousWait(500));
                miniaction.add(autonomousCircle(RIGHT, 8, 0.3));
                miniaction.add(autonomousJulieDown());
                julieSensor.enableLed(false);
                boolean isRed = (julieSensor.red() > julieSensor.blue());
                //                permanent_messages.add("Color: "+color.red()+" "+color.green()+" "+color.blue());
                if (team == RED_TEAM) {
                    if (!isRed) {
                        miniaction.add(autonomousCircle(LEFT, 18, 0.2));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(RIGHT, 10, 0.5));
                    } else {
                        miniaction.add(autonomousCircle(RIGHT, 18, 0.2));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(LEFT, 26, 0.5));
                    }
                } else {
                    if (isRed) {
                        miniaction.add(autonomousCircle(LEFT, 18, 0.2));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(RIGHT, 10, 0.5));
                    } else {
                        miniaction.add(autonomousCircle(RIGHT, 18, 0.2));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(LEFT, 26, 0.5));
                    }
                }
            }

            @Override
            public boolean onLoop() {
                if (miniaction.size() != 0) {
                    if (miniaction.get(0).isAlive()) {
                        if (!miniaction.get(0).isSetup()) {
                            miniaction.get(0).setup();
                        } else {
                            miniaction.get(0).loop();
                        }
                    } else {
                        miniaction.remove(0);
                    }
                }
                return (miniaction.size() == 0);
            }
        });
    }

    public Action autonomousVuforia(final Scenario s) {
        return new Action(new Action.Execute() {
            ArrayList<Action> miniaction = new ArrayList<>();
            OpenGLMatrix lastLocation = null;
            VuforiaLocalizer vuforia;
            RelicRecoveryVuMark mark;
            VuforiaTrackable relicTemplate;
            VuforiaTrackables relicTrackables;
            boolean foundMark = false;

            private void initVuforia() {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                parameters.vuforiaLicenseKey = Stats.Vuforia.key;
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
                relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                relicTemplate = relicTrackables.get(0);
                relicTrackables.activate();
            }

            private void searchVuforia() {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    mark = vuMark;
                    foundMark = true;
                    addActions();
                }
            }

            private void addActions() {
                if (mark == RelicRecoveryVuMark.CENTER) {
                    miniaction.addAll(Arrays.asList(s.getC()));
                } else if (mark == RelicRecoveryVuMark.LEFT) {
                    miniaction.addAll(Arrays.asList(s.getL()));
                } else if (mark == RelicRecoveryVuMark.RIGHT) {
                    miniaction.addAll(Arrays.asList(s.getR()));
                }
            }

            @Override
            public void onSetup() {
                initVuforia();
            }

            @Override
            public boolean onLoop() {
                if (foundMark) {
                    if (miniaction.size() != 0) {
                        if (miniaction.get(0).isAlive()) {
                            if (!miniaction.get(0).isSetup()) {
                                miniaction.get(0).setup();
                            } else {
                                miniaction.get(0).loop();
                            }
                        } else {
                            miniaction.remove(0);
                        }
                    }
                    return (miniaction.size() == 0);
                } else {
                    searchVuforia();
                    return false;
                }
            }
        });
    }

    public Action autonomousWait(final int millis) {
        return new Action(new Action.Execute() {
            int targetTime = 0;

            @Override
            public void onSetup() {
                targetTime = (int) runtime.milliseconds() + millis;
            }

            @Override
            public boolean onLoop() {
                return (runtime.milliseconds() >= targetTime);
            }
        });
    }

    public Action autonomousClawOpen() {
        return new Action(new Action.Execute() {
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
    }

    public Action autonomousClawClose() {
        return new Action(new Action.Execute() {
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
    }

    public Action autonomousDrive(final int centimeters, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (Stats.tickPerCentimeter * centimeters));
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (Stats.tickPerCentimeter * centimeters));
                leftDrive.setPower(power);
                rightDrive.setPower(power);
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
    }

    public Action autonomousDone() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
            }

            @Override
            public boolean onLoop() {
                resetRobot();
                MODE = Stats.MULTI;
                actions.clear();
                return true;
            }
        });
    }

    static class Scenario {
        private Action[] L, C, R;

        public Scenario(Action[] L, Action[] C, Action[] R) {
            this.L = L;
            this.C = C;
            this.R = R;
        }

        public Action[] getL() {
            return L;
        }

        public Action[] getC() {
            return C;
        }

        public Action[] getR() {
            return R;
        }
    }
}
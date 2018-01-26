package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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

public class Robot {
    private HardwareMap hardwareMap;
    private ElapsedTime runtime;
    private Telemetry telemetry;
    private DcMotor leftDrive, rightDrive, arm;
    private Servo clawRight, clawLeft, julie;
    private ColorSensor julieSensor;
    private BNO055IMU gyro;
    private ArrayList<Action> actions = new ArrayList<>();
    private ArrayList<String> permanent_messages = new ArrayList<>();
    private int MODE = Stats.MULTI;
    //Vuforia
    private boolean vuforiaStatus = false;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private RelicRecoveryVuMark mark = null;
    private VuforiaTrackable relicTemplate = null;
    private VuforiaTrackables relicTrackables = null;

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
        actions.add(a);
    }

    public void addActions(Action[] a) {
        actions.addAll(Arrays.asList(a));
    }

    public void addActions(ArrayList<Action> a) {
        actions.addAll(a);
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
        showPower();
        telemetry.addData("Gyro 1:", gyro.getAngularOrientation().firstAngle);
        telemetry.addData("Gyro 2:", gyro.getAngularOrientation().secondAngle);
        telemetry.addData("Gyro 3:", gyro.getAngularOrientation().thirdAngle);
        //        telemetry.addData("Seconds", (int) runtime.seconds());
        //        telemetry.addData("Motor", leftDrive.getPower() + " " + rightDrive.getPower());
    }

    private void showPower() {
        if (rightDrive.getPower() == leftDrive.getPower()) {
            telemetry.addData("Power:", rightDrive.getPower());
        } else {
            telemetry.addData("Power-L:", leftDrive.getPower());
            telemetry.addData("Power-R:", leftDrive.getPower());
        }
    }

    private void resetRobot() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetArm();
    }

    private void resetArm() {
        arm.setPower(0);
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
        initGyro();
    }

    private void collapse() {
        clawClose();
        collapseJulie();
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
        clawLeft.setDirection(Servo.Direction.FORWARD);
        clawRight.setDirection(Servo.Direction.REVERSE);
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

    @Deprecated
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

    private void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
    }

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = Stats.Vuforia.key;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        vuforiaStatus = true;
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

    public double getTurnGyro() {
        return gyro.getAngularOrientation().firstAngle;
    }

    @Deprecated
    public Action autonomousCircleByEncoder(final int direction, final double degree, final double power) {
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

    public Action autonomousCircle(final int direction, final double degree) {
        return new Action(new Action.Execute() {
            double gyroBegin, gyroToGo;
            int times = 0;
            Formula noMiss = new Formula() {

                @Override
                public double calculate(double progress) {
                    double marginalError=0.02;
                    progress*=direction;
                    if(progress>1-marginalError&&progress<1+marginalError){
                        return 0;
                    }else {
                        if (progress > 1) {
                            if (progress >= -0.2) {
                                return 0.1;
                            } else {
                                return 0.3;
                            }
                        } else {
                            if (progress >= 0.8) {
                                return -0.1;
                            } else {
                                return -0.3;
                            }
                        }
                    }
                }
            };

            @Override
            public void onSetup() {
                gyroBegin = getTurnGyro();
                if(gyroBegin+direction*degree>180){
                    gyroToGo=gyroBegin+direction*degree-(direction)*360;
                }else{
                    gyroToGo=gyroBegin+direction*degree;
                }
//                gyroToGo = gyroBegin + (degree * direction);
            }

            @Override
            public boolean onLoop() {
                double p = getTurnGyro();
                double progress=(p - gyroBegin) / (gyroToGo - gyroBegin);
                telemetry.addData("Progress:", progress);
                telemetry.addData("ToGo:", gyroToGo);
                telemetry.addData("Start:", gyroBegin);
                telemetry.addData("Current:", p);
                if ((int) gyroToGo == (int) p) {
                    if (times >= 3) {
                        resetRobot();
                    } else {
                        rightDrive.setPower(0);
                        leftDrive.setPower(0);
                        times++;
                    }
                } else {
                    times = 0;
                    double of = noMiss.calculate(progress);
                    rightDrive.setPower(of);
                    leftDrive.setPower(-of);
                }
                double of = noMiss.calculate(progress);
                return (of==0);
            }
        });
    }

    @Deprecated
    public Action autonomousTurn(final int direction, final double degree, final double power) {
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

    public Action autonomousArmMove(final double centimeters, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition((int) (arm.getCurrentPosition() + (Stats.tickPerArmCentimeter * centimeters)));
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
                miniaction.add(autonomousCircle(RIGHT, 8));
                miniaction.add(autonomousJulieDown());
                julieSensor.enableLed(false);
                boolean isRed = (julieSensor.red() > julieSensor.blue());
                //                permanent_messages.add("Color: "+color.red()+" "+color.green()+" "+color.blue());
                if (team == RED_TEAM) {
                    if (!isRed) {
                        miniaction.add(autonomousCircle(LEFT, 18));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(RIGHT, 10));
                    } else {
                        miniaction.add(autonomousCircle(RIGHT, 18));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(LEFT, 26));
                    }
                } else {
                    if (isRed) {
                        miniaction.add(autonomousCircle(LEFT, 18));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(RIGHT, 10));
                    } else {
                        miniaction.add(autonomousCircle(RIGHT, 18));
                        miniaction.add(autonomousJulieUp());
                        miniaction.add(autonomousCircle(LEFT, 26));
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
            boolean foundMark = false;

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
                if (!vuforiaStatus)
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

    public Action autonomousVuforiaInit() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                if (!vuforiaStatus)
                    initVuforia();
            }

            @Override
            public boolean onLoop() {
                return true;
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

    public Action autonomousDrive(final double centimeters, final double power) {
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

    public Action autonomousDriveToGyro(final float angle, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            @Override
            public boolean onLoop() {
                double angletogo = gyro.getAngularOrientation().firstAngle;
                if (angletogo < angle) {
                    drive(power);
                } else if (angletogo > angle) {
                    drive(-power);
                } else if (angletogo == angle) {
                    drive(0);
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

    static class Field {
        static final double field = 365.76;
        static final double balacing = 60;
    }

    interface Formula {
        double calculate(double progress);
    }
}

class Stats {
    static final int MULTI = 1;
    static final int LINEBYLINE = 2;
    static final int RIGHT = 1;
    static final int LEFT = -1;
    static final int BLUE_TEAM = 1;
    static final int RED_TEAM = 2;
    static final double ofthebalance = 30;
    static final double westR = ofthebalance + 44;
    static final double westL = ofthebalance + 19 + 19;
    static final double westCenterForwardCm = ofthebalance + 19;
    static final int eastLeftCm = 54;
    static final int eastCenterCm = 36;
    static final int eastRightCm = 19;
    static final int motorTickPerRevolution = (1440 / 86) * 100;//1674.41
    static final int armMotorTickPerRevolution = (1440);
    static final double PI = 3.14;
    static final double wheelDiameter = 2 * 5.1 * PI;
    static final double distanceBetween = 39;
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
    static final double clawOpen = 0.2;
    static final double clawClose = 0.0;
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
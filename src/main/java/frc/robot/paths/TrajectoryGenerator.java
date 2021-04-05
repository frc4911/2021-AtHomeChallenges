package frc.robot.paths;

import frc.robot.Constants;
import frc.robot.planners.DriveMotionPlanner;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectoryGenerator {
    private boolean skipThisOne = false;
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 60.0; //120.0;
    private static final double kMaxDecel = 72.0; //72.0;
    private static final double kMaxVoltage = 9.0;

    private static final double defSpeed = 40; // bounce 120
    private static final double defAccel = 180; // bounce 180
    private static final double defDecel = 180; // bounce 180

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if(mTrajectorySet == null) {
        double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
    static final Pose2d autoStartingPose = new Pose2d(Constants.kRobotLeftStartingPose.getTranslation().translateBy(new Translation2d(/*-0.5*/0.0, 0.0)), Rotation2d.fromDegrees(-90.0));

    static final Pose2d closeHatchScoringPose = Constants.closeHatchPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 3.5, -2.0)));
    public static final Pose2d farHatchScoringPose = Constants.farHatchPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 5.0, 8.0)));
    static final Pose2d humanLoaderPose = Constants.humanLoaderPosition.transformBy(Pose2d.fromTranslation(new Translation2d(Constants.kRobotHalfLength - 4.0, 2.0)));
    static final Pose2d ballIntakePose = new Pose2d(Constants.autoBallPosition.transformBy(Pose2d.fromTranslation(new Translation2d(Constants.kRobotHalfLength + 9.0, 0.0))).getTranslation(), Rotation2d.fromDegrees(0.0));
    static final Pose2d portScoringPose = Constants.rocketPortPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 6.0, 0.0)));

    static final Pose2d farShipScoringPose = Constants.farShipPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 4.0, 0.0)));
    static final Pose2d midShipScoringPose = Constants.midShipPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 4.0, 0.0)));
    static final Pose2d closeShipScoringPose = Constants.closeShipPosition.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength - 4.0, 6.0)));

    // Test Poses Alex
    private static final Pose2d startingPose = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    private static final Pose2d midPose = new Pose2d(new Translation2d(60.0, 0.0), Rotation2d.fromDegrees(0.0));
    private static final Pose2d mid2Pose = new Pose2d(new Translation2d(60.0, 100.0), Rotation2d.fromDegrees(90.0));
    private static final Pose2d endingPose = new Pose2d(new Translation2d(115.0, 100.0), Rotation2d.fromDegrees(0.0));


    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }


        public final MirroredTrajectory barrelPath;
        public final MirroredTrajectory slalomPath;
        public final MirroredTrajectory slalomPathStraightF;
        public final MirroredTrajectory slalomPathRound;
        public final MirroredTrajectory slalomPathStraightB;
        public final MirroredTrajectory slalomPathEnd;
        public final MirroredTrajectory bouncePathA;
        public final MirroredTrajectory bouncePathB;
        public final MirroredTrajectory bouncePathC;
        public final MirroredTrajectory bouncePathD;

        // public final MirroredTrajectory galacticSearchARedPath1;
        // public final MirroredTrajectory galacticSearchARedPath2;
        // public final MirroredTrajectory galacticSearchARedPath3;
        // public final MirroredTrajectory galacticSearchARedPath4;
        // public final MirroredTrajectory galacticSearchABluePath1;
        // public final MirroredTrajectory galacticSearchABluePath2;
        // public final MirroredTrajectory galacticSearchABluePath3;
        // public final MirroredTrajectory galacticSearchBRedPath1;
        // public final MirroredTrajectory galacticSearchBRedPath2;
        // public final MirroredTrajectory galacticSearchBRedPath3;
        // public final MirroredTrajectory galacticSearchBBluePath1;
        // public final MirroredTrajectory galacticSearchBBluePath2;
        // public final MirroredTrajectory galacticSearchBBluePath3;


        public final MirroredTrajectory powerPortForwardPath;
        public final MirroredTrajectory powerPortBackwardPath;

        // public final MirroredTrajectory testPath3;
        // public final MirroredTrajectory testPath4;

        // public final MirroredTrajectory testPathBrian;

        private TrajectorySet() {
            barrelPath = new MirroredTrajectory(getBarrelPath());
            // System.out.println(testPath.left.toString());
            slalomPath          = new MirroredTrajectory(getSlalomPath());
            slalomPathStraightF = new MirroredTrajectory(getSlalomPathStraightF());
            slalomPathRound     = new MirroredTrajectory(getSlalomPathRound());
            slalomPathStraightB = new MirroredTrajectory(getSlalomPathStraightB());
            slalomPathEnd       = new MirroredTrajectory(getSlalomPathEnd());
            //slalomPath = new MirroredTrajectory(getSlalomPath());
            bouncePathA = new MirroredTrajectory(getBouncePathA());
            bouncePathB = new MirroredTrajectory(getBouncePathB());
            bouncePathC = new MirroredTrajectory(getBouncePathC());
            bouncePathD = new MirroredTrajectory(getBouncePathD());
            // System.out.println(testPath2.left.toString());
            // bouncePath = new MirroredTrajectory(getBouncePath());
            // System.out.println(testPath2.left.toString());
            // testPath3 = new MirroredTrajectory(getTestPath3());
            // System.out.println(testPath3.left.toString());
            // testPath4 = new MirroredTrajectory(getTestPath4());
            // System.out.println(testPath4.left.toString());

            // galacticSearchARedPath1 = new MirroredTrajectory(getGalacticSearchARedPath1());
            // galacticSearchARedPath2 = new MirroredTrajectory(getGalacticSearchARedPath2());
            // galacticSearchARedPath3 = new MirroredTrajectory(getGalacticSearchARedPath3());
            // galacticSearchARedPath4 = new MirroredTrajectory(getGalacticSearchARedPath4());

            // galacticSearchABluePath1 = new MirroredTrajectory(getGalacticSearchABluePath1());
            // galacticSearchABluePath2 = new MirroredTrajectory(getGalacticSearchABluePath2());
            // galacticSearchABluePath3 = new MirroredTrajectory(getGalacticSearchABluePath3());

            // galacticSearchBRedPath1 = new MirroredTrajectory(getGalacticSearchBRedPath1());
            // galacticSearchBRedPath2 = new MirroredTrajectory(getGalacticSearchBRedPath2());
            // galacticSearchBRedPath3 = new MirroredTrajectory(getGalacticSearchBRedPath3());

            // galacticSearchBBluePath1 = new MirroredTrajectory(getGalacticSearchBBluePath1());
            // galacticSearchBBluePath2 = new MirroredTrajectory(getGalacticSearchBBluePath2());
            // galacticSearchBBluePath3 = new MirroredTrajectory(getGalacticSearchBBluePath3());

            powerPortForwardPath = new MirroredTrajectory(getPowerPortForwardPath());
            powerPortBackwardPath = new MirroredTrajectory(getPowerPortBackwardPath());

            // testPathBrian = new MirroredTrajectory(getBrianPath());
            // System.out.println("Brian's path");
            // System.out.println(testPathBrian.left.toString());
            // System.out.println("Brian's path end");

        }

        /*************************************Rendezvous Paths*****************************************/

        /*************************************Test Paths*****************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getBackAwayFromLinePath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-60.0, 0.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0, 60.0, 60.0, kMaxVoltage, 120.0, 1);
        }

        private class ThreePoints{
            double x;
            double y;
            double a;

            public ThreePoints(double x,double y, double a){
                this.x = x;
                this.y = y;
                this.a = a;
            }
        }

        private void addStraightSegment(ArrayList<ThreePoints> tpal, boolean addStart, double x0, double y0, double xf, double yf ){
            double stepLength = 12;
            double angleRads = Math.atan2(yf-y0,xf-x0);
            double angleDegrees = convertToDegrees(angleRads);
            double length = Math.sqrt(Math.pow(xf-x0,2)+Math.pow(yf-y0,2));
            double steps = Math.round(length/stepLength);
            System.out.println("addStraightSegment steps:"+steps+" angle:"+angleDegrees+" length:"+length);
            int startI = 1;
            if (addStart){
                startI=0;
            }
            for (int i=startI; i<steps; i++){
                tpal.add(new ThreePoints(x0+i*(xf-x0)/steps,y0+i*(yf-y0)/steps,angleDegrees));
            }
            tpal.add(new ThreePoints(xf,yf,angleDegrees));
        }

        private void addArcSegment(ArrayList<ThreePoints> tpal, boolean addStart, double startx, double starty, double centerx, double centery, double endAngleRads ){
            double arcLengthStep = 30; //degrees
            double endAngleDegrees = convertToDegrees(endAngleRads);
            double radius = Math.sqrt(Math.pow(centerx-startx,2)+Math.pow(centery-starty,2));
            double startAngleRads = Math.atan2(starty-centery,startx-centerx);
            double startAngleDegrees = convertToDegrees(startAngleRads);
            double arcLengthDegrees = endAngleDegrees-startAngleDegrees;
            System.out.println("arcLengthDegrees " + arcLengthDegrees);
            double leftright = 90;
            int steps = Math.round(Math.abs((int)(arcLengthDegrees/arcLengthStep)));
            double stepSize = arcLengthDegrees/(double)steps;
            if(steps == 1){
                stepSize = arcLengthDegrees;
            }
            System.out.println("addArcSegment steps:"+steps+" stepSize:"+stepSize+" startAngle:"+startAngleDegrees+" arcLength:"+arcLengthDegrees+" radius"+radius);
            int startI = 1;
            if (addStart){
                startI=0;
            }
            if(stepSize<0){
                leftright = leftright*-1;
            }
            for (int i=startI; i<=steps; i++){
                double angleDegrees = startAngleDegrees+((double)i)*stepSize;
                double angleRads = convertToRads(angleDegrees);
                ThreePoints tp = new ThreePoints(centerx+radius*Math.cos(angleRads), centery+radius*Math.sin(angleRads), angleDegrees+leftright);
                tpal.add(tp);
                System.out.println("Arc coords - "+tp.x+","+tp.y+","+tp.a+",("+angleDegrees+")");
            }
        }

        private double convertToRads(double angleDegrees){
            final double conversion = Math.PI/180.0;
            return angleDegrees*conversion;
        }

        private double convertToDegrees(double angleRads){
            final double conversion = 180.0/Math.PI;
            return angleRads*conversion;
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBarrelPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            double startx = 46;
            double starty = 90;

            waypoints.add(new Pose2d(new Translation2d(startx-startx, starty-starty), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(165-startx, 90-starty), Rotation2d.fromDegrees(45)));
            waypoints.add(new Pose2d(new Translation2d(150-startx, 150-starty), Rotation2d.fromDegrees(200)));
            waypoints.add(new Pose2d(new Translation2d(120-startx, 70-starty), Rotation2d.fromDegrees(340)));
            waypoints.add(new Pose2d(new Translation2d(255-startx, 70-starty), Rotation2d.fromDegrees(330)));
            waypoints.add(new Pose2d(new Translation2d(240-startx, -5-starty), Rotation2d.fromDegrees(200)));
            waypoints.add(new Pose2d(new Translation2d(215-startx, 90-starty), Rotation2d.fromDegrees(45)));

            waypoints.add(new Pose2d(new Translation2d(270-startx, 120-starty), Rotation2d.fromDegrees(15)));

            waypoints.add(new Pose2d(new Translation2d(300-startx, 120-starty), Rotation2d.fromDegrees(315)));

            waypoints.add(new Pose2d(new Translation2d(300-startx, 75-starty), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(startx-60-startx, starty-30-starty), Rotation2d.fromDegrees(180)));

            return generateTrajectory(   false, waypoints, Arrays.asList(),        125,        180,        180,           12, 120, 1);
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getBarrelPath() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();

        //     if (false){
        //         waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)));
        //         waypoints.add(new Pose2d(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0)));
        //     }
        //     else {
        //         double exitAngleRads;
        //         double enterAngleRads;
        //         double rc = 28/2;
        //         double radius = 30;

        //         double startx = 60-rc;
        //         double starty = 120-radius; 

        //         double d0x = 145; //150
        //         double d0y = 120;

        //         double d1x = 222; //240
        //         double d1y = 45; //60

        //         double d2x = 292; //300
        //         double d2y = 110; //120

        //         double endx = startx-30;
        //         double endy = starty-30;

        //         enterAngleRads = convertToRads(-90);
        //         addStraightSegment(tpal, true, startx,                              starty, 
        //                                     d0x+radius*Math.cos(enterAngleRads), d0y+radius*Math.sin(enterAngleRads));
        //         exitAngleRads = convertToRads(265);
        //         addArcSegment(tpal,     false, d0x+radius*Math.cos(enterAngleRads), d0y+radius*Math.sin(enterAngleRads),
        //                                     d0x,                                 d0y,                 exitAngleRads);
        //         enterAngleRads = convertToRads(75);
        //         addStraightSegment(tpal,false, d0x+radius*Math.cos(exitAngleRads),  d0y+radius*Math.sin(exitAngleRads),
        //                                     d1x+radius*Math.cos(enterAngleRads), d1y+radius*Math.sin(enterAngleRads));
        //         exitAngleRads = convertToRads(-225);
        //         addArcSegment(tpal,      false,d1x+radius*Math.cos(enterAngleRads), d1y+radius*Math.sin(enterAngleRads),
        //                                     d1x,                                 d1y,                 exitAngleRads);
        //         enterAngleRads = convertToRads(135);
        //         addStraightSegment(tpal,false, d1x+radius*Math.cos(exitAngleRads),  d1y+radius*Math.sin(exitAngleRads),
        //                                     d2x+radius*Math.cos(enterAngleRads), d2y+radius*Math.sin(enterAngleRads));
        //         exitAngleRads = convertToRads(-90);
        //         addArcSegment(tpal,      false,d2x+radius*Math.cos(enterAngleRads), d2y+radius*Math.sin(enterAngleRads),
        //                                     d2x,                                 d2y,                 exitAngleRads);

        //         addStraightSegment(tpal, false,d2x+radius*Math.cos(exitAngleRads),  d2y+radius*Math.sin(exitAngleRads),
        //                                     endx,                              endy);

        //         System.out.println("Barrel Path Coordinates - ");
        //         for (ThreePoints tp : tpal){
        //             System.out.println(tp.x+","+tp.y+","+tp.a);
        //             waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
        //         }

        //         // double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //         // if(speed == -1){
        //         //     SmartDashboard.putNumber("Autopath Speed", 20);
        //         //     speed = 20;
        //         // }
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        120,        40,        100,           9, 120, 1);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSlalomPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            ArrayList<ThreePoints> tpal = new ArrayList<>();
            
            double exitAngleRads;
            double enterAngleRads;
            double rc = 28/2;
            double radius = 30; //30
            
            double startx = 60-rc;
            double starty = 120+30;

            double d0x = 60;
            double d1x = 120;
            double d2x = 235; //240
            double d3x = 305; //300

            double dy = 120;

            double endx = startx-120;
            double endy = 120-30;

            //1
            enterAngleRads = convertToRads(118);
            exitAngleRads = convertToRads(0);
            addArcSegment(tpal, true,      d0x-rc,                              dy+radius*Math.sin(enterAngleRads),
                                            d0x,                                 dy,                 exitAngleRads);
            enterAngleRads = convertToRads(-179);
            exitAngleRads = convertToRads(-90);
            addArcSegment(tpal, false,      d1x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
                                            d1x,                                 dy,                 exitAngleRads);

                                            //Split
            //2                            
            // enterAngleRads = convertToRads(-90);
            // addStraightSegment(tpal, false, d1x+radius*Math.cos(exitAngleRads),  dy+radius*Math.sin(exitAngleRads),
            //                                 d2x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads));

                                            //Split
            //3                                
            // exitAngleRads = convertToRads(0);
            // addArcSegment(tpal, false,      d2x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
            //                                 d2x,                                 dy,                 exitAngleRads);                             
            // enterAngleRads = convertToRads(150);
            // exitAngleRads = convertToRads(0);
            // addArcSegment(tpal, true,       d3x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
            //                                 d3x,                                 dy,                 exitAngleRads);
            // radius += 5;
            // enterAngleRads = convertToRads(0);
            // exitAngleRads = convertToRads(-179);
            // addArcSegment(tpal, false,       d3x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
            //                                 d3x,                                 dy,                 exitAngleRads);
            // //? radius -= 5;
            // enterAngleRads = convertToRads(0);
            // exitAngleRads = convertToRads(90);
            // addArcSegment(tpal, false,      d2x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
            //                                 d2x,                                 dy,                 exitAngleRads);

                                            //Split
            //4                                
            // enterAngleRads = convertToRads(90);
            // addStraightSegment(tpal, false, d2x+radius*Math.cos(exitAngleRads),  dy+radius*Math.sin(exitAngleRads),
            //                                 d1x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads));

                                            //Split
            //5                                
            // exitAngleRads = convertToRads(179);
            // dy -= 15;
            // endy -=15;
            // addArcSegment(tpal, false,      d1x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
            //                                 d1x,                                 dy,                 exitAngleRads);
            // enterAngleRads = convertToRads(0);
            // exitAngleRads = convertToRads(-90);
            // addArcSegment(tpal, false,      d1x-radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
            //                                 d0x,                                 dy,                 exitAngleRads);
            // addStraightSegment(tpal, false, d0x-radius*Math.cos(exitAngleRads),  dy+radius*Math.sin(exitAngleRads),
            //                                 endx,                                endy);

            // double speed = SmartDashboard.getNumber("Autopath Speed", -1);
            // if(speed == -1){
            //     SmartDashboard.putNumber("Autopath Speed", 20);
            //     speed = 20;
            // }

            System.out.println("Slalom Path Coordinates - ");
            for (ThreePoints tp : tpal){
                System.out.println(tp.x+","+tp.y+","+tp.a);
                waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
            }
            double speed = 40;
             return generateTrajectory(   false, waypoints, Arrays.asList(), speed,        40,        40,           12, speed, 1);
            // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSlalomPathStraightF() {
            List<Pose2d> waypoints = new ArrayList<>();
            ArrayList<ThreePoints> tpal = new ArrayList<>();
            
            double exitAngleRads;
            double enterAngleRads;
            double rc = 28/2;
            double radius = 30; //30

            double startx = 60-rc;
            double starty = 120+30;

            double d0x = 60;
            double d1x = 120;
            double d2x = 235; //240

            double dy = 120;

            exitAngleRads = convertToRads(-90);
            enterAngleRads = convertToRads(-90);
            addStraightSegment(tpal, true, d1x+radius*Math.cos(exitAngleRads),  dy+radius*Math.sin(exitAngleRads),
                                            d2x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads));

            //System.out.println("Slalom Path Coordinates - ");
            for (ThreePoints tp : tpal){
                System.out.println(tp.x+","+tp.y+","+tp.a);
                waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
            }
            double speed = 40;
            return generateTrajectory(   false, waypoints, Arrays.asList(), speed,        40,        40,           12, speed, 1);
            // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);

        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSlalomPathRound() {
            List<Pose2d> waypoints = new ArrayList<>();
            ArrayList<ThreePoints> tpal = new ArrayList<>();
            
            double exitAngleRads;
            double enterAngleRads;
            double rc = 28/2;
            double radius = 30; //30
            
            double startx = 60-rc;
            double starty = 120+30;

            double d2x = 235; //240
            double d3x = 305; //300

            double dy = 120;

            enterAngleRads = convertToRads(-90);
            exitAngleRads = convertToRads(0);
            addArcSegment(tpal, true,      d2x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
                                            d2x,                                 dy,                 exitAngleRads);                             
            enterAngleRads = convertToRads(150);
            exitAngleRads = convertToRads(0);
            addArcSegment(tpal, true,       d3x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
                                            d3x,                                 dy,                 exitAngleRads);
            radius += 5;
            enterAngleRads = convertToRads(0);
            exitAngleRads = convertToRads(-179);
            addArcSegment(tpal, false,       d3x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
                                            d3x,                                 dy,                 exitAngleRads);
            //? radius -= 5;
            enterAngleRads = convertToRads(0);
            exitAngleRads = convertToRads(90);
            addArcSegment(tpal, false,      d2x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
                                            d2x,                                 dy,                 exitAngleRads);

            //System.out.println("Slalom Path Coordinates - ");
            for (ThreePoints tp : tpal){
                System.out.println(tp.x+","+tp.y+","+tp.a);
                waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
            }
            double speed = 40;
            return generateTrajectory(   false, waypoints, Arrays.asList(), speed,        40,        40,           12, speed, 1);
            // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);

        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSlalomPathStraightB() {
            List<Pose2d> waypoints = new ArrayList<>();
            ArrayList<ThreePoints> tpal = new ArrayList<>();
            
            double exitAngleRads;
            double enterAngleRads;
            double rc = 28/2;
            double radius = 30; //30

            double startx = 60-rc;
            double starty = 120+30;

            double d0x = 60;
            double d1x = 120;
            double d2x = 235; //240

            double dy = 120;

            enterAngleRads = convertToRads(90);
            exitAngleRads = convertToRads(90);
            addStraightSegment(tpal, true, d2x+radius*Math.cos(exitAngleRads),  dy+radius*Math.sin(exitAngleRads),
                                            d1x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads));

            //System.out.println("Slalom Path Coordinates - ");
            for (ThreePoints tp : tpal){
                System.out.println(tp.x+","+tp.y+","+tp.a);
                waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
            }
            double speed = 40;
            return generateTrajectory(   false, waypoints, Arrays.asList(), speed,        40,        40,           12, speed, 1);
            // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);

        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSlalomPathEnd() {
            List<Pose2d> waypoints = new ArrayList<>();
            ArrayList<ThreePoints> tpal = new ArrayList<>();
            
            double exitAngleRads;
            double enterAngleRads;
            double rc = 28/2;
            double radius = 30; //30
            
            double startx = 60-rc;
            double starty = 120+30;

            double d0x = 60;
            double d1x = 120;

            double dy = 120;

            double endx = startx-120;
            double endy = 120-30;

            enterAngleRads = convertToRads(90);
            exitAngleRads = convertToRads(179);
            dy -= 15;
            endy -=15;
            addArcSegment(tpal, true,      d1x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
                                            d1x,                                 dy,                 exitAngleRads);
            enterAngleRads = convertToRads(0);
            exitAngleRads = convertToRads(-90);
            addArcSegment(tpal, false,      d1x-radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
                                            d0x,                                 dy,                 exitAngleRads);
            addStraightSegment(tpal, false, d0x-radius*Math.cos(exitAngleRads),  dy+radius*Math.sin(exitAngleRads),
                                            endx,                                endy);

            //System.out.println("Slalom Path Coordinates - ");
            for (ThreePoints tp : tpal){
                System.out.println(tp.x+","+tp.y+","+tp.a);
                waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
            }
            double speed = 40;
            return generateTrajectory(   false, waypoints, Arrays.asList(), speed,        40,        40,           12, speed, 1);
            // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);

        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBouncePathA() {
            List<Pose2d> waypoints = new ArrayList<>();
            double startx = 46;
            double starty = 90;
            waypoints.add(new Pose2d(new Translation2d(startx-startx, starty-starty), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(90-startx, 40-starty), Rotation2d.fromDegrees(-89)));
            return generateTrajectory(
                false, // reversed
                waypoints, // waypoints
                Arrays.asList(), // constraints
                defSpeed, // max vel
                defAccel, // max accel
                defDecel, // max decel
                12, // max voltage
                defSpeed, // default vel
                1 // slowdown chunks?
            );
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBouncePathB() {
            List<Pose2d> waypoints = new ArrayList<>();
            double startx = 46;
            double starty = 90;
            waypoints.add(new Pose2d(new Translation2d(90 - startx, 40-starty), Rotation2d.fromDegrees(89)));
            waypoints.add(new Pose2d(new Translation2d(120-startx, 130-starty), Rotation2d.fromDegrees(60)));
            waypoints.add(new Pose2d(new Translation2d(180-startx, 120-starty), Rotation2d.fromDegrees(-89)));
            waypoints.add(new Pose2d(new Translation2d(180-startx, 33-starty), Rotation2d.fromDegrees(-89)));
            return generateTrajectory(   false, waypoints, Arrays.asList(), defSpeed,        defAccel,        defDecel,           12, defSpeed, 1);
        }
        // had a theory that a single path may time out if it extends 15 seconds. Unverified.
        private Trajectory<TimedState<Pose2dWithCurvature>> getBouncePathC() {
            List<Pose2d> waypoints = new ArrayList<>();
            double startx = 46;
            double starty = 90;
            waypoints.add(new Pose2d(new Translation2d(180 - startx, 33-starty), Rotation2d.fromDegrees(89)));
            waypoints.add(new Pose2d(new Translation2d(180-startx, 105-starty), Rotation2d.fromDegrees(89)));
            waypoints.add(new Pose2d(new Translation2d(270-startx, 105-starty), Rotation2d.fromDegrees(-91)));
            waypoints.add(new Pose2d(new Translation2d(270-startx, 33-starty), Rotation2d.fromDegrees(-89)));
            return generateTrajectory(   false, waypoints, Arrays.asList(), defSpeed,        defAccel,        defDecel,           12, defSpeed, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getBouncePathD() {
            List<Pose2d> waypoints = new ArrayList<>();
            double startx = 46;
            double starty = 90;
            waypoints.add(new Pose2d(new Translation2d(270 - startx, 33-starty), Rotation2d.fromDegrees(89)));
            waypoints.add(new Pose2d(new Translation2d(300-startx, 90-starty), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(315-startx, 90-starty), Rotation2d.fromDegrees(0)));
            return generateTrajectory(   false, waypoints, Arrays.asList(), defSpeed,        defAccel,        defDecel,           12, defSpeed, 1);
        }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getBouncePathA() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();         
        //     if (skipThisOne){
        //         waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)));
        //         waypoints.add(new Pose2d(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0)));
        //     }
        //     else {
        //         double exitAngleRads;
        //         double enterAngleRads;
        //         double rc = 28/2;
        //         double radius = 28; //30

        //         double d0x = 60; //B2
        //         double d0y = 60; //B2

        //         double startx = d0x+radius*Math.cos(118);
        //         double starty = d0y+radius*Math.sin(118);

        //         double a3x = 90;
        //         double a3y = 30; 

        //         double d1x = 150; //D5
        //         double d1y = 120; //D5

        //         double a6x = 180;
        //         double a6y = 30;

        //         double d2x = 210; //D7
        //         double d2y = 120; //D7

        //         double d3x = 240; //D8
        //         double d3y = 120; //D8

        //         double a9x = 270;
        //         double a9y = 30; 

        //         double d4x = 300; //B10
        //         double d4y = 60;  //B10
            
        //         double endx = 330;
        //         double endy = 90;

        //         // start leg A
        //         enterAngleRads = convertToRads(118);
        //         exitAngleRads = convertToRads(0);
        //         addArcSegment(tpal,      true,  d0x+radius*Math.cos(enterAngleRads), d0y+radius*Math.sin(enterAngleRads), d0x, d0y, exitAngleRads);
        //         addStraightSegment(tpal, false, d0x+radius*Math.cos(exitAngleRads),  d0y+radius*Math.sin(exitAngleRads),  a3x, a3y+rc-2);

        //         System.out.println("Bounce Path A Coordinates - ");
        //         for (ThreePoints tp : tpal){
        //             System.out.println(tp.x+","+tp.y+","+tp.a);
        //             // waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
        //         }
        //         double x = 46.85479624;
        //         double y = 84.7225326;

        //         // 46.8547962484.722532628
        //         // 65.5024650787.45401388-11.33333333
        //         // 81.657204677.74726709-50.66666667
        //         // 8860-90
        //         // 8951-83.65980825
        //         // 9042-83.65980825

        //         waypoints.add(new Pose2d(new Translation2d(46.85479624-x,84.7225326-y), Rotation2d.fromDegrees(28)));
        //         waypoints.add(new Pose2d(new Translation2d(65.50246507-x,87.45401388-y), Rotation2d.fromDegrees(-11.33333333)));
        //         waypoints.add(new Pose2d(new Translation2d(81.6572046-x,77.74726709-y), Rotation2d.fromDegrees(-50.66666667)));
        //         waypoints.add(new Pose2d(new Translation2d(88-x,60-y), Rotation2d.fromDegrees(-90)));
        //         waypoints.add(new Pose2d(new Translation2d(89-x,51-y), Rotation2d.fromDegrees(-83.65980825)));
        //         waypoints.add(new Pose2d(new Translation2d(90-x,36-y), Rotation2d.fromDegrees(-83.65980825)));
        //         // double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //         // if(speed == -1){
        //         //     SmartDashboard.putNumber("Autopath Speed", 20);
        //         //     speed = 20;
        //         // }
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        20,        80,        80,           9, 20, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getBouncePathB() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();         
        //     if (skipThisOne){
        //         waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)));
        //         waypoints.add(new Pose2d(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0)));
        //     }
        //     else {
        //         double exitAngleRads;
        //         double enterAngleRads;
        //         double rc = 28/2;
        //         double radius = 28; //30

        //         double d0x = 60; //B2
        //         double d0y = 60; //B2

        //         double startx = d0x+radius*Math.cos(118);
        //         double starty = d0y+radius*Math.sin(118);

        //         double a3x = 90;
        //         double a3y = 30; 

        //         double d1x = 150; //D5
        //         double d1y = 120; //D5

        //         double a6x = 180;
        //         double a6y = 30;

        //         double d2x = 210; //D7
        //         double d2y = 120; //D7

        //         double d3x = 240; //D8
        //         double d3y = 120; //D8

        //         double a9x = 270;
        //         double a9y = 30; 

        //         double d4x = 300; //B10
        //         double d4y = 60;  //B10
            
        //         double endx = 330;
        //         double endy = 90;

        //         // start leg B
        //         enterAngleRads = convertToRads(180);
        //         addStraightSegment(tpal, false, a3x, a3y-rc+2, d1x+radius*Math.cos(enterAngleRads), d1y+radius*Math.sin(enterAngleRads));
        //         exitAngleRads = convertToRads(0);
        //         addArcSegment(tpal,      false, d1x+radius*Math.cos(enterAngleRads), d1y+radius*Math.sin(enterAngleRads),d1x, d1y, exitAngleRads);
        //         addStraightSegment(tpal, false, d1x+radius*Math.cos(exitAngleRads),  d1y+radius*Math.sin(exitAngleRads), a6x, a6y);

        //         System.out.println("Bounce Path B Coordinates - ");
        //         for (ThreePoints tp : tpal){
        //             System.out.println(tp.x+","+tp.y+","+tp.a);
        //             //waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
        //         }
        //         double x = 93.55555;
        //         double y = 29.33333;
        //         waypoints.add(new  Pose2d(new  Translation2d(93.55555556-x,29.33333333-y),  Rotation2d.fromDegrees(72.58202921)));
        //         waypoints.add(new  Pose2d(new  Translation2d(97.11111111-x,40.66666667-y),  Rotation2d.fromDegrees(72.58202921)));
        //         waypoints.add(new  Pose2d(new  Translation2d(100.6666667-x,52-y),  Rotation2d.fromDegrees(72.58202921)));
        //         waypoints.add(new  Pose2d(new  Translation2d(104.2222222-x,63.33333333-y),  Rotation2d.fromDegrees(72.58202921)));
        //         waypoints.add(new  Pose2d(new  Translation2d(107.7777778-x,74.66666667-y),  Rotation2d.fromDegrees(72.58202921)));
        //         waypoints.add(new  Pose2d(new  Translation2d(111.3333333-x,86-y),  Rotation2d.fromDegrees(72.58202921)));
        //         waypoints.add(new  Pose2d(new  Translation2d(114.8888889-x,97.33333333-y),  Rotation2d.fromDegrees(72.58202921)));
        //         waypoints.add(new  Pose2d(new  Translation2d(118.4444444-x,108.6666667-y),  Rotation2d.fromDegrees(72.58202921)));
        //         waypoints.add(new  Pose2d(new  Translation2d(122-x,120-y),  Rotation2d.fromDegrees(72.58202921)));
        //         waypoints.add(new  Pose2d(new  Translation2d(125.7512887-x,134-y),  Rotation2d.fromDegrees(60)));
        //         waypoints.add(new  Pose2d(new  Translation2d(136-x,144.2487113-y),  Rotation2d.fromDegrees(30)));
        //         waypoints.add(new  Pose2d(new  Translation2d(150-x,148-y),  Rotation2d.fromDegrees(0)));
        //         waypoints.add(new  Pose2d(new  Translation2d(164-x,144.2487113-y),  Rotation2d.fromDegrees(-30)));
        //         waypoints.add(new  Pose2d(new  Translation2d(174.2487113-x,134-y),  Rotation2d.fromDegrees(-60)));
        //         waypoints.add(new  Pose2d(new  Translation2d(178-x,120-y),  Rotation2d.fromDegrees(-90)));
        //         waypoints.add(new  Pose2d(new  Translation2d(178.25-x,108.75-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(178.5-x,97.5-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(178.75-x,86.25-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(179-x,75-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(179.25-x,63.75-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(179.5-x,52.5-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(179.75-x,41.25-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(180-x,30-y),  Rotation2d.fromDegrees(-88.72696998)));


        //         // double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //         // if(speed == -1){
        //         //     SmartDashboard.putNumber("Autopath Speed", 20);
        //         //     speed = 20;
        //         // }
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        20,        80,        80,           9, 20, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getBouncePathC() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();         
        //     if (skipThisOne){
        //         waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)));
        //         waypoints.add(new Pose2d(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0)));
        //     }
        //     else {
        //         double exitAngleRads;
        //         double enterAngleRads;
        //         double rc = 28/2;
        //         double radius = 28; //30

        //         double d0x = 60; //B2
        //         double d0y = 60; //B2

        //         double startx = d0x+radius*Math.cos(118);
        //         double starty = d0y+radius*Math.sin(118);

        //         double a3x = 90;
        //         double a3y = 30; 

        //         double d1x = 150; //D5
        //         double d1y = 120; //D5

        //         double a6x = 180;
        //         double a6y = 30;

        //         double d2x = 210; //D7
        //         double d2y = 120; //D7

        //         double d3x = 240; //D8
        //         double d3y = 120; //D8

        //         double a9x = 270;
        //         double a9y = 30; 

        //         double d4x = 300; //B10
        //         double d4y = 60;  //B10
            
        //         double endx = 330;
        //         double endy = 90;

        //         // start leg C
        //         enterAngleRads = convertToRads(180);
        //         addStraightSegment(tpal, false, a6x, a6y, d2x+radius*Math.cos(enterAngleRads), d2y+radius*Math.sin(enterAngleRads));
        //         exitAngleRads = convertToRads(90);
        //         addArcSegment(tpal, false, d2x+radius*Math.cos(enterAngleRads),  d2y+radius*Math.sin(enterAngleRads), d2x, d2y, exitAngleRads);
        //         enterAngleRads = convertToRads(90);
                
        //         addStraightSegment(tpal, false, d2x+radius*Math.cos(exitAngleRads), d2y+radius*Math.sin(exitAngleRads), d3x+radius*Math.cos(enterAngleRads), d3y+radius*Math.sin(enterAngleRads));
                
        //         exitAngleRads = convertToRads(0);
        //         addArcSegment(tpal, false,      d3x+radius*Math.cos(enterAngleRads), d3y+radius*Math.sin(enterAngleRads), d3x,d3y, exitAngleRads);
        //         addStraightSegment(tpal, false, d3x+radius*Math.cos(exitAngleRads),  d3y+radius*Math.sin(exitAngleRads), a9x, a9y);

        //         System.out.println("Bounce Path C Coordinates - ");
        //         for (ThreePoints tp : tpal){
        //             System.out.println(tp.x+","+tp.y+","+tp.a);
        //             //waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
        //         }
        //         double x = startx;
        //         double y = starty;
        //         waypoints.add(new  Pose2d(new  Translation2d(180.25-x,41.25-y),  Rotation2d.fromDegrees(88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(180.5-x,52.5-y),  Rotation2d.fromDegrees(88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(180.75-x,63.75-y),  Rotation2d.fromDegrees(88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(181-x,75-y),  Rotation2d.fromDegrees(88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(181.25-x,86.25-y),  Rotation2d.fromDegrees(88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(181.5-x,97.5-y),  Rotation2d.fromDegrees(88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(181.75-x,108.75-y),  Rotation2d.fromDegrees(88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(182-x,120-y),  Rotation2d.fromDegrees(88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(185.7512887-x,134-y),  Rotation2d.fromDegrees(60)));
        //         waypoints.add(new  Pose2d(new  Translation2d(196-x,144.2487113-y),  Rotation2d.fromDegrees(30)));
        //         waypoints.add(new  Pose2d(new  Translation2d(210-x,148-y),  Rotation2d.fromDegrees(0)));
        //         waypoints.add(new  Pose2d(new  Translation2d(220-x,148-y),  Rotation2d.fromDegrees(0)));
        //         waypoints.add(new  Pose2d(new  Translation2d(230-x,148-y),  Rotation2d.fromDegrees(0)));
        //         waypoints.add(new  Pose2d(new  Translation2d(240-x,148-y),  Rotation2d.fromDegrees(0)));
        //         waypoints.add(new  Pose2d(new  Translation2d(254-x,144.2487113-y),  Rotation2d.fromDegrees(-30)));
        //         waypoints.add(new  Pose2d(new  Translation2d(264.2487113-x,134-y),  Rotation2d.fromDegrees(-60)));
        //         waypoints.add(new  Pose2d(new  Translation2d(268-x,120-y),  Rotation2d.fromDegrees(-90)));
        //         waypoints.add(new  Pose2d(new  Translation2d(268.25-x,108.75-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(268.5-x,97.5-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(268.75-x,86.25-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(269-x,75-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(269.25-x,63.75-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(269.5-x,52.5-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(269.75-x,41.25-y),  Rotation2d.fromDegrees(-88.72696998)));
        //         waypoints.add(new  Pose2d(new  Translation2d(270-x,30-y),  Rotation2d.fromDegrees(-88.72696998)));

        //         // double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //         // if(speed == -1){
        //         //     SmartDashboard.putNumber("Autopath Speed", 20);
        //         //     speed = 20;
        //         // }
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        20,        80,        80,           9, 20, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getBouncePathD() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();         
        //     if (skipThisOne){
        //         waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)));
        //         waypoints.add(new Pose2d(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0)));
        //     }
        //     else {
        //         double exitAngleRads;
        //         double enterAngleRads;
        //         double rc = 28/2;
        //         double radius = 28; //30

        //         double d0x = 60; //B2
        //         double d0y = 60; //B2

        //         double startx = d0x+radius*Math.cos(118);
        //         double starty = d0y+radius*Math.sin(118);

        //         double a3x = 90;
        //         double a3y = 30; 

        //         double d1x = 150; //D5
        //         double d1y = 120; //D5

        //         double a6x = 180;
        //         double a6y = 30;

        //         double d2x = 210; //D7
        //         double d2y = 120; //D7

        //         double d3x = 240; //D8
        //         double d3y = 120; //D8

        //         double a9x = 270;
        //         double a9y = 30; 

        //         double d4x = 300; //B10
        //         double d4y = 60;  //B10
            
        //         double endx = 330;
        //         double endy = 90;


        //         // start leg D

        //         enterAngleRads = convertToRads(180);
        //         addStraightSegment(tpal, false, a9x, a9y,d4x+radius*Math.cos(enterAngleRads), d4y+radius*Math.sin(enterAngleRads));
        //         exitAngleRads = convertToRads(90);
        //         addArcSegment(tpal, false, d4x+radius*Math.cos(enterAngleRads), d4y+radius*Math.sin(enterAngleRads), d4x, d4y,exitAngleRads);

        //         System.out.println("Bounce Path D Coordinates - ");
        //         for (ThreePoints tp : tpal){
        //             System.out.println(tp.x+","+tp.y+","+tp.a);
        //             //waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
        //         }
        //         double x = startx;
        //         double y = starty;
        //         waypoints.add(new Pose2d(new Translation2d(270.6666667-x,40-y), Rotation2d.fromDegrees(86.18592517)));
        //         waypoints.add(new  Pose2d(new  Translation2d(271.3333333-x,50-y),  Rotation2d.fromDegrees(86.18592517)));
        //         waypoints.add(new  Pose2d(new  Translation2d(272-x,60-y),  Rotation2d.fromDegrees(86.18592517)));
        //         waypoints.add(new  Pose2d(new  Translation2d(275.7512887-x,74-y),  Rotation2d.fromDegrees(60)));
        //         waypoints.add(new  Pose2d(new  Translation2d(286-x,84.24871131-y),  Rotation2d.fromDegrees(30)));
        //         waypoints.add(new  Pose2d(new  Translation2d(300-x,88-y),  Rotation2d.fromDegrees(0)));

        //         // double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //         // if(speed == -1){
        //         //     SmartDashboard.putNumber("Autopath Speed", 20);
        //         //     speed = 20;
        //         // }
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        20,        80,        80,           9, 20, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }

        double xscale = 1; 
        double yscale = 1;
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchARedPath1() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

            
        //     //TEST
        //     // double startx = 30+rc;
        //     // double starty = 60; 

        //     // double d0x = 120; 
        //     // double d0y = 120;

        //     // addStraightSegment(tpal, true, startx,                              starty, 
        //     //                                d0x-rc,                              d0y);

        //     double startx = 30+rc;
        //     double starty = 60; 

        //     double d0x = 90; 
        //     double d0y = 90;

        //     double d1x = 150;
        //     double d1y = 120; 

        //     double d2x = 180;
        //     double d2y = 50;

        //     double endx = 330;
        //     double endy = 60;

        //     addStraightSegment(tpal, true, startx,                              starty, 
        //                                    d0x-rc,                              d0y);

        //     System.out.println("Galactic Search A Red Path Coordinates - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 10;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchARedPath2() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30+rc;
        //     double starty = 60; 

        //     double d0x = 90; 
        //     double d0y = 90;

        //     double d1x = 150;
        //     double d1y = 120; 

        //     double d2x = 180;
        //     double d2y = 50;

        //     double endx = 330;
        //     double endy = 60;

        //     addStraightSegment(tpal,false, d0x-rc,  d0y,
        //                                    d1x-rc,  d1y);

        //     System.out.println("Galactic Search A Red Path Coordinates - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchARedPath3() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30+rc;
        //     double starty = 60; 

        //     double d0x = 90; 
        //     double d0y = 90;

        //     double d1x = 150;
        //     double d1y = 120; 

        //     double d2x = 180;
        //     double d2y = 50;

        //     double endx = 330;
        //     double endy = 60;

        //     addStraightSegment(tpal,false, d1x-rc,  d1y,
        //                                    d2x-rc,  d2y);

        //     System.out.println("Galactic Search A Red Path - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchARedPath4() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30+rc;
        //     double starty = 60; 

        //     double d0x = 90; 
        //     double d0y = 90;

        //     double d1x = 150;
        //     double d1y = 120; 

        //     double d2x = 180;
        //     double d2y = 50;

        //     double endx = 330;
        //     double endy = 60;

        //     addStraightSegment(tpal, false, d2x-rc,  d2y,
        //                                     endx,    endy);

        //     System.out.println("Galactic Search A Red PathCoordinates - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchABluePath1() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 180; 
        //     double d0y = 30;

        //     double d1x = 210;
        //     double d1y = 120; 

        //     double d2x = 270;
        //     double d2y = 90;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal, true, startx, starty, 
        //                                    d0x-rc, d0y);

        //     System.out.println("Galactic Search A Blue Path Coordinates - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchABluePath2() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 180; 
        //     double d0y = 30;

        //     double d1x = 210;
        //     double d1y = 120; 

        //     double d2x = 270;
        //     double d2y = 90;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal, true, d0x-rc, d0y,
        //                                    d1x-rc, d1y);
    
        //     System.out.println("Galactic Search A Blue Path Coordinates - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchABluePath3() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 180; 
        //     double d0y = 30;

        //     double d1x = 210;
        //     double d1y = 120; 

        //     double d2x = 270;
        //     double d2y = 90;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal, false, d1x-rc,  d1y,
        //                              d2x-rc,  d2y);                               
        //     System.out.println("Galactic Search A Blue Path Coordinates - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchABluePath4() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 180; 
        //     double d0y = 30;

        //     double d1x = 210;
        //     double d1y = 120; 

        //     double d2x = 270;
        //     double d2y = 90;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal, false, d2x-rc,  d2y,
        //                                     endx,  endy);                               
        //     System.out.println("Galactic Search A Blue Path Coordinates - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchBRedPath1() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 90; 
        //     double d0y = 120;

        //     double d1x = 150;
        //     double d1y = 60; 

        //     double d2x = 210;
        //     double d2y = 120;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal,false, d0x-rc,  d0y,
        //                                    d1x-rc,  d1y);

        //     System.out.println("Galactic Search A Red Path - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchBRedPath2() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 90; 
        //     double d0y = 120;

        //     double d1x = 150;
        //     double d1y = 60; 

        //     double d2x = 210;
        //     double d2y = 120;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal,false, d1x-rc,  d1y,
        //                                    d2x-rc,  d2y);

        //     System.out.println("Galactic Search A Red Path - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchBRedPath3() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 90; 
        //     double d0y = 120;

        //     double d1x = 150;
        //     double d1y = 60; 

        //     double d2x = 210;
        //     double d2y = 120;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal,false, d2x-rc,  d2y,
        //                                    endx,    endy);

        //     System.out.println("Galactic Search A Red Path - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchBBluePath1() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 180; 
        //     double d0y = 60;

        //     double d1x = 240;
        //     double d1y = 120; 

        //     double d2x = 300;
        //     double d2y = 60;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal,false, startx,  starty,
        //                                    d1x-rc,    d1y);

        //     System.out.println("Galactic Search A Red Path - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchBBluePath2() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 180; 
        //     double d0y = 60;

        //     double d1x = 240;
        //     double d1y = 120; 

        //     double d2x = 300;
        //     double d2y = 60;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal,false, d1x-rc,  d1y,
        //                                    d2x-rc,     d2y);

        //     System.out.println("Galactic Search A Red Path - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getGalacticSearchBBluePath3() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 30;
        //     double starty = 120+radius; 

        //     double d0x = 180; 
        //     double d0y = 60;

        //     double d1x = 240;
        //     double d1y = 120; 

        //     double d2x = 300;
        //     double d2y = 60;

        //     double endx = 330;
        //     double endy = 90;

        //     addStraightSegment(tpal,false, d2x-rc,  d2y,
        //                                    endx,     endy);

        //     System.out.println("Galactic Search A Red Path - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d((tp.x-startx)*xscale, (tp.y-starty)*yscale), Rotation2d.fromDegrees(tp.a)));
        //     }

        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     double rc = 28/2;
        //     double radius = 30;

        //     double startx = 60-rc;
        //     double starty = 120-radius; 

        //     double d0x = 120; 
        //     double d0y = starty;
        //     addStraightSegment(tpal, true, startx,                              starty, 
        //                                    d0x+radius*Math.cos(enterAngleRads), d0y+radius*Math.sin(enterAngleRads));
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),           20,        80,        80,           9, 20.0, 1);
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath2() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(new Pose2d(new Translation2d(200, 0.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/40.0, 60.0, 60.0, kMaxVoltage, 40.0, 1);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath3() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(135.0)));
        //     waypoints.add(new Pose2d(new Translation2d(-40.0, 40.0), Rotation2d.fromDegrees(135.0)));
        //     waypoints.add(new Pose2d(new Translation2d(-185.0, 61.0), Rotation2d.fromDegrees(180.0)));
            
        //     // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        //     return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBlueBPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(210)));
            waypoints.add(new Pose2d(new Translation2d(-120.0, -60.0), Rotation2d.fromDegrees(210)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 12.0, 12.0, 12.0, kMaxVoltage, 12.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPowerPortForwardPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-100.0, 0.0), Rotation2d.fromDegrees(0)));
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, 60.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPowerPortBackwardPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(new Translation2d(-100.0, 0.0), Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints, Arrays.asList(), 20.0, 60.0, 60.0, kMaxVoltage, 20.0, 1);
        }
        

    }
    
}

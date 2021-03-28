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
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 60.0; //120.0;
    private static final double kMaxDecel = 72.0; //72.0;
    private static final double kMaxVoltage = 9.0;

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

        // Rendezvous Paths
        public final MirroredTrajectory startToThreePowerCells;
        public final MirroredTrajectory startToThreePowerCells2;
        public final MirroredTrajectory collectThreePowerCells;      
        public final MirroredTrajectory moveToTwoPowerCells;
        public final MirroredTrajectory moveToTwoPowerCells2;            
        public final MirroredTrajectory collectTwoPowerCells; 
        
        // Steal Paths
        public final MirroredTrajectory startToStealPath;
        public final MirroredTrajectory stealToGoalPath;
        public final MirroredTrajectory goalToRendezvousPath;
        public final MirroredTrajectory threeBallsPath;
        public final MirroredTrajectory threeBallsToGoalPath;

        // Trench Paths
        public final MirroredTrajectory startToTrenchPath;
        public final MirroredTrajectory trenchToStartPath;

        // Test Paths Alex
        public final MirroredTrajectory backAwayFromLinePath;

        public final MirroredTrajectory startToEndPath;
        public final MirroredTrajectory barrelPath;
        public final MirroredTrajectory slalomPath;
        public final MirroredTrajectory bouncePath;
        public final MirroredTrajectory testPath3;
        public final MirroredTrajectory testPath4;

        public final MirroredTrajectory testPathBrian;

        private TrajectorySet() {
            // Rendezvous Paths
            startToThreePowerCells = new MirroredTrajectory(getStartToThreePowerCells());
            startToThreePowerCells2 = new MirroredTrajectory(getStartToThreePowerCells2());
            // System.out.println(startToThreePowerCells.left.toString());
            collectThreePowerCells = new MirroredTrajectory(getCollectThreePowerCells());
            // System.out.println(collectThreePowerCells.left.toString());
            moveToTwoPowerCells = new MirroredTrajectory(getMoveToTwoPowerCells());
            // System.out.println(moveToTwoPowerCells.left.toString());
            moveToTwoPowerCells2 = new MirroredTrajectory(getMoveToTwoPowerCells2());
            // System.out.println(moveToTwoPowerCells2.left.toString());
            collectTwoPowerCells = new MirroredTrajectory(getCollectTwoPowerCells());
            // System.out.println(collectTwoPowerCells.left.toString());

            // Trench Paths
            startToTrenchPath = new MirroredTrajectory(getStartToTrench());
            trenchToStartPath = new MirroredTrajectory(getTrenchToStart());

            // Steal Paths
            startToStealPath = new MirroredTrajectory(getStartToSteal());
            stealToGoalPath = new MirroredTrajectory(getStealToGoal());
            goalToRendezvousPath = new MirroredTrajectory(getGoalToRendezvous());
            threeBallsPath = new MirroredTrajectory(getThreeBalls());
            threeBallsToGoalPath = new MirroredTrajectory(getThreeBallsToGoal());

            // Test Paths Alex
            backAwayFromLinePath = new MirroredTrajectory(getBackAwayFromLinePath());

            startToEndPath = new MirroredTrajectory(getStartToEndPath());
            // System.out.println(startToEndPath.left.toString());
            barrelPath = new MirroredTrajectory(getBarrelPath());
            // System.out.println(testPath.left.toString());
            slalomPath = new MirroredTrajectory(getSlalomPath());
            // System.out.println(testPath2.left.toString());
            bouncePath = new MirroredTrajectory(getBouncePath());
            // System.out.println(testPath2.left.toString());
            testPath3 = new MirroredTrajectory(getTestPath3());
            // System.out.println(testPath3.left.toString());
            testPath4 = new MirroredTrajectory(getTestPath4());
            // System.out.println(testPath4.left.toString());

            testPathBrian = new MirroredTrajectory(getBrianPath());
            // System.out.println("Brian's path");
            // System.out.println(testPathBrian.left.toString());
            // System.out.println("Brian's path end");

        }

        /*************************************Rendezvous Paths*****************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToThreePowerCells() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(-133.5)));
            waypoints.add(new Pose2d(new Translation2d(-100.0, -95.0), Rotation2d.fromDegrees(-133.5)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToThreePowerCells2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-100.0, -95.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-105.0, -95.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-93.0, -71.0), Rotation2d.fromDegrees(63.4)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCollectThreePowerCells() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-95.0, -95.0), Rotation2d.fromDegrees(60.0)));
            waypoints.add(new Pose2d(new Translation2d(-83.0, -71.0), Rotation2d.fromDegrees(60.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMoveToTwoPowerCells() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-83.0, -71.0), Rotation2d.fromDegrees(-30.0)));
            waypoints.add(new Pose2d(new Translation2d(-70.0, -81.0), Rotation2d.fromDegrees(-30.0)));
            waypoints.add(new Pose2d(new Translation2d(-33.0, -15.0), Rotation2d.fromDegrees(60.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMoveToTwoPowerCells2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-33.0, -15.0), Rotation2d.fromDegrees(150.0)));
            waypoints.add(new Pose2d(new Translation2d(-118.0, 15.0), Rotation2d.fromDegrees(150.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }        

        private Trajectory<TimedState<Pose2dWithCurvature>> getCollectTwoPowerCells() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-95.0, -95.0), Rotation2d.fromDegrees(60.0)));
            waypoints.add(new Pose2d(new Translation2d(-83.0, -71.0), Rotation2d.fromDegrees(60.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        /*************************************Trench Paths*****************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToTrench() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(135.0)));
            waypoints.add(new Pose2d(new Translation2d(-65.0, 65.0), Rotation2d.fromDegrees(135.0)));
            waypoints.add(new Pose2d(new Translation2d(-195.0, 65.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchToStart() {
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(new Pose2d(new Translation2d(-185.0, 61.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(-40.0, 40.0), Rotation2d.fromDegrees(-45.0)));
            // waypoints.add(new Pose2d(new Translation2d(-20.0, 20.0), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(new Pose2d(new Translation2d(-195.0, 65.0), Rotation2d.fromDegrees(-18.4)));
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(-18.4)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/20.0, 20.0, 60.0, kMaxVoltage, 20.0, 1);
        }

        /*************************************Steal Paths*****************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToSteal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-122.0, 0.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0, 60.0, 60.0, kMaxVoltage, 120.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStealToGoal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-122.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(-100.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(-10.0, 160.0), Rotation2d.fromDegrees(60.6)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0 * 0.75, 60.0, 60.0, kMaxVoltage, 120.0 * 0.75, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGoalToRendezvous() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-10.0, 160.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-80.0, 160.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0 * 0.5, 60.0, 60.0, kMaxVoltage, 120.0 * 0.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThreeBalls() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-80.0, 160.0), Rotation2d.fromDegrees(120.0)));
            waypoints.add(new Pose2d(new Translation2d(-108.0, 112.0), Rotation2d.fromDegrees(120.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0 * 0.5, 60.0, 60.0, kMaxVoltage, 120.0 * 0.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getThreeBallsToGoal() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-48.0, 112.0), Rotation2d.fromDegrees(32.0)));
            waypoints.add(new Pose2d(new Translation2d(-10.0, 160.0), Rotation2d.fromDegrees(32.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0 * 0.5, 60.0, 60.0, kMaxVoltage, 120.0 * 0.5, 1);
        }

        /*************************************Test Paths*****************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getBackAwayFromLinePath() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(-60.0, 0.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), /*kMaxVelocity*/120.0, 60.0, 60.0, kMaxVoltage, 120.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBrianPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d((10*12), 0.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 30, kMaxAccel, kMaxDecel, kMaxVoltage, 30.0, 1);
        }
        
        // private Trajectory<TimedState<Pose2dWithCurvature>> getStraightPath(){
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(Constants.kRobotLeftStartingPose);
        //     waypoints.add(Constants.kRobotLeftStartingPose.transformBy(Pose2d.fromTranslation(new Translation2d(72.0, 0.0))));

        //     return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        // }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToEndPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(startingPose);
            waypoints.add(midPose);
            waypoints.add(mid2Pose);
            waypoints.add(endingPose);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
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
            ArrayList<ThreePoints> tpal = new ArrayList<>();

                        //Temporary
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(200, 0.0), Rotation2d.fromDegrees(0.0)));

            // double exitAngleRads;
            // double enterAngleRads;
            // double rc = 28/2;
            // double radius = 30;

            // double startx = 60-rc;
            // double starty = 120-radius; 

            // double d0x = 147; //150
            // double d0y = 120;

            // double d1x = 235; //240
            // double d1y = 63; 

            // double d2x = 292; //300
            // double d2y = 117; //120

            // double endx = startx;
            // double endy = starty;


            // enterAngleRads = convertToRads(-90);
            // addStraightSegment(tpal, true, startx,                              starty, 
            //                                d0x+radius*Math.cos(enterAngleRads), d0y+radius*Math.sin(enterAngleRads));
            // exitAngleRads = convertToRads(265);
            // addArcSegment(tpal,     false, d0x+radius*Math.cos(enterAngleRads), d0y+radius*Math.sin(enterAngleRads),
            //                                d0x,                                 d0y,                 exitAngleRads);
            // enterAngleRads = convertToRads(75);
            // addStraightSegment(tpal,false, d0x+radius*Math.cos(exitAngleRads),  d0y+radius*Math.sin(exitAngleRads),
            //                                d1x+radius*Math.cos(enterAngleRads), d1y+radius*Math.sin(enterAngleRads));
            // exitAngleRads = convertToRads(-225);
            // addArcSegment(tpal,      false,d1x+radius*Math.cos(enterAngleRads), d1y+radius*Math.sin(enterAngleRads),
            //                                d1x,                                 d1y,                 exitAngleRads);
            // enterAngleRads = convertToRads(135);
            // addStraightSegment(tpal,false, d1x+radius*Math.cos(exitAngleRads),  d1y+radius*Math.sin(exitAngleRads),
            //                                d2x+radius*Math.cos(enterAngleRads), d2y+radius*Math.sin(enterAngleRads));
            // exitAngleRads = convertToRads(-90);
            // addArcSegment(tpal,      false,d2x+radius*Math.cos(enterAngleRads), d2y+radius*Math.sin(enterAngleRads),
            //                                d2x,                                 d2y,                 exitAngleRads);

            // addStraightSegment(tpal, false,d2x+radius*Math.cos(exitAngleRads),  d2y+radius*Math.sin(exitAngleRads),
            //                                startx,                              starty);

            // System.out.println("Barrel Path Coordinates - ");
            // for (ThreePoints tp : tpal){
            //     System.out.println(tp.x+","+tp.y+","+tp.a);
            //     waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
            // }

            // double speed = SmartDashboard.getNumber("Autopath Speed", -1);
            // if(speed == -1){
            //     SmartDashboard.putNumber("Autopath Speed", 20);
            //     speed = 20;
            // }
            // return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSlalomPath() {
            List<Pose2d> waypoints = new ArrayList<>();
            //Temporary
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(200, 0.0), Rotation2d.fromDegrees(0.0)));

        //     ArrayList<ThreePoints> tpal = new ArrayList<>();
            
        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 28; //30
            
        //     double startx = 60-rc;
        //     double starty = 180-rc;

        //     double d0x = 60;
        //     double d1x = 120;
        //     double d2x = 235; //240
        //     double d3x = 295; //300

        //     double dy = 120;

        //     double endx = startx;
        //     double endy = 60+rc;

        //     enterAngleRads = convertToRads(-135);
        //     addStraightSegment(tpal, true,  startx,                              starty,
        //                                     d1x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads));
        //     exitAngleRads = convertToRads(-90);
        //     addArcSegment(tpal, false,      d1x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
        //                                     d1x,                                 dy,                 exitAngleRads);
        //     enterAngleRads = convertToRads(-90);
        //     addStraightSegment(tpal, false, d1x+radius*Math.cos(exitAngleRads),  dy+radius*Math.sin(exitAngleRads),
        //                                     d2x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads));
        //     exitAngleRads = convertToRads(0);
        //     addArcSegment(tpal, false,      d2x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
        //                                     d2x,                                 dy,                 exitAngleRads);
        //     enterAngleRads = convertToRads(150);
        //     exitAngleRads = convertToRads(-179);
        //     addArcSegment(tpal, true,       d3x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
        //                                     d3x,                                 dy,                 exitAngleRads);
        //     enterAngleRads = convertToRads(0);
        //     exitAngleRads = convertToRads(90);
        //     addArcSegment(tpal, false,      d2x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
        //                                     d2x,                                 dy,                 exitAngleRads);
        //     enterAngleRads = convertToRads(90);
        //     addStraightSegment(tpal, false, d2x+radius*Math.cos(exitAngleRads),  dy+radius*Math.sin(exitAngleRads),
        //                                     d1x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads));
        //     exitAngleRads = convertToRads(135);
        //     addArcSegment(tpal, false,      d1x+radius*Math.cos(enterAngleRads), dy+radius*Math.sin(enterAngleRads),
        //                                     d1x,                                 dy,                 exitAngleRads);

        //     addStraightSegment(tpal, false, d1x+radius*Math.cos(exitAngleRads),  dy+radius*Math.sin(exitAngleRads),
        //                                     endx,                                endy);

        //     System.out.println("Slalom Path Coordinates - ");
        //     for (ThreePoints tp : tpal){
        //         System.out.println(tp.x+","+tp.y+","+tp.a);
        //         waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
        //     }
        //     double speed = SmartDashboard.getNumber("Autopath Speed", -1);
        //     if(speed == -1){
        //         SmartDashboard.putNumber("Autopath Speed", 20);
        //         speed = 20;
        //     }
        //     return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
             return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBouncePath() {
            List<Pose2d> waypoints = new ArrayList<>();
            ArrayList<ThreePoints> tpal = new ArrayList<>();         
            //Temporary
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(200, 0.0), Rotation2d.fromDegrees(0.0)));

        //     double exitAngleRads;
        //     double enterAngleRads;
        //     double rc = 28/2;
        //     double radius = 28; //30
            
        //     double startx = 60-rc;
        //     double starty = 180-rc;

        //     double d0x = 60;
        //     double d1x = 120;
        //     double d2x = 235; //240
        //     double d3x = 295; //300

        //     double dy = 120;

        //     double endx = startx;
        //     double endy = 60+rc;

        //     // placeholder
        //     waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(new Pose2d(new Translation2d(200, 0.0), Rotation2d.fromDegrees(0.0)));

            // System.out.println("Bounce Path Coordinates - ");
            // for (ThreePoints tp : tpal){
            //     System.out.println(tp.x+","+tp.y+","+tp.a);
            //     waypoints.add(new Pose2d(new Translation2d(tp.x-startx, tp.y-starty), Rotation2d.fromDegrees(tp.a)));
            // }
            // double speed = SmartDashboard.getNumber("Autopath Speed", -1);
            // if(speed == -1){
            //     SmartDashboard.putNumber("Autopath Speed", 20);
            //     speed = 20;
            // }
            // return generateTrajectory(   false, waypoints, Arrays.asList(),        speed,        80,        80,           9, speed, 1);
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(200, 0.0), Rotation2d.fromDegrees(0.0)));
            return generateTrajectory(   false, waypoints, Arrays.asList(),           40,        80,        80,           9, 40.0, 1);
            // return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }

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

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath4() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(-185.0, 61.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(-40.0, 40.0), Rotation2d.fromDegrees(-45.0)));
            waypoints.add(new Pose2d(new Translation2d(-20.0, 20.0), Rotation2d.fromDegrees(-45.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 60.0, 1);
        }
    }
    
}

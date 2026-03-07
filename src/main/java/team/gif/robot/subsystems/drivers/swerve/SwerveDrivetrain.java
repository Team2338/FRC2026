package team.gif.robot.subsystems.drivers.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import team.gif.lib.LimelightHelpers;
import team.gif.lib.drivePace;
import team.gif.robot.Robot;
import team.gif.robot.subsystems.drivers.swerve.utilities.DriveMotor;
import team.gif.robot.subsystems.drivers.swerve.utilities.DriveMotorFactory;
import team.gif.robot.subsystems.drivers.swerve.utilities.Encoder;
import team.gif.robot.subsystems.drivers.swerve.utilities.EncoderFactory;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConfiguration;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConstants;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveMap;
import team.gif.robot.subsystems.drivers.swerve.utilities.TurnMotor;
import team.gif.robot.subsystems.drivers.swerve.utilities.TurnMotorFactory;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;


public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveMap deviceMap;
    private final SwerveConstants constants;
    private final DriveMotorFactory driveMotorFactory;
    private final TurnMotorFactory turnMotorFactory;
    private final EncoderFactory encoderFactory;

    public SwerveModule fL;
    public SwerveModule fR;
    public SwerveModule rL;
    public SwerveModule rR;

    private DriveMotor fLDriveMotor;
    private DriveMotor fRDriveMotor;
    private DriveMotor rLDriveMotor;
    private DriveMotor rRDriveMotor;

    private TurnMotor fLTurnMotor;
    private TurnMotor fRTurnMotor;
    private TurnMotor rLTurnMotor;
    private TurnMotor rRTurnMotor;

    private Encoder fLEncoder;
    private Encoder fREncoder;
    private Encoder rLEncoder;
    private Encoder rREncoder;

    public SwerveDrivePoseEstimator poseEstimator;
    private drivePace drivePace;

    private boolean visionEnabled = true;
    private String[] limelightNames = new String[] {};
    private PhotonCamera[] photonCameras = new PhotonCamera[] {};
    private PhotonPoseEstimator[] photonPoseEstimators = new PhotonPoseEstimator[] {};
    private static AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private Matrix<N3, N1> photonCurrStdDevs;
    private Rotation2d oneEighty = Rotation2d.fromDegrees(180);

    private boolean isRedAlliance = false;
    
    public boolean debugMode = false;

    // Network Table publishers for the swerve
    // states so that we can use them in advantage scope
    private static final StructArrayPublisher<SwerveModuleState> targetPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("TargetSwerveState", SwerveModuleState.struct).publish();
    private static final StructArrayPublisher<SwerveModuleState> actualPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("ActualSwerveState", SwerveModuleState.struct).publish();
    private static final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("EstimatedPose", Pose2d.struct).publish();
    private static final StructPublisher<Pose2d> estPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("EstimatedVisionPose", Pose2d.struct).publish();
    private static final StructPublisher<ChassisSpeeds> chassisSpeedsStructPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("ChassisSpeeds", ChassisSpeeds.struct).publish();


    /**
     * Constructs a SwerveDrivetrain with the specified configuration
     * @param config The SwerveConfig for the drivetrain
     */
    public SwerveDrivetrain(SwerveConfiguration config) {
        super();

        this.deviceMap = config.deviceMap;
        this.constants = config.constants;
        this.driveMotorFactory = config.driveMotorFactory;
        this.turnMotorFactory = config.turnMotorFactory;
        this.encoderFactory = config.encoderFactory;

        configModules();

        resetDriveEncoders();

        Rotation2d rotation = Robot.pigeon.getRotation2d();

        if(checkRedAlliance()) {
            rotation = rotation.rotateBy(oneEighty);
        }

        poseEstimator = new SwerveDrivePoseEstimator(constants.DRIVE_KINEMATICS, rotation, getPosition(), new Pose2d(0, 0, rotation));

        drivePace = team.gif.lib.drivePace.COAST_FR;

        configPathPlanner();
    }

    /**
     * Periodic function
     * - constantly update the odometry
     */
    @Override
    public void periodic() {

        isRedAlliance = checkRedAlliance();

        Rotation2d rotation = Robot.pigeon.getRotation2d();

        if(isRedAlliance) {
            rotation = rotation.rotateBy(oneEighty);
        }

        poseEstimator.update(
            rotation,
            getPosition()
        );

        if (Robot.pigeon.getYawRate() < 720 && visionEnabled) {

            for (String limelightName : limelightNames) {
                LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
                if (visionEnabled && estimate != null && estimate.tagCount > 0) {
                    poseEstimator.addVisionMeasurement(
                            estimate.pose,
                            estimate.timestampSeconds,
                            VecBuilder.fill(.7, .7, 9999999));

                }
            }
            for (var i = 0; i < photonCameras.length; i++) {
                PhotonCamera camera = photonCameras[i];
                PhotonPoseEstimator estimator = photonPoseEstimators[i];
                Optional<EstimatedRobotPose> visionEst;
                for (var result : camera.getAllUnreadResults()) {
                    visionEst = estimator.estimateCoprocMultiTagPose(result);
                    if (visionEst.isEmpty()) {
                        visionEst = estimator.estimateLowestAmbiguityPose(result);
                    }

                    updateEstimationStdDevs(visionEst, result.getTargets(), estimator);

                    visionEst.ifPresent(
                            est -> {
                                // Change our trust in the measurement based on the tags we can see
                                var estStdDevs = getEstimationStdDevs();

                                if (debugMode) {
                                    estPublisher.set(est.estimatedPose.toPose2d());
                                }

                                Rotation2d estRotation = isRedAlliance ? est.estimatedPose.getRotation().toRotation2d().rotateBy(oneEighty) : est.estimatedPose.getRotation().toRotation2d();
                                var newPose = new Pose2d(est.estimatedPose.getX(), est.estimatedPose.getY(), estRotation);
                                poseEstimator.addVisionMeasurement(newPose, est.timestampSeconds, estStdDevs);
                            });
                }

            }
        }

        if (debugMode) {
            posePublisher.set(poseEstimator.getEstimatedPosition());
            updateShuffleboardDebug();
        }
    }


    //region Vision
    /**
     * Set the limelight enabled status
     * @param enabled - enable limelight vision updates
     */
    public void setVisionEnabled(boolean enabled) {
        visionEnabled = enabled;
    }

    /**
     * Get the limelight enabled status
     * @return boolean for the current limelight enabled status
     */
    public boolean getVisionEnabled() {
        return visionEnabled;
    }

    /**
     * Add a limelight to the list of limelights used for vision based odometry
     * @param newLimelightName - the name of the limelight to add
     */
    public void addLimelight(String newLimelightName) {
        String[] newArray = new String[limelightNames.length + 1];
        System.arraycopy(limelightNames, 0, newArray, 0, limelightNames.length);
        newArray[newArray.length - 1] = newLimelightName;
        limelightNames = newArray;
    }

    /**
     * Get the list of limelights used for vision based odometry
     * @return String array of limelight names
     */
    public String[] getLimelightNames() {
        return limelightNames;
    }

    public void addPhotonCamera(String newPhotonCamName, Transform3d cameraLocation) {
        PhotonCamera[] newArray = new PhotonCamera[photonCameras.length + 1];
        System.arraycopy(photonCameras, 0, newArray, 0, photonCameras.length);
        newArray[newArray.length - 1] = new PhotonCamera(newPhotonCamName);
        photonCameras = newArray;

        PhotonPoseEstimator[] newEstArray = new PhotonPoseEstimator[photonPoseEstimators.length + 1];
        System.arraycopy(photonPoseEstimators, 0, newEstArray, 0, photonPoseEstimators.length);
        newEstArray[newEstArray.length - 1] = new PhotonPoseEstimator(tagLayout, cameraLocation);
        photonPoseEstimators = newEstArray;
    }

    public PhotonCamera[] getPhotonCameras() {
        return photonCameras;
    }

    public PhotonPoseEstimator[] getPhotonPoseEstimators() {
        return photonPoseEstimators;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator estimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            photonCurrStdDevs = VecBuilder.fill(0.7, 0.7, 9999999);

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VecBuilder.fill(0.7, 0.7, 9999999);
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                photonCurrStdDevs = VecBuilder.fill(0.7, 0.7, 9999999);;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 999999);
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                photonCurrStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from
     * Photonvision, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return photonCurrStdDevs;
    }

    //endregion

   //region get/set states
    /**
     * Reset the odometry to a given pose
     *
     * @param pose the pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(Robot.pigeon.getRotation2d(), new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()}, pose);
    }

    /**
     * Get the robot relative speed
     * @return ChassisSpeeds of the robot relative speed
     */
    public ChassisSpeeds getRobotRelativeSpeed() {
        SwerveModuleState frontLeftState = new SwerveModuleState(fL.getDriveVelocity(), Rotation2d.fromDegrees(fL.getTurningHeadingDegrees()));
        SwerveModuleState frontRightState = new SwerveModuleState(fR.getDriveVelocity(), Rotation2d.fromDegrees(fR.getTurningHeadingDegrees()));
        SwerveModuleState rearLeft = new SwerveModuleState(rL.getDriveVelocity(), Rotation2d.fromDegrees(rL.getTurningHeadingDegrees()));
        SwerveModuleState rearRight = new SwerveModuleState(rR.getDriveVelocity(), Rotation2d.fromDegrees(rR.getTurningHeadingDegrees()));

        ChassisSpeeds speed = constants.DRIVE_KINEMATICS.toChassisSpeeds(frontLeftState, frontRightState, rearLeft, rearRight);

        if (debugMode) {
            chassisSpeedsStructPublisher.set(speed);
        }

        return speed;
    }

    /**
     * Drive the bot with given params - always field relative
     *
     * @param x   dForward
     * @param y   dLeft
     * @param rot dRot
     */
    public void drive(double x, double y, double rot) {
        SwerveModuleState[] swerveModuleStates =
                constants.DRIVE_KINEMATICS.toSwerveModuleStates(
                        drivePace.getIsFieldRelative() ?
                                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, Robot.pigeon.getRotation2d())
                                : new ChassisSpeeds(x, y, rot));
        if (debugMode) {
            SwerveModuleState[] actualStates = { fL.getState(), fR.getState(), rL.getState(), rR.getState()};
            targetPublisher.set(swerveModuleStates);
            actualPublisher.set(actualStates);
        }
        setModuleStates(swerveModuleStates);
    }

    public void setMaxDrive() {
        fLDriveMotor.set(1);
        fRDriveMotor.set(1);
        rLDriveMotor.set(1);
        rRDriveMotor.set(1);
    }

    public void stopDrive() {
        drive(0,0,0);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a SwerveModuleState array
     *
     * @param desiredStates SwerveModuleState array of desired states for each of the modules
     * @implNote Only for use in the SwerveDrivetrain class and by pathplanner, for any general use {@link SwerveDrivetrain#drive(double x, double y, double rot)}
     */
     public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, drivePace.getValue()
        );

        fL.setDesiredState(desiredStates[0]);
        fR.setDesiredState(desiredStates[1]);
        rL.setDesiredState(desiredStates[2]);
        rR.setDesiredState(desiredStates[3]);
    }

    /**
     * Set the desired states for each of the 4 swerve modules using a ChassisSpeeds class
     * @param chassisSpeeds Robot Relative ChassisSpeeds to apply to wheel speeds
     * @implNote Use only in {@link SwerveDrivetrain}
     */
    public void setModuleChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = constants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, drivePace.getValue()
        );

        for (SwerveModuleState state : swerveModuleStates) {
            state.speedMetersPerSecond = Math.min(state.speedMetersPerSecond, drivePace.getValue());
        }

        fL.setDesiredState(swerveModuleStates[0]);
        fR.setDesiredState(swerveModuleStates[1]);
        rL.setDesiredState(swerveModuleStates[2]);
        rR.setDesiredState(swerveModuleStates[3]);

        if(debugMode) {
            SwerveModuleState[] actualStates = { fL.getState(), fR.getState(), rL.getState(), rR.getState()};
            actualPublisher.set(actualStates);
            chassisSpeedsStructPublisher.set(chassisSpeeds);
            targetPublisher.set(swerveModuleStates);
        }
    }

    /**
     * This set moves all the modules to 90 degrees. It turns the modules inward to prevent the robot from moving
     */
    public void modulesTo90() {
        SwerveModuleState state90 = new SwerveModuleState(0, Rotation2d.fromDegrees(90));
        fL.setDesiredState(state90, true);
        fR.setDesiredState(state90, false);
        rL.setDesiredState(state90, true);
        rR.setDesiredState(state90, false);

    }


    /**
     * Stop all the modules
     */
    public void stopModules() {
        fL.stop();
        fR.stop();
        rR.stop();
        rL.stop();
    }

    //endregion
    /**
     * Get the current pose of the robot
     *
     * @return The current pose of the robot (Pose2D)
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Get the current position of each of the swerve modules
     *
     * @return An array in form fL -> fR -> rL -> rR of each of the module positions
     */
    public SwerveModulePosition[] getPosition() {

        return new SwerveModulePosition[]{fL.getPosition(), fR.getPosition(), rL.getPosition(), rR.getPosition()};
    }

    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0
        );
    }

    /**
     * Reset the drive encoders
     */
    public void resetDriveEncoders() {
        fL.resetDriveEncoders();
        fR.resetDriveEncoders();
        rL.resetDriveEncoders();
        rR.resetDriveEncoders();
    }

    /**
     * set the drivePace settings for the drivetrain
     *
     * @param drivePace the drivePace to set
     */
    public void setDrivePace(drivePace drivePace) {
        this.drivePace = drivePace;
    }

    /**
     * Get the current drivePace settings
     *
     * @return the current drivePace settings
     */
    public drivePace getDrivePace() {
        return drivePace;
    }

    public SwerveConstants getConstants() {
        return constants;
    }

    private boolean checkRedAlliance() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return true;
        } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return false;
        } else {
            return true;
            //This state should never happen unless we are not connected.
            //It is set to red because that is what we are set up for in the shop.
        }

    }

    private void configModules() {

        fLDriveMotor = driveMotorFactory.create(deviceMap.FRONT_LEFT_DRIVE_MOTOR_ID, constants);
        fRDriveMotor = driveMotorFactory.create(deviceMap.FRONT_RIGHT_DRIVE_MOTOR_ID, constants);
        rLDriveMotor = driveMotorFactory.create(deviceMap.REAR_LEFT_DRIVE_MOTOR_ID, constants);
        rRDriveMotor = driveMotorFactory.create(deviceMap.REAR_RIGHT_DRIVE_MOTOR_ID, constants);

        fLTurnMotor = turnMotorFactory.create(deviceMap.FRONT_LEFT_TURNING_MOTOR_ID);
        fRTurnMotor = turnMotorFactory.create(deviceMap.FRONT_RIGHT_TURNING_MOTOR_ID);
        rLTurnMotor = turnMotorFactory.create(deviceMap.REAR_LEFT_TURNING_MOTOR_ID);
        rRTurnMotor = turnMotorFactory.create(deviceMap.REAR_RIGHT_TURNING_MOTOR_ID);

        // In case the turn motor is a TalonSRX that has the encoder connected to the data port
        if(encoderFactory == null && fLTurnMotor instanceof TalonSRXTurnMotorEncoder) {
            fLEncoder = (Encoder) fLTurnMotor;
            fREncoder = (Encoder) fRTurnMotor;
            rLEncoder = (Encoder) rLTurnMotor;
            rREncoder = (Encoder) rRTurnMotor;
        } else {
            assert encoderFactory != null;
            fLEncoder = encoderFactory.create(deviceMap.FRONT_LEFT_ENCODER_ID);
            fREncoder = encoderFactory.create(deviceMap.FRONT_RIGHT_ENCODER_ID);
            rLEncoder = encoderFactory.create(deviceMap.REAR_LEFT_ENCODER_ID);
            rREncoder = encoderFactory.create(deviceMap.REAR_RIGHT_ENCODER_ID);
        }


        fL = new SwerveModule(
                fLDriveMotor,
                fLTurnMotor,
                fLEncoder,
                constants.FL_TURN_INVERTED,
                constants.FL_DRIVE_INVERTED,
                constants.FRONT_LEFT_OFFSET,
                constants.FL_DRIVE_FF,
                constants.FL_FF,
                constants.FL_P,
                constants.MAX_DRIVE_TEMP
        );

        fR = new SwerveModule (
                fRDriveMotor,
                fRTurnMotor,
                fREncoder,
                constants.FR_TURN_INVERTED,
                constants.FR_DRIVE_INVERTED,
                constants.FRONT_RIGHT_OFFSET,
                constants.FR_DRIVE_FF,
                constants.FR_FF,
                constants.FR_P,
                constants.MAX_DRIVE_TEMP
                );

        rL = new SwerveModule (
                rLDriveMotor,
                rLTurnMotor,
                rLEncoder,
                constants.RL_TURN_INVERTED,
                constants.RL_DRIVE_INVERTED,
                constants.REAR_LEFT_OFFSET,
                constants.RL_DRIVE_FF,
                constants.RL_FF,
                constants.RL_P,
                constants.MAX_DRIVE_TEMP
        );

        rR = new SwerveModule (
                rRDriveMotor,
                rRTurnMotor,
                rREncoder,
                constants.RR_TURN_INVERTED,
                constants.RR_DRIVE_INVERTED,
                constants.REAR_RIGHT_OFFSET,
                constants.RR_DRIVE_FF,
                constants.RR_FF,
                constants.RR_P,
                constants.MAX_DRIVE_TEMP
        );
    }

    private void configPathPlanner() {
        RobotConfig ppConfig;
        try{
            ppConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            ModuleConfig moduleConfig = constants.PATHPLANNER_MODULE_CONFIG;
            ppConfig = new RobotConfig(constants.MASS_KG, constants.MOI_KGM2, moduleConfig, constants.TRACK_WIDTH_METERS);
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeed,
                this::setModuleChassisSpeeds,
                constants.AUTO_DRIVE_CONTROLLER,
                ppConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if( alliance.isPresent() ){
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
    }

    public double fLDriveTemp() { return fLDriveMotor.getTemp(); }
    public double fRDriveTemp() { return fRDriveMotor.getTemp(); }
    public double rLDriveTemp() { return rLDriveMotor.getTemp(); }
    public double rRDriveTemp() { return rRDriveMotor.getTemp(); }

    /**
     * Enables debug mode. Adds the following to the network table:
     * <ul>
     *     <li>Heading for each module (Gyro Widget)</li>
     *     <li>Actual Swerve State (Elastic Widget)</li>
     *     <li>Target Swerve State (Elastic Widget)</li>
     *     <li>Actual Swerve State (AdvantageScope)</li>
     *     <li>Target Swerve State (AdvantageScope)</li>
     *     <li>Target Chassis Speed (AdvantageScope</li>
     *     <li>Raw Encoder Readings</li>
     *     <li>Raw Encoder Degrees</li>
     *     <li>Raw Encoder Radians</li>
     *     <li>Raw Drive Encoder</li>
     * </ul>
     */
    public void enableDebugMode() {
        debugMode = true;
        SmartDashboard.putData("Swerve/FL Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", fL::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData("Swerve/FR Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", fR::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData("Swerve/RL Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", rL::getTurningHeadingDegrees, null);
        });
        SmartDashboard.putData("Swerve/RR Heading", builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", rR::getTurningHeadingDegrees, null);
        });

        SmartDashboard.putData("Swerve/Actual Swerve State", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> fL.getTurningHeading(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> fL.getDriveVelocity(), null);

                builder.addDoubleProperty("Front Right Angle", () -> fR.getTurningHeading(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> fR.getDriveVelocity(), null);

                builder.addDoubleProperty("Back Left Angle", () -> rL.getTurningHeading(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> rL.getDriveVelocity(), null);

                builder.addDoubleProperty("Back Right Angle", () -> rR.getTurningHeading(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> rR.getDriveVelocity(), null);

                builder.addDoubleProperty("Robot Angle", () -> Units.degreesToRadians(Robot.pigeon.get360Heading()), null);
            }
        });

        SmartDashboard.putData("Swerve/Target Swerve State", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", fL::getTargetAngle, null);
                builder.addDoubleProperty("Front Left Velocity", fL::getTargetVelocity, null);

                builder.addDoubleProperty("Front Right Angle", fR::getTargetAngle, null);
                builder.addDoubleProperty("Front Right Velocity", fR::getTargetVelocity, null);

                builder.addDoubleProperty("Back Left Angle", rL::getTargetAngle, null);
                builder.addDoubleProperty("Back Left Velocity", rL::getTargetVelocity, null);

                builder.addDoubleProperty("Back Right Angle", rR::getTargetAngle, null);
                builder.addDoubleProperty("Back Right Velocity", rR::getTargetVelocity, null);

                builder.addDoubleProperty("Robot Angle", () -> Units.degreesToRadians(Robot.pigeon.get360Heading()), null);
            }
        });
    }
    private void updateShuffleboardDebug() {
        SmartDashboard.putNumber("Swerve/FR Raw Degrees", fR.encoderDegrees());
        SmartDashboard.putNumber("Swerve/FL Raw Degrees", fL.encoderDegrees());
        SmartDashboard.putNumber("Swerve/RR Raw Degrees", rR.encoderDegrees());
        SmartDashboard.putNumber("Swerve/RL Raw Degrees", rL.encoderDegrees());

        SmartDashboard.putNumber("Swerve/FL Raw Encoder", fLEncoder.getTicks());
        SmartDashboard.putNumber("Swerve/FR Raw Encoder", fREncoder.getTicks());
        SmartDashboard.putNumber("Swerve/RL Raw Encoder", rLEncoder.getTicks());
        SmartDashboard.putNumber("Swerve/RR Raw Encoder", rREncoder.getTicks());

        SmartDashboard.putNumber("Swerve/FR Raw Radians", fR.getTurningHeading());
        SmartDashboard.putNumber("Swerve/FL Raw Radians", fL.getTurningHeading());
        SmartDashboard.putNumber("Swerve/RR Raw Radians", rR.getTurningHeading());
        SmartDashboard.putNumber("Swerve/RL Raw Radians", rL.getTurningHeading());

        SmartDashboard.putNumber("Swerve/FL Drive Encoder", fLDriveMotor.getPosition());
        SmartDashboard.putNumber("Swerve/FR Drive Encoder", fRDriveMotor.getPosition());
        SmartDashboard.putNumber("Swerve/RL Drive Encoder", rLDriveMotor.getPosition());
        SmartDashboard.putNumber("Swerve/RR Drive Encoder", rRDriveMotor.getPosition());

        SmartDashboard.putNumber("Swerve/FL Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber("Swerve/FR Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber("Swerve/RL Rot Output", fLTurnMotor.getOutput());
        SmartDashboard.putNumber("Swerve/RR Rot Output", fLTurnMotor.getOutput());
    }

    //region SysID
    public SysIdRoutine getSysIdRoutine(String motors) {
        MutVoltage voltMut = Volts.mutable(0);


        if (motors.equals("drive")) {
            MutDistance posMut = Meters.mutable(0);
            MutLinearVelocity vMut= MetersPerSecond.mutable(0);

            return new SysIdRoutine(new SysIdRoutine.Config(null, voltMut.mut_replace(8, Volts), null),
                    new SysIdRoutine.Mechanism(
                            voltage -> {
                                fLDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                fL.turnHoldZero();
                                fRDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                fR.turnHoldZero();
                                rLDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                rL.turnHoldZero();
                                rRDriveMotor.setVoltage(voltage.baseUnitMagnitude());
                                rR.turnHoldZero();
                            },
                            log -> {
                                log.motor("fLDrive")
                                        .voltage(voltMut.mut_replace(fLDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(fLDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(fLDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("fRDrive")
                                        .voltage(voltMut.mut_replace(fRDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(fRDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(fRDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("rLDrive")
                                        .voltage(voltMut.mut_replace(rLDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(rLDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(rLDriveMotor.getVelocity(), MetersPerSecond));
                                log.motor("rRDrive")
                                        .voltage(voltMut.mut_replace(rRDriveMotor.getVoltage(), Volts))
                                        .linearPosition(posMut.mut_replace(rRDriveMotor.getPosition(), Meters))
                                        .linearVelocity(vMut.mut_replace(rRDriveMotor.getVelocity(), MetersPerSecond));

                            },
                            this));
        } else if (motors.equals("turn")) {
            MutAngle thetaMut = Radians.mutable(0);
            MutAngularVelocity thetaVMut = RadiansPerSecond.mutable(0);

            return new SysIdRoutine(new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                    voltage -> {
                            fLTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            fRTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            rLTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                            rRTurnMotor.setVoltage(voltage.baseUnitMagnitude());
                        }, log -> {
                            log.motor("fLTurn")
                                    .voltage(voltMut.mut_replace(fLTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(fLEncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(fLEncoder.getVelocity(), RadiansPerSecond));
                            log.motor("fRTurn")
                                    .voltage(voltMut.mut_replace(fRTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(fREncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(fREncoder.getVelocity(), RadiansPerSecond));
                            log.motor("rLTurn")
                                    .voltage(voltMut.mut_replace(rLTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(rLEncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(rLEncoder.getVelocity(), RadiansPerSecond));
                            log.motor("rRTurn")
                                    .voltage(voltMut.mut_replace(rRTurnMotor.getVoltage(), Volts))
                                    .angularPosition(thetaMut.mut_replace(rREncoder.getRadians(), Radians))
                                    .angularVelocity(thetaVMut.mut_replace(rREncoder.getVelocity(), RadiansPerSecond));
                    }, this));

        } else {
            DriverStation.reportError("Invalid motor type at SwerveDrivetrainMk4.getSysIdRoutine", false);
            return null;
        }
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param motor The motor to run the test on either "drive" or "turn"
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(String motor, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(motor).quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param motor The motor to run the test on either "drive" or "turn"
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(String motor, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(motor).dynamic(direction);
    }
    //endregion
}
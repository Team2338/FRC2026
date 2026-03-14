// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import team.gif.robot.commands.agitator.AgitatorAutonPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPercent;
import team.gif.robot.commands.collector.collectorautos.CollectorAutoPivotDown;
import team.gif.robot.commands.autos.AutonCollectDown;
import team.gif.robot.commands.autos.AutonShoot;
import team.gif.robot.commands.autos.AutonInitialShoot;

import java.util.ArrayList;
import java.util.function.Function;
import java.util.stream.Stream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private  SendableChooser<Command> autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("Collector Run", new CollectorAutoPercent(0.5).alongWith(new AgitatorAutonPercent()));
        NamedCommands.registerCommand("Hold Collector Down", new CollectorAutoPivotDown(0.05));
        NamedCommands.registerCommand("CC-autonshoot", new AutonInitialShoot());
        NamedCommands.registerCommand("OC-autonshoot", new AutonShoot());
        NamedCommands.registerCommand("AutonCollectorDown", new AutonCollectDown());

        // Configure the trigger bindings
        configureBindings();

        autoChooser = build2338AutoChooser((stream) -> stream);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static SendableChooser<Command> build2338AutoChooser(Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {

        if(!AutoBuilder.isConfigured()) { throw new RuntimeException("AutoBuilder was not configured before attempting to build an auto chooser."); }

        SendableChooser<Command> autoChooser = new SendableChooser<>();

        ArrayList<String> names = new ArrayList<>();
        names.add("CC-autonshoot-center");
        names.add("OC-autonshoot-center");
        names.add("CollectFirst");
        names.add("New Auto");

        ArrayList<PathPlannerAuto> autoChoices = new ArrayList<>();

        for(String n : names) {
            PathPlannerAuto auto = new PathPlannerAuto(n);
            autoChoices.add(auto);
        }

        autoChooser.setDefaultOption("** None **", Commands.none());

        optionsModifier.apply(autoChoices.stream()).forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        return autoChooser;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

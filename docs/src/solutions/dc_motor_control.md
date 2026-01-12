# Controlling a DC Motor (E)

```@example dc_motor
using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D
using ModelingToolkitStandardLibrary.Electrical
using ModelingToolkitStandardLibrary.Mechanical.Rotational
using ModelingToolkitStandardLibrary.Blocks
using DifferentialEquations, Plots, ControlSystemsBase

# Define the DC Motor model
@mtkmodel Motor begin
    @structural_parameters begin
        R = 0.5      # [Ohm] armature resistance
        L = 4.5e-3   # [H] armature inductance
        k = 0.5      # [N.m/A] motor constant
        J = 0.02     # [kg.m²] inertia
        f = 0.01     # [N.m.s/rad] friction factor
    end
    @components begin
        ground = Ground()
        source = Voltage()
        resistor = Resistor(R = R)
        inductor = Inductor(L = L)
        emf = EMF(k = k)
        fixed = Fixed()
        load = Torque(use_support = false)
        inertia = Inertia(J = J)
        friction = Damper(d = f)
    end
    @equations begin
        connect(fixed.flange, emf.support, friction.flange_b)
        connect(emf.flange, friction.flange_a, inertia.flange_a)
        connect(inertia.flange_b, load.flange)
        connect(source.p, resistor.p)
        connect(resistor.n, inductor.p)
        connect(inductor.n, emf.p)
        connect(emf.n, source.n, ground.g)
    end
end

# Define the complete control system
@mtkmodel MotorControlSystem begin
    @structural_parameters begin
        pi_k = 1.1
        pi_T = 0.05
        tau_L_step = -0.3  # [N.m] amplitude of the load torque step
    end
    @components begin
        motor = Motor()
        ref = Blocks.Step(height = 1, start_time = 0)
        pi_controller = Blocks.LimPI(k = pi_k, T = pi_T, u_max = 10, Ta = 0.035)
        feedback = Blocks.Feedback()
        load_step = Blocks.Step(height = tau_L_step, start_time = 3)
        speed_sensor = SpeedSensor()
    end
    @equations begin
        connect(motor.load.flange, speed_sensor.flange)
        connect(ref.output, feedback.input1)
        connect(speed_sensor.w, :y, feedback.input2)
        connect(load_step.output, motor.load.tau)
        connect(feedback.output, pi_controller.err_input)
        connect(pi_controller.ctr_output, :u, motor.source.V)
    end
end

# Compile and solve
@mtkcompile model = MotorControlSystem()

prob = ODEProblem(model, [], (0, 6.0))
sol = solve(prob, Rodas4())

p1 = plot(sol.t, sol[model.motor.inertia.w], ylabel = "Angular Vel. in rad/s",
                label = "Measurement", title = "DC Motor with Speed Controller")
Plots.plot!(sol.t, sol[model.ref.output.u], label = "Reference")
p2 = plot(sol.t, sol[model.motor.load.tau.u], ylabel = "Disturbance in Nm", label = "")

# Linear analysis
mat, simplified_sys = get_sensitivity(model, :y)
S = ss(mat...)
bplot = bodeplot(S, plotphase=false)
nplot = nyquistplot(-ss(get_looptransfer(model, :u)[1]...))
plot(p1, p2, bplot, nplot, layout = (2, 2))
```

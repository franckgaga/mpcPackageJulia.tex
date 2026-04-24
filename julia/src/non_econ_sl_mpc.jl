# ==========================================
# ========== GLOBAL SETTINGS ===============
# ==========================================
run_benchmarks   = false
benchmark_uno    = true 
benchmark_madnlp = false

# ==========================================
# ========== STATE ESTIMATOR ===============
# ==========================================
using ModelPredictiveControl
function f!(ẋ, x, u, _ , p)
    g, L, K, m = p       # [m/s²], [m], [kg/s], [kg]
    θ, ω = x[1], x[2]    # [rad], [rad/s]
    τ = u[1]             # [Nm]
    ẋ[1] = ω
    ẋ[2] = -g/L*sin(θ) - K/m*ω + τ/m/L^2
end
h!(y, x, _ , _ ) = (y[1] = 180/π*x[1])   # [°]
p = [9.8, 0.4, 1.2, 0.3]
nu = 1; nx = 2; ny = 1; Ts = 0.1
model = NonLinModel(f!, h!, Ts, nu, nx, ny; p)
vu = ["\$τ\$ (Nm)"]
vx = ["\$θ\$ (rad)", "\$ω\$ (rad/s)"]
vy = ["\$θ\$ (°)"]
model = setname!(model; u=vu, x=vx, y=vy)

## =========================================
σQ = [0.1, 1.0]; σR=[5.0]; nint_u=[1]; σQint_u=[0.1]
estim = UnscentedKalmanFilter(model; σQ, σR, nint_u, σQint_u)

## =========================================
p_plant = copy(p); p_plant[3] = 1.25*p[3]
plant = NonLinModel(f!, h!, Ts, nu, nx, ny; p=p_plant)
N = 35; u = [0.5]; 
res = sim!(estim, N, u; plant, y_noise=[0.5])
using Plots; plot(res, plotu=false, plotxwithx̂=true)

## =========================================
## ========= Plot PDF ======================
## =========================================
using PlotThemes, Plots.PlotMeasures
theme(:default)
default(fontfamily="Computer Modern");
plt = plot(res, plotu=false, plotxwithx̂=true, size=(425, 275))
yticks!(plt[2], [0.0, 0.25, 0.5, 0.75])
yticks!(plt[3], [-0.5, 0, 0.5, 1.0, 1.5])
yticks!(plt[4], [-0.10, -0.05, 0, 0.05, 0.1])
display(plt)
savefig(plt, "$(@__DIR__())/../../fig/plot_NonLinMPC1.pdf")

## ==========================================
## ========== NONLINEAR MPC =================
## ==========================================

## =========================================
Hp, Hc, Mwt, Nwt, Cwt = 20, 2, [0.5], [2.5], Inf
transcription = MultipleShooting()
nmpc = NonLinMPC(estim; Hp, Hc, Mwt, Nwt, Cwt, transcription)
umin, umax = [-1.5], [+1.5]
nmpc = setconstraint!(nmpc; umin, umax)

## =========================================
using JuMP; unset_time_limit_sec(nmpc.optim)

## =========================================
x_0 = [0, 0]; x̂_0 = [0, 0, 0]; ry = [180]
res_r = sim!(nmpc, N, ry; plant, x_0, x̂_0)
plot(res_r)

## =========================================
## ========= Plot PDF ======================
## =========================================
using PlotThemes, Plots.PlotMeasures 
theme(:default)
default(fontfamily="Computer Modern")
plt = plot(res_r, size=(425, 200), bottom_margin=10px)
display(plt)
savefig(plt, "$(@__DIR__())/../../fig/plot_NonLinMPC2.pdf")

## =========================================
## ========= Benchmark =====================
## =========================================
using BenchmarkTools
using JuMP, Ipopt, UnoSolver

using MadNLP

if run_benchmarks
    optim = JuMP.Model(Ipopt.Optimizer, add_bridges=false)
    nmpc_ipopt = NonLinMPC(estim; Hp, Hc, Mwt, Nwt, Cwt, optim, transcription)
    nmpc_ipopt = setconstraint!(nmpc_ipopt; umin, umax)
    JuMP.unset_time_limit_sec(nmpc_ipopt.optim)
    bm = @benchmark(
            sim!($nmpc_ipopt, $N, $ry; plant=$plant, x_0=$x_0, x̂_0=$x̂_0, progress=false),
            samples=50, 
            seconds=10*60
        )
    @show btime_NMPC_track_solver_IP = median(bm)

    if benchmark_madnlp
        optim = JuMP.Model(MadNLP.Optimizer, add_bridges=false)
        nmpc_madnlp = NonLinMPC(estim; Hp, Hc, Mwt, Nwt, Cwt, optim, transcription)
        nmpc_madnlp = setconstraint!(nmpc_madnlp; umin, umax)
        JuMP.unset_time_limit_sec(nmpc_madnlp.optim)
        bm = @benchmark(
            sim!($nmpc_madnlp, $N, $ry; plant=$plant, x_0=$x_0, x̂_0=$x̂_0, progress=false),
            samples=50, 
            seconds=10*60
        )
        @show btime_NMPC_track_solver_IP2 = median(bm)
    end

    if benchmark_uno
        optim = Model(UnoSolver.Optimizer, add_bridges=false)
        set_attribute(optim, "preset", "funnelsqp")
        set_attribute(optim, "globalization_mechanism", "LS") 
        nmpc_uno = NonLinMPC(estim; Hp, Hc, Mwt, Nwt, Cwt, optim, transcription)
        nmpc_uno = setconstraint!(nmpc_uno; umin, umax)
        JuMP.unset_time_limit_sec(nmpc_uno.optim)
        bm = @benchmark(
                sim!($nmpc_uno, $N, $ry; plant=$plant, x_0=$x_0, x̂_0=$x̂_0, progress=false),
                samples=50,
                seconds=10*60
            )
        @show btime_NMPC_track_solver_SQ = median(bm)
    end
end

## =========================================
x_0 = [π, 0]; x̂_0 = [π, 0, 0]; y_step = [10]
res_d = sim!(nmpc, N, [180.0]; plant, x_0, x̂_0, y_step)
plot(res_d)

## =========================================
## ========= Plot PDF ======================
## =========================================
using PlotThemes, Plots.PlotMeasures
theme(:default)
default(fontfamily="Computer Modern")
plt = plot(res_d, size=(425, 200), bottom_margin=10px)
display(plt)
savefig(plt, "$(@__DIR__())/../../fig/plot_NonLinMPC3.pdf")

## =========================================
## ========= Benchmark =====================
## =========================================
if run_benchmarks
    bm = @benchmark(
            sim!($nmpc_ipopt, $N, $[180.0]; plant=$plant, x_0=$x_0, x̂_0=$x̂_0, y_step=$y_step, progress=false),
            samples=50,
            seconds=10*60
        )
    @show btime_NMPC_regul_solver_IP = median(bm)

    if benchmark_madnlp
        bm = @benchmark(
            sim!($nmpc_madnlp, $N, $[180.0]; plant=$plant, x_0=$x_0, x̂_0=$x̂_0, y_step=$y_step, progress=false),
            samples=50,
            seconds=10*60
        )
        @show btime_NMPC_regul_solver_IP2 = median(bm)
    end

    if benchmark_uno
        bm = @benchmark(
                sim!($nmpc_uno, $N, $[180.0]; plant=$plant, x_0=$x_0, x̂_0=$x̂_0, y_step=$y_step, progress=false),
                samples=50,
                seconds=10*60
            )
        @show btime_NMPC_regul_solver_SQ = median(bm)
    end
end

# ==========================================
# ========== ECONOMIC MPC ==================
# ==========================================
h2!(y, x, _ , _ ) = (y[1] = 180/π*x[1]; y[2]=x[2])
nu, nx, ny = 1, 2, 2
model2 = NonLinModel(f!, h2!, Ts, nu, nx, ny; p)
plant2 = NonLinModel(f!, h2!, Ts, nu, nx, ny; p=p_plant)
model2 = setname!(model2, u=vu, x=vx, y=[vy; vx[2]])
plant2 = setname!(plant2, u=vu, x=vx, y=[vy; vx[2]])
estim2 = UnscentedKalmanFilter(model2; σQ, σR, 
                               nint_u, σQint_u, i_ym=[1])


## =========================================
function JE(UE, ŶE, _ , p , _ )
    Ts = p
    τ, ω = UE[1:end-1], ŶE[2:2:end-1]
    return Ts*sum(τ.*ω)
end
p = Ts; Mwt2 = [Mwt; 0.0]; Ewt = 3.5e3
empc = NonLinMPC(estim2; Hp, Hc, Mwt=Mwt2, 
                 Nwt, Cwt, JE, Ewt, p, transcription)
empc = setconstraint!(empc; umin, umax)

## =========================================
using JuMP; unset_time_limit_sec(empc.optim)

## =========================================
x_0 = [0, 0]; x̂_0 = [0, 0, 0]; ry = [180; 0]
res2_r = sim!(empc, N, ry; plant=plant2, x_0, x̂_0)
plot(res2_r, ploty=[1])

## =========================================
## ========= Plot PDF ======================
## =========================================
using PlotThemes, Plots.PlotMeasures 
theme(:default)
default(fontfamily="Computer Modern")
plt = plot(res2_r, ploty=[1], size=(425, 200), bottom_margin=10px)
display(plt)
savefig(plt, "$(@__DIR__())/../../fig/plot_EconomMPC1.pdf")

## =========================================
function calcW(res)
    τ, ω = res.U_data[1, 1:end-1], res.X_data[2, 1:end-1]
    return Ts*sum(τ.*ω)
end
display(Dict(:W_nmpc => calcW(res_r), :W_empc => calcW(res2_r)))

## =========================================
## ========= Benchmark =====================
## =========================================
using BenchmarkTools
using JuMP, Ipopt, UnoSolver

if run_benchmarks
    optim = JuMP.Model(Ipopt.Optimizer, add_bridges=false)
    empc_ipopt = NonLinMPC(estim2; Hp, Hc, Nwt, Mwt=Mwt2, Cwt, JE, Ewt, optim, p, transcription)
    empc_ipopt = setconstraint!(empc_ipopt; umin, umax)
    JuMP.unset_time_limit_sec(empc_ipopt.optim)
    bm = @benchmark(
            sim!($empc_ipopt, $N, $ry; plant=$plant2, x_0=$x_0, x̂_0=$x̂_0, progress=false),
            samples=50, 
            seconds=10*60
        )
    @show btime_EMPC_track_solver_IP = median(bm)

    if benchmark_madnlp
        optim = JuMP.Model(MadNLP.Optimizer, add_bridges=false)
        empc_madnlp = NonLinMPC(estim2; Hp, Hc, Nwt, Mwt=Mwt2, Cwt, JE, Ewt, optim, p, transcription)
        empc_madnlp = setconstraint!(empc_madnlp; umin, umax)
        JuMP.unset_time_limit_sec(empc_madnlp.optim)
        bm = @benchmark(
                sim!($empc_madnlp, $N, $ry; plant=$plant2, x_0=$x_0, x̂_0=$x̂_0, progress=false),
                samples=50, 
                seconds=10*60
            )
        @show btime_EMPC_track_solver_IP2 = median(bm)
    end

    if benchmark_uno
        optim = Model(UnoSolver.Optimizer, add_bridges=false)
        set_attribute(optim, "preset", "funnelsqp")
        set_attribute(optim, "globalization_mechanism", "LS") 
        empc_uno = NonLinMPC(estim2; Hp, Hc, Nwt, Mwt=Mwt2, Cwt, JE, Ewt, optim, p, transcription)
        empc_uno = setconstraint!(empc_uno; umin, umax)
        JuMP.unset_time_limit_sec(empc_uno.optim)
        bm = @benchmark(
                sim!($empc_uno, $N, $ry; plant=$plant2, x_0=$x_0, x̂_0=$x̂_0, progress=false),
                samples=50,
                seconds=10*60
            )
        @show btime_EMPC_track_solver_SQ = median(bm)
    end
end

## =========================================
x_0 = [π, 0]; x̂_0 = [π, 0, 0]; y_step = [10; 0]
res2_d = sim!(empc, N, ry; plant=plant2, x_0, x̂_0, y_step)
plot(res2_d, ploty=[1])

## =========================================
display(Dict(:W_nmpc => calcW(res_d), :W_empc => calcW(res2_d)))

## =========================================
## ========= Plot PDF ======================
## =========================================
using PlotThemes, Plots.PlotMeasures
theme(:default)
default(fontfamily="Computer Modern")
plt = plot(res2_d, ploty=[1], size=(425, 200), bottom_margin=10px)
display(plt)
savefig(plt, "$(@__DIR__())/../../fig/plot_EconomMPC2.pdf")

## =========================================
## ========= Benchmark =====================
## =========================================
if run_benchmarks
    bm = @benchmark(
            sim!($empc_ipopt, $N, $ry; plant=$plant2, x_0=$x_0, x̂_0=$x̂_0, y_step=$y_step, progress=false),
            samples=50,
            seconds=10*60
        )
    @show btime_EMPC_regul_solver_IP = median(bm)

    if benchmark_madnlp
        bm = @benchmark(
            sim!($empc_madnlp, $N, $ry; plant=$plant2, x_0=$x_0, x̂_0=$x̂_0, y_step=$y_step, progress=false),
            samples=50,
            seconds=10*60
        )
        @show btime_EMPC_regul_solver_IP2 = median(bm)
    end

    if benchmark_uno
        bm = @benchmark(
                sim!($empc_uno, $N, $ry; plant=$plant2, x_0=$x_0, x̂_0=$x̂_0, y_step=$y_step, progress=false),
                samples=50,
                seconds=10*60
            )
        @show btime_EMPC_regul_solver_SQ = median(bm)
    end
end

## ==========================================
## ====== SUCCESSIVE LINEARIZATION MPC ======
## ==========================================
# using Pkg; Pkg.add(["JuMP","DAQP"])
using JuMP, DAQP
optim = JuMP.Model(DAQP.Optimizer, add_bridges=false)

## ==========================================
linmodel = linearize(model, x=[0, 0], u=[0])
kf = KalmanFilter(linmodel; σQ, σR, nint_u, σQint_u)
mpc3 = LinMPC(kf; Hp, Hc, Mwt, Nwt, Cwt, optim)
mpc3 = setconstraint!(mpc3; umin, umax)

## ==========================================
function sim2!(mpc, nlmodel, N, ry, plant, x, x̂, y_step)
    U, Y, Ry = zeros(1, N), zeros(1, N), zeros(1, N)
    setstate!(plant, x); setstate!(mpc, x̂)
    initstate!(mpc, [0], plant())
    linmodel = linearize(nlmodel; u=[0], x=x̂[1:2])
    setmodel!(mpc, linmodel)
    for i = 1:N
        y = plant() + y_step
        x̂ = preparestate!(mpc, y)
        u = mpc(ry)
        linearize!(linmodel, nlmodel; u, x=x̂[1:2])
        setmodel!(mpc, linmodel) 
        U[:,i], Y[:,i], Ry[:,i] = u, y, ry
        updatestate!(mpc, u, y)
        updatestate!(plant, u)
    end
    U_data, Y_data, Ry_data = U, Y, Ry
    return SimResult(mpc, U_data, Y_data; Ry_data)
end

## ==========================================
x_0 = [0, 0]; x̂_0 = [0, 0, 0]; ry = [180]
res3_r = sim2!(mpc3, model, N, ry, plant, x_0, x̂_0, [0])
plot(res3_r)

## =========================================
## ========= Plot PDF ======================
## =========================================
using PlotThemes, Plots.PlotMeasures 
theme(:default)
default(fontfamily="Computer Modern")
plt = plot(res3_r, size=(425, 200), bottom_margin=10px)
display(plt)
savefig(plt, "$(@__DIR__())/../../fig/plot_SuccLinMPC1.pdf")

## =========================================
## ========= Benchmark =====================
## =========================================
using BenchmarkTools

if run_benchmarks
    x_0 = [0, 0]; x̂_0 = [0, 0, 0]; ry = [180]; y_step=[0]
    bm = @benchmark(
            sim2!($mpc3, $model, $N, $ry, $plant, $x_0, $x̂_0, $y_step),
            samples=500, 
            seconds=10*60
        )
    @show btime_SLMPC_track_solver_AS = median(bm)
end

## =========================================
x_0 = [π, 0]; x̂_0 = [π, 0, 0]; ry = [180]
res3_d = sim2!(mpc3, model, N, ry, plant, x_0, x̂_0, [10])
plot(res3_d)

## =========================================
## ========= Plot PDF ======================
## =========================================
using PlotThemes, Plots.PlotMeasures 
theme(:default)
default(fontfamily="Computer Modern")
plt = plot(res3_d, size=(425, 200), bottom_margin=10px)
display(plt)
savefig(plt, "$(@__DIR__())/../../fig/plot_SuccLinMPC2.pdf")

## =========================================
## ========= Benchmark =====================
## =========================================
if run_benchmarks
    x_0 = [π, 0]; x̂_0 = [π, 0, 0]; ry = [180]; y_step=[10]
    bm = @benchmark(
            sim2!($mpc3, $model, $N, $ry, $plant, $x_0, $x̂_0, $y_step),
            samples=500, 
            seconds=10*60
        )
    @show btime_SLMPC_regul_solver_AS = median(bm)
end
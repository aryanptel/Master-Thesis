
#lorentz system

using DifferentialEquations
using Plots

function lorenz(du,u,p,t)
    mt = p[4]
 du[1] = p[4]*(p[1]*(u[2]-u[1]))
 du[2] = p[4]*(u[1]*(p[2]-u[3]) - u[2])
 du[3] = p[4]*(u[1]*u[2] - p[3]*u[3])
end

u0 = [1.0;0.0;0.0]
tspan = (0.0,100.0)
p = (10.0,24.0,8/3, 1) #(alpha, beta, gamma, mt)
prob = ODEProblem(lorenz, u0, tspan,p)
sol = solve(prob);
xyzt = plot(sol[1:200], plotdensity=10000,lw=1.5)
#xy = plot(sol, plotdensity=10000, vars=(1,2))
#xz = plot(sol, plotdensity=10000, vars=(1,3))
#yz = plot(sol, plotdensity=10000, vars=(2,3))
#xyz = plot(sol, plotdensity=10000, vars=(1,2,3))
#plot(plot(xyzt,xyz),plot(xy, xz, yz, layout=(1,3),w=1), layout=(2,1))

p = (10.0,25.0,8/3, 0.5) #(alpha, beta, gamma, mt)
prob = ODEProblem(lorenz, u0, tspan,p)
sol = solve(prob);
xyzt = plot(sol[1:200], plotdensity=10000,lw=1.0)
xz = plot(sol, plotdensity=10000, vars=(1,2))

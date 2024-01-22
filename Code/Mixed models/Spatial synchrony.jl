using DifferentialEquations, Plots
using Dates

#system
function rossler_system(du, u1, mu, t)
    x1, x2, y1, y2 = u1
    
    a1= 4
    a2  = 4

    

    du[1] = x2
    du[2] =  mu*x2*(1- x1^2) - x1

    du[3] = x2+x1-y1-a1
    du[4] = mu*x2*(1- x1^2) - y2 +x2 -x1 -a2

end

# Set the time span for integration
t_start = 0.0
t_end = 1000.0
t_step = 0.002
t_span = (t_start, t_end)

mu = 0.5
# Set the initial conditions
initial_state = [10, 2, 5, 5]



# Solve the coupled Lorenz systems
prob = ODEProblem(rossler_system, initial_state, t_span, mu)
alg = Tsit5()
solution = solve(prob,alg, saveat=t_step );
lim =length(solution)

#li = 1:200000
li = 1:lim
#li = 1:1000001

x1 = solution[1, li];
y1 = solution[2, li];
x2 = solution[3, li];
y2 = solution[4, li];

t = solution.t[li];
length(y2)

#error for projective syncronisation
errrx = []
for i in 1:length(x1)
    errx = ((x1[i] - x2[i]))
    push!(errrx, errx)
end

errry = []
for i in 1:length(y1)
    erry = ((y1[i] - y2[i]))
    push!(errry, erry)
end



Plots.plot(x1,y1, title = "rossler system ",lw=1, label ="Drive",color = :blue)
Plots.plot!(x2,y2, title = "rossler system ",lw=1, label ="Response",color = :red);
xlabel!("x");
ylabel!("y");
zlabel!("z")

Plots.plot(t, (errrx), title = "rossler system with 2 c", lw=1, label ="Z^$m", color = :green)
xlabel!("t");
ylabel!("|z1^$m/z2 -1|")

Plots.plot(x1, y1,lw=1, label="",title = "X1^$s vs X2 ")
Plots.plot(x2, y2, lw=1, label="",title ="Y1^$n vs Y2 ")

Plots.plot(z1.^m, z2, lw=1,title ="Z1^$m vs Z2 ", label="", color = :black )
mn = 0:0.1:100
Plots.plot!(mn, mn, lw=1, label="")
plot(t, errrx, lw=1, label ="X axis sync", color = :blue)




Plots.plot(t,x1.^s, title = "rossler system X", lw=1, label ="X-Drive", color = :blue);
Plots.plot!(t,x2, title = "rossler system ", lw=1, label ="X Response", color = :red)

Plots.plot(t,y1.^n, title = "rossler system Y", lw=1, label ="Y-drive", color = :blue);
Plots.plot!(t,y2, title = "rossler system ", lw=1, label ="Y-Response", color = :red)

Plots.plot(t,z1.^m, title = "rossler system Z", lw=1, label ="Z-drive", color = :blue);
Plots.plot!(t,z2, title = "rossler system ", lw=1,label ="Z-response", color = :red)

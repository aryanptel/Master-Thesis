using DifferentialEquations, Plots
using Dates

#system
function rossler_system(du, u1, p1, t)
    x1, y1, z1, x2, y2, z2 = u1
    a, b, c,m,n,s = p1


    s4 = +y2 + s *x1^(-1 + s)* (-y1 - z1) + z2
    s5 = -x2 + n* y1^(-1 + n)* (x1 + a *y1) - a* y2
    s6 = -b + m* z1^(-1 + m) *(b + (-c + x1)* z1) - (-c + x2)* z2
    j1 = x1 - x2
    j2 = y1 - y2
    j3 = c*m*(z1^m - z2)
            

    du[1] = -y1 - z1
    du[2] =  x1 + a*y1
    du[3] = b + z1*(x1 - c)
    
    du[4] = -y2 - z2   +s4 + j1
    du[5] = x2 + a*y2   +s5 + j2
    du[6] =  b+z2*(x2-c)   +s6 + j3

end

# Set the parameters

a = 0.2
b = 0.2
c = 14

# Set the time span for integration
t_start = 0.0
t_end = 1000.0
t_step = 0.002
t_span = (t_start, t_end)

#non-linear dependence
m = 1
n = 1
s = 1
p1 = (a,b,c,m,n,s)

# Set the initial conditions
initial_state = [10, 2, 5, 5, 0, 19]



# Solve the coupled Lorenz systems
prob = ODEProblem(rossler_system, initial_state, t_span, p1)
alg = Tsit5()
solution = solve(prob,alg, saveat=t_step );
lim =length(solution)

#li = 1:200000
li = 400000:lim
#li = 1:1000001

x1 = solution[1, li];
y1 = solution[2, li];
z1 = solution[3, li];
x2 = solution[4, li];
y2 = solution[5, li];
z2 = solution[6, li];

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

errrz = []
for i in 1:length(z1)
    errz = abs((z1[i]^m / z2[i])-1)
    push!(errrz, errz)
end


Plots.plot(x1,y1,z1, title = "rossler system ",lw=1, label ="Drive",color = :blue)
Plots.plot!(x2,y2,z2, title = "rossler system ",lw=1, label ="Response",color = :red);
xlabel!("x");
ylabel!("y");
zlabel!("z")

Plots.plot(t, (errrz), title = "rossler system with 2 c", lw=1, label ="Z^$m", color = :green)
xlabel!("t");
ylabel!("|z1^$m/z2 -1|")

Plots.plot(x1.^s, x2,lw=1, label="",title = "X1^$s vs X2 ")
Plots.plot(y1.^n, y2, lw=1, label="",title ="Y1^$n vs Y2 ")

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

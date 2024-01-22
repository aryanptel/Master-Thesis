using DifferentialEquations, Plots
using Dates

#system
function rossler_system(du, u1, p1, t)
    x1, y1, z1, x2, y2, z2 = u1
    a, b, c,m,n,s = p1

    
    
        
    du[1] = -y1 - z1
    du[2] =  x1 + a*y1
    du[3] = b + z1*(x1 - c)
    
    du[4] = -y2 - z2   
    du[5] = x2 + a*y2  -(y2 - y1)
    du[6] =  b+z2*(x2-c)
end

# Set the parameters

a = 0.2

b = 0.2
c = 9

#non-linear dependence
m = 1
n = 1
s = 1
p1 = (a,b,c,m,n,s)

# Set the initial conditions
initial_state = [15.0, 2.0, 5.0,20, 10, 20.0]

# Set the time span for integration
t_start = 0.0
t_end = 1000.0
t_step = 0.001
t_span = (t_start, t_end)

# Solve the coupled Lorenz systems
prob = ODEProblem(rossler_system, initial_state, t_span, p1)
alg = Tsit5()
solution = solve(prob,alg, saveat=0.001 );
lim =length(solution)

#li = 1:200000
li = 1:lim
#li = 1:1000001
#title!("Synchronization of Linearly Coupled Lorenz Systems")
#plot!(legend=true, grid=true, camera = (40,40))
x1 = solution[1, li];
y1 = solution[2, li];
z1 = solution[3, li];
x2 = solution[4, li];
y2 = solution[5, li];
z2 = solution[6, li];
t = solution.t[li];
length(x1)

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
z_sqr = []

for i in 1:length(z1)
    z_s = z1[i]^m
    push!(z_sqr, z_s)
end

Plots.plot(x1,y1,z1, title = "rossler system ",lw=1, label ="Drive",color = :blue)
Plots.plot!(x2,y2,z2, title = "rossler system ",lw=1, label ="Response",color = :red);
xlabel!("x");
ylabel!("y");
zlabel!("z")

Plots.plot(x1.^s, x2,lw=1, label="",title = "X1^$s vs X2 ")
Plots.plot(y1.^n, y2, lw=1, label="",title ="Y1^$n vs Y2 ")

Plots.plot(z1.^m, z2, lw=1,title ="Z1^$m vs Z2 ", label="", color = :black )
mn = 0:0.1:100
Plots.plot!(mn, mn, lw=1, label="")

#=
fig = figure()
# Create a 1x3 grid of subplots
ax1 = subplot(3, 1, 1)
ax2 = subplot(3, 1, 2)
ax3 = subplot(3, 1, 3)
ax1.plot(t, errrx, lw=1, label ="X axis sync", color = :blue)
#xlabel!("t")
#ylabel!("x2-x1")
ax2.plot(t, errry, lw=1, label ="Y axis sync", color = :red)
#xlabel!("t")
#ylabel!("y2-y1")
ax3.plot(t, errrz, lw=1, label ="Z axis sync", color = :green)
#xlabel!("t")
#ylabel!("z1^4/z2 -1")
subplots_adjust(hspace=0.5, wspace=0.5)
ax1.set_title("X2-X1")
ax2.set_title("Y2-Y1")
ax3.set_title("|Z2^$m/Z1 - 1|")
show()
=#

Plots.plot(t, (errry), title = "rossler system ", lw=1, label ="Z axis sync", color = :green);
xlabel!("t");
ylabel!("log|z1^$m/z2 -1|")


Plots.plot(t,x1.^s, title = "rossler system X", lw=1, label ="X-Drive", color = :blue);
Plots.plot!(t,x2, title = "rossler system ", lw=1, label ="X Response", color = :red)

Plots.plot(t,y1.^n, title = "rossler system Y", lw=1, label ="Y-drive", color = :blue);
Plots.plot!(t,y2, title = "rossler system ", lw=1, label ="Y-Response", color = :red)

Plots.plot(t,z1.^m, title = "rossler system Z", lw=1, label ="Z-drive", color = :blue);
Plots.plot!(t,z2, title = "rossler system ", lw=1,label ="Z-response", color = :red)

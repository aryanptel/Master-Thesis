using DifferentialEquations, Plots
using Dates
using PyPlot

#system
function rossler_system(du, u1, p1, t)
    x1, y1, z1, x2, y2, z2 = u1
    a, b, c,m = p1

    
    du[1] = -y1 - z1
    du[2] =  x1 + a*y1
    du[3] = b + z1*(x1 - c)
    
    du[4] = -y2 - z2   -(y1 - y2 + z1 - z2) -(x2 - x1)
    du[5] = x2 + a*y2   -(-x1 + x2 - a* y1 + a* y2)-(y2 - y1)
    du[6] =  b+z2*(x2-c)    -b + m *(z1^(-1 + m)) *(b + (-c + x1)* z1) - (-c + x2) *z2 -c*(z2 - z1^m)
    
end

# Set the parameters

a = 0.2
b = 0.2
c = 14
m = 3/2

p1 = (a,b,c,m)

# Set the initial conditions
initial_state = [15.0, 2.0, 5.0, -200.0, -1000.0, -190.0]

# Set the time span for integration
t_start = 0.0
t_end = 1000.0
t_step = 0.001
t_span = (t_start, t_end)

# Solve the coupled Lorenz systems
prob = ODEProblem(rossler_system, initial_state, t_span, p1)
alg = Tsit5()
solution = solve(prob,alg, saveat=t_step );
lim =length(solution)

#li = 10000:200000
li = 800000:lim
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
    errz = ((z1[i]^m / z2[i])-1)
    push!(errrz, errz)
end
z_sqr = []

for i in 1:length(z1)
    z_s = z1[i]^m
    push!(z_sqr, z_s)
end

plot(x1,y1,z1, title = "rossler system ",lw=1, label ="Drive",color = :blue)
plot!(x2,y2,z2, title = "rossler system ",lw=1, label ="Response",color = :red);
xlabel!("x");
ylabel!("y");
zlabel!("z")

plot(x1, x2,lw=1,title = "X1 vs X2 ")
plot(y1, y2, lw=1,title ="Y1 vs Y2 ")

plot(z1, z2, lw=1, lable ="z1vsz2", color = :black );
mn = 0:0.1:100
plot!(mn, mn.^m, lw=1,title ="y = x^$m \nZ1 vs z2", lable ="y = x^$m")

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

plot(t, errrz, title = "rossler system ",ylimits=(-1,1), lw=1, label ="Z axis sync", color = :green);
xlabel!("t");
ylabel!("z1^$m/z2 -1")


plot(t,x1, title = "rossler system ", lw=1, label ="X-Drive", color = :blue);
plot!(t,x2, title = "rossler system ", lw=1, label ="X Response", color = :red)

plot(t,y1, title = "rossler system ", lw=1, label ="Y-drive", color = :blue);
plot!(t,y2, title = "rossler system ", lw=1, label ="Y-Response", color = :red)

plot(t,z_sqr, title = "rossler system ", lw=1, label ="Z-drive", color = :blue);
plot!(t,z2, title = "rossler system ", lw=1,label ="Z-response", color = :red)

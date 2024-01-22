using DifferentialEquations, Plots
pyplot()
#system
function rossler_system(du, u1, p1, t)
    x1, y1, z1, x2, y2, z2 = u1
    a, b, c,p,q,r = p1

    #p = 3
    #q = 3
    #r = 3

    du[1] = -y1 - z1
    du[2] =  x1 + a*y1
    du[3] = b + z1*(x1 - c)
    
    du[4] = -y2 - z2   + y2 + p *(-y1 - z1) + z2    -10*(x2 - p* x1)
    du[5] = x2 + a*y2   -x2 + q *(x1 + a *y1) - a *y2
    du[6] =  b+z2*(x2-c)    -b + r *(b + (-c + x1)* z1)-(-c + x2)* z2
    
end

# Set the parameters

a = 0.2
b = 0.2
c = 14
p = 2
q = 2
r = 2

p1 = (a,b,c,p,q,r)

# Set the initial conditions
initial_state = [15.0, 0.0, 5.0, 200.0, 0.0, 7.0]

# Set the time span for integration
t_start = 0.0
t_end = 5000.0
t_step = 0.0001
t_span = (t_start, t_end)



# Solve the coupled Lorenz systems
prob = ODEProblem(rossler_system, initial_state, t_span, p1)
alg = Tsit5()
solution = solve(prob,alg, saveat=t_step )
#length(solution)

#plot(solution[:, 50000:100000],vars=(1,2,3), title = "rossler system ",lw=1, label ="Drive",color = :blue)

#plot!(solution[:, 50000:100000],vars=(4,5,6), title = "Syncronisation of rossler system\nwith Non-linear(z2 = z1^2) coupling",lw=1,color = :red, label ="Response")

# Create the 3D plot
li = 400000:500000
li = 1:500000
#title!("Synchronization of Linearly Coupled Lorenz Systems")
#plot!(legend=true, grid=true, camera = (40,40))
x1 = solution[1, li]
y1 = solution[2, li]
z1 = solution[3, li]
x2 = solution[4, li]
y2 = solution[5, li]
z2 = solution[6, li]
t = solution.t[li]

plot(x1,y1,z1, title = "rossler system ",lw=1, label ="Drive",color = :blue)
plot!(x2,y2,z2, title = "rossler system ",lw=1, label ="Response",color = :red)

xlabel!("x")
ylabel!("y")
zlabel!("z")



errrx = []
for i in 1:length(x1)
    errx = ((p * x1[i] - x2[i]))
    push!(errrx, errx)
end

errry = []
for i in 1:length(y1)
    erry = ((q*y1[i] - y2[i]))
    push!(errry, erry)
end

errrz = []
for i in 1:length(z1)
    errz = ((r*z1[i] - z2[i]))
    push!(errrz, errz)
end

plot(t, errrx, title = "rossler system ", lw=1, label ="X axis sync", color = :blue)
plot(t, errry, title = "rossler system ", lw=1, label ="Y axis sync", color = :red)
plot(t, errrz, title = "rossler system ", lw=1, label ="Z axis sync", color = :green)

plot(t,(p*x1-x2), title = "rossler system ", lw=1, label ="X axis", color = :blue)
plot!(t,x2, title = "rossler system ", lw=1, label ="X axis", color = :red)

plot(t,y1, title = "rossler system ", lw=1, label ="X axis", color = :blue)
plot!(t,y2, title = "rossler system ", lw=1, label ="X axis", color = :red)

plot(t,x1, title = "rossler system ", lw=1, label ="X axis", color = :blue)
plot!(t,x2, title = "rossler system ", lw=1, label ="X axis", color = :red)

(*
function process_list(input_list)
    output_list = similar(input_list)  # Create an output list of the same size as the input list
    
    for i in 1:length(input_list)
        if input_list[i] > 0
            output_list[i] = 1
        elseif input_list[i] < 0
            output_list[i] = -1
        else
            output_list[i] = 0
        end
    end
    
    return output_list
end

x11 = process_list(x1)
x22 = process_list(x2)
z11 = process_list(z1)
plot(x11)
plot!(x22)
plot!(z11)

scatter(solution.t, x11, legend=false, markersize=4, xlabel="X", ylabel="Y", title="Scatter Plot")
*)
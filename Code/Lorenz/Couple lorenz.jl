#syncronisation of coupled lorentz oscillator


using DifferentialEquations, Plots
gr()
# Define the Lorenz system

#=bi-directional coupling
    du[1] = sigma * (y - x) + coupling *sigma* (v/p[1][1])
    du[2] = x * (rho - z) - y  + coupling * (rho*u-u*w)/p[2][2]
    du[3] = x * y - beta * z + coupling * (u*v)/p[3][3]
    
    du[4] = sigma * (v - u) + coupling *sigma* (p[1][1]*y)
    du[5] = u * (rho - w) - v + coupling * (rho*x - x*z)*p[2][2]
    du[6] = u * v - beta * w + coupling * (x*y)*p[3][3]

#master-slave coupling
    du[1] = sigma * (y - x) 
    du[2] = x * (rho - z) - y  
    du[3] = x * y - beta * z 
    
    du[4] = sigma * (v - u) + coupling *sigma* (p[1][1]-p[2][2])*y
    du[5] = u * (rho - w) - v + coupling * (-rho*(p[1][1]-p[2][2])*x + x*z*(p[1][1]*p[3][3]-p[2][2]))
    du[6] = u * v - beta * w - coupling *(p[1][1]*p[2][2]-p[3][3])* (x*y)
    
=#



#system
function lorenz_system(du, u1, p1, t)
    x, y, z, u, v, w = u1
    sigma, rho, beta = p1
    coupling = 1

    a = 2.6
    b = 1.8
    c = 3.6
    p = [[a, 0, 0], [0, b, 0], [0, 0, c]]
    #p = funcform()

    du[1] = sigma * (y - x) 
    du[2] = x * (rho - z) - y  
    du[3] = x * y - beta * z 
    
    du[4] = sigma * (v - u) + coupling *sigma* (p[1][1]-p[2][2])*y
    du[5] = u * (rho - w) - v + coupling * (-rho*(p[1][1]-p[2][2])*x + x*z*(p[1][1]*p[3][3]-p[2][2]))
    du[6] = u * v - beta * w - coupling *(p[1][1]*p[2][2]-p[3][3])* (x*y)
    
end

# Set the parameters

sigma = 10.0
rho = 28.0
beta = 8/3

p = (sigma,rho,beta)

# Set the initial conditions
initial_state = [1.0, 0.0, 0.0, 2.0, 0.0, 0.0]

# Set the time span for integration
t_start = 0.0
t_end = 100.0
t_step = 0.0001
t_span = (t_start, t_end)



# Solve the coupled Lorenz systems
prob = ODEProblem(lorenz_system, initial_state, t_span, p)
alg = RK4()
solution = solve(prob,alg, dt = t_step )

length(solution)

plot(solution,vars=(1,2,3), title = "Lorenz Attractor",lw=1, label ="Drive",color = :blue)
plot!(solution,vars=(4,5,6), title = "Lorenz Attractor with master-slave coupling",lw=1,color = :red, label ="Response")

# Create the 3D plot


xlabel!("x")
ylabel!("y")
zlabel!("z")

#title!("Synchronization of Linearly Coupled Lorenz Systems")
plot!(legend=true, grid=true, camera = (45,10))


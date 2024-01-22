using DifferentialEquations, Plots
function param()
    N = 1000
    A = 1
    B = 1
    J = 0.1
    K = 1
    return N,A,B,J,K

end

N,A,B,J,K = param()

function model(u)

    N,A,B,J,K = param()
    
    N = length(u) ÷ 3
    
    
    x = u[1,:]
    y = u[2,:]

    vx = u[3,:]
    vy = u[4,:]

    dx = sigma * (y - x) 
    dy = x * (rho - z) - y  + coupling * (rho*u-u*w)/b
    dz = x * y - beta * z
    
    du = similar(u)
    
    for i in 1:N

        cplx = 0
        for j in 1:N

            if i != j
                cplx += (x[j]-x[i])*(A + J*cos(theta[j]- theta[i]))/((x[j]-x[i])^2 + (y[j]-y[i])^2 )^(1/2) - B*(x[j] - x[i])/((x[j]-x[i])^2 + (y[j]-y[i])^2) 

            end
        end

        cply = 0
        for j in 1:N
            if i != j
                cply += (y[j]-y[i])*(A+J * cos(theta[j]- theta[i]))/((x[j]-x[i])^2 + (y[j]-y[i])^2 )^(1/2) - B*(y[j] - y[i])/((x[j]-x[i])^2 + (y[j]-y[i])^2)
    
            end

        end

        cpl2 = 0
        for j in 1:N
            if i != j
                cpl2 = sin(theta[j]- theta[i])/((x[j]-x[i])^2 + (y[j]-y[i])^2)^(1/2)

            end

        end


        dx1 = vx1 
        dvx1 = 0.5*(vx2-vx1) + (x2-x1) - K*(x2-x1)/(norm + eps) - 0.2*x1

        dy1 = vy1
        dvy1 = 0.5*(vy2-vy1) + (y2-y1) - K*(y2-y1)/(norm + eps) - 0.2*y1
        
        dx2 = vx2
        dvx2 = 0.5*(vx1-vx2) + (x1-x2) - K*(x1-x2)/(norm + eps) 
        
        dy2 = vy2
        dvy2 = 0.5*(vy1-vy2) + (y1-y2) - K*(y1-y2)/(norm + eps)

        dx = sigma * (y - x) 
        dy = x * (rho - z) - y  + coupling * (rho*u-u*w)/b
        dz = x * y - beta * z
        
        du = sigma * (v - u) + coupling *sigma * (y)*a
        dv = u * (rho - w) - v + coupling * (rho*x - x*z)*b
        dw = u * v - beta * w + coupling * (x*y)*c

    """
        du[1,i] =  (1/N)* cplx
        du[2,i] =  (1/N)*cply
        du[3,i] =  (K/N)* cpl2
    """
    end
    return du

end

function rk4_step(u, dt)
    k1 = dt * model(u)
    k2 = dt * (model(u + k1/2))
    k3 = dt * (model(u + k2/2))
    k4 = dt * (model(u + k3))
    return u + (k1 + 2*k2 + 2*k3 + k4) / 6
end

# Set the time span for integration

t0 = 0.0
tfinal = 100.0
dt = 0.1
num_steps = Int(round((tfinal - t0) / dt))
u0  = rand(3,N)

# Initialize solution array
solution = zeros(3,N, num_steps+1);
solution[:,:, 1] = u0

# Time integration using RK4
u = copy(u0)
for i in 1:num_steps
    u = rk4_step(u, dt)
    solution[:,:, i+1] = u
end

# Extract time points and solutions
t = collect(t0:dt:tfinal)
u = solution

@gif for i in 1:1000
    plot(u[1,:,i], u[2,:,i], seriestype=:scatter, label="data")
end

plot(u[1,:,end], u[2,:,end], seriestype=:scatter, label="data")

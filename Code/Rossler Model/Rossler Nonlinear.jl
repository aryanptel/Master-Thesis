{\rtf1\ansi\ansicpg1252\cocoartf2639
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\paperw11900\paperh16840\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\pard\tx566\tx1133\tx1700\tx2267\tx2834\tx3401\tx3968\tx4535\tx5102\tx5669\tx6236\tx6803\pardirnatural\partightenfactor0

\f0\fs24 \cf0 using DifferentialEquations, Plots\
#system\
function rossler_system(du, u1, p1, t)\
    x1, y1, z1, x2, y2, z2 = u1\
    a, b, c = p1\
\
    \
    du[1] = -y1 - z1\
    du[2] =  x1 + a*y1\
    du[3] = b + z1*(x1 - c)\
    \
    du[4] = -y2 - z2   -(y1 - y2 + z1 - z2) -(x2 - x1)\
    du[5] = x2 + a*y2   -(-x1 + x2 - a* y1 + a* y2)-(y2 - y1)\
    du[6] =  b+z2*(x2-c)    -b + 2 *z1* (b + (-c + x1) *z1) - (-c + x2)* z2-(z2 - z1^2)\
    \
end\
\
# Set the parameters\
\
a = 0.2\
b = 0.2\
c = 14\
\
\
p1 = (a,b,c)\
\
# Set the initial conditions\
initial_state = [15.0, 2.0, 5.0, 200.0, 100.0, 190.0]\
\
# time\
t_start = 0.0\
t_end = 1000.0\
t_step = 0.001\
t_span = (t_start, t_end)\
\
# Solve the coupled Lorenz systems\
prob = ODEProblem(rossler_system, initial_state, t_span, p1)\
alg = Tsit5()\
solution = solve(prob,alg, saveat=t_step )\
#li = 70000:100000\
#li = 200000:300000\
li = 1:100000\
#title!("Synchronization of Linearly Coupled Lorenz Systems")\
#plot!(legend=true, grid=true, camera = (40,40))\
x1 = solution[1, li]\
y1 = solution[2, li]\
z1 = solution[3, li]\
x2 = solution[4, li]\
y2 = solution[5, li]\
z2 = solution[6, li]\
t = solution.t[li]\
\
\
\
plot(x1,y1,z1, title = "rossler system ",lw=1, label ="Drive",color = :blue)\
plot!(x2,y2,z2, title = "rossler system ",lw=1, label ="Response",color = :red)\
\
xlabel!("x")\
ylabel!("y")\
zlabel!("z")\
}
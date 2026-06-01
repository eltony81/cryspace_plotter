require "cryspace"

# 1. Plant (RLC) - Standard parameters from UMich Tutorial
R = 1.0; L = 0.5; C = 0.1
a = [[0.0, 1/C], [-1/L, -R/L]].to_tensor
b = [[0.0], [1/L]].to_tensor
c = [[1.0, 0.0]].to_tensor
d = [[0.0]].to_tensor
rlc_plant = CrySpace::StateSpace.new(a, b, c, d)

# 2. PID Controller (Forward Path - optimized to eliminate overshoot)
kp, ki, kd = 0.8, 1.0, 0.4
tf = 0.01 # Derivative filter for realizability
pid_num = [(kp*tf + kd), (kp + ki*tf), ki].to_tensor
pid_den = [tf, 1.0, 0.0].to_tensor
pid_controller = CrySpace::TransferFunction.new(pid_num, pid_den).to_statespace

# Forward Path System (Plant * Controller)
sys_forward = rlc_plant * pid_controller

# 3. Sensor Dynamics in the Feedback Path (More complex feedback)
# In many practical systems, the feedback sensor is not instantaneous. 
# Here we model a sensor with a low-pass measurement lag (time constant tau = 0.05 seconds):
# H(s) = 1 / (0.05s + 1)
sensor_tau = 0.05
sensor_num = [1.0].to_tensor
sensor_den = [sensor_tau, 1.0].to_tensor
sensor_dynamics = CrySpace::TransferFunction.new(sensor_num, sensor_den).to_statespace

# 4. Closed-loop with sensor dynamics in the feedback path
# Under negative feedback, y = sys_forward(u - sensor_dynamics(y))
sys_cl = sys_forward.feedback(sensor_dynamics)

# 5. Simulation
t_vec = Float64Tensor.linear_space(0.0, 15.0, 1501)
u_step = Float64Tensor.ones([1, 1501])

_, _, y_open = rlc_plant.simulate(t_vec, u: u_step.dup)
_, _, y_cl = sys_cl.simulate(t_vec, u: u_step.dup)

# 6. Export and Plot
File.open("results_sensor.csv", "w") do |file|
  file.puts "Time,OpenLoop,ClosedLoopSensorPID"
  1501.times { |i| file.puts "#{t_vec[i].value},#{y_open[i, 0].value},#{y_cl[i, 0].value}" }
end

system "gnuplot -e \"set datafile separator ','; set key autotitle columnheader; set term png size 800,600; set output 'simulation_sensor_plot.png'; set title 'RLC Response with Sensor Dynamics Lag'; set xlabel 'Time (s)'; set ylabel 'Voltage (V)'; set yrange [-0.2:1.6]; plot 'results_sensor.csv' using 1:2 with lines, '' using 1:3 with lines\""

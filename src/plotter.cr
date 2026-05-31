require "cryspace"

# 1. Plant (RLC) - Parametri standard da UMich Tutorial
R = 1.0; L = 0.5; C = 0.1
a = [[0.0, 1/C], [-1/L, -R/L]].to_tensor
b = [[0.0], [1/L]].to_tensor
c = [[1.0, 0.0]].to_tensor
d = [[0.0]].to_tensor
rlc_plant = CrySpace::StateSpace.new(a, b, c, d)

# 2. PID Controller - Guadagni stabili da UMich
kp, ki, kd = 4.0, 8.0, 1.3
tf = 0.01 # Filtro derivativo per stabilità
pid_num = [(kp*tf + kd), (kp + ki*tf), ki].to_tensor
pid_den = [tf, 1.0, 0.0].to_tensor
pid_controller = CrySpace::TransferFunction.new(pid_num, pid_den).to_statespace

# 3. Closed-loop
sys_cl = (rlc_plant * pid_controller).feedback([[1.0]].to_tensor)

# 4. Simulation
t_vec = Float64Tensor.linear_space(0.0, 15.0, 1501)
u_step = Float64Tensor.ones([1, 1501])

_, _, y_open = rlc_plant.simulate(t_vec, u: u_step.dup)
_, _, y_cl = sys_cl.simulate(t_vec, u: u_step.dup)

# 5. Export and Plot
File.open("results.csv", "w") do |file|
  file.puts "Time,OpenLoop,ClosedLoopPID"
  1501.times { |i| file.puts "#{t_vec[i].value},#{y_open[i, 0].value},#{y_cl[i, 0].value}" }
end

system "gnuplot -e \"set datafile separator ','; set key autotitle columnheader; set term png size 800,600; set output 'simulation_plot.png'; set title 'RLC Response (UMich PID)'; set xlabel 'Time (s)'; set ylabel 'Voltage (V)'; set yrange [-0.2:1.6]; plot 'results.csv' using 1:2 with lines, '' using 1:3 with lines\""

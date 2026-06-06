require "./spec_helper"
require "cryspace"

describe CryspacePlotter do
  it "successfully simulates the closed-loop RLC system with PID" do
    r, l, c = 1.0, 0.5, 0.1
    a = [[0.0, 1/c], [-1/l, -r/l]].to_tensor
    b = [[0.0], [1/l]].to_tensor
    c = [[1.0, 0.0]].to_tensor
    d = [[0.0]].to_tensor
    rlc_plant = CrySpace::StateSpace.new(a, b, c, d)

    kp, ki, kd = 3.0, 5.0, 1.5
    tf = 0.01
    pid_num = [(kp*tf + kd), (kp + ki*tf), ki].to_tensor
    pid_den = [tf, 1.0, 0.0].to_tensor
    pid_controller = CrySpace::TransferFunction.new(pid_num, pid_den).to_statespace

    sys_cl = (rlc_plant * pid_controller).feedback([[1.0]].to_tensor)
    sys_cl.should_not be_nil
    sys_cl.n_states.should eq(4)
  end
end


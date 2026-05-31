# CrySpace Plotter

An application simulating an RLC plant controlled by a continuous-time PID controller in a closed-loop unity feedback configuration, powered by the `cryspace` control systems library.

## Mathematical Formulations

### 1. The RLC Plant Model
The plant is a standard series RLC circuit where the states are defined as:
* $x_1(t) = v_C(t)$ (voltage across the capacitor, which is also the output $y(t)$)
* $x_2(t) = i_L(t)$ (current through the inductor)

#### State-Space Representation
$$
\dot{x}(t) = A x(t) + B u(t)
$$
$$
y(t) = C x(t) + D u(t)
$$

With the physical parameters $R = 1.0\,\Omega$, $L = 0.5\,\text{H}$, and $C = 0.1\,\text{F}$, the matrices are:

$$
A = \begin{bmatrix} 0 & \frac{1}{C} \\ -\frac{1}{L} & -\frac{R}{L} \end{bmatrix} = \begin{bmatrix} 0 & 10 \\ -2 & -2 \end{bmatrix}
$$

$$
B = \begin{bmatrix} 0 \\ \frac{1}{L} \end{bmatrix} = \begin{bmatrix} 0 \\ 2 \end{bmatrix}
$$

$$
C = \begin{bmatrix} 1 & 0 \end{bmatrix}, \quad D = \begin{bmatrix} 0 \end{bmatrix}
$$

#### Transfer Function

$$
G_p(s) = C(sI - A)^{-1} B + D = \frac{20}{s^2 + 2s + 20}
$$

---

### 2. The PID Controller
To make the controller physically realizable and stable against high-frequency noise, a first-order derivative filter with time constant $t_f = 0.01\,\text{s}$ is added.

#### Transfer Function

$$
C_{PID}(s) = K_p + \frac{K_i}{s} + \frac{K_d s}{t_f s + 1} = \frac{(K_p t_f + K_d) s^2 + (K_p + K_i t_f) s + K_i}{t_f s^2 + s}
$$

By dividing the numerator and denominator by $t_f$ to normalize the leading denominator coefficient, we obtain:

$$
C_{PID}(s) = \frac{b_0 s^2 + b_1 s + b_2}{s^2 + a_1 s + a_2}
$$

where:
* $b_0 = K_p + \frac{K_d}{t_f}$
* $b_1 = \frac{K_p}{t_f} + K_i$
* $b_2 = \frac{K_i}{t_f}$
* $a_1 = \frac{1}{t_f}$
* $a_2 = 0$

#### State-Space (Controllable Canonical Form)
`TransferFunction#to_statespace` converts the controller to:

$$
A_c = \begin{bmatrix} -a_1 & -a_2 \\ 1 & 0 \end{bmatrix}, \quad B_c = \begin{bmatrix} 1 \\ 0 \end{bmatrix}
$$

$$
C_c = \begin{bmatrix} b_1 - b_0 a_1 & b_2 - b_0 a_2 \end{bmatrix}, \quad D_c = \begin{bmatrix} b_0 \end{bmatrix}
$$

---

### 3. Closed-Loop Connection
To track a setpoint $r(t) = 1.0$ (unit step), the controller is placed in the **forward path** under **unity negative feedback**:

```
          e(t)       +-------+  u_c(t)  +-------+
  r(t) ----(O)------>|  PID  |--------->| Plant |-----> y(t)
            ^ -      +-------+          +-------+  |
            |                                      |
            +--------------------------------------+
```

In the codebase, this is constructed using series multiplication (`*`) followed by the `feedback` operator:

$$
G_{cl}(s) = \frac{G_p(s) C_{PID}(s)}{1 + G_p(s) C_{PID}(s)}
$$

```crystal
sys_cl = (rlc_plant * pid_controller).feedback([[1.0]].to_tensor)
```

#### Explanation of `[[1.0]].to_tensor`
The `.feedback` method in the `StateSpace` class closes the loop by subtracting the output scaled by a feedback gain matrix $H$ from the input:
$$
e(t) = r(t) - H y(t)
$$

For a **SISO** (Single-Input Single-Output) system where we want a **unity feedback loop** (meaning the output $y(t)$ is directly compared to the reference $r(t)$ to calculate the error $e(t) = r(t) - y(t)$):
1. **Unity Factor**: The feedback factor $H$ must be exactly $1.0$.
2. **Matrix Dimensions**: The feedback gain must be a $1 \times 1$ matrix (matching the 1 output and 1 input of our SISO system).
3. **Syntax**: In the Crystal `num` tensor library, a $1 \times 1$ matrix is represented as a nested array `[[1.0]]` and converted via `.to_tensor`.

---

## PID Tuning Parameters
To achieve a fast rise time with minimal overshoot (sovraelongation), the gains have been tuned to:
* $K_p = 4.0$
* $K_i = 8.0$
* $K_d = 1.3$
* $t_f = 0.01\,\text{s}$ (derivative filter pole at $-100\,\text{rad/s}$)

### Performance Metrics:
* **Rise Time ($90\%$)**: $\approx 0.5\,\text{seconds}$
* **Settling Time ($99.9\%$)**: $\approx 2.0\,\text{seconds}$
* **Overshoot**: $< 0.3\%$

---

## Simulation Details
Due to the fast pole introduced by the derivative filter ($\approx -88.5\,\text{rad/s}$ in closed loop), the system equations are stiff. An explicit Runge-Kutta 4 (RK4) solver requires a step size $dt \le 0.02\,\text{s}$ to avoid numerical divergence.
The simulation is executed with:
* **Time span**: $0.0$ to $15.0\,\text{seconds}$
* **Time step ($dt$)**: $0.01\,\text{seconds}$ ($1501$ simulation steps)

---

## Second Example: Complex Feedback (Sensor Dynamics)
In real-world control systems, physical sensors do not respond instantaneously. They introduce a measurement lag (dynamics).

In [src/plotter_sensor.cr](file:///home/tony/Projects/cryspace_plotter/cryspace_plotter/src/plotter_sensor.cr), we model a low-pass filter representing sensor dynamics in the feedback path:

$$
H(s) = \frac{1}{\tau_s s + 1}
$$

with a measurement time constant $\tau_s = 0.05\,\text{s}$.

The closed-loop system is then:

$$
G_{cl}(s) = \frac{G_p(s) C_{PID}(s)}{1 + G_p(s) C_{PID}(s) H(s)}
$$

In code, this is closed by passing the `sensor_dynamics` state space system directly to the `.feedback` method:

```crystal
sys_cl = sys_forward.feedback(sensor_dynamics)
```

---

## Launching the App
1. Ensure the dependencies are installed:
   ```bash
   shards install
   ```
2. Run the first example (unity feedback):
   ```bash
   crystal run src/plotter.cr
   ```
   This generates `results.csv` and compiles the plot to `simulation_plot.png`.
3. Run the second example (with sensor dynamics lag):
   ```bash
   crystal run src/plotter_sensor.cr
   ```
   This generates `results_sensor.csv` and compiles the plot to `simulation_sensor_plot.png`.

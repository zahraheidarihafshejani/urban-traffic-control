# Traffic Signal Control using Store-and-Forward Model & LQ Regulator

MATLAB implementation of urban traffic signal control based on Dr. Diakaki's Store-and-Forward modeling framework and Linear Quadratic (LQ) optimal control, compared against traditional Fixed-Time control.

---

## Project Structure

The project follows a clean, modular architecture:
- **`config.m`**: Central configuration script containing plant parameters (saturation flows, cycle time, lost time, link capacities), demand scenarios, and controller settings.
- **`plant_step.m`**: Cycle-averaged simulation plant modeling vehicle conservation, queue dynamics, and outflow constraints ($T = C$).
- **`lq_gain.m`**: Computes the optimal stationary LQ feedback gain matrix using the robust **Backward Riccati Iteration** method (Diakaki's Appendix A.2) to handle $A = I$ numerical stability issues.
- **`controller.m`**: Computes green time commands for different control modes (Fixed-Time and LQ) and enforces phase constraints using the **Equal-Deficit Algorithm** (§4.3.6).
- **`main.m`**: Main execution script running the simulation, applying demand surges, evaluating performance indices, and plotting comparative results.

---

## Key Features

- **Store-and-Forward Plant Model**: Simulates cycle-averaged queue accumulation and discharge dynamics.
- **LQ Optimal Regulation**: Regulates queue lengths and dynamically redistributes green times during demand surges.
- **Constraint Enforcement**: Strictly maintains cycle length effectiveness and bounds ($g_{\min} \le g \le g_{\max}$) via the Equal-Deficit algorithm.
- **Performance Comparison**: Compares Fixed-Time control against LQ control across metrics such as maximum queue length, delay/accumulation area, and total departures.

---

## Getting Started

1. Open MATLAB and navigate to the project directory.
2. Run `main.m` to execute the simulation and generate comparative performance plots and metrics.

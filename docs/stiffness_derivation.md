# Central Backbone Bending Stiffness Modeling (`K_b = E · I`)

This note summarizes the static bending stiffness model for the stretchable PTFE (Polytetrafluoroethylene / Teflon) central elastic backbone used in the continuum robot.

## Governing Equations

- **Bending Stiffness**:
  $$K_b = E \cdot I \quad (\text{N}\cdot\text{m}^2)$$
- **Area Moment of Inertia (Hollow Circular Tube)**:
  $$I = \frac{\pi}{64} \left(D_{\text{tube}}^4 - d_{\text{tube}}^4\right) \quad (\text{m}^4)$$
- **Material Elastic Modulus (PTFE)**:
  $$E \approx 0.5\text{ GPa} = 0.5 \times 10^9\text{ Pa}$$
- **Static Moment Equilibrium**:
  $$\tau_{\text{act}} = \tau_{\text{res}} \implies n F r = K_b \kappa$$

## Design Parameters & Computed Results

### 1. Early Prototype Tube
- Inner diameter $d_{\text{tube}} = 30\text{ mm} = 3.0 \times 10^{-2}\text{ m}$
- Outer diameter $D_{\text{tube}} = 35\text{ mm} = 3.5 \times 10^{-2}\text{ m}$
- Area moment of inertia:
  $$I = \frac{\pi}{64}\left((3.5 \times 10^{-2})^4 - (3.0 \times 10^{-2})^4\right) \approx 1.407 \times 10^{-5}\text{ m}^4$$
- Bending stiffness:
  $$K_b = (0.5 \times 10^9) \times (1.407 \times 10^{-5}) \approx 7034.65\text{ N}\cdot\text{m}^2$$

### 2. Final Prototype Tube
- Inner diameter $d_{\text{tube}} = 4\text{ mm} = 4.0 \times 10^{-3}\text{ m}$
- Outer diameter $D_{\text{tube}} = 6\text{ mm} = 6.0 \times 10^{-3}\text{ m}$
- Area moment of inertia:
  $$I = \frac{\pi}{64}\left((6.0 \times 10^{-3})^4 - (4.0 \times 10^{-3})^4\right) \approx 5.105 \times 10^{-8}\text{ m}^4$$
- Bending stiffness:
  $$K_b = (0.5 \times 10^9) \times (5.105 \times 10^{-8}) \approx 25.53\text{ N}\cdot\text{m}^2$$

---

## 中文说明（中心软管抗弯刚度推导）

- 中心弹性骨架采用聚四氟乙烯（PTFE / Teflon）空心软管，弹性模量取 $E \approx 0.5\text{ GPa}$。
- 截面惯性矩按空心圆管公式 $I = \frac{\pi}{64}(D_{\text{tube}}^4 - d_{\text{tube}}^4)$ 计算（需统一换算为国际单位 $\text{m}$）。
- 最终样机选用的中心管规格为内径 $d_{\text{tube}} = 4\text{ mm}$、外径 $D_{\text{tube}} = 6\text{ mm}$，以平衡结构自重支撑能力与舵机驱动力矩需求。

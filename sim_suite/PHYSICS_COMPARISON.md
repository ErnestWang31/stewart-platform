# Physics Model Comparison: Original vs Overshoot Sim

## Key Differences Explained

### 1. Physics Constant: `K_plant = 0.6 * g` vs `g`

**What it means:**
- **`g` (9.81 m/s²)**: Assumes a **point mass sliding** down an incline
  - Acceleration: `a = g * sin(θ)`
  - No rotational inertia effects

- **`K_plant = 0.6 * g` (5.886 m/s²)**: Accounts for a **hollow sphere rolling** without slipping
  - For a hollow sphere: moment of inertia `I = (2/3) * m * r²`
  - Rolling acceleration: `a = (g * sin(θ)) / (1 + I/(m*r²)) = 0.6 * g * sin(θ)`
  - The 0.6 factor comes from the rotational inertia reducing linear acceleration

**Why it matters:**
- A ping pong ball is a hollow sphere, so `K_plant = 0.6 * g` is more physically accurate
- This makes the ball respond **slower** to tilt (40% less acceleration than sliding)
- This naturally provides some "damping" effect through rotational inertia

**Current state:** ✅ Using `K_plant = 0.6 * g` (correct for hollow sphere)

---

### 2. Damping/Friction: Velocity-Dependent Term

**overshoot_sim.py:**
```python
accel = K_plant * np.sin(self.angle)  # NO friction term
```
- **Zero damping** - only the rolling inertia provides natural damping
- Ball can oscillate freely
- Good for exploring overshoot behavior, but not realistic

**Original sim:**
```python
ax = g * np.sin(self.theta_x) - (self.friction_coefficient / self.ball_mass) * self.vx
# With friction_coefficient = 0.2
```
- **Viscous damping**: `- (c/m) * v` where `c = 0.2 N·s/m`
- This is **velocity-dependent friction** (like air resistance or surface friction)
- Higher velocity → more damping force
- With `c = 0.2` and `m = 0.1 kg`: damping coefficient = `0.2/0.1 = 2.0 s⁻¹`
- This was **too high**, causing overdamped response (no oscillations visible)

**Current sim:**
```python
ax = K_plant * np.sin(self.theta_x) - (self.friction_coefficient / self.ball_mass) * self.vx
# With friction_coefficient = 0.05
```
- Damping coefficient = `0.05/0.1 = 0.5 s⁻¹` (4x less than original)
- Still has some damping, but allows oscillations

**Physical sources of damping:**
1. **Air resistance** (velocity-dependent)
2. **Surface friction** (rolling resistance)
3. **Rotational inertia** (already in K_plant = 0.6)
4. **Motor lag** (acts like damping in the control loop)

**Recommendation:** Use moderate damping (0.05-0.1) to match real behavior

---

### 3. Motor Actuation Models

#### overshoot_sim.py (Simple Fixed-Speed Model):
```python
servo_speed = 0.15  # Fixed coefficient
self.angle = self.angle * (1 - servo_speed) + target_angle_rad * servo_speed
# Equivalent to: angle = angle * 0.85 + target * 0.15
```

**Characteristics:**
- **Independent of timestep** - always moves 15% toward target per update
- At 30 Hz (dt=0.033s): moves ~15% per step
- At 500 Hz (dt=0.002s): still moves ~15% per step (but updates more frequently)
- **Time constant**: Approximately `τ ≈ -dt / ln(0.85) ≈ dt / 0.163 ≈ 6*dt`
  - At 30 Hz: τ ≈ 0.2s
  - At 500 Hz: τ ≈ 0.012s (much faster!)
- **Problem**: Response speed depends on simulation timestep (not physical)

#### Original Sim (Time-Constant Model):
```python
alpha = dt / motor_time_constant  # e.g., dt / 0.1
dtheta = (theta_cmd - theta) * alpha
theta += dtheta
```

**Characteristics:**
- **Depends on timestep** - but in a physically consistent way
- Time constant `τ = 0.1s` means it takes ~0.1s to reach 63% of target
- At any timestep, the response time is the same (0.1s)
- Can add **rate limiting**: `max_dtheta = motor_max_rate * dt`
- More physically accurate

**Comparison:**
- **Simple model**: Easy to tune, but timestep-dependent (not ideal)
- **Time-constant model**: Physically consistent, but needs rate limiting

**Recommendation:** Use time-constant model with rate limiting for realism

---

## Summary Table

| Aspect | overshoot_sim.py | Original Sim | Current Sim | Best Choice |
|--------|------------------|--------------|-------------|-------------|
| **Physics constant** | `0.6 * g` ✅ | `g` ❌ | `0.6 * g` ✅ | `0.6 * g` (hollow sphere) |
| **Damping** | None (0.0) | High (0.2) | Low (0.05) | Moderate (0.08-0.12) |
| **Motor model** | Fixed-speed (0.15) | Time-constant (0.1s) | Fixed-speed (0.15) | Time-constant + rate limit |
| **Motor lag** | ~0.2s @ 30Hz | 0.1s (fixed) | ~0.2s @ 30Hz | 0.1s (fixed) |

---

## Recommended "Best of Both Worlds" Configuration

```json
{
  "simulation": {
    "ball_mass": 0.1,
    "friction_coefficient": 0.1,        // Moderate damping (between 0.05 and 0.2)
    "use_simple_motor_model": false,   // Use time-constant model
    "motor_time_constant": 0.1,        // 0.1s response time
    "motor_max_rate_deg_per_s": 180.0, // Rate limiting
    ...
  }
}
```

**Rationale:**
1. ✅ Keep `K_plant = 0.6 * g` (physically correct for hollow sphere)
2. ✅ Use moderate damping (0.1) - allows oscillations but matches real behavior
3. ✅ Use time-constant motor model - physically consistent, independent of timestep
4. ✅ Add rate limiting - prevents unrealistic fast motor movements

This should give you:
- Realistic overshoot (from PID + low damping)
- Visible oscillations that damp out (from moderate friction)
- Physically consistent motor behavior (from time-constant model)
- Better match to real platform behavior

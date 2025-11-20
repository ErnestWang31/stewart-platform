# Mathematical Explanation: Stewart Platform Motor Mapping and Inverse Kinematics

## Table of Contents
1. [Coordinate Systems and Notation](#coordinate-systems)
2. [Roll and Pitch to Platform Orientation](#roll-pitch-to-orientation)
3. [Simplified Trigonometric Mapping](#simplified-mapping)
4. [Full Inverse Kinematics](#full-inverse-kinematics)
5. [Complete Mathematical Flow](#complete-flow)

---

## 1. Coordinate Systems and Notation {#coordinate-systems}

### Platform Coordinate System
- **X-axis**: Left (-) to Right (+)
- **Y-axis**: Backward (-) to Forward (+)
- **Z-axis**: Down (-) to Up (+)
- **Origin**: Platform center

### Motor Positions
The three motors are arranged at 120° intervals around the circular platform:

- **Motor 1**: At angle φ₁ = 90° (top, along +Y axis)
  - Unit vector: **u₁** = [0, 1, 0]
  
- **Motor 2**: At angle φ₂ = 210° (bottom-left)
  - Unit vector: **u₂** = [cos(210°), sin(210°), 0] = [-√3/2, -1/2, 0]
  
- **Motor 3**: At angle φ₃ = 330° (bottom-right)
  - Unit vector: **u₃** = [cos(330°), sin(330°), 0] = [√3/2, -1/2, 0]

### Angles
- **Roll (ϕ)**: Rotation around X-axis (positive = tilt right, right side down)
- **Pitch (θ)**: Rotation around Y-axis (positive = tilt forward, front down)
- **Motor angles**: θ₁₁, θ₂₁, θ₃₁ (primary joint angles for motors 1, 2, 3)

---

## 2. Roll and Pitch to Platform Orientation {#roll-pitch-to-orientation}

### Rotation Matrices

To convert roll and pitch angles to platform orientation, we use rotation matrices:

#### Roll Rotation (around X-axis)
```
R_roll(ϕ) = [1       0           0       ]
           [0   cos(ϕ)    -sin(ϕ)  ]
           [0   sin(ϕ)     cos(ϕ)  ]
```

#### Pitch Rotation (around Y-axis)
```
R_pitch(θ) = [ cos(θ)   0   sin(θ) ]
            [   0      1     0    ]
            [-sin(θ)   0   cos(θ) ]
```

#### Combined Rotation
The platform normal vector **n** (pointing upward) is rotated by:
```
n = R_pitch(θ) · R_roll(ϕ) · n₀
```

Where **n₀** = [0, 0, 1] is the initial normal vector (pointing up).

Expanding:
```
n = R_pitch(θ) · R_roll(ϕ) · [0]  =  R_pitch(θ) · [  0   ]
                             [0]                   [-sin(ϕ)]
                             [1]                   [ cos(ϕ)]
```

```
n = [ sin(θ)           ]
    [-sin(ϕ)cos(θ)     ]
    [  cos(ϕ)cos(θ)    ]
```

### Components of Normal Vector
- **nₓ** = sin(θ)
- **nᵧ** = -sin(ϕ)cos(θ)
- **nᵤ** = cos(ϕ)cos(θ)

For small angles (typical case: |ϕ|, |θ| < 15°):
- cos(ϕ) ≈ 1, cos(θ) ≈ 1
- sin(ϕ) ≈ ϕ, sin(θ) ≈ θ

So:
```
n ≈ [  θ  ]
    [ -ϕ  ]
    [  1  ]
```

---

## 3. Simplified Trigonometric Mapping {#simplified-mapping}

### Assumption
The simplified method assumes **small angles** and a **direct linear relationship** between motor height changes and platform tilt.

### Height Change Calculation

For each motor at position angle φᵢ, the required height change Δhᵢ to achieve roll ϕ and pitch θ is:

```
Δhᵢ = -ϕ·cos(φᵢ) - θ·sin(φᵢ)
```

**Why the negative signs?**
- When platform tilts right (positive roll), the right side goes down
- Motors on the right side need to go down (negative height change)
- cos(φ) is positive for right-side motors, so we need a negative sign

### Motor-Specific Height Changes

#### Motor 1 (φ₁ = 90°)
```
Δh₁ = -ϕ·cos(90°) - θ·sin(90°)
    = -ϕ·0 - θ·1
    = -θ
```
**Motor 1 primarily controls pitch** (forward/backward tilt)

#### Motor 2 (φ₂ = 210°)
```
Δh₂ = -ϕ·cos(210°) - θ·sin(210°)
    = -ϕ·(-√3/2) - θ·(-1/2)
    = (√3/2)ϕ + (1/2)θ
```
**Motor 2 contributes to both roll and pitch**

#### Motor 3 (φ₃ = 330°)
```
Δh₃ = -ϕ·cos(330°) - θ·sin(330°)
    = -ϕ·(√3/2) - θ·(-1/2)
    = -(√3/2)ϕ + (1/2)θ
```
**Motor 3 contributes to both roll and pitch**

### Matrix Form

The relationship can be written as:

```
[Δh₁]   [-cos(90°)  -sin(90°) ]   [ϕ]   [ 0   -1 ]   [ϕ]
[Δh₂] = [-cos(210°) -sin(210°)] · [θ] = [√3/2  1/2] · [θ]
[Δh₃]   [-cos(330°) -sin(330°)]          [-√3/2 1/2]
```

Or:
```
[Δh₁]   [ 0     -1  ]   [ϕ]
[Δh₂] = [√3/2   1/2] · [θ]
[Δh₃]   [-√3/2  1/2]
```

### Conversion to Servo Angles

The height change is converted to servo angle change:

```
θᵢ = θ_neutral + Δhᵢ · scale_factor · direction_i
```

Where:
- **θ_neutral** = 15° (neutral position)
- **scale_factor** = 1.0 (default, can be calibrated)
- **direction_i** = ±1 (motor direction correction)

### Complete Simplified Mapping

```
θ₁ = 15° - θ·scale_factor
θ₂ = 15° + [(√3/2)ϕ + (1/2)θ]·scale_factor
θ₃ = 15° + [-(√3/2)ϕ + (1/2)θ]·scale_factor
```

---

## 4. Full Inverse Kinematics {#full-inverse-kinematics}

The full inverse kinematics method accounts for the **actual geometric constraints** of the Stewart platform mechanism.

### Platform Geometry

The platform is a triangle with:
- **Side length**: `l` (triangle side length)
- **Center position**: **S** = [0, 0, S_z] (platform center at height S_z)

### Motor Geometry

Each motor has:
- **Base length**: `l_i` (distance from base center to motor pivot)
- **Link 1 length**: `l_i1` (first link length)
- **Link 2 length**: `l_i2` (second link length)

### Step 1: Normal Vector to Platform Vertices

Given the platform normal vector **n** = [nₓ, nᵧ, nᵤ], we need to find the three platform vertices **x₁**, **x₂**, **x₃**.

#### Finding Platform Vertices

1. Create an orthonormal basis:
   - **v̂** = (n × **i**) / ||n × **i**||  (perpendicular to n and x-axis)
   - **û** = (v̂ × n) / ||v̂ × n||  (perpendicular to both)
   - **ŵ** = û × v̂  (completes the basis)

2. Platform vertex positions:
   ```
   a = l / (2·cos(30°))
   x₁ = S + a·v̂
   x₂ = S - a·sin(30°)·v̂ + a·cos(30°)·û
   x₃ = S - a·sin(30°)·v̂ - a·cos(30°)·û
   ```

### Step 2: Solve for Motor Base Positions

The motors are positioned at base points **P₁**, **P₂**, **P₃** such that:
- The distance from each motor base to its corresponding platform vertex equals the link lengths
- The platform maintains its desired orientation

#### Motor Base Position Vectors

```
P₁ = d₁·X₁̂ + c₁·ẑ
P₂ = d₂·X₂̂ + c₂·ẑ
P₃ = d₃·X₃̂ + c₃·ẑ
```

Where:
- **X₁̂** = [0, 1, 0]  (Motor 1 direction)
- **X₂̂** = [cos(30°), -sin(30°), 0]  (Motor 2 direction)
- **X₃̂** = [-cos(30°), -sin(30°), 0]  (Motor 3 direction)
- **ẑ** = [0, 0, 1]  (Vertical direction)

#### Constraint Equations

The distance from motor base **Pᵢ** to platform vertex **xᵢ** must equal the total link length:

```
||Pᵢ - xᵢ||² = (l_i1 + l_i2)²
```

This creates a system of equations that must be solved numerically.

#### Solving for d₁, d₂, d₃

The system uses the constraint equation `eeq1(d₁, n)`:

```
A₂ = 1 + (nₓ·√3/2 - nᵧ/2)² / nᵤ²
A₃ = 1 + (-nₓ·√3/2 - nᵧ/2)² / nᵤ²

BB = d₁ - 2·nᵧ·d₁·(nₓ·√3/2 - nᵧ/2) / nᵤ²
BBB = d₁ - 2·nᵧ·d₁·(-nₓ·√3/2 - nᵧ/2) / nᵤ²

CC = d₁² + nᵧ²·d₁² / nᵤ² - l²
CCC = d₁² + nᵧ²·d₁² / nᵤ² - l²

s₂ = √(max(0, -4·A₂·CC + BB²))
s₃ = √(max(0, -4·A₃·CCC + BBB²))

d₂ = (-BB + s₂) / (2·A₂)
d₃ = (-BBB + s₃) / (2·A₃)
```

The value of **d₁** is found by solving `eeq1(d₁, n) = 0` using numerical methods (e.g., `scipy.optimize.fsolve`).

#### Solving for c₁, c₂, c₃

Once d₁, d₂, d₃ are known:

```
c₁ = 12  (base height)
c₂ = c₁ + (1/nᵤ)·(-d₂·cos(30°)·nₓ + d₂·sin(30°)·nᵧ + d₁·nᵧ)
c₃ = c₂ + (1/nᵤ)·((d₂ + d₃)·cos(30°)·nₓ - (-d₃ + d₂)·sin(30°)·nᵧ)
```

### Step 3: Calculate Motor Joint Angles

For each motor, we need to find the joint angles that position the end of the links at the platform vertex.

#### Motor 1: Two-Link Inverse Kinematics

Given:
- Base position: **P₁** = [P₁ₓ, P₁ᵧ, P₁ᵤ]
- Target position: **x₁** = [x₁ₓ, x₁ᵧ, x₁ᵤ]
- Link lengths: l₁₁, l₁₂

The joint angles are:

```
AA_tmp = l₁₂² - (l₁ - x₁ᵧ)² - l₁₁² - x₁ᵤ²
bb_tmp = 2·(l₁ - x₁ᵧ)·l₁₁
cc_tmp = 2·x₁ᵤ·l₁₁
denom_AA = √(bb_tmp² + cc_tmp²)

cc = arccos(AA_tmp / denom_AA)
bb₁ = arccos(bb_tmp / denom_AA)

θ₁₁ = cc - bb₁
θ₁₂ = arcsin((x₁ᵤ - l₁₁·sin(θ₁₁)) / l₁₂)
```

#### Motor 2: Two-Link Inverse Kinematics

Similar calculation but with different geometry:

```
y_tmp = √(x₂ₓ² + x₂ᵧ²)
z_tmp = x₂ᵤ

AA_tmp = -(l₂₂² - (l₂ - y_tmp)² - l₂₁² - z_tmp²)
bb_tmp = 2·(l₂ - y_tmp)·l₂₁
cc_tmp = 2·z_tmp·l₂₁
denom_AA = √(bb_tmp² + cc_tmp²)

cc = arcsin(AA_tmp / denom_AA)
bb₁ = arcsin(bb_tmp / denom_AA)

θ₂₁ = cc + bb₁
θ₂₂ = arcsin((z_tmp - l₂₁·sin(θ₂₁)) / l₂₂)
```

#### Motor 3: Two-Link Inverse Kinematics

```
y_tmp = √(x₃ₓ² + x₃ᵧ²)
z_tmp = x₃ᵤ

AA_tmp = l₃₂² - (l₃ - y_tmp)² - l₃₁² - z_tmp²
bb_tmp = 2·(l₃ - y_tmp)·l₃₁
cc_tmp = 2·z_tmp·l₃₁
denom_AA = √(bb_tmp² + cc_tmp²)

cc = arccos(AA_tmp / denom_AA)
bb₂ = arcsin(cc_tmp / denom_AA)

θ₃₁ = cc - bb₂
θ₃₂ = arcsin((z_tmp - l₃₁·sin(θ₃₁)) / l₃₂)
```

### Step 4: Platform Center Adjustment

After calculating motor base positions, we adjust so the platform center matches the desired position **S**:

```
pp = (P₁ + P₂ + P₃)/3 - S
P₁ = P₁ - pp
P₂ = P₂ - pp
P₃ = P₃ - pp
```

This ensures the platform center is at the correct position.

---

## 5. Complete Mathematical Flow {#complete-flow}

### Simplified Method Flow

```
PID Output (roll, pitch)
    ↓
Convert to radians (ϕ, θ)
    ↓
Calculate height changes:
  Δh₁ = -θ
  Δh₂ = (√3/2)ϕ + (1/2)θ
  Δh₃ = -(√3/2)ϕ + (1/2)θ
    ↓
Convert to servo angles:
  θ₁ = 15° + Δh₁·scale
  θ₂ = 15° + Δh₂·scale
  θ₃ = 15° + Δh₃·scale
    ↓
Send to Arduino (0-30° range)
```

### Full Inverse Kinematics Flow

```
PID Output (roll, pitch)
    ↓
Convert to normal vector:
  n = R_pitch(θ) · R_roll(ϕ) · [0, 0, 1]ᵀ
    ↓
Calculate platform vertices:
  x₁, x₂, x₃ from normal vector n
    ↓
Solve constraint equations:
  Find d₁, d₂, d₃, c₁, c₂, c₃
    ↓
Calculate motor base positions:
  P₁, P₂, P₃
    ↓
Adjust for platform center:
  P₁, P₂, P₃ = P₁, P₂, P₃ - offset
    ↓
Two-link inverse kinematics:
  Calculate θ₁₁, θ₁₂, θ₂₁, θ₂₂, θ₃₁, θ₃₂
    ↓
Extract primary angles:
  θ₁₁, θ₂₁, θ₃₁ (servo angles)
    ↓
Convert to servo range (0-30°)
    ↓
Send to Arduino
```

---

## Key Differences

### Simplified Method
- ✅ **Fast**: Direct trigonometric calculation
- ✅ **Simple**: No numerical solving required
- ✅ **Good for small angles**: Accurate for |roll|, |pitch| < 15°
- ❌ **Approximation**: Assumes linear relationship
- ❌ **No geometric constraints**: Doesn't account for link lengths

### Full Inverse Kinematics
- ✅ **Accurate**: Accounts for actual geometry
- ✅ **Works for large angles**: Not limited to small angles
- ✅ **Physical constraints**: Respects link length limits
- ❌ **Slow**: Requires numerical solving
- ❌ **Complex**: More parameters to calibrate
- ❌ **Sensitive**: May fail if geometry is incorrect

---

## Example Calculation

### Input
- Roll: ϕ = 5° (tilt right)
- Pitch: θ = 3° (tilt forward)

### Simplified Method

```
Δh₁ = -3° = -3°
Δh₂ = (√3/2)·5° + (1/2)·3° = 4.33° + 1.5° = 5.83°
Δh₃ = -(√3/2)·5° + (1/2)·3° = -4.33° + 1.5° = -2.83°

θ₁ = 15° - 3° = 12°
θ₂ = 15° + 5.83° = 20.83°
θ₃ = 15° - 2.83° = 12.17°
```

### Full Inverse Kinematics

```
1. Normal vector:
   n ≈ [sin(3°), -sin(5°)cos(3°), cos(5°)cos(3°)]
     ≈ [0.052, -0.087, 0.995]

2. Solve for d₁, d₂, d₃ (numerical method)
3. Calculate P₁, P₂, P₃
4. Two-link IK for each motor
5. Extract θ₁₁, θ₂₁, θ₃₁

Result: Similar to simplified method but accounts for geometry
```

---

## Conclusion

The **simplified trigonometric mapping** is used by default because:
1. It's fast enough for real-time control
2. It's accurate enough for small angles (< 15°)
3. It's easier to tune and debug
4. It doesn't require precise geometric calibration

The **full inverse kinematics** is available as an option for:
1. Large angle control
2. More accurate positioning
3. Systems with known geometry
4. Research applications

Both methods achieve the same goal: converting desired platform orientation (roll, pitch) into motor commands (servo angles).


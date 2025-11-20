# PID Tuning Guide for 3-Motor Stewart Platform

## System Understanding

### Motor Configuration
- **Motor 1**: 90° (pointing up, primarily affects Y-axis)
- **Motor 2**: 210° (pointing down-left, affects both X and Y)
- **Motor 3**: 330° (pointing down-right, affects both X and Y)

### Projection Math
Each motor responds to ball position projected onto its axis:
- `proj = X * cos(angle) + Y * sin(angle)`
- Motor 1 (90°): `proj = X*0 + Y*1 = Y` (pure Y-axis)
- Motor 2 (210°): `proj = X*cos(210°) + Y*sin(210°) = -0.866*X - 0.5*Y`
- Motor 3 (330°): `proj = X*cos(330°) + Y*sin(330°) = 0.866*X - 0.5*Y`

## Tuning Strategy

### Phase 1: Single Motor Tuning (Isolate Each Motor)

**Goal**: Tune one motor at a time with others disabled (gains = 0)

#### Step 1: Tune Motor 1 (Y-axis primary)
1. Set Motor 2 and Motor 3 gains to **0, 0, 0**
2. Set Motor 1 gains to **Kp=1.0, Ki=0, Kd=0** (start conservative)
3. Place ball at center, then manually push it **only in Y direction** (up/down)
4. Observe response:
   - **Too slow/weak**: Increase Kp gradually (1.0 → 2.0 → 3.0...)
   - **Oscillating**: Reduce Kp, add Kd (start with Kd = Kp * 0.5)
   - **Steady-state error**: Add Ki (start with Ki = Kp * 0.05)
5. Fine-tune until ball returns to center smoothly without overshoot

#### Step 2: Tune Motor 2 (X-Y combined)
1. Set Motor 1 and Motor 3 gains to **0, 0, 0**
2. Set Motor 2 gains to **Kp=1.0, Ki=0, Kd=0**
3. Place ball at center, push it **diagonally** (toward Motor 2 direction)
4. Tune using same process as Motor 1

#### Step 3: Tune Motor 3 (X-Y combined)
1. Set Motor 1 and Motor 2 gains to **0, 0, 0**
2. Set Motor 3 gains to **Kp=1.0, Ki=0, Kd=0**
3. Place ball at center, push it **diagonally** (toward Motor 3 direction)
4. Tune using same process

### Phase 2: Two Motor Coordination

**Goal**: Test how motors work together

1. Enable Motor 1 (Y-axis) and Motor 2
2. Disable Motor 3
3. Test X-axis movement (Motor 2 should respond)
4. Test Y-axis movement (Motor 1 should respond)
5. Adjust if one motor is too aggressive

### Phase 3: All Three Motors Together

**Goal**: Fine-tune with all motors active

1. Enable all three motors with tuned gains
2. Test diagonal movements
3. Test circular movements
4. Look for:
   - **Coupling issues**: One motor fighting another
   - **Oscillations**: Reduce gains slightly
   - **Sluggish response**: Increase gains slightly

## Tuning Rules of Thumb

### Starting Values
- **Kp**: Start at 1.0-2.0 (proportional to error)
- **Ki**: Start at Kp * 0.05-0.1 (eliminates steady-state error)
- **Kd**: Start at Kp * 0.3-0.5 (reduces overshoot/oscillation)

### Adjustment Guidelines
- **Oscillation**: Reduce Kp by 20%, increase Kd by 50%
- **Too slow**: Increase Kp by 20-30%
- **Steady-state error**: Increase Ki by 50%
- **Overshoot**: Increase Kd by 30-50%
- **Integral windup**: Reduce Ki or add integral limits

### Typical Ranges
- **Kp**: 0.5 - 5.0 (most systems work well around 1.5-3.0)
- **Ki**: 0.01 - 0.5 (usually 5-10% of Kp)
- **Kd**: 0.5 - 5.0 (usually 30-50% of Kp)

## Testing Procedures

### Test 1: Step Response
1. Place ball at center
2. Quickly move ball to edge (e.g., X=0.05m, Y=0)
3. Release and observe:
   - **Rise time**: How fast it responds
   - **Overshoot**: Does it overshoot center?
   - **Settling time**: How long to stabilize
   - **Steady-state error**: Final position offset

### Test 2: Disturbance Rejection
1. Place ball at center
2. Gently tap ball in random directions
3. Observe recovery:
   - **Fast recovery**: Good
   - **Oscillation**: Too aggressive (reduce gains)
   - **Slow recovery**: Too conservative (increase gains)

### Test 3: Tracking
1. Slowly move ball in circle
2. Observe if motors track smoothly
3. Look for lag or overshoot

## Common Issues

### Issue: Motors Fighting Each Other
**Solution**: Check motor directions (flip checkboxes), reduce gains slightly

### Issue: Oscillation
**Solution**: Reduce Kp, increase Kd, check for integral windup

### Issue: Slow Response
**Solution**: Increase Kp, check if Ki is too high (causing lag)

### Issue: Steady-State Error
**Solution**: Increase Ki gradually (but watch for windup)

## Recommended Tuning Order

1. **Start with Motor 1 only** (easiest, pure Y-axis)
2. **Tune Motor 1** to perfection
3. **Add Motor 2**, tune it
4. **Add Motor 3**, tune it
5. **Fine-tune all together** for coordination

## Quick Reference

| Symptom | Action |
|---------|--------|
| Oscillating | ↓ Kp, ↑ Kd |
| Too slow | ↑ Kp |
| Steady-state error | ↑ Ki |
| Overshoot | ↑ Kd |
| Motors fighting | Check directions, ↓ gains |
| Integral windup | ↓ Ki or add limits |


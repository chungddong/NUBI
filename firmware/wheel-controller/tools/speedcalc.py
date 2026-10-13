"""Turn a motor constant into speeds, or a measured speed into a motor constant.

    speedcalc.py                      assume Kv = 14 rpm/V, print the table
    speedcalc.py --kv 12.5            use a known Kv
    speedcalc.py --measured 15.2      derive Kv from a no-load sweep result
    speedcalc.py --measured 15.2 --mass 95

Everything downstream of Kv is arithmetic; Kv itself is the one number worth
measuring, because hoverboard hub motors run anywhere from 10 to 16 rpm/V and
that alone swings top speed from 11 to 18 km/h.
"""

import argparse
import math

WHEEL_IN = 6.5
DIAMETER_M = WHEEL_IN * 0.0254
CIRCUM_M = math.pi * DIAMETER_M
RADIUS_M = DIAMETER_M / 2

R_PHASE = 0.5      # ohm, effective winding resistance -- typical hub motor
CRR = 0.015        # rolling resistance, solid tyre on pavement
CDA = 0.6          # m^2, drag area of an upright rider
RHO = 1.2          # kg/m^3
G = 9.81


def kmh(mps):
    return mps * 3.6


def no_load(kv, volts):
    rpm = kv * volts
    return rpm, rpm / 60 * CIRCUM_M


def loaded(kv, volts, mass, grade=0.0, motors=1):
    """Solve for the speed where motor torque equals road load.

    Returns per-motor amps. Sharing one road load across N motors cuts each
    one's torque, current and IR droop by N, so more motors means slightly
    *more* top speed, not less -- and vastly more thermal headroom.
    """
    ke = 60 / (2 * math.pi * kv)     # V per rad/s
    kt = ke                          # N*m per A, same number in SI
    v = 1.0
    for _ in range(200):             # fixed-point iteration, converges quickly
        drag = 0.5 * RHO * CDA * v * v
        roll = CRR * mass * G
        hill = mass * G * grade
        force = drag + roll + hill
        amps = force * RADIUS_M / kt / motors
        eff_v = volts - amps * R_PHASE
        if eff_v <= 0:
            return 0.0, amps
        omega = eff_v / ke                      # rad/s at the wheel
        v_new = omega * RADIUS_M
        if abs(v_new - v) < 1e-6:
            v = v_new
            break
        v = 0.5 * v + 0.5 * v_new               # damped, avoids oscillation
    return v, amps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kv", type=float, default=14.0, help="rpm per volt")
    ap.add_argument("--measured", type=float,
                    help="no-load km/h at full duty; derives Kv and overrides --kv")
    ap.add_argument("--volts", type=float, default=36.8)
    ap.add_argument("--mass", type=float, default=80.0, help="rider + vehicle, kg")
    ap.add_argument("--motors", type=int, default=1, help="driven wheels sharing the load")
    args = ap.parse_args()

    kv = args.kv
    if args.measured:
        rpm = args.measured / 3.6 / CIRCUM_M * 60
        kv = rpm / args.volts
        print(f"measured {args.measured:.2f} km/h at {args.volts:.1f} V "
              f"-> {rpm:.0f} rpm -> Kv = {kv:.1f} rpm/V\n")
    else:
        print(f"assuming Kv = {kv:.1f} rpm/V (generic hoverboard motor)\n")

    print(f"  wheel {WHEEL_IN}\" -> circumference {CIRCUM_M:.4f} m\n")
    print(f"  {args.motors} driven wheel(s), {args.mass:.0f} kg total\n")
    print(f"  {'condition':<28} {'rpm':>6} {'m/s':>7} {'km/h':>7} {'A/mot':>6} {'A tot':>6}")
    print("  " + "-" * 65)

    for volts, label in ((args.volts, f"no load, {args.volts:.1f} V"),
                         (42.0, "no load, 42 V (full)")):
        rpm, mps = no_load(kv, volts)
        print(f"  {label:<28} {rpm:>6.0f} {mps:>7.2f} {kmh(mps):>7.1f} "
              f"{'-':>6} {'-':>6}")

    for volts, grade, label in (
        (args.volts, 0.00, "flat"),
        (42.0, 0.00, "flat, full battery"),
        (args.volts, 0.05, "5 % grade"),
        (args.volts, 0.10, "10 % grade"),
        (args.volts, 0.20, "20 % grade"),
    ):
        mps, amps = loaded(kv, volts, args.mass, grade, args.motors)
        rpm = mps / CIRCUM_M * 60
        print(f"  {label:<28} {rpm:>6.0f} {mps:>7.2f} {kmh(mps):>7.1f} "
              f"{amps:>6.1f} {amps*args.motors:>6.1f}")

    mps, _ = loaded(kv, args.volts, args.mass, 0.0, args.motors)
    if mps > 0:
        print(f"\n  at {mps:.2f} m/s:  100 m in {100/mps:.0f} s, "
              f"1 km in {1000/mps/60:.1f} min")
    print("\n  'A tot' is what the battery supplies; 'A/mot' is what each winding")
    print("  sees, and heating goes with its square -- that is where extra motors")
    print("  buy you something, not in top speed.")


if __name__ == "__main__":
    main()

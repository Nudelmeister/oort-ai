// Copyright (c) 2015, Mikhail Vorotilov
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

use std::f64::consts::FRAC_PI_3;

#[derive(Debug)]
pub enum Roots {
    /// Equation has no roots
    No([f64; 0]),
    /// Equation has one root (or all roots of the equation are the same)
    One([f64; 1]),
    /// Equation has two roots
    Two([f64; 2]),
    /// Equation has three roots
    Three([f64; 3]),
    /// Equation has four roots
    Four([f64; 4]),
}

impl AsRef<[f64]> for Roots {
    fn as_ref(&self) -> &[f64] {
        match self {
            Roots::No(x) => x,
            Roots::One(x) => x,
            Roots::Two(x) => x,
            Roots::Three(x) => x,
            Roots::Four(x) => x,
        }
    }
}

impl Roots {
    fn check_new_root(&self, new_root: f64) -> (bool, usize) {
        let mut pos = 0;
        let mut exists = false;

        for x in self.as_ref().iter() {
            if *x == new_root {
                exists = true;
                break;
            }
            if *x > new_root {
                break;
            }
            pos += 1;
        }

        (exists, pos)
    }

    /// Add a new root to existing ones keeping the list of roots ordered and unique.
    fn add_new_root(self, new_root: f64) -> Self {
        match self {
            Roots::No(_) => Roots::One([new_root]),
            _ => {
                let (exists, pos) = self.check_new_root(new_root);

                if exists {
                    self
                } else {
                    let old_roots = self.as_ref();
                    match (old_roots.len(), pos) {
                        (1, 0) => Roots::Two([new_root, old_roots[0]]),
                        (1, 1) => Roots::Two([old_roots[0], new_root]),
                        (2, 0) => Roots::Three([new_root, old_roots[0], old_roots[1]]),
                        (2, 1) => Roots::Three([old_roots[0], new_root, old_roots[1]]),
                        (2, 2) => Roots::Three([old_roots[0], old_roots[1], new_root]),
                        (3, 0) => Roots::Four([new_root, old_roots[0], old_roots[1], old_roots[2]]),
                        (3, 1) => Roots::Four([old_roots[0], new_root, old_roots[1], old_roots[2]]),
                        (3, 2) => Roots::Four([old_roots[0], old_roots[1], new_root, old_roots[2]]),
                        (3, 3) => Roots::Four([old_roots[0], old_roots[1], old_roots[2], new_root]),
                        _ => panic!("Cannot add root"),
                    }
                }
            }
        }
    }
}

/// Solves a quartic equation a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0 = 0.
///
/// Returned roots are ordered.
/// Precision is about 5e-15 for f64, 5e-7 for f32.
/// WARNING: f32 is often not enough to find multiple roots.
///
/// # Examples
///
/// ```
/// use roots::find_roots_quartic;
///
/// let one_root = find_roots_quartic(1f64, 0f64, 0f64, 0f64, 0f64);
/// // Returns Roots::One([0f64]) as 'x^4 = 0' has one root 0
///
/// let two_roots = find_roots_quartic(1f32, 0f32, 0f32, 0f32, -1f32);
/// // Returns Roots::Two([-1f32, 1f32]) as 'x^4 - 1 = 0' has roots -1 and 1
///
/// let multiple_roots = find_roots_quartic(-14.0625f64, -3.75f64, 29.75f64, 4.0f64, -16.0f64);
/// // Returns Roots::Two([-1.1016116464173349f64, 0.9682783130840016f64])
///
/// let multiple_roots_not_found = find_roots_quartic(-14.0625f32, -3.75f32, 29.75f32, 4.0f32, -16.0f32);
/// // Returns Roots::No([]) because of f32 rounding errors when trying to calculate the discriminant
/// ```
pub fn find_roots_quartic(a4: f64, a3: f64, a2: f64, a1: f64, a0: f64) -> Roots {
    // Handle non-standard cases
    if a4 == 0.0 {
        // a4 = 0; a3*x^3 + a2*x^2 + a1*x + a0 = 0; solve cubic equation
        find_roots_cubic(a3, a2, a1, a0)
    } else if a0 == 0.0 {
        // a0 = 0; x^4 + a2*x^2 + a1*x = 0; reduce to cubic and arrange results
        find_roots_cubic(a4, a3, a2, a1).add_new_root(0.0)
    } else if a1 == 0.0 && a3 == 0.0 {
        // a1 = 0, a3 =0; a4*x^4 + a2*x^2 + a0 = 0; solve bi-quadratic equation
        find_roots_biquadratic(a4, a2, a0)
    } else {
        // Discriminant
        // https://en.wikipedia.org/wiki/Quartic_function#Nature_of_the_roots
        // Partially simplifed to keep intermediate values smaller (to minimize rounding errors).
        let discriminant =
            a4 * a0 * a4 * (256.0 * a4 * a0 * a0 + a1 * (144.0 * a2 * a1 - 192.0 * a3 * a0))
                + a4 * a0 * a2 * a2 * (16.0 * a2 * a2 - 80.0 * a3 * a1 - 128.0 * a4 * a0)
                + (a3
                    * a3
                    * (a4 * a0 * (144.0 * a2 * a0 - 6.0 * a1 * a1)
                        + (a0 * (18.0 * a3 * a2 * a1 - 27.0 * a3 * a3 * a0 - 4.0 * a2 * a2 * a2)
                            + a1 * a1 * (a2 * a2 - 4.0 * a3 * a1))))
                + a4 * a1 * a1 * (18.0 * a3 * a2 * a1 - 27.0 * a4 * a1 * a1 - 4.0 * a2 * a2 * a2);
        let pp = 8.0 * a4 * a2 - 3.0 * a3 * a3;
        let rr = a3 * a3 * a3 + 8.0 * a4 * a4 * a1 - 4.0 * a4 * a3 * a2;
        let delta0 = a2 * a2 - 3.0 * a3 * a1 + 12.0 * a4 * a0;
        let dd = 64.0 * a4 * a4 * a4 * a0 - 16.0 * a4 * a4 * a2 * a2 + 16.0 * a4 * a3 * a3 * a2
            - 16.0 * a4 * a4 * a3 * a1
            - 3.0 * a3 * a3 * a3 * a3;

        // Handle special cases
        let double_root = discriminant == 0.0;
        if double_root {
            let triple_root = double_root && delta0 == 0.0;
            let quadruple_root = triple_root && dd == 0.0;
            let no_roots = dd == 0.0 && pp > 0.0 && rr == 0.0;
            if quadruple_root {
                // Wiki: all four roots are equal
                Roots::One([-a3 / (4.0 * a4)])
            } else if triple_root {
                // Wiki: At least three roots are equal to each other
                // x0 is the unique root of the remainder of the Euclidean division of the quartic by its second derivative
                //
                // Solved by SymPy (ra is the desired reminder)
                // a, b, c, d, e = symbols('a,b,c,d,e')
                // f=a*x**4+b*x**3+c*x**2+d*x+e     // Quartic polynom
                // g=6*a*x**2+3*b*x+c               // Second derivative
                // q, r = div(f, g)                 // SymPy only finds the highest power
                // simplify(f-(q*g+r)) == 0         // Verify the first division
                // qa, ra = div(r/a,g/a)            // Workaround to get the second division
                // simplify(f-((q+qa)*g+ra*a)) == 0 // Verify the second division
                // solve(ra,x)
                // ----- yields
                // (−72*a^2*e+10*a*c^2−3*b^2*c)/(9*(8*a^2*d−4*a*b*c+b^3))
                let x0 = (-72.0 * a4 * a4 * a0 + 10.0 * a4 * a2 * a2 - 3.0 * a3 * a3 * a2)
                    / (9.0 * (8.0 * a4 * a4 * a1 - 4.0 * a4 * a3 * a2 + a3 * a3 * a3));
                let roots = Roots::One([x0]);
                roots.add_new_root(-(a3 / a4 + 3.0 * x0))
            } else if no_roots {
                // Wiki: two complex conjugate double roots
                Roots::No([])
            } else {
                find_roots_via_depressed_quartic(a4, a3, a2, a1, a0, pp, rr, dd)
            }
        } else {
            let no_roots = discriminant > 0.0 && (pp > 0.0 || dd > 0.0);
            if no_roots {
                // Wiki: two pairs of non-real complex conjugate roots
                Roots::No([])
            } else {
                find_roots_via_depressed_quartic(a4, a3, a2, a1, a0, pp, rr, dd)
            }
        }
    }
}

pub fn find_roots_cubic(a3: f64, a2: f64, a1: f64, a0: f64) -> Roots {
    // Handle non-standard cases
    if a3 == 0.0 {
        // a3 = 0; a2*x^2+a1*x+a0=0; solve quadratic equation
        find_roots_quadratic(a2, a1, a0)
    } else if a2 == 0.0 {
        // a2 = 0; a3*x^3+a1*x+a0=0; solve depressed cubic equation
        find_roots_cubic_depressed(a1 / a3, a0 / a3)
    } else if a3 == 1.0 {
        // solve normalized cubic expression
        find_roots_cubic_normalized(a2, a1, a0)
    } else {
        // standard case
        let d = 18.0 * a3 * a2 * a1 * a0 - 4.0 * a2 * a2 * a2 * a0 + a2 * a2 * a1 * a1
            - 4.0 * a3 * a1 * a1 * a1
            - 27.0 * a3 * a3 * a0 * a0;
        let d0 = a2 * a2 - 3.0 * a3 * a1;
        let d1 = 2.0 * a2 * a2 * a2 - 9.0 * a3 * a2 * a1 + 27.0 * a3 * a3 * a0;
        if d < 0.0 {
            // one real root
            let sqrt = (-27.0 * a3 * a3 * d).sqrt();
            let c = (if d1 < 0.0 { d1 - sqrt } else { d1 + sqrt } / 2.0).cbrt();
            let x = -(a2 + c + d0 / c) / (3.0 * a3);
            Roots::One([x])
        } else if d == 0.0 {
            // multiple roots
            if d0 == 0.0 {
                // triple root
                Roots::One([-a2 / (a3 * 3.0)])
            } else {
                // single root and double root
                Roots::One([(9.0 * a3 * a0 - a2 * a1) / (d0 * 2.0)]).add_new_root(
                    (4.0 * a3 * a2 * a1 - 9.0 * a3 * a3 * a0 - a2 * a2 * a2) / (a3 * d0),
                )
            }
        } else {
            // three real roots
            let c3_img = (27.0 * a3 * a3 * d).sqrt() / 2.0;
            let c3_real = d1 / 2.0;
            let c3_module = (c3_img * c3_img + c3_real * c3_real).sqrt();
            let c3_phase = 2.0 * (c3_img / (c3_real + c3_module)).atan();
            let c_module = c3_module.cbrt();
            let c_phase = c3_phase / 3.0;
            let c_real = c_module * c_phase.cos();
            let c_img = c_module * c_phase.sin();
            let x0_real = -(a2 + c_real + (d0 * c_real) / (c_module * c_module)) / (3.0 * a3);

            let e_real = -1.0 / 2.0;
            let e_img = 3.0f64.sqrt() / 2.0;
            let c1_real = c_real * e_real - c_img * e_img;
            let c1_img = c_real * e_img + c_img * e_real;
            let x1_real = -(a2 + c1_real + (d0 * c1_real) / (c1_real * c1_real + c1_img * c1_img))
                / (3.0 * a3);

            let c2_real = c1_real * e_real - c1_img * e_img;
            let c2_img = c1_real * e_img + c1_img * e_real;
            let x2_real = -(a2 + c2_real + (d0 * c2_real) / (c2_real * c2_real + c2_img * c2_img))
                / (3.0 * a3);

            Roots::One([x0_real])
                .add_new_root(x1_real)
                .add_new_root(x2_real)
        }
    }
}

pub fn find_roots_cubic_depressed(a1: f64, a0: f64) -> Roots {
    let two_third_pi = 2.0 * FRAC_PI_3;

    if a1 == 0.0 {
        Roots::One([-a0.cbrt()])
    } else if a0 == 0.0 {
        find_roots_quadratic(1.0, 0.0, a1).add_new_root(0.0)
    } else {
        let d = a0 * a0 / 4.0 + a1 * a1 * a1 / 27.0;
        if d < 0.0 {
            // n*a0^2 + m*a1^3 < 0 => a1 < 0
            let a = (-4.0 * a1 / 3.0).sqrt();

            let phi = (-4.0 * a0 / (a * a * a)).acos() / 3.0;
            Roots::One([a * phi.cos()])
                .add_new_root(a * (phi + two_third_pi).cos())
                .add_new_root(a * (phi - two_third_pi).cos())
        } else {
            let sqrt_d = d.sqrt();
            let a0_div_2 = a0 / 2.0;
            let x1 = (sqrt_d - a0_div_2).cbrt() - (sqrt_d + a0_div_2).cbrt();
            if d == 0.0 {
                // one real root and one double root
                Roots::One([x1]).add_new_root(a0_div_2)
            } else {
                // one real root
                Roots::One([x1])
            }
        }
    }
}

fn find_roots_via_depressed_quartic(
    a4: f64,
    a3: f64,
    a2: f64,
    a1: f64,
    a0: f64,
    pp: f64,
    rr: f64,
    dd: f64,
) -> Roots {
    // Depressed quartic
    // https://en.wikipedia.org/wiki/Quartic_function#Converting_to_a_depressed_quartic

    // a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0 = 0 => y^4 + p*y^2 + q*y + r.
    let a4_pow_2 = a4 * a4;
    let a4_pow_3 = a4_pow_2 * a4;
    let a4_pow_4 = a4_pow_2 * a4_pow_2;
    // Re-use pre-calculated values
    let p = pp / (8.0 * a4_pow_2);
    let q = rr / (8.0 * a4_pow_3);
    let r =
        (dd + 16.0 * a4_pow_2 * (12.0 * a0 * a4 - 3.0 * a1 * a3 + a2 * a2)) / (256.0 * a4_pow_4);

    let mut roots = Roots::No([]);
    for y in find_roots_quartic_depressed(p, q, r).as_ref().iter() {
        roots = roots.add_new_root(*y - a3 / (4.0 * a4));
    }
    roots
}

fn find_roots_quartic_depressed(a2: f64, a1: f64, a0: f64) -> Roots {
    // Handle non-standard cases
    if a1 == 0.0 {
        // a1 = 0; x^4 + a2*x^2 + a0 = 0; solve biquadratic equation
        find_roots_biquadratic(0.0, a2, a0)
    } else if a0 == 0.0 {
        // a0 = 0; x^4 + a2*x^2 + a1*x = 0; reduce to normalized cubic and add zero root
        find_roots_cubic_normalized(0.0, a2, a1).add_new_root(0.0)
    } else {
        // Solve the auxiliary equation y^3 + (5/2)*a2*y^2 + (2*a2^2-a0)*y + (a2^3/2 - a2*a0/2 - a1^2/8) = 0
        let a2_pow_2 = a2 * a2;
        let a1_div_2 = a1 / 2.0;
        let b2 = a2 * 5.0 / 2.0;
        let b1 = 2.0 * a2_pow_2 - a0;
        let b0 = (a2_pow_2 * a2 - a2 * a0 - a1_div_2 * a1_div_2) / 2.0;

        // At least one root always exists. The last root is the maximal one.
        let resolvent_roots = find_roots_cubic_normalized(b2, b1, b0);
        let y = resolvent_roots.as_ref().iter().last().unwrap();

        let _a2_plus_2y = a2 + 2.0 * *y;
        if _a2_plus_2y > 0.0 {
            let sqrt_a2_plus_2y = _a2_plus_2y.sqrt();
            let q0a = a2 + *y - a1_div_2 / sqrt_a2_plus_2y;
            let q0b = a2 + *y + a1_div_2 / sqrt_a2_plus_2y;

            let mut roots = find_roots_quadratic(1.0, sqrt_a2_plus_2y, q0a);
            for x in find_roots_quadratic(1.0, -sqrt_a2_plus_2y, q0b)
                .as_ref()
                .iter()
            {
                roots = roots.add_new_root(*x);
            }
            roots
        } else {
            Roots::No([])
        }
    }
}

fn find_roots_biquadratic(a4: f64, a2: f64, a0: f64) -> Roots {
    // Handle non-standard cases
    if a4 == 0.0 {
        // a4 = 0; a2*x^2 + a0 = 0; solve quadratic equation
        find_roots_quadratic(a2, 0.0, a0)
    } else if a0 == 0.0 {
        // a0 = 0; a4*x^4 + a2*x^2 = 0; solve quadratic equation and add zero root
        find_roots_quadratic(a4, 0.0, a2).add_new_root(0.0)
    } else {
        // solve the corresponding quadratic equation and order roots
        let mut roots = Roots::No([]);
        for x in find_roots_quadratic(a4, a2, a0).as_ref().iter() {
            if *x > 0.0 {
                let sqrt_x = x.sqrt();
                roots = roots.add_new_root(-sqrt_x).add_new_root(sqrt_x);
            } else if *x == 0.0 {
                roots = roots.add_new_root(0.0);
            }
        }
        roots
    }
}

fn find_roots_cubic_normalized(a2: f64, a1: f64, a0: f64) -> Roots {
    let two_third_pi = 2.0 * FRAC_PI_3;

    let q = (3.0 * a1 - a2 * a2) / 9.0;
    let r = (9.0 * a2 * a1 - 27.0 * a0 - 2.0 * a2 * a2 * a2) / 54.0;
    let q3 = q * q * q;
    let d = q3 + r * r;
    let a2_div_3 = a2 / 3.0;

    if d < 0.0 {
        let phi_3 = (r / (-q3).sqrt()).acos() / 3.0;
        let sqrt_q_2 = 2.0 * (-q).sqrt();

        Roots::One([sqrt_q_2 * phi_3.cos() - a2_div_3])
            .add_new_root(sqrt_q_2 * (phi_3 - two_third_pi).cos() - a2_div_3)
            .add_new_root(sqrt_q_2 * (phi_3 + two_third_pi).cos() - a2_div_3)
    } else {
        let sqrt_d = d.sqrt();
        let s = (r + sqrt_d).cbrt();
        let t = (r - sqrt_d).cbrt();

        if s == t {
            if s + t == 0.0 {
                Roots::One([s + t - a2_div_3])
            } else {
                Roots::One([s + t - a2_div_3]).add_new_root(-(s + t) / 2.0 - a2_div_3)
            }
        } else {
            Roots::One([s + t - a2_div_3])
        }
    }
}

fn find_roots_quadratic(a2: f64, a1: f64, a0: f64) -> Roots {
    // Handle non-standard cases
    if a2 == 0.0 {
        // a2 = 0; a1*x+a0=0; solve linear equation
        find_roots_linear(a1, a0)
    } else {
        // Rust lacks a simple way to convert an integer constant to generic type F
        let discriminant = a1 * a1 - 4.0 * a2 * a0;
        if discriminant < 0.0 {
            Roots::No([])
        } else {
            let a2x2 = 2.0 * a2;
            if discriminant == 0.0 {
                Roots::One([-a1 / a2x2])
            } else {
                // To improve precision, do not use the smallest divisor.
                // See https://people.csail.mit.edu/bkph/articles/Quadratics.pdf
                let sq = discriminant.sqrt();

                let (same_sign, diff_sign) = if a1 < 0.0 {
                    (-a1 + sq, -a1 - sq)
                } else {
                    (-a1 - sq, -a1 + sq)
                };

                let (x1, x2) = if same_sign.abs() > a2x2.abs() {
                    let a0x2 = 2.0 * a0;
                    if diff_sign.abs() > a2x2.abs() {
                        // 2*a2 is the smallest divisor, do not use it
                        (a0x2 / same_sign, a0x2 / diff_sign)
                    } else {
                        // diff_sign is the smallest divisor, do not use it
                        (a0x2 / same_sign, same_sign / a2x2)
                    }
                } else {
                    // 2*a2 is the greatest divisor, use it
                    (diff_sign / a2x2, same_sign / a2x2)
                };

                // Order roots
                if x1 < x2 {
                    Roots::Two([x1, x2])
                } else {
                    Roots::Two([x2, x1])
                }
            }
        }
    }
}

fn find_roots_linear(a1: f64, a0: f64) -> Roots {
    if a1 == 0.0 {
        if a0 == 0.0 {
            Roots::One([0.0])
        } else {
            Roots::No([])
        }
    } else {
        Roots::One([-a0 / a1])
    }
}

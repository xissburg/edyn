#include "edyn/util/tire.hpp"
#include "edyn/math/constants.hpp"

namespace edyn {


void calculate_tire_forces(const tire_specification& spec, tire_state& state) {
    // Advanced brush model Treadsim from the Master Thesis "Implementing inflation pressure and velocity effects 
    // into the  Magic Formula tyre model" by J. de Hoogh.

    const scalar Fz = state.vertical_load;
    const scalar r0 = spec.radius;
    const scalar defl = state.vertical_deflection;
    const scalar re = r0 - defl;
    scalar a = scalar(0.4) * r0 * (defl/r0 + scalar(2.25) * std::sqrt(defl/r0));

    if (Fz < EDYN_EPSILON || a < EDYN_EPSILON) {
        state.Fx = 0; state.Fy = 0; state.Mz = 0; state.Mx = 0;
        return;
    }

    scalar b = spec.tread_width/2;

    scalar a2 = 2*a;

    state.contact_patch_length = a2;
    state.contact_patch_width = b*2;

    //scalar cir = r0*SIMD_2_PI;
    scalar dx1 = 2*a/spec.num_bristles;
    //scalar dx2 = (cir - dx1*(mNumBristles - 1))/(mNumBeams - (mNumBristles - 1));
    scalar dy1 = 2*b/spec.num_rows;

    //scalar aphit = R > SIMD_EPSILON ? a/R : 0;
    scalar b0 = b - dy1/2;

    scalar kappa = state.slip_ratio;
    scalar alpha = state.slip_angle;
    scalar singam = state.sin_camber;

    scalar Vc = state.speed;
    scalar Vcx = Vc * std::cos(alpha);
    scalar Vsx = -Vcx * kappa;
    scalar Vsy = Vcx * std::tan(alpha);
    scalar Vr = Vcx - Vsx;
    scalar omega = Vr/re;

    const scalar epsdrgam = 0.8;
    const scalar cpx = spec.longitudinal_tread_stiffness;
    const scalar cpy = spec.lateral_tread_stiffness;

    scalar psidot = -state.yaw_rate;

    scalar Fx = 0, Fy = 0, Mz = 0;

    const scalar c1 = 0.25;
    const scalar c2 = 0.25;

    state.slide_factor = 0;

    for (unsigned int row = 0; row < spec.num_rows; ++row) {
        scalar bRow = b0 - dy1*row;
        //scalar VcxRow = Vcx - bRow*psidot;
        scalar VsxRow = Vsx - bRow*psidot + bRow*omega*singam*(epsdrgam - 1);
        scalar VsyRow = Vsy;
        scalar deltatmax = 1;
        scalar deltat = std::abs(Vr) > EDYN_EPSILON ? std::min(2*a/(spec.num_bristles*std::abs(Vr)), deltatmax) : deltatmax;
        scalar xi = a + dx1/2;
        scalar exi = 0;
        scalar eyi = 0;
        scalar Fxim1 = 0;
        scalar Fyim1 = 0;
        scalar Mzim1 = 0;

        for (unsigned int i = 0; i < spec.num_bristles; ++i) {
            xi -= dx1;
            scalar dybdximean = singam * std::cos(xi/r0 + half_pi);
            scalar ybi = bRow + r0*singam * std::sin(xi/r0 + half_pi);
            scalar Vbxi = VsxRow;
            scalar Vbyimean = VsyRow + (xi + dx1/2)*psidot - Vr*dybdximean;
            scalar nVbi = std::sqrt(Vbxi*Vbxi + Vbyimean*Vbyimean);

            scalar Pmax = Fz/(2*b*(a2 - a2*c1/2 - a2*c2/3));
            scalar x1 = xi;
            x1 = -(x1 - a);
            scalar pz = 0;

            if (x1 <= c1*a2) {
                pz = Pmax*(x1/a2/c1);
            }
            else if (x1 <= a2 - a2*c2 && x1 > c1*a2) {
                pz = Pmax;
            }
            else {
                pz = Pmax*(1 - (x1 - a2 + a2*c2)*(x1 - a2 + a2*c2)/(a2*a2*c2*c2));
            }

            // simple correction for alpha dependency for pressure distribution
            scalar p = pz;//*(1 - bRow*btFabs(alpha));
            
            scalar dsix = deltat*Vbxi;
            scalar dsiy = deltat*Vbyimean;
            scalar mu0 = spec.friction_coefficient * state.friction_coefficient * std::exp(-0.001 * spec.load_sensitivity * Fz);
            scalar amu = spec.friction_velocity_sensitivity;
            scalar mu = mu0 / (1 + amu*nVbi);
            scalar axi = exi - dsix;
            scalar ayi = eyi - dsiy;
            scalar deff = std::sqrt(axi*axi*cpx*cpx + ayi*ayi*cpy*cpy);

            if (deff <= mu*p) {
                exi = axi;
                eyi = ayi;
            }
            else {
                auto fx = cpx*axi/deff;
                auto fy = cpy*ayi/deff;

                auto cor = mu*p/deff;
                auto cormin = 0.66;

                if (cor < cormin) {
                    auto ai = std::sqrt(axi*axi + ayi*ayi);
                    fx = fx*cor/cormin + (cormin - cor)/cormin*axi/ai;
                    fy = fy*cor/cormin + (cormin - cor)/cormin*ayi/ai;
                }

                auto qa = cpx*cpx*fx*fx + cpy*cpy*fy*fy;
                auto qb = 2*cpx*cpx*axi*fx + 2*cpy*cpy*ayi*fy;
                auto qc = cpx*cpx*axi*axi + cpy*cpy*ayi*ayi - mu*mu*p*p;
                auto d = qb*qb - 4*qa*qc;
                d = std::sqrt(std::max(scalar(0), d));
                auto c = (-qb + d)/(2*qa);

                auto slidex = c*fx;
                auto slidey = c*fy;
                exi = axi + slidex;
                eyi = ayi + slidey;

                auto slidedist = std::sqrt(slidex*slidex + slidey*slidey);
                auto slideVel = slidedist/deltat;
                state.slide_factor += std::min(slideVel*0.05, 1.0);
            }

            scalar px = cpx*exi;
            scalar py = cpy*eyi;

            Fxim1 += px*dx1*dy1;
            Fyim1 += py*dx1*dy1;
            Mzim1 += (xi*py - ybi*px)*dx1*dy1;
        }

        Fx += Fxim1;
        Fy += Fyim1;
        Mz += Mzim1;
    }

    state.slide_factor /= spec.num_rows * spec.num_bristles;

    //scalar t = -Mz/Fy;

    state.Fx = Fx;
    state.Fy = Fy;
    state.Mz = Mz;
}

scalar velocity_dependent_vertical_stiffness(scalar nominal_stiffness, scalar speed, scalar inflation_pressure) {
    scalar pinfl = inflation_pressure * 1.1;
    return nominal_stiffness * (pinfl/inflation_pressure) * (1 + 2.4e-3*(speed - 16.67));
}

}
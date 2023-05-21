#include <string>
#include <cmath>

// TODO: Align with ADAMS 5.2 conventions (see pdf annotations)
// TODO: Special care has to be taken for Fz, it should be saturated
// before computing the MF variables, but the output needs to be scaled back
// TODO: Transient slip
// TODO: Saturate slip
// TODO: Custom tire friction

typedef struct TireSlipState
{
    double kappa;
    double alpha;
    double v_sx;
    double v_sy;
    double v_re;
};

typedef struct TireParMF52
{
    // $-------------------------------------------------------------------------------
    // [MDI_HEADER]
    std::string FILE_TYPE = "tir";
    int FILE_VERSION = 2;
    std::string FILE_FORMAT = "ASCII";
    // (COMMENTS)
    // {comment_string}
    // 'Hoosier  6.0 / 18.0 - 10  LCO on 7 in. wide rim'
    // $-------------------------------------------------------------------------------
    // [UNITS]
    std::string LENGTH = "meter";
    std::string FORCE = "newton";
    std::string ANGLE = "radians";
    std::string MASS = "kg";
    std::string TIME = "second";
    // $-------------------------------------------------------------------------------
    // [MODEL]
    std::string PROPERTY_FILE_FORMAT = "MF_05";
    int USE_MODE = 14;
    int FITTYP = 5;
    double MFSAFE1 = -528;
    double MFSAFE2 = 0;
    double MFSAFE3 = 0;
    double VXLOW = 1;
    double LONGVL = 11.113889;
    // $-------------------------------------------------------------------------------
    // [DIMENSION]
    double UNLOADED_RADIUS = 0.22524021;
    double WIDTH = 0.1524;
    double RIM_RADIUS = 0.127;
    double RIM_WIDTH = 0.1778;
    // $-------------------------------------------------------------------------------
    // [VERTICAL]
    double VERTICAL_STIFFNESS = 86998.407;
    double VERTICAL_DAMPING = 500;
    double BREFF = 9.801034;
    double DREFF = 0.13264966;
    double FREFF = 0.210098;
    double FNOMIN = 700;
    // $-------------------------------------------------------------------------------
    // [LONG_SLIP_RANGE]
    double KPUMIN = -0.18;
    double KPUMAX = 0.18;
    // $-------------------------------------------------------------------------------
    // [SLIP_ANGLE_RANGE]
    double ALPMIN = -0.26179939;
    double ALPMAX = 0.26179939;
    // $-------------------------------------------------------------------------------
    // [INCLINATION_ANGLE_RANGE]
    double CAMMIN = 0;
    double CAMMAX = 0.06981317;
    // $-------------------------------------------------------------------------------
    // [VERTICAL_FORCE_RANGE]
    double FZMIN = 222.41108;
    double FZMAX = 1112.0554;
    // $-------------------------------------------------------------------------------
    // [SCALING_COEFFICIENTS]
    double LFZO = 1;
    double LCX = 1;
    double LMUX = 1;
    double LEX = 1;
    double LKX = 1;
    double LHX = 1;
    double LVX = 1;
    double LGAX = 1;
    double LCY = 1;
    double LMUY = 1;
    double LEY = 1;
    double LKY = 1;
    double LHY = 1;
    double LVY = 1;
    double LGAY = 1;
    double LTR = 1;
    double LRES = 1;
    double LGAZ = 1;
    double LXAL = 1;
    double LYKA = 1;
    double LVYKA = 1;
    double LS = 1;
    double LSGKP = 1;
    double LSGAL = 1;
    double LGYR = 1;
    double LMX = 1;
    double LVMX = 1;
    double LMY = 1;
    // $-------------------------------------------------------------------------------
    // [LONGITUDINAL_COEFFICIENTS]
    double PCX1 = 1.6665293;
    double PDX1 = 2.4768519;
    double PDX2 = -0.53147643;
    double PDX3 = 8.3517929;
    double PEX1 = 0.56037969;
    double PEX2 = -0.24251264;
    double PEX3 = 0.49734973;
    double PEX4 = 0.16134733;
    double PKX1 = 53.832396;
    double PKX2 = -0.10990728;
    double PKX3 = -0.44967965;
    double PHX1 = 0.0018085003;
    double PHX2 = 0.00019128982;
    double PVX1 = -0.10229506;
    double PVX2 = 0.009897706;
    double RBX1 = 4.2118702;
    double RBX2 = 9.5390909;
    double RCX1 = 1.4367635;
    double REX1 = -1.5486587;
    double REX2 = 1.870066;
    double RHX1 = -0.10211311;
    double PTX1 = 0.22524021;
    double PTX2 = -0.00045986321;
    double PTX3 = 0.44967965;
    // $-------------------------------------------------------------------------------
    // [OVERTURNING_COEFFICIENTS]
    double QSX1 = -0.0012736444;
    double QSX2 = 1.3374604;
    double QSX3 = -0.057725142;
    // $-------------------------------------------------------------------------------
    // [LATERAL_COEFFICIENTS]
    double PCY1 = 1.4043704;
    double PDY1 = 2.4867716;
    double PDY2 = -0.2045912;
    double PDY3 = 9.153668;
    double PEY1 = 0.39575701;
    double PEY2 = -0.067998402;
    double PEY3 = 0.16282804;
    double PEY4 = 8.5348072;
    double PKY1 = -45.373871;
    double PKY2 = 1.7713032;
    double PKY3 = 0.82067367;
    double PHY1 = -0.0035959889;
    double PHY2 = -0.0022229713;
    double PHY3 = -0.10723752;
    double PVY1 = -0.05267503;
    double PVY2 = 0.0016656414;
    double PVY3 = 0.063476006;
    double PVY4 = 1.7890555;
    double RBY1 = 14.717478;
    double RBY2 = 17.187177;
    double RBY3 = -0.047768723;
    double RCY1 = 1.0492159;
    double REY1 = 0.50066317;
    double REY2 = -0.29737153;
    double RHY1 = 0.012336864;
    double RHY2 = 0.018931134;
    double RVY1 = 3.79157;
    double RVY2 = -1.0101257;
    double RVY3 = 4.93289;
    double RVY4 = 17.138;
    double RVY5 = -2.3871522e-15;
    double RVY6 = -1.8457702e-15;
    double PTY1 = 0.45048041;
    double PTY2 = 1.7713032;
    // $-------------------------------------------------------------------------------
    // [ROLLING_COEFFICIENTS]
    double QSY1 = 0.01;
    double QSY2 = 0;
    double QSY3 = 0;
    double QSY4 = 0;
    // $-------------------------------------------------------------------------------
    // [ALIGNING_COEFFICIENTS]
    double QBZ1 = 5.0855072;
    double QBZ2 = 0.58498877;
    double QBZ3 = -0.83462989;
    double QBZ4 = -1.9253989e-15;
    double QBZ5 = 0.052667096;
    double QBZ9 = -7.3462502;
    double QBZ10 = -1.9988843;
    double QCZ1 = 1.9397849;
    double QDZ1 = 0.12862462;
    double QDZ2 = -0.014500336;
    double QDZ3 = -1.6877525;
    double QDZ4 = 5.8783689;
    double QDZ6 = 0.00918515;
    double QDZ7 = -0.018964835;
    double QDZ8 = 1.7345128;
    double QDZ9 = 0.27461419;
    double QEZ1 = 0.046997996;
    double QEZ2 = 1.7594722;
    double QEZ3 = -1.5258915;
    double QEZ4 = -0.11709783;
    double QEZ5 = 3.1681091;
    double QHZ1 = -0.013467777;
    double QHZ2 = -0.0092399665;
    double QHZ3 = 0.073063753;
    double QHZ4 = 0.55218275;
    double SSZ1 = 0.0373861;
    double SSZ2 = -0.050880155;
    double SSZ3 = -1.4782946;
    double SSZ4 = 1.0508605;
    double QTZ1 = 0;
    double MBELT = 0;
};

double sign(double val)
{
    return copysign(1.0, val);
}

class TireMdlMF52
{
    TireParMF52 par;

    bool flip_side = false;

private:
    double m_fx;
    double m_fy;
    double m_svyk;

public:
    double Fx(double fz, double kappa, double alpha, double gamma)
    {
        auto SL = kappa;
        auto FZ = fz;
        auto GAMMA = gamma;
        auto ALPHA = alpha;

        auto GAMMAX = GAMMA * par.LGAX;
        auto FZ0PR = par.FNOMIN * par.LFZO;
        auto DFZ = (FZ - FZ0PR) / FZ0PR;

        // Longitudinal force (pure longitudinal slip)
        auto SHX = (par.PHX1 + par.PHX2 * DFZ) * par.LHX;
        auto SLX = SL + SHX;
        auto CX = par.PCX1 * par.LCX;
        auto MUX = (par.PDX1 + par.PDX2 * DFZ) * (1.0 - par.PDX3 * (GAMMAX * GAMMAX)) * par.LMUX;
        auto DX = MUX * FZ;
        auto KX = FZ * (par.PKX1 + par.PKX2 * DFZ) * exp(par.PKX3 * DFZ) * par.LKX;
        auto BX = KX / (CX * DX);
        auto EX = fmin((par.PEX1 + DFZ * (par.PEX2 + par.PEX3 * DFZ)) * (1.0 - par.PEX4 * sign(SLX)) * par.LEX, 1.0);
        auto SVX = FZ * (par.PVX1 + par.PVX2 * DFZ) * par.LVX * par.LMUX;
        auto FX0 = DX * sin(CX * atan(BX * SLX - EX * (BX * SLX - atan(BX * SLX)))) + SVX;

        // Longitudinal force(combined slip)
        auto epsilon = 0.00001;
        auto SHXA = par.RHX1;
        auto EXA = par.REX1 + par.REX2 * DFZ;
        auto CXA = par.RCX1;
        auto BXA = par.RBX1 * cos(atan(par.RBX2 * SL) * par.LXAL);
        auto ALPHAS = ALPHA + SHXA;
        auto GXA0 = cos(CXA * atan(BXA * SHXA - EXA * (BXA * SHXA - atan(BXA * SHXA))));
        auto GXA = fmax(cos(CXA * atan(BXA * ALPHAS - EXA * (BXA * ALPHAS - atan(BXA * ALPHAS)))) / GXA0, epsilon);
        auto FX = GXA * FX0;

        return FX;
    }

    double Fy(double fz, double kappa, double alpha, double gamma)
    {
        // Apply corrections to SI units and sign convention
        auto SL = kappa;
        auto FZ = fz;
        auto GAMMA = gamma;
        auto ALPHA = alpha;

        // Compute Gamma and DFZ
        auto GAMMAY = GAMMA * par.LGAY;
        auto GAMMAZ = GAMMA * par.LGAZ;
        auto FZ0PR = par.FNOMIN * par.LFZO;
        auto DFZ = (FZ - FZ0PR) / FZ0PR;

        // Lateral force (pure side slip)
        auto SHY = (par.PHY1 + par.PHY2 * DFZ) * par.LHY + par.PHY3 * GAMMAY; // % removed  - 1 term
        auto ALPHAY = ALPHA + SHY;
        auto CY = par.PCY1 * par.LCY;
        auto MUY = (par.PDY1 + par.PDY2 * DFZ) * (1.0 - par.PDY3 * GAMMAY * GAMMAY) * par.LMUY;
        auto DY = MUY * FZ;
        auto KY = par.PKY1 * FZ0PR * sin(2.0 * atan(FZ / (par.PKY2 * FZ0PR))) * (1.0 - par.PKY3 * abs(GAMMAY)) * par.LKY;
        auto BY = KY / (CY * DY);
        auto EY = (par.PEY1 + par.PEY2 * DFZ) * (1.0 - (par.PEY3 + par.PEY4 * GAMMAY) * sign(ALPHAY)) * par.LEY;
        auto SVY = FZ * ((par.PVY1 + par.PVY2 * DFZ) * par.LVY + (par.PVY3 + par.PVY4 * DFZ) * GAMMAY) * par.LMUY;
        auto FY0 = DY * sin(CY * atan(BY * ALPHAY - EY * (BY * ALPHAY - atan(BY * ALPHAY)))) + SVY;

        // Lateral force (combined slip)
        auto epsilon = 0.00001;
        auto MUY = (par.PDY1 + par.PDY2 * DFZ) * (1.0 - par.PDY3 * (GAMMAY * GAMMAY)) * par.LMUY;
        auto DVYK = MUY * FZ * (par.RVY1 + par.RVY2 * DFZ + par.RVY3 * GAMMAZ) * cos(atan(par.RVY4 * ALPHA));
        auto SVYK = DVYK * sin(par.RVY5 * atan(par.RVY6 * SL)) * par.LVYKA;
        auto SHYK = par.RHY1 + par.RHY2 * DFZ;
        auto EYK = par.REY1 + par.REY2 * DFZ;
        auto CYK = par.RCY1;
        auto BYK = par.RBY1 * cos(abs(atan(par.RBY2 * (ALPHA - par.RBY3)))) * par.LYKA;
        auto KAPPAS = SL + SHYK;
        auto GYK0 = cos(CYK * atan(BYK * SHYK - EYK * (BYK * SHYK - atan(BYK * SHYK))));
        auto GYK = cos(CYK * atan(BYK * KAPPAS - EYK * (BYK * KAPPAS - atan(BYK * KAPPAS)))) / GYK0;
        auto FY = GYK * FY0 + SVYK;

        return FY;
    }

    double Fz(double rho, double drho)
    {
        // auto RL = rl;
        auto RHO = rho; // positive under compression
        auto DEL_RHO = drho;

        // Vertical force
        auto CZ = par.VERTICAL_STIFFNESS;
        auto DZ = par.VERTICAL_DAMPING;

        // auto RHO = par.UNLOADED_RADIUS - rl;
        auto FZ = RHO > 0.0 ? RHO * CZ + DEL_RHO * DZ : 0.0;

        return FZ;
    }

    double Mx(double fz, double kappa, double alpha, double gamma)
    {
        // Apply corrections to SI units and sign convention
        auto SL = kappa;
        auto FZ = fz;
        auto GAMMA = gamma;
        auto ALPHA = alpha;

        // get internal channels (from fx, fy)
        auto FX = m_fx;
        auto FY = m_fy;
        auto SVYK = m_svyk;

        // Compute Gamma and DFZ
        auto GAMMAY = GAMMA * par.LGAY;
        auto GAMMAZ = GAMMA * par.LGAZ;
        auto FZ0PR = par.FNOMIN * par.LFZO;
        auto DFZ = (FZ - FZ0PR) / FZ0PR;

        // Overturning moment (pure side slip)
        auto SHY = (par.PHY1 + par.PHY2 * DFZ) * par.LHY + par.PHY3 * GAMMAY;                                             // %38,  (%55)
        auto ALPHAY = ALPHA + SHY;                                                                                        // %30 (%47)
        auto CY = par.PCY1 * par.LCY;                                                                                     // %32 (%49)
        auto MUY = (par.PDY1 + par.PDY2 * DFZ) * (1.0 - par.PDY3 * GAMMAY * GAMMAY) * par.LMUY;                           // %34 (%51)
        auto DY = MUY * FZ;                                                                                               // %33 (%50)
        auto KY = par.PKY1 * FZ0PR * sin(2.0 * atan(FZ / (par.PKY2 * FZ0PR))) * (1.0 - par.PKY3 * abs(GAMMAY)) * par.LKY; // %36 (%53)
        auto BY = KY / (CY * DY);                                                                                         // %37 (%54)
        // % NOTE, PER SVEN @TNO: "SIGN(ALPHAY)"IS CORRECT AS IN DOCUMENTATION & BELOW; IT'S NOT SUPPOSED TO BE "SIGN(GAMMAY)"
        auto EY = (par.PEY1 + par.PEY2 * DFZ) * (1.0 - (par.PEY3 + par.PEY4 * GAMMAY) * sign(ALPHAY)) * par.LEY; // %35 (%52)
        // % NOTE: LVY MULTIPLIES ONLY PVY1&2 IN DOCUMENTATION; ORIG VERSION MULT ALL TERMS
        auto SVY = FZ * ((par.PVY1 + par.PVY2 * DFZ) * par.LVY + (par.PVY3 + par.PVY4 * DFZ) * GAMMAY) * par.LMUY; // %39 (%56)
        auto FY0 = DY * sin(CY * atan(BY * ALPHAY - EY * (BY * ALPHAY - atan(BY * ALPHAY)))) + SVY;                // %29 (%46)
        auto FY = FY0;                                                                                             // %28

        // auto MX = FZ * par.UNLOADED_RADIUS * (par.QSX1 - par.QSX2 * GAMMA + par.QSX3 * FY / FZ0PR) * par.LMX; // %84  SWIFT DOES NOT IMPLEMENT MX
        auto MX = FZ * par.UNLOADED_RADIUS * (par.QSX1 * par.LMX + (-par.QSX2 * GAMMA + par.QSX3 * FY / FZ0PR) * par.LMX); // %84  SWIFT DOES NOT IMPLEMENT MX

        return MX;
    }

    double My(double fz, double omega, double re)
    {
        auto FX = m_fx;
        auto FZ = fz;

        auto R0 = par.UNLOADED_RADIUS;
        auto FZ0PR = par.FNOMIN * par.LFZO;

        auto VX = omega * re;
        auto VREF = par.LONGVL;

        auto vx_vref_4 = pow(VX / VREF, 4.0);
        auto MY = -FZ * R0 * (par.QSY1 + par.QSY2 * FX / FZ0PR + par.QSY3 * abs(VX / VREF) + par.QSY4 * vx_vref_4);

        // TODO: Alternative formula iff QSY1 and QSY2 equal 0

        return 0;
    }

    double Mz(double fz, double kappa, double alpha, double gamma)
    {
        // Apply corrections to SI units and sign convention
        auto SL = kappa;
        auto FZ = fz;
        auto GAMMA = gamma;
        auto ALPHA = alpha;

        // get internal channels (from fx, fy)
        auto FX = m_fx;
        auto FY = m_fy;
        auto SVYK = m_svyk;

        // Compute Gamma and DFZ
        auto GAMMAY = GAMMA * par.LGAY;
        auto GAMMAZ = GAMMA * par.LGAZ;
        auto FZ0PR = par.FNOMIN * par.LFZO;
        auto DFZ = (FZ - FZ0PR) / FZ0PR;

        // Aligning moment (pure side slip)
        auto SHY = (par.PHY1 + par.PHY2 * DFZ) * par.LHY + par.PHY3 * GAMMAY;                                      // %38,  (%55)
        auto SVY = FZ * ((par.PVY1 + par.PVY2 * DFZ) * par.LVY + (par.PVY3 + par.PVY4 * DFZ) * GAMMAY) * par.LMUY; // %39 (%56)
        auto ALPHAY = ALPHA + SHY;                                                                                 // %30 (%47)

        auto SHT = par.QHZ1 + par.QHZ2 * DFZ + (par.QHZ3 + par.QHZ4 * DFZ) * GAMMAZ;                                      // %52 ( %68)
        auto ALPHAT = ALPHA + SHT;                                                                                        //  %43 (%59)
        auto KY = par.PKY1 * FZ0PR * sin(2.0 * atan(FZ / (par.PKY2 * FZ0PR))) * (1.0 - par.PKY3 * abs(GAMMAY)) * par.LKY; // %36 (%53)
        // % NOTE: PER SVEN, "EQUATION 45 IS WRONG DOCUMENTATION, THERE IT SHOULD BE SHF INSTEAD OF SHR"
        auto SHF = SHY + SVY / KY; // %46 (%62)
        auto ALPHAR = ALPHA + SHF; // %45 (%61)

        auto BT = (par.QBZ1 + DFZ * (par.QBZ2 + par.QBZ3 * DFZ)) * (1.0 + par.QBZ4 * GAMMAZ + par.QBZ5 * abs(GAMMAZ)) * par.LKY / par.LMUY;          // % 48(% 64)
        auto CT = par.QCZ1;                                                                                                                          //% 49(% 65)
        auto DT = FZ * (par.QDZ1 + par.QDZ2 * DFZ) * (1.0 + GAMMAZ * (par.QDZ3 + par.QDZ4 * GAMMAZ)) * (par.UNLOADED_RADIUS / par.FNOMIN) * par.LTR; // % 50(% 66)
        // % NOTE : EQUATION FOR ET HAS CHANGED FROM PAC97 EQUATION; 2 / PI TERM IS NEW.
        auto ET = (par.QEZ1 + DFZ * (par.QEZ2 + par.QEZ3 * DFZ)) * (1.0 + (par.QEZ4 + par.QEZ5 * GAMMAZ) * (2.0 / M_PI) * atan(BT * CT * ALPHAT)); // % 51(% 67)
        auto CY = par.PCY1 * par.LCY;                                                                                                              //% 32(% 49)
        auto MUY = (par.PDY1 + par.PDY2 * DFZ) * (1.0 - par.PDY3 * GAMMAY * GAMMAY) * par.LMUY;                                                    // % 34(% 51)

        auto DY = MUY * FZ;                                            // % 33(% 50)
        auto BY = KY / (CY * DY);                                      // % 37(% 54)
        auto BR = par.QBZ9 * par.LKY / par.LMUY + par.QBZ10 * BY * CY; // % 53(% 69)
        // % NOTE : LRES MULTIPLIES EVERYTHING IN ORIG EQN; BELOW MATCHES DOCUMENTATION
        auto DR = FZ * ((par.QDZ6 + par.QDZ7 * DFZ) * par.LRES + (par.QDZ8 + par.QDZ9 * DFZ) * GAMMAZ) * par.UNLOADED_RADIUS * par.LMUY; // % 54(% 70 LRES = LMR)
        auto TRAIL = DT * cos(CT * atan(BT * ALPHAT - ET * (BT * ALPHAT - atan(BT * ALPHAT)))) * cos(ALPHA);                             // % 42(% 58)
        auto MZR = DR * cos(atan(BR * ALPHAR)) * cos(ALPHA);                                                                             // % 44(% 60)
        auto EY = (par.PEY1 + par.PEY2 * DFZ) * (1.0 - (par.PEY3 + par.PEY4 * GAMMAY) * sign(ALPHAY)) * par.LEY;                         // % 35(% 52)
        // % NOTE : LVY MULTIPLIES ONLY PVY1 & 2 IN DOCUMENTATION; ORIG VERSION MULT ALL TERMS
        auto SVY = FZ * ((par.PVY1 + par.PVY2 * DFZ) * par.LVY + (par.PVY3 + par.PVY4 * DFZ) * GAMMAY) * par.LMUY; // % 39(% 56)
        auto FY0 = DY * sin(CY * atan(BY * ALPHAY - EY * (BY * ALPHAY - atan(BY * ALPHAY)))) + SVY;                // % 29(% 46)
        auto MZ0 = -TRAIL * FY0 + MZR;                                                                             // % 41(% 57)

        // Lateral force (combined slip)
        auto KXK = FZ * (par.PKX1 + par.PKX2 * DFZ) * exp(par.PKX3 * DFZ) * par.LKX;
        auto ALPHATEQ = atan(sqrt(tan(ALPHAT * ALPHAT) + (KXK / KY) * (KXK / KY) * SL * SL)) * sign(ALPHAT);
        auto ALPHAREQ = atan(sqrt(tan(ALPHAR * ALPHAR) + (KXK / KY) * (KXK / KY) * SL * SL)) * sign(ALPHAR);
        MZR = DR * cos(atan(BR * ALPHAREQ)) * cos(ALPHA);
        TRAIL = DT * cos(CT * atan(BT * ALPHATEQ - ET * (BT * ALPHATEQ - atan(BT * ALPHATEQ)))) * cos(ALPHA); // %42 (%58)
        auto FYP = FY - SVYK;
        auto S = par.UNLOADED_RADIUS * (par.SSZ1 + par.SSZ2 * (FY / FZ0PR) + (par.SSZ3 + par.SSZ4 * DFZ) * GAMMAZ) * par.LS;
        auto MZ = -TRAIL * FYP + MZR + S * FX;
    }

    double RadiusRolling(double fz)
    {
        auto FZ = fz;

        // Effective rolling radius(free rolling)
        auto CZ = par.VERTICAL_STIFFNESS;

        auto RHO = FZ / CZ;
        auto RHOFZ0 = par.FNOMIN / CZ;
        auto RHOD = RHO / RHOFZ0;

        auto RE = par.UNLOADED_RADIUS - RHOFZ0 * (par.FREFF * RHOD + par.DREFF * atan(par.BREFF * RHOD));

        return RE;
    }

    double RadiusUnloaded()
    {
        return par.UNLOADED_RADIUS;
    }

    TireSlipState Slip(double vcx, double vcy, double omega, double re)
    {
        auto RE = re;
        auto OMEGA = omega;

        auto V_CX = vcx;
        auto V_CY = vcy;

        auto VR = OMEGA * RE;

        // Protect at low velocity
        auto VMIN = par.VXLOW;

        // protect V_CX for low speed
        auto V_CX_S = sign(V_CX) * fmax(abs(V_CX), VMIN);

        // The longitudinal slip speed is defined as:
        auto V_SX = V_CX - VR;
        auto V_SY = V_CY;

        auto kappa = -V_SX / V_CX_S;
        auto alpha = atan(V_SY / abs(V_CX_S));

        // define tire state output
        TireSlipState ts = {};
        ts.alpha = alpha;
        ts.kappa = kappa;
        ts.v_re = VR;
        ts.v_sx = V_SX;
        ts.v_sy = V_SY;

        return ts;
    }

    void CalcForceMoments(double vcx, double vcy, double omega, double gamma, double rho, double drho)
    {
        // flip side
        if (flip_side)
        {
            vcy *= -1.0;
            gamma *= -1.0; // do we really want this?
        }

        // calculate fz (based on the loaded radius / deflection input)
        auto fz = Fz(rho, drho);

        // saturate fz, gamma
        auto fz_sat = fmin(fmax(fz, par.FZMIN), par.FZMAX);
        auto fz_scl = fmax(fz, 0.0) / fz_sat;
        auto gamma_sat = fmin(fmax(gamma, par.CAMMIN), par.CAMMAX);

        // calculate rolling radius
        auto re = RadiusRolling(fz_sat);

        // calculate slip
        auto slip = Slip(vcx, vcy, omega, re);

        // calculate remaining forces and moments
        auto alpha = slip.alpha;
        auto kappa = slip.kappa;

        auto fx = fz_scl * Fx(fz_sat, kappa, alpha, gamma_sat);
        auto fy = fz_scl * Fy(fz_sat, kappa, alpha, gamma_sat);
        auto mx = fz_scl * Mx(fz_sat, kappa, alpha, gamma_sat);
        auto my = fz_scl * My(fz_sat, omega, re);
        auto mz = fz_scl * Mz(fz_sat, kappa, alpha, gamma_sat);

        if (flip_side)
        {
            fy *= -1.0;
            mx *= -1.0;
            mz *= -1.0;
        }
    }
};

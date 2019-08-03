using Test
using FlightMechanics
using FlightMechanics.Models
using FlightMechanics.Aircrafts

#  Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
#  and simulation: dynamics, controls design, and autonomous systems. John Wiley
#  & Sons. (page 205)
pos = PositionEarth(0, 0, 0)
env = Environment(pos, atmos="ISAF16", wind="NoWind", grav="const")

FlightMechanics.Aircrafts.get_empty_cg(ac::F16) = [-0.30 * get_chord(ac), 0, 0]

ac = F16()
fcs = F16FCS()

h = 0.0 * M2FT
psi = 0.234077 # rad
gamma = 0.0
turn_rate = 0.3  # rad/s
tas_fts = 502.0  # ft/s

tas =  tas_fts * FT2M

# Initial conditions for trimmer
exp_α    =  2.392628e-1  # rad
exp_β    =  5.061803e-4  # rad
exp_thtl =  0.8349601
exp_de   = -1.481766     # deg
exp_da   =  9.553108e-2  # deg
exp_dr   = -4.118124e-1  # deg
α0 = exp_α + 10*DEG2RAD
β0 = 0.
stick_lon0 = exp_de/25.0 + 0.5
stick_lat0 = exp_da/21.5 + 0.5
pedals0 = exp_dr/30.0 + 0.5
thtl0 = exp_thtl + 0.3

set_stick_lon(fcs, stick_lon0)
set_stick_lat(fcs, stick_lat0)
set_pedals(fcs, pedals0)
set_thtl(fcs, thtl0)

ac, aerostate, state, fcs = steady_state_trim(
    ac, fcs, env, tas, pos, psi, gamma, turn_rate, α0, β0, show_trace=false
);

linear_system = Linearize(ac, state, aerostate, fcs, env);

A = get_A(linear_system)
A_ = [
    # u      v      theta  q      w      phi    p      r
    A[1,1] A[1,2] A[1,8] A[1,5] A[1,3] A[1,9] A[1,4] A[1,5];
    A[2,1] A[2,2] A[2,8] A[2,5] A[2,3] A[2,9] A[2,4] A[2,5];
    A[8,1] A[8,2] A[8,8] A[8,5] A[8,3] A[8,9] A[8,4] A[8,5];
    A[5,1] A[5,2] A[5,8] A[5,5] A[5,3] A[5,9] A[5,4] A[5,5];
    A[3,1] A[3,2] A[3,8] A[3,5] A[3,3] A[3,9] A[3,4] A[3,5];
    A[9,1] A[9,2] A[9,8] A[9,5] A[9,3] A[9,9] A[9,4] A[9,5];
    A[4,1] A[4,2] A[4,8] A[4,5] A[4,3] A[4,9] A[4,4] A[4,5];
    A[5,1] A[5,2] A[5,8] A[5,5] A[5,3] A[5,9] A[5,4] A[5,5]
]

# from book
A_exp = [                                
    -.090 -169 -31.2 -7.75 31.4 -7.73 5e-4 2e-3;
    -5e-4 -1.05 .0151 .903 3e-4 -.0607 -5e-4 -1e-4;
    .0 .0 .0 .203 .0 -.300 .0 -.979;
    1e-3 1.26 0 -1.66 1e-3 .0 .0589 -.0157;
    -1e-4 1.3e-4 -0.0032 7e-6 -.322 .0130 .248 -.961;
    .0 .0 .300 .508 .0 .0 1.0 .0105;
    -3e-4 .0578 .0 -.0469 -59.4 .0 -3.19 1.64;
    5e-5 -.0617 .0 .0123 8.88 .0 -.299 -.564
]



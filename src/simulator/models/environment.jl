using FlightMechanics


export AtmosphereISA, calculate_atmosphere
export ConstantWind, calculate_wind
export EarthConstantGravity, calculate_gravity
export Environment, DefaultEnvironment, calculate_environment,
    get_temperature, get_pressure, get_density, get_sound_velocity,
    get_wind, get_wind_NED, get_wind_direction, get_wind_intensity, get_wind_vertical,
    get_gravity_accel, get_grav_body_vector, get_grav_body_versor


# -------- ATMOSPHERE --------
abstract type Atmosphere end

struct AtmosphereISA<:Atmosphere
    temperature::Number  # K
    pressure::Number  # Pa
    density::Number  # kg/m³
    sound_velocity::Number  # m/s
end

# Sea level
AtmosphereISA() = AtmosphereISA(T0, P0, RHO0, A0)

get_temperature(atmos::Atmosphere) = atmos.temperature
get_pressure(atmos::Atmosphere) = atmos.pressure
get_density(atmos::Atmosphere) = atmos.density
get_sound_velocity(atmos::Atmosphere) = atmos.sound_velocity

function calculate_atmosphere(atmos::AtmosphereISA, state::State)
    AtmosphereISA(atmosphere_isa(get_height(state))...)
end

# -------- WIND --------
abstract type Wind end

struct ConstantWind<:Wind
    direction::Number  # coming from [rad]
    intensity::Number  # positive blowing to ac [m/s]
    vertical::Number   # positive blowing to ac [m/s]
end

ConstantWind() = ConstantWind(0, 0, 0)

get_wind(wind::Wind) = [wind.direction, wind.intensity, wind.vertical]
get_direction(wind::Wind) = wind.direction
get_intensity(wind::Wind) = wind.intensity
get_vertical(wind::Wind) = wind.vertical

function get_wind_NED(wind::Wind)
    # coming from
    wind_N = wind.intensity * cos(wind.direction + π)
    wind_E = wind.intensity * sin(wind.direction + π)
    wind_D = wind.vertical
    return [wind_N, wind_E, wind_D]
end

calculate_wind(wind::ConstantWind, state::State) = wind

# -------- GRAVITY --------
abstract type Gravity end

struct EarthConstantGravity<:Gravity
    value::Number  # m/s²
    body_vector::Array{T, 1} where T<:Number
end

EarthConstantGravity() = EarthConstantGravity(GRAVITY_ACCEL, [0, 0, GRAVITY_ACCEL])

function EarthConstantGravity(state::State)
    grav = EarthConstantGravity()
    calculate(grav, state)
end

get_gravity_accel(grav::Gravity) = grav.value
get_grav_body_vector(grav::Gravity) = grav.body_vector
get_grav_body_versor(grav::Gravity) = grav.body_vector / grav.value

function calculate_gravity(grav::EarthConstantGravity, state::State)
    grav.body_vector[:] = hor2body(0, 0, grav.value, get_euler_angles(state)...)
    return grav
end

# -------- ENVRIONMENT --------
struct Environment
    atmos::Atmosphere
    wind::Wind
    grav::Gravity
end

DefaultEnvironment() = Environment(AtmosphereISA(), ConstantWind(), EarthConstantGravity())

get_temperature(env::Environment) = get_temperature(env.atmos)
get_pressure(env::Environment) = get_pressure(env.atmos)
get_density(env::Environment) = get_density(env.atmos)
get_sound_velocity(env::Environment) = get_sound_velocity(env.atmos)

get_wind(env::Environment) = get_wind(env.wind)
get_wind_NED(env::Environment) = get_wind_NED(env.wind)
get_wind_direction(env::Environment) = get_wind_direction(env.wind)
get_wind_intensity(env::Environment) = get_wind_intensity(env.wind)
get_wind_vertical(env::Environment) = get_wind_vertical(env.wind)

get_gravity_accel(env::Environment) = get_gravity_accel(env.grav)
get_grav_body_vector(env::Environment) = get_grav_body_vector(env.grav)
get_grav_body_versor(env::Environment) = get_grav_body_versor(env.grav)


function calculate_environment(env::Environment, state::State)
    atmos = calculate_atmosphere(env.atmos, state)
    wind = calculate_wind(env.wind, state)
    grav = calculate_gravity(env.grav, state)
    Environment(atmos, wind, grav)
end

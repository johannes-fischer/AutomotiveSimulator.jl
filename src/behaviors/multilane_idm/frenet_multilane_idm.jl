mutable struct VirtualMultiLaneIDMTarget{I, M}
    id::I
    roadind::RoadIndex # represents relevant side of target vehicle (rear for front targets, ...)
    Δs::Float64 # signed distance s_ego - s_target
    v::Float64
    a::Float64
    T::Float64
    m::Union{Nothing, M}# For front target represents target_rear - ego_front, for rear target represents target_front - ego_rear; or different sign?
end

abstract type ManeuverStyle end
struct FiniteTime <: ManeuverStyle
    T::Float64 # time T available for whole maneuver
end
struct FiniteDistance <: ManeuverStyle
    ind::RoadIndex # road index until which maneuver has to be finished
end

Base.@kwdef mutable struct FrenetMultiLaneIDM{I, M} <: AbstractMultiLaneIDM{I}
    idm::IntelligentDriverModel = IntelligentDriverModel()
    a_cmf::Float64 = idm.d_cmf # comfortable acceleration [m/s²]
    target_style::MultiLaneIDMTargetStyle = TargetNeighbors()
    virtual_target_style::OptimizationCriterion = LinearPositionVelocityCriterion()
    maneuver_style::ManeuverStyle = FiniteTime(8.0)
    dt::Float64 = 1.0# simulation timestep (for replanning)
    virtual_target_lookahead::Float64 = dt/2 # should be dt/2 for midpoint rule (?)
    use_virtual_velocity::Bool = false # if false, only the virtual target's position is used to compute IDM formula, but not its velocity
    # fields for storing target results
    front_targets::Vector{MultiLaneIDMTarget{I}} = MultiLaneIDMTarget{Int}[]
    rear_targets::Vector{MultiLaneIDMTarget{I}} = MultiLaneIDMTarget{Int}[]
    virtual_front_targets::Dict{I,VirtualMultiLaneIDMTarget{I, M}} = Dict{Int,VirtualMultiLaneIDMTarget{Int, trajtype(virtual_target_style)}}()
    virtual_rear_targets::Dict{I,VirtualMultiLaneIDMTarget{I, M}} = Dict{Int,VirtualMultiLaneIDMTarget{Int, trajtype(virtual_target_style)}}()
    previous_front_target_ids::Set{I} = Set{Int}()
    previous_rear_target_ids::Set{I} = Set{Int}()
end

steady_state_distance(model::FrenetMultiLaneIDM, v) = model.idm.s_min + v * model.idm.T
steady_state_velocity(model::FrenetMultiLaneIDM, v) = v

function observe!(
    model::FrenetMultiLaneIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    egoid::I,
) where {S, D, I}
    veh_ego = get_by_id(scene, egoid)

    # set front and rear targets
    empty!(model.front_targets)
    empty!(model.rear_targets)

    set_targets!(model, scene, roadway, egoid, model.target_style)
    update_virtual_targets!(model, scene, roadway, egoid)
    set_virtual_targets!(model, model.front_targets, model.virtual_front_targets, veh_ego)
    set_virtual_targets!(model, model.rear_targets, model.virtual_rear_targets, veh_ego)

    track_longitudinal!(model, scene, roadway, egoid)

    model.previous_front_target_ids = Set([target.id for target in model.front_targets])
    model.previous_rear_target_ids = Set([target.id for target in model.rear_targets])
    return model
end

function update_virtual_targets!(
    model::FrenetMultiLaneIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    egoid::I,
) where {S, D, I}
    front_target_ids = [target.id for target in model.front_targets]
    rear_target_ids = [target.id for target in model.rear_targets]
    # Remove virtual targets that are not targets anymore
    filter!(∈(front_target_ids) ∘ first, model.virtual_front_targets)
    filter!(∈(rear_target_ids) ∘ first, model.virtual_rear_targets)

    veh_ego = get_by_id(scene, egoid)
    update_virtual_targets!(model, scene, roadway, veh_ego, model.front_targets, model.virtual_front_targets, model.previous_front_target_ids, false)
    update_virtual_targets!(model, scene, roadway, veh_ego, model.rear_targets, model.virtual_rear_targets, model.previous_rear_target_ids, true)
end

function update_virtual_targets!(
    model::FrenetMultiLaneIDM{I,M},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    veh_ego::Entity{S, D, I},
    targets::Vector{MultiLaneIDMTarget{I}},
    virtual_targets::Dict{I,VirtualMultiLaneIDMTarget{I,M}},
    previous_targets::Set{I},
    rear::Bool,
) where {S, D, I, M}
    v_ego = vel(veh_ego)

    for target in targets
        veh_target = get_by_id(scene, target.id)

        if target.id ∈ keys(virtual_targets)
            # virtual target already exists
            virtual_target = virtual_targets[target.id]

            if virtual_target.T - model.dt/2 <= 0.0
                # virtual target has expired
                delete!(virtual_targets, target.id)
                continue
            else
                @assert !isnothing(virtual_target.m)
                # update virtual target
                update_virtual_target_state!(model, roadway, veh_ego, virtual_target, rear)
                update_maneuver_duration!(virtual_target, model.maneuver_style, model, veh_ego, veh_target, roadway)
            end
        # elseif -target.Δs >= 0.99 * ss_distance
        else
            # virtual target does not exist

            # check if new target and if interaction term would result in comfortable acceleration/decleration
            new_target = target.id ∉ previous_targets
            cmf_accel = rear ? model.a_cmf : model.idm.d_cmf
            if new_target && interaction_term(model, v_ego, target, rear=rear) >= sqrt(1 + cmf_accel / model.idm.a_max)
                # if not, create new virtual target point based on neutral virtual front vehicle
                virtual_target = create_virtual_target(model, roadway, target, veh_ego, veh_target, rear)
                push!(virtual_targets, target.id => virtual_target)
            else
                # if yes it is not necessary to create virtual target
                continue
            end
        end

        update_virtual_target_motion!(virtual_target, target, model.virtual_target_style)
    end
end

function create_virtual_target(
    model::FrenetMultiLaneIDM{I, M},
    roadway::Roadway,
    target::MultiLaneIDMTarget,
    veh_ego::Entity{S, D, I},
    veh_target::Entity{S, D, I},
    rear::Bool,
) where {S, D, I, M}

    v_ego = vel(veh_ego)
    distance = steady_state_distance(model, v_ego)
    velocity = steady_state_velocity(model, v_ego)

    if rear
        move_distance = target.Δs - distance + targetpoint_delta(VehicleTargetPointFront(), veh_target)
    else
        # target.Δs contains vehicle dimensions. i.e. moving is by only target.Δs would result in ego front kissing target rear
        move_distance = target.Δs + distance + targetpoint_delta(VehicleTargetPointRear(), veh_target)
    end

    virtual_roadind = move_along(
        posf(veh_target).roadind,
        roadway,
        move_distance
    )
    frel = get_frenet_relative_position(veh_ego, virtual_roadind, roadway)
    if rear
        Δs = frel.Δs + targetpoint_delta(VehicleTargetPointRear(), veh_ego)
        # @show Δs frel.Δs distance
        @assert Δs ≈ distance
    else
        # Δs is negative (for virtual front target) hence ADD target point to rduce distance
        Δs = frel.Δs + targetpoint_delta(VehicleTargetPointFront(), veh_ego)
        # Verify that roadindex is at correct position
        # @show Δs frel.Δs distance
        @assert -Δs ≈ distance
    end
    v = velocity
    # Start target acceleration at comfortable values
    a = rear ? model.a_cmf : -model.idm.d_cmf
    T = maneuver_duration(model.maneuver_style, veh_ego, veh_target, roadway)
    virtual_target = VirtualMultiLaneIDMTarget{I, M}(
        target.id,
        virtual_roadind,
        Δs,
        v,
        a,
        T,
        nothing,
    )
    return virtual_target
end

function maneuver_duration(maneuver::FiniteTime, ego::Entity, target::Entity, roadway)
    return maneuver.T
end

function maneuver_duration(maneuver::FiniteDistance, ego::Entity, target::Entity, roadway)
    d = get_frenet_relative_position(ego, maneuver.ind, roadway).Δs
    # d2 = get_frenet_relative_position(target, maneuver.ind, roadway).Δs
    # @show d, d2
    # d is negative, hence adding target point delta to reduce distance
    d += targetpoint_delta(VehicleTargetPointFront(), ego)
    return -d / vel(target)

end

function update_maneuver_duration!(
    virtual_target::VirtualMultiLaneIDMTarget,
    maneuver::FiniteTime, model, ego::Entity, target::Entity, roadway
)
    virtual_target.T -= model.dt
end

function update_maneuver_duration!(
    virtual_target::VirtualMultiLaneIDMTarget,
    maneuver::FiniteDistance, model, ego::Entity, target::Entity, roadway
)
    virtual_target.T = maneuver_duration(maneuver, ego, target, roadway)
end

# TODO: LQR
function update_virtual_target_state!(
    model::FrenetMultiLaneIDM{I, M},
    roadway::Roadway,
    veh_ego::Entity{S, D, I},
    virtual_target::VirtualMultiLaneIDMTarget{I, M},
    rear::Bool
) where {S, D, I, M<:OptimalTrajectory}
    target_Δs = xt(virtual_target.m, model.dt) - xt(virtual_target.m, 0.0)
    virtual_target.roadind = move_along(
        virtual_target.roadind,
        roadway,
        target_Δs,
    )
    frel = get_frenet_relative_position(veh_ego, virtual_target.roadind, roadway)
    ego_targetpoint = rear ? VehicleTargetPointRear() : VehicleTargetPointFront()
    virtual_target.Δs = frel.Δs + targetpoint_delta(ego_targetpoint, veh_ego)
    virtual_target.v = vt(virtual_target.m, model.dt)
    virtual_target.a = at(virtual_target.m, model.dt)
    return virtual_target
end

function update_virtual_target_motion!(
    virtual_target::VirtualMultiLaneIDMTarget,
    target::MultiLaneIDMTarget,
    style::OptimizationCriterion,
)
    # Virtual target trajectory start and end values
    s_start = -virtual_target.Δs
    v_start = virtual_target.v
    a_start = virtual_target.a
    s_end = virtual_target.T * target.v - target.Δs
    v_end = target.v
    a_end = 0.0
    # @show virtual_target.Δs virtual_target.v virtual_target.a
    # @show target.Δs
    # @show s_end v_end

    horizon = virtual_target.T
    # horizon = max(model.dt, virtual_target.T) # TODO: Safeguard this?
    start = @SVector[s_start, v_start, a_start]
    goal = @SVector[s_end, v_end, a_end]

    # @show start goal horizon

    # virtual_target.m = optimize(style, start, goal, horizon)
    update!(virtual_target, style, start, goal, horizon)
    return virtual_target.m
end

# TODO: LQR
function set_virtual_targets!(
    model::FrenetMultiLaneIDM{I, M},
    targets::Vector{MultiLaneIDMTarget{I}},
    virtual_targets::Dict{I,VirtualMultiLaneIDMTarget{I, M}},
    veh_ego::Entity{S, D, I},
    # scene::EntityScene{S, D, I},
    # roadway::Roadway,
    # egoid::I,
) where {S, D, I, M}
    # TODO: Fix virtual target lookahead for large dt simulation timesteps
    ego_traveled_distance = model.virtual_target_lookahead * vel(veh_ego)
    for (i, target) in pairs(targets)
        if target.id in keys(virtual_targets)
            virtual_target = virtual_targets[target.id]
            m = virtual_target.m
            Δs = ego_traveled_distance - xt(m, model.virtual_target_lookahead)
            # @show Δs
            if model.use_virtual_velocity
                v = vt(m, model.virtual_target_lookahead)
            else
                v = target.v
            end
            targets[i] = MultiLaneIDMTarget(target.id, Δs, v)
        end
    end
end


function track_longitudinal!(
    model::FrenetMultiLaneIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    egoid::I,
) where {S, D, I}
    idm = model.idm

    veh_ego = get_by_id(scene, egoid)
    v_ego = vel(veh_ego)
    v_ratio = model.idm.v_des > 0.0 ? (v_ego / model.idm.v_des) : Inf

    max_front_interaction = maximum(
        target -> interaction_term(model, v_ego, target, rear=false),
        model.front_targets,
        init=-Inf,
    )
    max_rear_interaction = maximum(
        target -> interaction_term(model, v_ego, target, rear=true),
        model.rear_targets,
        init=-Inf,
    )

    if max_rear_interaction == Inf && (max_front_interaction == Inf || v_ratio == Inf)
        # front interaction overwrites rear interaction
        idm.a = -Inf
    else
        free_flow_term = 1.0 - v_ratio^idm.δ
        front_term = 1.0 - max_front_interaction^2
        rear_term = max_rear_interaction^2 - 1.0

        # check existence of targets, otherwise nonexistant rear target leads to infinite rear_term, similarly for front target
        front_target_exists = length(model.front_targets) > 0
        rear_target_exists = length(model.rear_targets) > 0

        front_active = front_target_exists && front_term < free_flow_term
        rear_active = rear_target_exists && rear_term > free_flow_term
        gap_active = front_target_exists && rear_target_exists && front_term < rear_term

        if gap_active
            f = 0.5 * (front_term + rear_term)
        elseif front_active
            f = front_term
        elseif rear_active
            f = rear_term
        else
            f = free_flow_term
        end

        idm.a = idm.a_max * f
    end


    idm.a = clamp(idm.a, -idm.d_max, idm.a_max)
    return model
end

"""
    Δs is positive distance between vehicles
"""
function interaction_term(
    model::FrenetMultiLaneIDM,
    v_ego::Float64,
    other::MultiLaneIDMTarget{I};
    rear::Bool,
) where {I}
    Δv = v_ego - other.v
    Δs = -other.Δs # s_other - s_ego, to make it positive for regular IDM car following
    v_approach = v_ego
    # if target is rear target of ego vehicle, reverse sign of Δs and Δv and use approach velocity of rear vehicle
    if rear
        Δv = -Δv
        Δs = -Δs
        v_approach = other.v
    end
    Δs = max(Δs, 0.0)
    @assert Δs >= 0

    (; s_min, T, a_max, d_cmf) = model.idm # named destructuring
    # if Δv is so negative that s_des < 0, resulting force should not become stronger again, so max(0, ...)
    # e.g. if front target drives much faster than ego (and is before ego)
    s_des = s_min + max(0., v_approach * T + v_approach * Δv / (2 * sqrt(a_max * d_cmf)))
    s_des / Δs
end


Base.@kwdef mutable struct MultiLaneIDM{I} <: AbstractMultiLaneIDM{I}
    idm::IntelligentDriverModel = IntelligentDriverModel()
    front_targets::Vector{MultiLaneIDMTarget{I}} = MultiLaneIDMTarget{Int}[]
    rear_targets::Vector{MultiLaneIDMTarget{I}} = MultiLaneIDMTarget{Int}[]
    target_style::MultiLaneIDMTargetStyle = TargetNeighbors()
    Ts::Union{Nothing, Real} = nothing # smoothing time delay
end

function track_longitudinal!(
    model::MultiLaneIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    egoid::I,
) where {S, D, I}
    idm = model.idm
    idm.a = Inf

    veh_ego = get_by_id(scene, egoid)
    v_ego = vel(veh_ego)

    goal_distance = steady_state_distance(model, v_ego)

    if isempty(model.front_targets) && isempty(model.rear_targets)
        # no target vehicles, just drive to match desired speed
        Δv = idm.v_des - v_ego
        idm.a = Δv * idm.k_spd # predicted accel to match target speed
    else
        # Switch between IDM and const accel as presented in summer seminar
        follow_idm = true

        max_front_interaction = 0.0
        for target in model.front_targets
            interaction = interaction_term(model, scene, veh_ego, target)
            # if a front target is behind ego, the interaction term will be Inf
            if !isnothing(model.Ts) && (interaction == Inf || -target.Δs < 0.9*goal_distance)
                # println("Front const accel")
                # @show 0.9*goal_distance + target.Δs
                Δv = v_ego - target.v
                Δs = target.Δs # s_ego - s_target
                idm.a = min(idm.a, 2 * ( -Δv * model.Ts - Δs - goal_distance ) / (model.Ts^2))
                follow_idm = false
            end
            max_front_interaction = max(max_front_interaction, interaction)
        end

        max_rear_interaction = 0.0
        for target in model.rear_targets
            interaction = interaction_term(model, scene, veh_ego, target, rear=true)
            # if a rear target is before ego, the interaction term will be Inf
            if !isnothing(model.Ts) && (interaction == Inf || target.Δs < 0.9*goal_distance)
                # println("Rear const accel")
                # @show 0.9*goal_distance - target.Δs
                Δv = v_ego - target.v
                Δs = -target.Δs # s_target - s_ego
                idm.a = min(idm.a, 2 * ( -Δv * model.Ts + Δs + goal_distance ) / (model.Ts^2))
                follow_idm = false
            end
            max_rear_interaction = max(max_rear_interaction, interaction)
        end

        if follow_idm
            if max_rear_interaction == Inf && (max_front_interaction == Inf || iszero(model.idm.v_des))
                # front interaction overwrites rear interaction
                idm.a = -Inf
            else
                v_ratio = v_ego / model.idm.v_des
                idm.a =
                    idm.a_max * (
                        1.0 - v_ratio^idm.δ - max(max_front_interaction)^2 +
                        max(max_rear_interaction)^2
                    )
                # if egoid == 3
                #     @show v_ego, model.idm.v_des
                #     @show 1 - v_ratio^idm.δ
                #     @show -max(max_front_interaction)^2
                #     @show max(max_rear_interaction)^2
                #     @show 1 - v_ratio^idm.δ + max(max_rear_interaction)^2-max(max_front_interaction)^2
                # end
            end
        end
    end

    idm.a = clamp(idm.a, -idm.d_max, idm.a_max)
    return model
end

"""
    Δs is positive distance between vehicles
"""
function interaction_term(
    model::MultiLaneIDM,
    scene::EntityScene{S, D, I},
    veh_ego::Entity{S, D, I},
    other::MultiLaneIDMTarget{I};
    rear::Bool=false,
) where {S, D, I}
    v_ego = vel(veh_ego)

    Δv = v_ego - other.v
    Δs = -other.Δs # s_other - s_ego, to make it positive for regular IDM car following
    v_approach = v_ego
    # if target is rear target of ego vehicle, reverse sign of Δs and Δv
    if rear
        Δv = -Δv
        Δs = -Δs
        v_approach = other.v
    end

    # @show veh_ego.id
    # @show Δs
    # @show posf(veh_ego).s
    # @show posf(get_by_id(scene, other.id)).s

    # If a front target is behind ego or a rear target in front of ego -> infinite repulsive force
    Δs <= 0 && return Inf
    @assert Δs > 0

    (; s_min, T, a_max, d_cmf) = model.idm # named destructuring
    # if Δv is so negative that s_des < 0, resulting force should not become stronger again, so max(0, ...)
    # e.g. if front target drives much faster than ego (and is before ego)
    s_des = s_min + max(0., v_approach * T + v_approach * Δv / (2 * sqrt(a_max * d_cmf)))
    s_des / Δs
end

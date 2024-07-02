
Base.@kwdef mutable struct SmoothMultiLaneIDM{I} <: AbstractMultiLaneIDM{I}
    idm::IntelligentDriverModel = IntelligentDriverModel()
    front_targets::Vector{MultiLaneIDMTarget{I}} = MultiLaneIDMTarget{Int}[]
    rear_targets::Vector{MultiLaneIDMTarget{I}} = MultiLaneIDMTarget{Int}[]
    target_style::MultiLaneIDMTargetStyle = TargetNeighbors()
    sharpness::Float64 = 1.0
    shift::Float64 = 0.0
end

function track_longitudinal!(
    model::SmoothMultiLaneIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    egoid::I,
) where {S, D, I}
    idm = model.idm

    veh_ego = get_by_id(scene, egoid)
    v_ego = vel(veh_ego)
    v_ratio = model.idm.v_des > 0.0 ? (v_ego / model.idm.v_des) : Inf

    if isempty(model.front_targets) && isempty(model.rear_targets)
        # no target vehicles, just drive to match desired speed
        Δv = idm.v_des - v_ego
        idm.a = Δv * idm.k_spd # predicted accel to match target speed
    else
        max_front_interaction = maximum(
            target -> interaction_term(model, scene, veh_ego, target),
            model.front_targets,
            init=0.0,
        )
        max_rear_interaction = maximum(
            target -> interaction_term(model, scene, veh_ego, target, rear=true),
            model.rear_targets,
            init=0.0,
        )

        if max_rear_interaction == Inf && (max_front_interaction == Inf || v_ratio == Inf)
            # front interaction overwrites rear interaction
            idm.a = -Inf
        else
            idm.a =
                idm.a_max * (
                    1.0 - v_ratio^idm.δ - max(max_front_interaction)^2 +
                    max(max_rear_interaction)^2
                )
        end
    end

    idm.a = clamp(idm.a, -idm.d_max, idm.a_max)
    return model
end

"""
Δs is positive distance between vehicles
"""
function interaction_term(
    model::SmoothMultiLaneIDM,
    scene::EntityScene{S, D, I},
    veh_ego::Entity{S, D, I},
    other::MultiLaneIDMTarget{I};
    rear::Bool=false,
) where {S, D, I}
    v_ego = vel(veh_ego)

    Δv = v_ego - other.v
    Δs = -other.Δs # = s_other - s_ego, to make it positive for regular IDM car following
    v_approach = v_ego
    # if target is rear target of ego vehicle, reverse sign of Δs and Δv
    if rear
        Δv = -Δv
        Δs = -Δs
        v_approach = other.v
    end

    (; s_min, T, a_max, d_cmf) = model.idm # named destructuring
    # if Δv is so negative that s_des < 0, resulting force should not become stronger again, so max(0, ...)
    # e.g. if front target drives much faster than ego (and is before ego)
    s_des = s_min + max(0., v_approach * T + v_approach * Δv / (2 * sqrt(a_max * d_cmf)))

    # https://en.wikipedia.org/wiki/Rectifier_(neural_networks)
    unshifted_Δs = log1pexp(model.sharpness * Δs) / model.sharpness
    # shifted_Δs = log(1 + model.shift + exp(model.sharpness * Δs)) / model.sharpness
    shifted_Δs = log1pxpexpy(model.shift, model.sharpness * Δs) / model.sharpness
    smooth_Δs = shifted_Δs

    s_des / smooth_Δs
end

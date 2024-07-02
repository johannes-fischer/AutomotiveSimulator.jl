
Base.@kwdef mutable struct GapIDM{I, M} <: AbstractMultiLaneIDM{I}
    idm::IntelligentDriverModel = IntelligentDriverModel()
    a_cmf::Float64 = idm.d_cmf # comfortable acceleration [m/s²]
    target_style::MultiLaneIDMTargetStyle = TargetNeighbors()
    # virtual_target_style::OptimizationCriterion = LinearPositionVelocityCriterion()
    maneuver_style::ManeuverStyle = FiniteTime(8.0)
    dt::Float64 = 1.0# simulation timestep (for replanning)
    # virtual_target_lookahead::Float64 = dt/2 # should be dt/2 for midpoint rule (?)
    # use_virtual_velocity::Bool = false # if false, only the virtual target's position is used to compute IDM formula, but not its velocity
    # fields for storing target results
    front_targets::Vector{MultiLaneIDMTarget{I}} = MultiLaneIDMTarget{Int}[]
    rear_targets::Vector{MultiLaneIDMTarget{I}} = MultiLaneIDMTarget{Int}[]
    # virtual_front_targets::Dict{I,VirtualMultiLaneIDMTarget{I, M}} = Dict{Int,VirtualMultiLaneIDMTarget{Int, trajtype(virtual_target_style)}}()
    # virtual_rear_targets::Dict{I,VirtualMultiLaneIDMTarget{I, M}} = Dict{Int,VirtualMultiLaneIDMTarget{Int, trajtype(virtual_target_style)}}()
    # previous_front_target_ids::Set{I} = Set{Int}()
    # previous_rear_target_ids::Set{I} = Set{Int}()
end

function observe!(
    model::GapIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    egoid::I,
) where {S, D, I}
    veh_ego = get_by_id(scene, egoid)

    # set front and rear targets
    empty!(model.front_targets)
    empty!(model.rear_targets)

    set_targets!(model, scene, roadway, egoid, model.target_style)
    # update_virtual_targets!(model, scene, roadway, egoid)
    # set_virtual_targets!(model, model.front_targets, model.virtual_front_targets, veh_ego)
    # set_virtual_targets!(model, model.rear_targets, model.virtual_rear_targets, veh_ego)

    track_longitudinal!(model, scene, roadway, egoid)

    # model.previous_front_target_ids = Set([target.id for target in model.front_targets])
    # model.previous_rear_target_ids = Set([target.id for target in model.rear_targets])
    return model
end

# # TODO: LQR
# function set_virtual_targets!(
#     model::FrenetMultiLaneIDM{I, M},
#     targets::Vector{MultiLaneIDMTarget{I}},
#     virtual_targets::Dict{I,VirtualMultiLaneIDMTarget{I, M}},
#     veh_ego::Entity{S, D, I},
#     # scene::EntityScene{S, D, I},
#     # roadway::Roadway,
#     # egoid::I,
# ) where {S, D, I, M}
#     # TODO: Fix virtual target lookahead for large dt simulation timesteps
#     ego_traveled_distance = model.virtual_target_lookahead * vel(veh_ego)
#     for (i, target) in pairs(targets)
#         if target.id in keys(virtual_targets)
#             virtual_target = virtual_targets[target.id]
#             m = virtual_target.m
#             Δs = ego_traveled_distance - xt(m, model.virtual_target_lookahead)
#             # @show Δs
#             if model.use_virtual_velocity
#                 v = vt(m, model.virtual_target_lookahead)
#             else
#                 v = target.v
#             end
#             targets[i] = MultiLaneIDMTarget(target.id, Δs, v)
#         end
#     end
# end

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

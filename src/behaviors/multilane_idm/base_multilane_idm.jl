abstract type AbstractMultiLaneIDM{I} <: LaneFollowingDriver end

struct MultiLaneIDMTarget{I}
    id::I
    Δs::Float64 # signed distance s_ego - s_other
    v::Float64
end

abstract type MultiLaneIDMTargetStyle end
Base.@kwdef mutable struct TargetByID{I} <: MultiLaneIDMTargetStyle
    front_target_ids::Vector{I} = Int[]
    rear_target_ids::Vector{I} = Int[]
end

Base.@kwdef mutable struct TargetNeighbors{I} <: MultiLaneIDMTargetStyle
    front_target_ids::Vector{I} = Int[]
    rear_target_ids::Vector{I} = Int[]
end

function set_desired_speed!(model::AbstractMultiLaneIDM, v_des::Float64)
    set_desired_speed!(model.idm, v_des)
    model
end
Base.rand(rng::AbstractRNG, model::AbstractMultiLaneIDM) = rand(rng, model.idm)

steady_state_distance(model::AbstractMultiLaneIDM, v) = model.idm.s_min + v * model.idm.T

function observe!(
    model::AbstractMultiLaneIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    egoid::I,
) where {S, D, I}
    # set front and rear targets
    empty!(model.front_targets)
    empty!(model.rear_targets)

    set_targets!(model, scene, roadway, egoid, model.target_style)

    track_longitudinal!(model, scene, roadway, egoid)
    return model
end

function set_targets!(
    model::AbstractMultiLaneIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    egoid::I,
    target_style::TargetByID{I}
) where {S, D, I}
    veh_ego = get_by_id(scene, egoid)
    for target_id in target_style.front_target_ids
        veh_oth = get_by_id(scene, target_id)
        frel = get_frenet_relative_position(veh_ego, veh_oth, roadway)
        # for front targets, Δs has to be negative (ego is behind target),
        # hence shrinking distance by vehicle dimensions means increasing Δs
        Δs =
            frel.Δs + targetpoint_delta(VehicleTargetPointFront(), veh_ego) -
            targetpoint_delta(VehicleTargetPointRear(), veh_oth)
        push!(
            model.front_targets,
            MultiLaneIDMTarget(veh_oth.id, Δs, vel(veh_oth)),
        )
    end
    for target_id in target_style.rear_target_ids
        veh_oth = get_by_id(scene, target_id)
        frel = get_frenet_relative_position(veh_ego, veh_oth, roadway)
        # for rear targets, Δs is positive,
        # hence shrinking the distance by vehicle dimensions means decreasing Δs
        Δs =
            frel.Δs + targetpoint_delta(VehicleTargetPointRear(), veh_ego) -
            targetpoint_delta(VehicleTargetPointFront(), veh_oth)
        push!(
            model.rear_targets,
            MultiLaneIDMTarget(veh_oth.id, Δs, vel(veh_oth)),
        )
    end
end

function set_targets!(
    model::AbstractMultiLaneIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    egoid::I,
    ::TargetNeighbors
) where {S, D, I}
    veh_ego = get_by_id(scene, egoid)

    lane = get_lane(roadway, veh_ego)
    add_lane_targets!(model, scene, roadway, lane, veh_ego)

    rlane = rightlane(roadway, lane)
    isnothing(rlane) || add_lane_targets!(model, scene, roadway, rlane, veh_ego)

    llane = leftlane(roadway, lane)
    isnothing(llane) || add_lane_targets!(model, scene, roadway, llane, veh_ego)
end

function add_lane_targets!(
    model::AbstractMultiLaneIDM{I},
    scene::EntityScene{S, D, I},
    roadway::Roadway,
    lane::Lane,
    veh_ego::Entity{S, D, I},
) where {S, D, I}
    front_neighbor = find_neighbor(
        scene,
        roadway,
        veh_ego,
        lane=lane,
        targetpoint_ego=VehicleTargetPointFront(),
        targetpoint_neighbor=VehicleTargetPointRear(),
    )
    if !isnothing(front_neighbor.ind)
        vehicle = scene[front_neighbor.ind]
        push!(
            model.front_targets,
            MultiLaneIDMTarget(vehicle.id, -front_neighbor.Δs, vel(vehicle)),
        )
        push!(model.target_style.front_target_ids, vehicle.id)
    end

    rear_neighbor = find_neighbor(
        scene,
        roadway,
        veh_ego,
        lane=lane,
        rear=true,
        targetpoint_ego=VehicleTargetPointFront(),
        targetpoint_neighbor=VehicleTargetPointFront(),
    )
    if !isnothing(rear_neighbor.ind)
        vehicle = scene[rear_neighbor.ind]
        push!(
            model.rear_targets,
            MultiLaneIDMTarget(vehicle.id, rear_neighbor.Δs, vel(vehicle)),
        )
        push!(model.target_style.rear_target_ids, vehicle.id)
    end
end

using Random
# using Printf
using LogExpFunctions
using StaticUnivariatePolynomials
using StaticUnivariatePolynomials: derivative
using StaticArrays
using OptimalFrenetTrajectory


export
    AbstractMultiLaneIDM,
    MultiLaneIDMTarget,
    TargetByID,
    TargetNeighbors
include("base_multilane_idm.jl")

export
    MultiLaneIDM
include("multilane_idm.jl")

export
    SmoothMultiLaneIDM
include("smooth_multilane_idm.jl")

export
    FrenetMultiLaneIDM,
    FiniteTime,
    FiniteDistance
include("frenet_multilane_idm.jl")
include("virtual_targets/frenet_polynomials.jl")
include("virtual_targets/lqr_trajectory.jl")

include("util.jl")
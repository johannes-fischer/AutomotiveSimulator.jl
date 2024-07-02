trajtype(::MinimumJerkCriterion) = PolynomialTrajectory{Polynomial{6,Float64}}
trajtype(::MinimumAccelerationCriterion) = PolynomialTrajectory{Polynomial{4,Float64}}
trajtype(::LinearPositionVelocityCriterion) = LinearTrajectory{Polynomial{2,Float64}, Polynomial{2,Float64}}

function update!(
    target::VirtualMultiLaneIDMTarget{<:Any, <:OptimalTrajectory},
    style::OptimizationCriterion,
    start,
    goal,
    t::Real
)
    target.m = optimize(style, start, goal, t)
end
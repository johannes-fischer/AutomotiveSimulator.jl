using AutomotiveSimulator


A = VecE2(0.0,0.0)
B = VecE2(100.0,0.0)

T = Float64
center_line = gen_straight_curve(A,B,4)
lane1 = Lane(LaneTag(1,1),center_line,[3.0,2.0,1.0,1.0])
lane2 = Lane(LaneTag(1,2),center_line)
seg = RoadSegment(1,[lane1])
road = Roadway([seg])

AutomotiveVisualization.colortheme["background"] = colorant"white";
snapshot = render([road])
write("merging_scenario.svg", snapshot)




for seg in road.segments
    ll = seg.lanes[end]
    for (i,pt) in enumerate(ll.curve)
        print(pt.pos+polar(ll.width[i]/2,pt.pos.θ + π/2))
    end
end
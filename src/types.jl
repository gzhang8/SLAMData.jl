import LinearAlgebra
using ColorTypes, FixedPointNumbers

abstract type AbstractRgbdData end

struct RGBDFrame
    id::Int64
    image::Array{RGB{Normed{UInt8,8}},2}
    depth::Array{UInt16,2}
    pose::Matrix{Float32}
end

function RGBDFrame(rgb::Array{RGB{Normed{UInt8,8}},2}, depth::Array{UInt16,2})
    # depthf = convert(Array{Float32}, depth)
    RGBDFrame(1, copy(rgb), depth, Matrix{Float32}(LinearAlgebra.I,4,4))
end

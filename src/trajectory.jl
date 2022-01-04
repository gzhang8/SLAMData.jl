using FileIO, DelimitedFiles
using LinearAlgebra
using Rotations#, StaticArrays
# import Makie
# import GeometryTypes

mutable struct Trajectory
    n::Int64
    fixed_id::Int64
    ids::Vector{Int64}
    poses::Dict{Int64, Matrix{Float64}}
    is_pose_Twc::Bool # whether poses are in Twc format
end

function Base.getindex(X::Trajectory, i::Int)
    return X.poses[X.ids[i]]
end

function getTno2Idx(traj::Trajectory, k::Int64)
    @assert k>1
    idn =traj.ids[k]
    ido = traj.ids[k-1]

    To = traj.poses[ido]
    Tn = traj.poses[idn]
    Tno = Tn*inv(To)
    return Tno
end

# function load_trajectory(traj_filename::String)
#     traj_data = readdlm(traj_filename)
#     n = size(traj_data, 1)
#     ids = Vector{Int64}(undef, n)
#     poses = Dict{Int64, Matrix{Float64}}()
#     fixed_id = 0
#     for i=1:n

#         ids[i] = convert(Int64, traj_data[i, 1])
#         if convert(Int64, traj_data[i, 2]) == 1
#             fixed_id = ids[i]
#         end
#         # load
#         T = Matrix{Float64}(I, 4, 4)
#         T[1:3, 4] = traj_data[i, 3:5]
#         x, y, z, w = traj_data[i, 6:9]
#         T[1:3, 1:3] = Array(Quat(w,x,y,z))
#         poses[ids[i]] = T
#     end
#     return Trajectory(n, fixed_id, ids, poses)
# end


function load_covs(cov_fname::String)
    cov_data = readdlm(cov_fname)
    n = size(cov_data, 1)
    covs = Dict{Int64, Matrix{Float64}}()
    for i=1:n
        id = Int64(cov_data[i, 1])
        covs[id] = copy(reshape(cov_data[i, 2:37], 6, 6))
    end
    return covs
end


"""
Redwood dataset use Twc to represent camera pose
"""
function load_redwood_traj(fname::String)
    ids = Vector{Int64}(undef, 0)
    poses = Dict{Int64, Matrix{Float64}}()
    open(fname) do file
        # do stuff with the open file
        while !eof(file)
            aline = readline(file)
            split_strs = split(aline, (' ', '\t'))
            # @show split_strs
            id = parse(Int64, split_strs[1])
            num2 = parse(Int64, split_strs[2])
            num3 = parse(Int64, split_strs[3])

            mat4 = Matrix{Float64}(undef, 4, 4)
            for i=1:4
                line_mat = readline(file)
                line_strs_raw = split(line_mat, (' ', '\t'))
                line_strs = [s for s = line_strs_raw if length(s)>0]
                for j=1:4
                    mat4[i, j] = parse(Float64, line_strs[j])
                end
            end

            # @show line1, line2, line3, line4
            # matchall(nmbr, line[linenum])[index]
            push!(ids, id)
            poses[id] = mat4
            # @show id, num2, num3
            # @show mat4
        end
    end
    sort!(ids)
    is_pose_Twc = true
    return Trajectory(length(ids), ids[1], ids, poses, is_pose_Twc)
end

# function save_redwood_poses(fname::String, poses::Dict{Tuple{Int64, Int64}, Matrix{Float64}}; num3=0)
#     open(fname, "w") do file
#         for ((num1, num2), T) = poses
#             if num1 > num2
#                 continue
#             end
#             write(file, "$num1\t$num2\t$num3\n")
#             DelimitedFiles.writedlm(file, round.(T, digits=7), " ")
#         end
#     end
# end


"""
pose should translate id1 to id2
id1 should be smaller than id2
Redwood has T12
"""
function save_redwood_poses(
    fname::String,
    poses::Dict{Tuple{Int64, Int64}, Matrix{Float64}};
    num3=0,
    keep_orginal::Bool=false)
    open(fname, "w") do file
        for ((num1, num2), T) = poses
            if num1 > num2
                continue
            end
            T_out = if keep_orginal
                T
            else
                LinearAlgebra.inv(T)
            end


            write(file, "$num1\t$num2\t$num3\n")
            DelimitedFiles.writedlm(file, round.(T_out, digits=7), " ")
        end
    end

end

"""
pose should translate id1 to id2
id1 should be smaller than id2
Redwood has T12
"""
function save_redwood_poses(
    fname::String,
    poses::Dict{Tuple{Int64, Int64, Int64}, Matrix{Float64}};
    keep_orginal::Bool=false)
    open(fname, "w") do file
        for ((num1, num2, num3), T) = poses
            if (num1 > num2) && !keep_orginal
                continue
            end
            T_out = if keep_orginal
                T
            else
                LinearAlgebra.inv(T)
            end


            write(file, "$num1\t$num2\t$num3\n")
            DelimitedFiles.writedlm(file, round.(T_out, digits=7), " ")
        end
    end

end

"""
pose should translate id1 to id2, unless use keep_orginal
This can also be used for writing information
"""
function save_redwood_file(
    fname::String,
    poses::Dict{Tuple{Int64, Int64}, Matrix{Float64}};
    num3=0,
    keep_orginal::Bool=false)
    open(fname, "w") do file
        for ((num1, num2), T) = poses
            if keep_orginal
                write(file, "$num1\t$num2\t$num3\n")
                DelimitedFiles.writedlm(file, round.(T, digits=7), " ")
            elseif (num1 > num2)
                write(file, "$num2\t$num1\t$num3\n")
                DelimitedFiles.writedlm(file, round.(T, digits=7), " ")
            else
                write(file, "$num1\t$num2\t$num3\n")
                DelimitedFiles.writedlm(file, round.(LinearAlgebra.inv(T), digits=7), " ")
            end
        end
    end
end

"""
pose should translate id1 to id2
default is_Twc should keeo T as original
"""
function save_redwood_traj(
    fname::String,
    poses::Dict{Int64, Matrix{NT}};
    is_Twc::Bool=true
) where NT<:Number
    open(fname, "w") do file
        for num1 = sort(collect(keys(poses)))
            T = poses[num1]
            write(file, "$num1\t$num1\t$(num1+1)\n")
            if is_Twc
                DelimitedFiles.writedlm(file, round.(T, digits=7), " ")
            else
                DelimitedFiles.writedlm(file, round.(inv(T), digits=7), " ")
            end
        end
    end
end

# """
# vector of (src_id, dst_id, T)
# pose should translate id1 to id2
# """
# function save_redwood_file(fname::String, poses::Vector{Tuple{Int64, Int64, Matrix{Float64}}}; num3=0)
#     open(fname, "w") do file
#         for (num1, num2, T) = poses
#             if num1 > num2
#                 write(file, "$num2\t$num1\t$num3\n")
#                 DelimitedFiles.writedlm(file, round.(T, digits=7), " ")
#             else
#                 write(file, "$num1\t$num2\t$num3\n")
#                 DelimitedFiles.writedlm(file, round.(LinearAlgebra.inv(T), digits=7), " ")
#             end
#
#         end
#     end
# end

"""
note that the file has dst_id, src_id, num and T_dst_src
but we read it to be [src_id, dst_id, Tds]
Unless keep_orginal=true, then data is read in the same way that it is stored in
the file.

When keep_orginal=true, the dict is [dst_id, src_id, num and Tds]
"""
function load_redwood_loop(fname::String; keep_orginal::Bool=false)
    # ids = Vector{Int64}(undef, 0)
    poses = Dict{Tuple{Int64, Int64, Int64}, Matrix{Float64}}()
    open(fname) do file
        # do stuff with the open file
        while !eof(file)
            aline = readline(file)
            split_strs = split(aline, (' ', '\t'))
            # @show split_strs
            num1 = parse(Int64, split_strs[1])
            num2 = parse(Int64, split_strs[2])
            num3 = parse(Int64, split_strs[3])

            mat4 = Matrix{Float64}(undef, 4, 4)
            for i=1:4
                line_mat = readline(file)
                line_strs_raw = split(line_mat, (' ', '\t'))
                line_strs = [s for s = line_strs_raw if length(s)>0]
                for j=1:4
                    mat4[i, j] = parse(Float64, line_strs[j])
                end
            end

            # @show line1, line2, line3, line4
            # matchall(nmbr, line[linenum])[index]
            # push!(ids, id)
            if keep_orginal
                poses[(num1, num2, num3)] = mat4
            else
                if num1 > num2
                    poses[(num2, num1, num3)] = mat4
                else
                    poses[(num1, num2, num3)] = LinearAlgebra.inv(mat4)
                end
            end

            # @show id, num2, num3
            # @show mat4
        end
    end
    # sort!(ids)
    return poses
end

function load_redwood_loop_info(fname::String)::Dict{Tuple{Int64, Int64, Int64}, Matrix{Float64}}
    # ids = Vector{Int64}(undef, 0)
    infos = Dict{Tuple{Int64, Int64, Int64}, Matrix{Float64}}()
    open(fname) do file
        # do stuff with the open file
        while !eof(file)
            aline = readline(file)
            split_strs = split(aline, (' ', '\t'))
            # @show split_strs
            num1 = parse(Int64, split_strs[1])
            num2 = parse(Int64, split_strs[2])
            num3 = parse(Int64, split_strs[3])

            mat6 = Matrix{Float64}(undef, 6, 6)
            for i=1:6
                line_mat = readline(file)
                line_strs_raw = split(line_mat, (' ', '\t'))
                line_strs = [s for s = line_strs_raw if length(s)>0]
                for j=1:6
                    mat6[i, j] = parse(Float64, line_strs[j])
                end
            end

            # @show line1, line2, line3, line4
            # matchall(nmbr, line[linenum])[index]
            # push!(ids, id)
            infos[(num1, num2, num3)] = mat6


            # @show id, num2, num3
            # @show mat4
        end
    end
    # sort!(ids)
    return infos
end


"""
build pose from odom for redwood data
returns Twc
"""
function odom2pose(
	    odom::Dict{Tuple{Int64, Int64, Int64}, Matrix{Float64}};
		start_id::Int64=0,
		dst_first=true)
	@assert dst_first == true
	num = first(odom).first[3]
	poses = Dict{Int64, Matrix{Float64}}()
	poses[start_id] = Matrix{Float64}(LinearAlgebra.I, 4, 4)
	for id = start_id+1:num-1 # last id is num -1?
		increment = odom[(id-1, id, num)]
		last_pose = poses[id-1]
		poses[id] = last_pose * increment

	end
	return poses
end

"""
load BundleFusion Trajectory file format
use Twc to represent camera pose
"""
function load_boundlefusion_traj(fname::String)
    ids = Vector{Int64}(undef, 0)
    poses = Dict{Int64, Matrix{Float64}}()
    open(fname) do file
        # do stuff with the open file
        while !eof(file)
            # read line for index
            id_line = readline(file)
            id = parse(Int64, strip(id_line))

            # aline = readline(file)
            # split_strs = split(aline, (' ', '\t'))
            # # @show split_strs
            # id = parse(Int64, split_strs[1])
            # num2 = parse(Int64, split_strs[2])
            # num3 = parse(Int64, split_strs[3])

            # read matrix
            mat4 = Matrix{Float64}(undef, 4, 4)
            for i=1:4
                line_mat = readline(file)
                line_strs_raw = split(line_mat, (' ', '\t'))
                line_strs = [s for s = line_strs_raw if length(s)>0]
                for j=1:4
                    mat4[i, j] = parse(Float64, line_strs[j])
                end
            end
            # read empty line
            readline(file)

            # @show line1, line2, line3, line4
            # matchall(nmbr, line[linenum])[index]
            push!(ids, id)
            poses[id] = mat4
            # @show id, num2, num3
            # @show mat4
        end
    end
    sort!(ids)
    return Trajectory(length(ids), ids[1], ids, poses)
end

# function view_traj!(org_graph_scene, traj::Trajectory; linecolor=:red)
#     # traj = load_redwood_traj(pose_fname)
#
#     # 1.2 visualize all pcds
#     locations = Vector{Makie.Point3f0}(undef, traj.n)
#     # edges = Vector{Pair{Point3f0, Point3f0}}(undef, 0)
#     arrow_dirs = Vector{Makie.Point3f0}(undef, traj.n-1)
#
#     for id = traj.ids
#         locations[id + 1] = Makie.Point3f0(traj.poses[id][1:3, 4])
#         if id != 0
#             a_edge = Pair(locations[id], locations[id+1])
#             # push!(edges, a_edge)
#             arrow_dirs[id] = locations[id+1] - locations[id]
#         end
#     end
#     # org_graph_scene = Makie.Scene()
#     # scatter!(org_graph_scene, locations)
#     # linesegments!(org_graph_scene, edges, color = :red)
#     Makie.arrows!(org_graph_scene,
#             locations[1:traj.n-1],
#             arrow_dirs,
#             arrowcolor=:black,
#             arrowsize=0.1,
#             linecolor=linecolor)
# end
#
#
# function view_traj(traj::Trajectory; linecolor=:red)
#     org_graph_scene = Makie.Scene()
#     view_traj!(org_graph_scene, traj, linecolor=linecolor)
#     return org_graph_scene
# end

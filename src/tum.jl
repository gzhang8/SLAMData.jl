

using FileIO, DelimitedFiles
using Images
# using NPZ
using LinearAlgebra
import Rotations

"""
time stamps are in μs
"""
mutable struct TumData <:RgbdReader
    k::Int64
    frame_num::Int64
    folder::String
    ids::Vector{Int64}
    timestamps::Vector{Int64}
    id2rgb::Dict{Int64, String}
    id2depth::Dict{Int64, String}
    cam::CameraModel.RgbdCamParams
end



"""
at k = 0 means it has not started yet
"""
function TumData(folder::String; k::Int64=0, use_ros_cam::Bool=false)
    idx_file = joinpath(folder, "index.txt")
    idx_data = readdlm(idx_file)
    n = size(idx_data, 1)

    id2rgb = Dict{Int64, String}()
    id2depth = Dict{Int64, String}()

    ids = Vector{Int64}(undef, n)
    timestamps::Vector{Int64} = Vector{Int64}(undef, n)
    time_factor = 1000000
    for i=1:n
        id = idx_data[i, 1]
        ids[i] = id
        timestamps[i] = Int64(round(idx_data[i, 2] * time_factor))
        id2rgb[id] = joinpath(folder, idx_data[i, 3])
        id2depth[id] = joinpath(folder, idx_data[i, 4])
    end
    # ids start from 0. timestamp starts from 1
    cam = if use_ros_cam
        CameraModel.TumCamROS()
    elseif occursin("freiburg1", folder)

        fx = 517.306408

        # IR projector baseline times fx (aprox.)
        bf = 40.0

        CameraModel.RgbdCamParams(
            width=640,
            height=480,
            fx = fx,
            fy = 516.469215,
            cx = 318.643040,
            cy = 255.313989,
            depth_factor=5000.0,
            baseline=bf/fx,
            k1 = 0.262383,
            k2 = -0.953104,
            p1 = -0.005358,
            p2 = 0.002628,
            k3 = 1.163314
        )

    elseif occursin("freiburg2", folder)

        fx = 520.908620

        # IR projector baseline times fx (aprox.)
        bf = 40.0

        CameraModel.RgbdCamParams(
            width=640,
            height=480,
            fx = fx,
            fy = 521.007327,
            cx = 325.141442,
            cy = 249.701764,
            depth_factor=5208.0,
            baseline=bf/fx,
            k1 = 0.231222,
            k2 = -0.784899,
            p1 = -0.003257,
            p2 = -0.000105,
            k3 = 0.917205,
        )

    elseif occursin("freiburg3", folder)

        fx=535.4

        # IR projector baseline times fx (aprox.)
        bf = 40.0

        CameraModel.RgbdCamParams(
            width=640,
            height=480,
            fx = fx,
            fy=539.2,
            cx=320.1,
            cy=247.6,
            depth_factor=5000.0,
            baseline=bf/fx,
            k1=0.0,
            k2=0.0,
            p1=0.0,
            p2=0.0,
        )
    else
        @warn "no camera type detected"
    end

    return TumData(k, n, folder, ids, timestamps, id2rgb, id2depth, cam)
end




function read(tum_data::TumData)
    tum_data.k += 1
    return readk(tum_data, tum_data.k)
    # id = tum_data.ids[tum_data.k]
    # img = load(tum_data.id2rgb[id])
    # depth = load(tum_data.id2depth[id])
    # depth_view = channelview(depth)
    # depth_uint16 = copy(reinterpret(UInt16, depth_view[1,:,:]))
    #
    # return img, depth_uint16
end

function readk(tum_data::TumData, k::Int64)

    id = tum_data.ids[k]
    return readid(tum_data, id)
    # img = load(tum_data.id2rgb[id])
    # depth = load(tum_data.id2depth[id])
    # depth_view = channelview(depth)
    # depth_uint16 = copy(reinterpret(UInt16, depth_view[1,:,:]))
    # return img, depth_uint16
end

function readlabelid(tum_data::TumData, id::Int64)::Matrix{Int32}
    rgb_fname = tum_data.id2rgb[id]
    label_fname = "$(splitext(rgb_fname)[1]).npy"
    return npzread(label_fname)
end

function loaddepth(fname::String)
    depth = load(fname)
    depth_view = channelview(depth)
    depth_uint16 = copy(reinterpret(UInt16, depth_view[1,:,:]))
    return depth_uint16
end

function readid(tum_data::TumData, id::Int64)
    # id = tum_data.ids[k]
    img = FileIO.load(tum_data.id2rgb[id])
    depth = FileIO.load(tum_data.id2depth[id])
    depth_view = Images.channelview(depth)
    # depth_uint16 = copy(reinterpret(UInt16, depth_view[1,:,:]))
    depth_uint16 = if ndims(depth_view) ==  3
        copy(reinterpret(UInt16, depth_view[1,:,:]))
    else
        copy(reinterpret(UInt16, depth_view))
    end
    return img, depth_uint16
end

"""
readRGBD(reader::Sun3DReader; id::Int64)
read rgbd images with id as input
"""
readRGBD(tum_data::TumData; id::Int64) = readid(tum_data, id)

function fastforward!(tum_data::TumData, Δk::Int64)
    tum_data.k += Δk
end

function fastbackward!(tum_data::TumData, Δk::Int64)
    tum_data.k -= Δk
end

function id2time(tum_data::TumData, id::Int64)
    idx = get_id_index(tum_data, id)
    return tum_data.timestamps[idx]
end


"""
mutable struct Trajectory
    n::Int64
    fixed_id::Int64
    ids::Vector{Int64}
    poses::Dict{Int64, Matrix{Float64}}
end
tum data use float number as time, we apply a time factor to make it an integer
x_idx: the index for the x colum with 1 start index
"""
function load_tum_traj(f_path::String; time_factor::Int64=1000000, x_idx=2)
    # ids = Vector{Int64}(undef, 0)
    id2poses = Dict{Int64, Matrix{Float64}}()
    ids = Vector{Int64}(undef, 0)
    open(f_path) do file
        # do stuff with the open file
        while !eof(file)
            aline = readline(file)
            split_strs = split(aline, (' ', '\t'))
            if split_strs[1] == "#"
                # skip comment
                continue
            end
            # @show split_strs
            raw_time = parse(Float64, split_strs[1])
            id = Int64(round(raw_time * time_factor))#parse(Int64, split_strs[1])
            xyzqxqyqzqw = [parse(Float64, x) for x = split_strs[x_idx:x_idx+6]]
            qx, qy, qz, qw = xyzqxqyqzqw[4:7]
            # convert to transformation matrix
            R = Rotations.Quat(qw, qx, qy, qz)
            T = Matrix{Float64}(LinearAlgebra.I, 4, 4)
            T[1:3, 1:3] = R
            T[1:3, 4] = xyzqxqyqzqw[1:3]
            # display(T)


            # @show id, num2, num3
            # @show mat4
            # @show id, xyzqxqyqzqw
            # save res
            push!(ids, id)
            id2poses[id] = T
        end
    end
    n = length(ids)
    fixed_id = minimum(ids)
    is_pose_Twc = true
    traj = Trajectory(n, fixed_id, ids, id2poses, is_pose_Twc)
    return traj
end

"""
The format of each line is 'timestamp tx ty tz qx qy qz qw'
https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
timestamp is loaded as us in Int64 type
"""
function save_tum_traj(
    f_path::String,
    traj_dict::Dict{Int64, Matrix{FT}};
    id2time::Dict{Int64, Int64}=Dict{Int64, Int64}(),
    time_factor::Int64=1000000,
    do_inv::Bool=false
) where FT<:Number

    traj_keys = sort!(collect(keys(traj_dict)))
    open(f_path, write=true) do file
        for traj_key = traj_keys
            T = traj_dict[traj_key]

            time_stamp = if length(id2time) > 0
                id2time[traj_key]
            else
                traj_key
            end
            T = if do_inv
                inv(T)
            else
                T
            end

            tx, ty, tz, = T[1:3, 4]
            q = Rotations.Quat(T[1:3, 1:3])
            qx = q.x
            qy = q.y
            qz = q.z
            qw = q.w
            out_string = if time_factor == 1
                out_string = Printf.@sprintf(
                    "%d %f %f %f %f %f %f %f",
                    time_stamp,
                    tx,
                    ty,
                    tz,
                    qx,
                    qy,
                    qz,
                    qw
                )
            else
                out_string = Printf.@sprintf(
                    "%.6f %f %f %f %f %f %f %f",
                    time_stamp/time_factor,
                    tx,
                    ty,
                    tz,
                    qx,
                    qy,
                    qz,
                    qw
                )
                # out_string = "$(time_stamp/time_factor) $tx $ty $tz $qx $qy $qz $qw"
            end
            println(file, out_string)
        end

    end

end

"""
bf index is in a format similar to TUM index
"""
function load_boundlefusion_index(
    idx_fpath::String;
    seq_folder::String="",
    only_keep_filename::Bool=true
)
    # idx_file = joinpath(folder, "index.txt")
    idx_data = readdlm(idx_fpath)
    n = size(idx_data, 1)

    id2rgb = Dict{Int64, String}()
    id2depth = Dict{Int64, String}()

    ids = Vector{Int64}(undef, n)
    # timestamps::Vector{Int64} = Vector{Int64}(undef, n)
    # time_factor = 1000000
    for i=1:n
        id = idx_data[i, 1]
        ids[i] = id
        # timestamps[i] = Int64(round(idx_data[i, 2] * time_factor))
        rgb_fname, depth_fname = if only_keep_filename
            splitpath(String(idx_data[i, 2]))[end], splitpath(String(idx_data[i, 3]))[end]
        else
            idx_data[i, 2], idx_data[i, 3]
        end
        id2rgb[id] = joinpath(seq_folder, rgb_fname)
        id2depth[id] = joinpath(seq_folder, depth_fname)
    end
    return ids, id2rgb, id2depth
end


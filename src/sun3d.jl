

mutable struct Sun3DReader <: RgbdReader
    frame_num::Int64
    f_id::Int64
    ids::Vector{Int64} # actually using image ids
    timestamps::Vector{Int64}
    data_folder::String
    depth_folder::String
    image_folder::String
    image_id2fname::Dict{Int64, String}
    image_id2time::Dict{Int64, Int64}
    depth_id2fname::Dict{Int64, String}
    depth_id2time::Dict{Int64, Int64}
    img_id_to_depth_id::Dict{Int64, Int64}
    cam::CameraModel.RgbdCamParams
    depth_factor::Float64
end


function Sun3DReader(data_folder::String)
    depth_folder = joinpath(data_folder, "depth")
    image_folder = joinpath(data_folder, "image")
    # get all depth file names
    depth_fnames = readdir(depth_folder)
    filter!(x->splitext(x)[2]==".png", depth_fnames)
    image_fnames = readdir(image_folder)
    filter!(x->splitext(x)[2]==".jpg", image_fnames)

    image_id2fname = Dict{Int64, String}()
    image_id2time = Dict{Int64, Int64}()

    for fname in image_fnames
        fname_wo_ext = splitext(fname)[1]
        id, time = parse.(Int64, split(fname_wo_ext, "-"))
        # id_time = parse(Int64,replace(splitext(fname)[1], "-"=>""))
        image_id2fname[id] = fname
        image_id2time[id] = time
    end

    depth_id2fname = Dict{Int64, String}()
    depth_id2time = Dict{Int64, Int64}()
    depth_ids = Vector{Int64}(undef, length(depth_fnames))
    depth_times = Vector{Int64}(undef, length(depth_fnames))

    for (idx, fname) in enumerate(depth_fnames)
        fname_wo_ext = splitext(fname)[1]
        id, time = parse.(Int64, split(fname_wo_ext, "-"))
        # id_time = parse(Int64,replace(splitext(fname)[1], "-"=>""))
        depth_id2fname[id] = fname
        depth_id2time[id] = time
        depth_ids[idx] = id
        depth_times[idx] = time
    end

    # synchronize: find a depth for each image
    img_id_to_depth_id = Dict{Int64, Int64}()
    for (img_id, img_time) = image_id2time
        time_diff = abs.(img_time .- depth_times)
        matched_depth_idx = argmin(time_diff)
        # @assert time_diff[matched_depth_idx] < 3000 # 3000 is 3ms
        if time_diff[matched_depth_idx] > 3000
            @warn "sun3d data image depth time diff is $(time_diff[matched_depth_idx]/1000) ms"
        end
        img_id_to_depth_id[img_id] = depth_ids[matched_depth_idx]
    end

    # load camera intrinsic
    intrinsic_fname = joinpath(data_folder, "intrinsics.txt")
    k = DelimitedFiles.readdlm(intrinsic_fname, ' ', Float64)
    fx = k[1,1]
    fy = k[2,2]
    cx = k[1,3]
    cy = k[2,3]
    w, h = 640, 480

    depth_factor=1000.0

    cam = CameraModel.RgbdCamParams(
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        width=w,
        height=h,
        depth_factor=depth_factor,
        baseline=50.0/fx
    )


    ids = sort!(collect(keys(img_id_to_depth_id)))
    f_id::Int64 = ids[1]

    # for sun2d, timestamp = id
    timestamps = copy(ids)


    res = Sun3DReader(
        length(ids),
        f_id,
        ids, # actually using image ids
        timestamps,
        data_folder,
        depth_folder,
        image_folder,
        image_id2fname,
        image_id2time,
        depth_id2fname,
        depth_id2time,
        img_id_to_depth_id,
        cam,
        depth_factor
    )

    return res
end



function Sun3DReader(
    data_folder::String, index_fpath::String; one_indexed::Bool=true
)
    depth_folder = joinpath(data_folder, "depth")
    image_folder = joinpath(data_folder, "image")
    # # get all depth file names
    # depth_fnames = readdir(depth_folder)
    # filter!(x->splitext(x)[2]==".png", depth_fnames)
    # image_fnames = readdir(image_folder)
    # filter!(x->splitext(x)[2]==".jpg", image_fnames)
    ids, id2rgb, id2depth = load_boundlefusion_index(
        index_fpath,
        seq_folder=""
    )

    image_id2fname = Dict{Int64, String}()
    image_id2time = Dict{Int64, Int64}()

    # for fname in image_fnames
    for (img_id, img_path) in id2rgb
        # fname_wo_ext = splitext(splitpath(img_path)[end])[1]
        fname_wo_ext = splitext(img_path)[1]
        id, time = parse.(Int64, split(fname_wo_ext, "-"))
        # id_time = parse(Int64,replace(splitext(fname)[1], "-"=>""))
        image_id2time[img_id+one_indexed] = time
        image_id2fname[img_id+one_indexed] = img_path
    end

    depth_id2fname = Dict{Int64, String}()
    depth_id2time = Dict{Int64, Int64}()
    # TODO what is the use of this ids
    depth_ids = Vector{Int64}(undef, length(id2depth))
    depth_times = Vector{Int64}(undef, length(id2depth))

    # for (idx, fname) in enumerate(depth_fnames)
    for (depth_id, depth_fpath) in id2depth
        # fname_wo_ext = splitext(splitpath(depth_fpath)[end])[1] #splitext(fname)[1]
        fname_wo_ext = splitext(depth_fpath)[1]
        id, time = parse.(Int64, split(fname_wo_ext, "-"))
        # id_time = parse(Int64,replace(splitext(fname)[1], "-"=>""))
        depth_id2fname[depth_id+one_indexed] = depth_fpath
        depth_id2time[depth_id+one_indexed] = time
        # depth_ids[depth_id+1] = id
        # depth_times[depth_id] = time
    end

    # synchronize: find a depth for each image
    ids .+= one_indexed
    img_id_to_depth_id = Dict{Int64, Int64}()
    for fid in ids
        img_id_to_depth_id[fid] = fid
    end
    # for (img_id, img_time) = image_id2time
    #     time_diff = abs.(img_time .- depth_times)
    #     matched_depth_idx = argmin(time_diff)
    #     # @assert time_diff[matched_depth_idx] < 3000 # 3000 is 3ms
    #     if time_diff[matched_depth_idx] > 3000
    #         @warn "sun3d data image depth time diff is $(time_diff[matched_depth_idx]/1000) ms"
    #     end
    #     img_id_to_depth_id[img_id] = depth_ids[matched_depth_idx]
    # end

    # load camera intrinsic
    intrinsic_fname = joinpath(data_folder, "intrinsics.txt")
    k = DelimitedFiles.readdlm(intrinsic_fname, ' ', Float64)
    fx = k[1,1]
    fy = k[2,2]
    cx = k[1,3]
    cy = k[2,3]
    w, h = 640, 480

    depth_factor=1000.0

    cam = CameraModel.RgbdCamParams(
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        width=w,
        height=h,
        depth_factor=depth_factor,
        baseline=50.0/fx
    )


    # ids = sort!(collect(keys(img_id_to_depth_id)))
    f_id::Int64 = ids[1]

    # for sun3d, timestamp = id
    timestamps = copy(ids)


    res = SLAMData.Sun3DReader(
        length(ids),
        f_id,
        ids, # actually using image ids
        timestamps,
        data_folder,
        depth_folder,
        image_folder,
        image_id2fname,
        image_id2time,
        depth_id2fname,
        depth_id2time,
        img_id_to_depth_id,
        cam,
        depth_factor
    )

    return res
end



function readid(reader::Sun3DReader, id::Int64)
    # id = tum_data.ids[k]
    img = FileIO.load(joinpath(reader.image_folder, reader.image_id2fname[id]))

    depth_id = reader.img_id_to_depth_id[id]
    depth = FileIO.load(joinpath(reader.depth_folder, reader.depth_id2fname[depth_id]))
    depth_view = Images.channelview(depth)
    depth_uint16 = if ndims(depth_view) ==  3
        copy(reinterpret(UInt16, depth_view[1,:,:]))
    else
        copy(reinterpret(UInt16, depth_view))
    end
    # depth_uint16 = copy(reinterpret(UInt16, depth_view))
    depth_uint16 = (depth_uint16 .<< 13) .| (depth_uint16 .>>> 3)
    return img, depth_uint16
end


# function depth = depthRead(filename)
#     depth = imread(filename);
#     depth = bitor(bitshift(depth,-3), bitshift(depth,16-3));
#     depth = single(depth)/1000;
# end
"""
readRGBD(reader::Sun3DReader; id::Int64)
read rgbd images with id as input
"""
readRGBD(reader::Sun3DReader; id::Int64) = readid(reader, id)

"""
sun3d traj use Twc
"""
function load_sun3d_extrinics(traj_fpath::String; start_id::Int64=0)
    sun3d_raw_data = DelimitedFiles.readdlm(traj_fpath)

    Twcs = Dict{Int64,Matrix{Float64}}()

    @assert size(sun3d_raw_data, 1) % 3 == 0

    n = Int64(size(sun3d_raw_data, 1) / 3)

    for i1 = 1:n
        row_offset = (i1-1)*3
        T = Matrix{Float64}(LinearAlgebra.I, 4, 4)
        T[1:3, 1:4] = sun3d_raw_data[row_offset+1:row_offset+3, 1:4]
        Twcs[i1-1+start_id] = T
    end
    return Twcs
end

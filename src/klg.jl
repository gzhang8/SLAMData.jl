# using Atom
# include("types.jl")
using FileIO
using Images
using ImageMagick # for load rgb
# using Images
"""
ids start from 0
"""
mutable struct KlgReader <: RgbdReader
    frame_num::Int64
    ids::Vector{Int64}
    timestamps::Vector{Int64}
    dw::Int64
    dh::Int64
    num_pixels::Int64
    f_idx::Int64
    klg_file_name::String
    klg_file::IOStream
    # store the position of each frame
    frame_position::Dict{Int64, Int64}

    depth_buffer::Array{UInt8}
    rgb_buffer::Array{UInt8}

    depth_uncompressed::Array{UInt16}
    #codec::ZlibDecompressor
    cam::CameraModel.RgbdCamParams
end

function destruct_klg!(klg_data::KlgReader)
    Base.close(klg_data.klg_file)
end

function KlgReader(
    klg_file_name::String;
    cam::CamT
) where CamT <: CameraModel.AbstractCamParams
    dw = cam.width
    dh = cam.height
    num_pixels = dw * dh
    klg_file::IOStream = open(klg_file_name,"r")
    FrameNum::Int64 = Base.read(klg_file, Int32)
    frame_position = Dict{Int64, Int64}()
    # FrameNum = Base.read(klg_file, Int32)
    # frame_position[0] = 4

    depth_buffer::Array{UInt8} = Array{UInt8}(undef, num_pixels * 2);
    rgb_buffer::Array{UInt8} = Array{UInt8}(undef, num_pixels * 3);

    depth_uncompressed =  Array{UInt8}(undef, num_pixels);

    #codec = ZlibDecompressor()
    #TranscodingStreams.initialize(codec)
    # finalizer(k, k->print("finalized!"))

    ids = collect(0:FrameNum-1)
    timestamps = ids .+ 1

    klg_data = KlgReader(
        FrameNum, ids, timestamps, dw, dh, num_pixels, -1,
        klg_file_name, klg_file,
        frame_position, depth_buffer, rgb_buffer, depth_uncompressed,
        cam)
    finalizer(destruct_klg!, klg_data)
    return klg_data
end

"""
`isrgb`: true for rgb order, false for bgr order
"""
function readRGBD_base(klg_data::KlgReader; isrgb::Bool=true)
    time_stamp = Base.read(klg_data.klg_file, Int64)
    depth_size = Base.read(klg_data.klg_file, Int32)
    rgb_size = Base.read(klg_data.klg_file, Int32)

#     println(time_stamp)
    # @show depth_size
    # @show rgb_size

    readbytes!(klg_data.klg_file, klg_data.depth_buffer, depth_size)
    readbytes!(klg_data.klg_file, klg_data.rgb_buffer, rgb_size)

    # # C++ function decompress
    # decompress(klg_data.depth_buffer[1:depth_size], klg_data.depth_uncompressed)

    if klg_data.num_pixels * 2 != depth_size
        # use ccall to perform decompression
        decompLength::UInt64 = size(klg_data.depth_uncompressed, 1) * 2;
        depth_buffer_ptr = Base.unsafe_convert(Ptr{Cuchar}, klg_data.depth_buffer)
        depth_uncompressed_ptr = Base.unsafe_convert(Ptr{Cuchar}, klg_data.depth_uncompressed)
        # //typedef unsigned char Byte;
        # ZEXTERN int ZEXPORT uncompress OF((Bytef *dest,   uLongf *destLen,
        #                                    const Bytef *source, uLong sourceLen));
        res = ccall((:uncompress, :libz),
                    Cint,
                    (Ptr{Cuchar}, Ptr{Culonglong}, Ptr{Cuchar}, Culonglong),
                    depth_uncompressed_ptr, Ref(decompLength), depth_buffer_ptr, depth_size)

        depth = permutedims(reshape(klg_data.depth_uncompressed, (klg_data.dw, klg_data.dh)), [2, 1])
    else
        depth = permutedims(reshape(reinterpret(UInt16, klg_data.depth_buffer), (klg_data.dw, klg_data.dh)), [2, 1])
    end

    if klg_data.num_pixels * 3 != rgb_size
        img = readblob(klg_data.rgb_buffer[1:rgb_size])
    else
        rgb_mat = permutedims(reshape(klg_data.rgb_buffer[1:rgb_size],
                                     (3, klg_data.dw, klg_data.dh)), [1, 3, 2])
        if isrgb
            img = RGB{Normed{UInt8,8}}.(rgb_mat[1, :, :]./255, rgb_mat[2, :, :]./255, rgb_mat[3, :, :]./255)
        else
            img = RGB{Normed{UInt8,8}}.(rgb_mat[3, :, :]./255, rgb_mat[2, :, :]./255, rgb_mat[1, :, :]./255)
        end
            # img = copy(dropdims(reinterpret(RGB{Normed{UInt8,8}}, rgb_mat), dims=1))
    end
    # @show size(img)
    return  img, depth;#,depth_uncompressed;#
end

# end

# klg_reader = io.KlgReader("/home/gzhang8/project/cavedrone/data/belize/2017-06-07.01.klg", 640, 480, 640*480);
# rgb, depth = io.readOneFrame(klg_reader);
# end

function readRGBDFrame(klg_reader::KlgReader; id::Int64, isrgb::Bool=true)
    rgb, depth = readRGBD(klg_reader, id=id, isrgb=isrgb);
    return RGBDFrame(rgb, depth)

end

"""
read data for frame id
id is the index for klg_data, starting from 0
    `isrgb::Bool` true for rgb order, false for bgr order
"""
function readRGBD(klg_data::KlgReader; id::Int64, isrgb::Bool=true)
    @assert(klg_data.frame_num > id)
    old_position = mark(klg_data.klg_file)
    # FrameNum = Base.read(klg_file, Int32)
    current_fid::Int64 = 0
    current_pos::Int64 = 4
    if length(klg_data.frame_position) > 0
        max_fid = maximum(keys(klg_data.frame_position))
        if max_fid >= id
            current_fid = id
            current_pos = klg_data.frame_position[id]
        else
            current_fid = max_fid
            current_pos = klg_data.frame_position[max_fid]
        end

    end
    seek(klg_data.klg_file, current_pos)

    # fast forward to the frame we want to read
    while current_fid < id
        # time_stamp = Base.read(klg_data.klg_file, Int64)
        skip(klg_data.klg_file, 8)
        klg_data.frame_position[current_fid] = current_pos
        # 4 bytes for depth_size
        depth_size = Base.read(klg_data.klg_file, Int32)
        # 4 bytes for rgb_size
        rgb_size = Base.read(klg_data.klg_file, Int32)
        skip(klg_data.klg_file, depth_size + rgb_size)
        current_fid += 1
        current_pos += (16 + depth_size + rgb_size)

    end
    # add the last one in the loop
    klg_data.frame_position[current_fid] = current_pos
    img, depth = readRGBD_base(klg_data, isrgb=isrgb)
    # reset file location
    reset(klg_data.klg_file)
    unmark(klg_data.klg_file)
    return img, depth
end

function view_depth(depth)
    return depth_view = Gray.(depth/4000);
end

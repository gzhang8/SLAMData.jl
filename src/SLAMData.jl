module SLAMData
using ImageMagick
import CameraModel
import Printf
# import Statistics

export TumData, TumCamROS, read, fastforward!, fastbackward!,
       Trajectory, load_trajectory, load_covs, 

       # draw_trajectory!, draw_trajectory,
       getTno2Idx, readid, readk,
      
       # klg
       KlgReader, readRGBD, readRGBDFrame,

       # traj
       load_redwood_traj, load_redwood_loop, save_redwood_file,
       load_tum_traj, save_tum_traj, load_boundlefusion_traj,
       load_redwood_loop_info, odom2pose,
       # load img to array
       loadpng,

       # sun3d reader
       Sun3DReader


include("types.jl")

include("rgbd_reader.jl")
include("stereo_reader.jl")
include("tum.jl")
include("sun3d.jl")
include("trajectory.jl")
include("klg.jl")


end # module

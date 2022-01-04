
function loadpng(fname::String)
    png_img = load(fname)
    img_channel_view = channelview(png_img)
    img_re = reinterpret(UInt16, img_channel_view[1,:,:])
    return copy(img_re)
end

abstract type RgbdReader end


function get_timestamp_index(reader::T, timestamp::Int64) where T<:RgbdReader
    index = findfirst(x->x==timestamp, reader.timestamps)
    if isa(index, Nothing)
        return -1
    else
        return index
    end
end

function get_id_index(reader::T, id::Int64) where T<:RgbdReader
    index = findfirst(x->x==id, reader.ids)
    if isa(index, Nothing)
        return -1
    else
        return index
    end
end

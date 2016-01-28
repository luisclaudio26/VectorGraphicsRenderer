local driver = require"driver"
local image = require"image"
local chronos = require"chronos"

local bezier, quadratic = require("lua.bezier"), require("lua.quadratic")
local unpack, pack = table.unpack, table.pack
local max, min = math.max, math.min 
local floor, ceil = math.floor, math.ceil
local abs = math.abs

local _M = driver.new()

local BGColor = require("lua.color").rgb(1,1,1,1)
local epsilon, max_iteration = 0.000000000001, 50

-----------------------------------------------------------------------------------------
-------------------------------- AUXILIAR FUNCTIONS -------------------------------------
-----------------------------------------------------------------------------------------
local function sign(v)
    if v < 0 then return -1
    elseif v > 0 then return 1
    else return 0 end
end

local function transform_point(x, y, xf)
    local _x, _y, w = xf : apply(x, y, 1)
    return _x / w, _y / w
end

local function truncate_parameter(t)
    if t < 0 or t == math.huge then t = 0 --????
    elseif t > 1 or t == -math.huge then t = 1
    end
    return t
end

local function root_bisection(t0, t1, func)
    local tm = (t0 + t1)*0.5

    -- Halting criterium
    local delta = t1 - t0
    if delta < epsilon then return tm end

    -- Recursively subdivide
    local y = func(tm)
    local sy = sign(y)

    if sy == sign( func(t0) ) then
        return root_bisection(tm, t1, func)
    elseif sy == sign( func(t1) ) then 
        return root_bisection(t0, tm, func)
    elseif y == 0 then 
        return tm
    end
end

local function compute_cubic_maxima(x0, x1, x2, x3)
    local c = 3*(x1 - x0)
    local b = 6*(x0 - 2*x1 + x2)
    local a = 3*(-x0 + 3*x1 - 3*x2 + x3)

    local n, t1, s1, t2, s2 = quadratic.quadratic(a,b,c)
    local out1, out2 = 0, 0

    if n > 0 then out1 = t1/s1 end
    if n > 1 then out2 = t2/s2 end

    out1, out2 = truncate_parameter( out1 ), truncate_parameter( out2 )
    return out1, out2
end

local function compute_rational_maxima(x0, x1, x2, w)
    local a = 2*(-1 + w)*(x0 - x2)
    local b = 2*(x0 - 2*w*x0 + 2*x1 - x2)
    local c = 2*(w*x0 - x1)

    n, r1, s1, r2, s2 = quadratic.quadratic(a, b, c)

    local out1, out2 = 0, 0
    if n > 0 then out1 = r1/s1 end
    if n > 1 then out2 = r2/s2 end

    out1, out2 = truncate_parameter(out1), truncate_parameter(out2)
    return out1, out2
end

local function alpha_composite(c1, c2, a)
    return c1 + (1-a)*c2
end

local function search_in_ramp(ramp, value)
    -- Just a linear search. Other versions may sample
    -- the ramp and just do a look-up table for this
    for i = 1, #ramp-2, 2 do
        if ramp[i] <= value and value < ramp[i+2] then
            return i
        end
    end

    return (#ramp-3)
end

local function interpolate_colors(color1, color2, t)
    local out = {}
    for i = 1, 4 do
        out[i] = (1-t)*color1[i] + t*color2[i]
    end
    return out
end

-----------------------------------------------------------------------------------------
-------------------------------- PATH PREPROCESSING -------------------------------------
-----------------------------------------------------------------------------------------
local prepare_table = {}
prepare_table.instructions = {}
prepare_table.push_functions = {}

--------------------------------
------ Push primitives ---------
--------------------------------
function prepare_table.push_functions.linear_segment(x0, y0, x1, y1, holder, virtual)
    
    -- Virtual flag: a Virtual linear segment must be considered when filling
    -- the shape, but not if we're stroking. This is just a "fake" linear segment
    -- we use to close and open path when we need to fill it.
    
    local n = #holder + 1
    holder[n] = {}

    holder[n].type = "linear_segment"
    holder[n].virtual = virtual
    holder[n].x0, holder[n].y0 = x0, y0
    holder[n].x1, holder[n].y1 = x1, y1

    -- Precompute implicit equation and bounding box
    local a = y1 - y0
    local b = x0 - x1
    local c = -a * x0 - b * y0
    local dysign = sign(a)
    a, b, c = a*dysign, b*dysign, c*dysign

    local xmin, xmax = min(x0, x1), max(x0, x1)
    local ymin, ymax = min(y0, y1), max(y0, y1)

    holder[n].a, holder[n].b, holder[n].c = a, b, c
    holder[n].dysign = dysign
    holder[n].xmin, holder[n].xmax = xmin, xmax
    holder[n].ymin, holder[n].ymax = ymin, ymax
end

function prepare_table.push_functions.degenerate_segment(x0, y0, dx0, dy0, dx1, dy1, holder)
    local n = #holder + 1
    holder[n] = {}

    holder[n].type = "degenerate_segment"
    holder[n].x, holder[n].y = x0, y0
    holder[n].dx0, holder[n].dy0 = dx0, dy0
    holder[n].dx1, holder[n].dy1 = dx1, dy1
end

function prepare_table.push_functions.quadratic_segment(u0, v0, u1, v1, u2, v2, holder)
    local n = #holder + 1 
    holder[n] = {}
    holder[n].type = "quadratic_segment"
    holder[n].x0, holder[n].y0 = u0, v0
    holder[n].x1, holder[n].y1 = u1, v1
    holder[n].x2, holder[n].y2 = u2, v2
    holder[n].dysign = sign(v2 - v0)

    local maxy, miny = max(v0, v2), min(v0, v2)
    local maxx, minx = max(u0, u2), min(u0, u2)

    holder[n].xmax, holder[n].xmin = maxx, minx
    holder[n].ymax, holder[n].ymin = maxy, miny
end

function prepare_table.push_functions.cubic_segment(u0, v0, u1, v1, u2, v2, u3, v3, holder)
    local n = #holder + 1
    holder[n] = {}
    holder[n].type = "cubic_segment"
    holder[n].x0, holder[n].y0 = u0, v0
    holder[n].x1, holder[n].y1 = u1, v1
    holder[n].x2, holder[n].y2 = u2, v2
    holder[n].x3, holder[n].y3 = u3, v3
    holder[n].dysign = sign(v3 - v0)

    local xmax, xmin = max(u0, u3), min(u0, u3)
    local ymax, ymin = max(v0, v3), min(v0, v3)

    holder[n].xmax, holder[n].xmin = xmax, xmin
    holder[n].ymax, holder[n].ymin = ymax, ymin
end

function prepare_table.push_functions.rational_quadratic_segment(u0, v0, u1, v1, u2, v2, w, holder)
    local n = #holder + 1
    holder[n] = {}
    holder[n].type = "rational_segment"
    holder[n].x0, holder[n].y0 = u0, v0
    holder[n].x1, holder[n].y1 = u1, v1
    holder[n].x2, holder[n].y2 = u2, v2
    holder[n].w = w

    holder[n].xmax, holder[n].xmin = max(u0, u2), min(u0, u2)
    holder[n].ymax, holder[n].ymin = max(v0, v2), min(v0, v2)
    holder[n].dysign = sign(v2 - v0)
end

--------------------------------
--------- Instructions ---------
--------------------------------
function prepare_table.instructions.begin_closed_contour(shape, offset, iadd)
    local xf, data = shape.xf, shape.data
    data[offset+1], data[offset+2] = transform_point(data[offset+1], data[offset+2], xf)
end

function prepare_table.instructions.end_closed_contour(shape, offset, iadd)
    -- Fetch first vertice and then add closing edge
    local data = shape.data
    local x, y = data[offset], data[offset+1]

    local instr_offset = data[offset + 2]
    local closing_instruction = shape.offsets[iadd - instr_offset]
    local first_x, first_y = data[closing_instruction+1], data[closing_instruction+2]

    prepare_table.push_functions.linear_segment(x, y, first_x, first_y, shape.primitives, false)
end

function prepare_table.instructions.begin_open_contour(shape, offset, iadd)
    prepare_table.instructions.begin_closed_contour(shape, offset, iadd)
end

function prepare_table.instructions.end_open_contour(shape, offset, iadd)
    -- Do the same as end_closed_contour, but mark last edge as Virtual
    prepare_table.instructions.end_closed_contour(shape, offset, iadd)
    shape.primitives[ #shape.primitives ].virtual = true
end

function prepare_table.instructions.linear_segment(shape, offset, iadd)
    local data = shape.data

    -- If everything went well, first point was already transformed
    -- (by a begin_xxxx_contour instruction, for example)
    data[offset+2], data[offset+3] = transform_point(data[offset+2], data[offset+3], shape.xf)

    local x0, y0 = data[offset], data[offset+1]
    local x1, y1 = data[offset+2], data[offset+3]

    prepare_table.push_functions.linear_segment(x0, y0, x1, y1, shape.primitives, false)
end

function prepare_table.instructions.degenerate_segment(shape, offset, iadd)
    local primitives, data = shape.primitives, shape.data
    local x0, y0 = data[offset], data[offset+1]

    -- The last two parameters in data are repeated but were not transformed!!!
    data[offset+6], data[offset+7] = x0, y0

    prepare_table.push_functions.linear_segment(x0, y0, x0, y0, shape.primitives, false)
end

function prepare_table.instructions.quadratic_segment(shape, offset, iadd)
    local primitives, data = shape.primitives, shape.data

    data[offset+2], data[offset+3] = transform_point(data[offset+2], data[offset+3], shape.xf)
    data[offset+4], data[offset+5] = transform_point(data[offset+4], data[offset+5], shape.xf)

    local x0, y0 = data[offset], data[offset+1]
    local x1, y1 = data[offset+2], data[offset+3]
    local x2, y2 = data[offset+4], data[offset+5]

    -- Calculate maxima points
    local t = {}
    t[1], t[4] = 0, 1
    t[2] = truncate_parameter( (x0-x1) / (x0 - 2*x1 + x2) )
    t[3] = truncate_parameter( (y0-y1) / (y0 - 2*y1 + y2) )
    table.sort( t )

    -- Split bézier
    for i = 2, 4 do
        if t[i-1] ~= t[i] then
            u0, v0, u1, v1, u2, v2 = bezier.cut2(t[i-1], t[i], x0, y0, x1, y1, x2, y2)
            prepare_table.push_functions.quadratic_segment(u0, v0, u1, v1, u2, v2, primitives)
        end
    end
end

function prepare_table.instructions.cubic_segment(shape, offset, iadd)
    local primitives, data = shape.primitives, shape.data

    for i = 2, 6, 2 do
        data[offset+i], data[offset+(i+1)] = transform_point(data[offset+i], data[offset+(i+1)], shape.xf)        
    end

    local x0, y0 = data[offset], data[offset+1]
    local x1, y1 = data[offset+2], data[offset+3]
    local x2, y2 = data[offset+4], data[offset+5]
    local x3, y3 = data[offset+6], data[offset+7]

    -- Calculate maxima
    local t = {}
    t[1], t[6] = 0, 1
    t[2], t[3] = compute_cubic_maxima(x0, x1, x2, x3)
    t[4], t[5] = compute_cubic_maxima(y0, y1, y2, y3)
    table.sort( t )

    for i = 2, 6 do
        if t[i-1] ~= t[i] then
            u0, v0, u1, v1, u2, v2, u3, v3 = bezier.cut3(t[i-1], t[i], x0, y0, x1, y1, x2, y2, x3, y3)
            prepare_table.push_functions.cubic_segment(u0, v0, u1, v1, u2, v2, u3, v3, primitives)
        end
    end
end

function prepare_table.instructions.rational_quadratic_segment(shape, start, iadd)
    local primitives, data, xf = shape.primitives, shape.data, shape.xf

    -- Unpack and transform values
    data[start+2], data[start+3] = transform_point(data[start+2], data[start+3], xf)
    data[start+5], data[start+6] = transform_point(data[start+5], data[start+6], xf)

    local x0, y0 = data[start], data[start+1]
    local x1, y1 = data[start+2], data[start+3]
    local w = data[start+4]
    local x2, y2 = data[start+5], data[start+6]

    -- Find maxima
    t = {}
    t[1], t[6] = 0, 1
    t[2], t[3] = compute_rational_maxima(x0, x1, x2, w)
    t[4], t[5] = compute_rational_maxima(y0, y1, y2, w)
    table.sort( t )

    for i = 2, 6 do
        if t[i-1] ~= t[i] then
            local u0, v0, u1, v1, r, u2, v2 = bezier.cut2rc(t[i-1], t[i], x0, y0, x1, y1, w, x2, y2)
            prepare_table.push_functions.rational_quadratic_segment(u0, v0, u1, v1, u2, v2, r, primitives)            
        end
    end
end

-----------------------------------------------------------------------------------------
-------------------------------- GEOMETRY PREPROCESSING ---------------------------------
-----------------------------------------------------------------------------------------
function prepare_table.triangle(element)
    -- We can a transformation that maps to a canonical 
    --triangle for speed!
    local shape = element.shape

    -- Transform vertices
    local x0, y0 = shape.x1, shape.y1
    local x1, y1 = shape.x2, shape.y2
    local x2, y2 = shape.x3, shape.y3

    x0, y0 = transform_point(x0, y0, shape.xf)
    x1, y1 = transform_point(x1, y1, shape.xf)
    x2, y2 = transform_point(x2, y2, shape.xf)

    -- Precompute implicit edges    
    shape.implicit = {}
    
    local compute_implicit = function(x0, y0, x1, y1)
        
        local a, b= y1-y0, -(x1-x0)
        local c = -a*x0-b*y0

        local n = #shape.implicit+1
        shape.implicit[n] = {}
        shape.implicit[n].a = a
        shape.implicit[n].b = b
        shape.implicit[n].c = c
    end

    compute_implicit(x0, y0, x1, y1)
    compute_implicit(x1, y1, x2, y2)
    compute_implicit(x2, y2, x0, y0)

    -- Bounding box info
    shape.xmax, shape.xmin = max(x2, max(x1, x0)), min(x2, min(x1, x0))
    shape.ymax, shape.ymin = max(y2, max(y1, y0)), min(y2, min(y1, y0))
end

function prepare_table.circle(element)
    -- Precompute inverse (we could precompute a transformation
    -- which maps to a canonical circle, also)
    local shape = element.shape
    shape.inversexf = shape.xf : inverse()
end

function prepare_table.path(element)
    local shape = element.shape
    shape.primitives = {}

    -- Build primitives
    for i, v in ipairs(shape.instructions) do
        local offset = shape.offsets[i]
        prepare_table.instructions[v](shape, offset, i)
    end
end

function prepare_table.polygon(element)
    local shape, data = element.shape, element.shape.data
    shape.primitives = {}

    data[1], data[2] = transform_point(data[1], data[2], shape.xf)

    -- We build primitives just as for linear_segments. We'll "trick" the
    -- rasterization function for paths.
    for i = 3, #shape.data, 2 do
        data[i], data[i+1] = transform_point(data[i], data[i+1], shape.xf)

        local x0, y0 = data[i-2], data[i-1]
        local x1, y1 = data[i], data[i+1]

        prepare_table.push_functions.linear_segment(x0, y0, x1, y1, shape.primitives, true)
    end

    -- Push closing edge
    prepare_table.push_functions.linear_segment(data[#data-1], data[#data], 
                                            data[1], data[2], shape.primitives, true)
end

-----------------------------------------------------------------------------------------
-------------------------------- PAINT PREPROCESSING ------------------------------------
-----------------------------------------------------------------------------------------
prepare_table.prepare_paint = {}

function prepare_table.prepare_paint.solid(paint)
    -- Just multiply color alpha channel by layer opacity
    paint.data[4] = paint.data[4] * paint.opacity
end

-- This is still the naive way to do!
function prepare_table.prepare_paint.lineargradient(paint)
    local data = paint.data
    local p1, p2 = data.p1, data.p2

    data.grad_length =  math.sqrt( (p1[1] - p2[1])^2 + (p1[2] - p2[2])^2 )
    data.unit = {}
    data.unit[1] = (p2[1] - p1[1]) / data.grad_length
    data.unit[2] = (p2[2] - p1[2]) / data.grad_length

    data.inversexf = paint.xf : inverse()

    -- If 0.0 and 1.0 are not defined in the ramp, define it
    local ramp = data.ramp

    if ramp[1] ~= 0 then
        table.insert(ramp, 1, ramp[2]) -- Insert color
        table.insert(ramp, 1, 0) -- Insert offset
    end

    if ramp[#ramp-1] ~= 1 then
        local v = ramp[#ramp]
        table.insert(ramp, 1) -- Insert offset
        table.insert(ramp, v) -- Insert value
    end
end

function prepare_table.prepare_paint.radialgradient(paint)


end

-- prepare scene for sampling and return modified scene
local function preparescene(scene)

    for i, element in ipairs(scene.elements) do
        element.shape.xf = scene.xf * element.shape.xf
        prepare_table.prepare_paint[element.paint.type](element.paint)
        prepare_table[element.shape.type](element)
    end

    return scene
end

-----------------------------------------------------------------------------------------
-------------------------------- PATH SAMPLING ------------------------------------------
-----------------------------------------------------------------------------------------
local sample_table = {}
sample_table.sample_path = {}

-- TODO: MUST FUSION ALL THESE TESTS AFTER

function sample_table.sample_path.linear_segment(primitive, x, y)
    -- Bounding box tests
    if y >= primitive.ymax or y < primitive.ymin then return 0 end
    if x > primitive.xmax then return 0 end
    if x <= primitive.xmin then return primitive.dysign end

    -- Implicit test
    local eval = sign( primitive.a * x + primitive.b * y + primitive.c )

    if eval < 0 then return primitive.dysign
    else return 0 end
end

function sample_table.sample_path.degenerate_segment(primitive, x, y)
    if y == primitive.y and x <= primitive.x then
        return sign (primitive.dy0 )
    else
        return 0
    end
end

-- TODO: Fusion these two functions
function sample_table.sample_path.quadratic_segment(primitive, x, y)

    -- Bounding box test
    if y >= primitive.ymax or y < primitive.ymin then return 0 end
    if x > primitive.xmax then return 0 end
    if x <= primitive.xmin then return primitive.dysign end

    local x0, y0 = primitive.x0, primitive.y0
    local x1, y1 = primitive.x1, primitive.y1
    local x2, y2 = primitive.x2, primitive.y2

    -- Compute intersection
    local t_ = root_bisection(0, 1, function(t) return y0*(1-t)^2 + 2*(1-t)*t*y1 + y2*t^2 - y end )
    local x_ = x0*(1-t_)^2 + 2*(1-t_)*t_*x1 + x2*t_^2

    if x < x_ then return primitive.dysign
    else return 0 end
end

function sample_table.sample_path.cubic_segment(primitive, x, y)
    if y >= primitive.ymax or y < primitive.ymin then return 0 end
    if x > primitive.xmax then return 0 end
    if x <= primitive.xmin then return primitive.dysign end

    local x0, y0 = primitive.x0, primitive.y0
    local x1, y1 = primitive.x1, primitive.y1
    local x2, y2 = primitive.x2, primitive.y2
    local x3, y3 = primitive.x3, primitive.y3

    -- Compute intersection
    t_ = root_bisection(0, 1, function(t) return y0*(1-t)^3 + 3*(1-t)^2*t*y1 + 3*(1-t)*t^2*y2 + t^3*y3 - y end )
    x_ = x0*(1-t_)^3 + 3*(1-t_)^2*t_*x1 + 3*(1-t_)*t_^2*x2 + t_^3*x3

    if x < x_ then return primitive.dysign
    else return 0 end
end

function sample_table.sample_path.rational_segment(primitive, x, y)
    if y >= primitive.ymax or y < primitive.ymin then return 0 end
    if x > primitive.xmax then return 0 end
    if x <= primitive.xmin then return primitive.dysign end

    local x0, y0 = primitive.x0, primitive.y0
    local x1, y1 = primitive.x1, primitive.y1
    local x2, y2 = primitive.x2, primitive.y2
    local w = primitive.w

    -- Compute intersection
    local func = function(t)
        local bx, by, bw = bezier.at2rc(t, x0, y0, x1, y1, w, x2, y2)
        return (by/bw - y) 
    end

    local t_ = root_bisection(0, 1, func )
    local x_, y_, w_ = bezier.at2rc(t_, x0, y0, x1, y1, w, x2, y2)
    x_ = x_/w_

    if x < x_ then return primitive.dysign
    else return 0 end
end

-----------------------------------------------------------------------------------------
------------------------------- PAINT SAMPLING ------------------------------------------
-----------------------------------------------------------------------------------------
sample_table.sample_paint = {}
sample_table.sample_paint.spread_table = {}

sample_table.sample_paint.spread_table = {
    ["repeat"] = function(v)
        if v > 1 then return v - floor(v) 
        elseif v < 0 then return 1 + (v - ceil(v))
        else return v end
    end
}

function sample_table.sample_paint.spread_table.reflect(v)
    if v >= 0 then
        local int = floor(v)
        if int % 2 == 0 then return v - int
        else return 1 - (v - int) end
    else
        local int = ceil(v)
        if int % 2 == 0 then return -(v - int)
        else return 1-(v - int) end
    end
end

function sample_table.sample_paint.spread_table.pad(v)
    if v > 1 then return 1
    elseif v < 0 then return 0
    else return v end
end

---------------
---------------

function sample_table.sample_paint.solid(paint, x, y)
    return paint.data
end

function sample_table.sample_paint.lineargradient(paint, x, y)

    local data, ramp = paint.data, paint.data.ramp
    local x0, y0 = data.p1[1], data.p1[2]
    x_, y_ = transform_point(x, y, data.inversexf)

    -- Dot product (p-p1) . (p2 - p1) / ||p2-p1||
    local k = (x_ - x0) * data.unit[1] + (y_ - y0) * data.unit[2]
    local k = k / data.grad_length

    local wrapped = sample_table.sample_paint.spread_table[ramp.spread](k)
    local off = search_in_ramp(ramp, wrapped)
    local out = interpolate_colors(ramp[off+1], ramp[off+3], wrapped - ramp[off])

    out[4] = out[4] * paint.opacity

    return out
end

function sample_table.sample_paint.radialgradient(paint, x, y)
    return {0,0,0,0}
end

-----------------------------------------------------------------------------------------
---------------------------- GEOMETRY SAMPLING ------------------------------------------
-----------------------------------------------------------------------------------------
function sample_table.triangle(element, x, y)
    local implicit = element.shape.implicit
    local xmin, xmax = element.shape.xmin, element.shape.xmax
    local ymin, ymax = element.shape.ymin, element.shape.ymax

    -- Bounding box tests (closed bottom, open top, closed right, open left)
    if y < ymin or y >= ymax then return BGColor end
    if x <= xmin or x > xmax then return BGColor end

    -- Implicit test
    local edge_sign = {}
    for i = 1, 3 do
        edge_sign[i] = sign( implicit[i].a*x + implicit[i].b*y + implicit[i].c )
    end

    if edge_sign[1] == edge_sign[2] and edge_sign[2] == edge_sign[3] then
        return element.paint.data
    else
        return BGColor
    end
end

function sample_table.circle(element, x, y)
    local shape = element.shape
    local cx, cy, r = shape.cx, shape.cy, shape.r

    -- Map point to untransformed circle
    tx, ty = transform_point(x, y, shape.inversexf)
    local d = math.sqrt( (cx-tx)^2 + (cy-ty)^2 )

    if d <= r then return element.paint.data
    else return BGColor end
end

function sample_table.path(element, x, y)
    local paint, shape, primitives = element.paint, element.shape, element.shape.primitives

    local count = 0
    for i,prim in ipairs(primitives) do
        count = count + sample_table.sample_path[prim.type](prim, x, y)
    end

    local paint_flag
    if element.type == "fill" then
        paint_flag = (count ~= 0)
    elseif element.type == "eofill" then
        paint_flag = (count % 2 ~= 0)
    end

    if paint_flag == true then 
        return sample_table.sample_paint[paint.type](paint, x, y)
    else return BGColor end
end

function sample_table.polygon(element, x, y)
    return sample_table.path(element, x, y)
end

-- sample scene at x,y and return r,g,b,a
local function sample(scene, x, y)
    local out = {0,0,0,0}

    for i = #scene.elements, 1, -1 do
        local element = scene.elements[i]
        local temp = sample_table[element.shape.type](element, x, y)

        -- Superpose images
        if temp ~= BGColor then

            -- Alpha blend current color with new layer
            for j = 1, 4 do
                out[j] = alpha_composite(out[j], temp[j], out[4])
            end

            -- Premultiply values in OUT
            for j = 1, 3 do out[j] = out[j]*out[4] end
        end
    end

    -- Compose with background
    for j = 1, 4 do
        out[j] = alpha_composite(out[j], BGColor[j], out[4])
    end

    return unpack(out)
end

-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------
-- verifies that there is nothing unsupported in the scene
local function checkscene(scene)
    for i, element in ipairs(scene.elements) do
        assert(element.type == "fill" or
               element.type == "eofill", "unsupported element")
        assert(element.shape.type == "circle" or
               element.shape.type == "triangle" or
               element.shape.type == "path" or
               element.shape.type == "polygon", "unsuported primitive")
        assert(element.paint.type == "solid" or
               element.paint.type == "lineargradient" or
               element.paint.type == "radialgradient", "unsupported paint")
    end
end

-- output formatted string to stderr
local function stderr(...)
    io.stderr:write(string.format(...))
end

function _M.render(scene, viewport, file)
local time = chronos.chronos()
    -- make sure scene does not contain any unsuported content
    checkscene(scene)
    -- transform and prepare scene for rendering
    scene = preparescene(scene)
    -- get viewport
    local vxmin, vymin, vxmax, vymax = unpack(viewport, 1, 4)
stderr("preprocess in %.3fs\n", time:elapsed())
time:reset()
    -- get image width and height from viewport
    local width, height = vxmax-vxmin, vymax-vymin
    -- allocate output image
    local img = image.image(width, height)
    -- render
    for i = 1, height do
stderr("\r%5g%%", floor(1000*i/height)/10)
        local y = vymin+i-1.+.5
        for j = 1, width do
            local x = vxmin+j-1.+.5
            img:set(j, i, sample(scene, x, y))
        end
    end
stderr("\n")
stderr("rendering in %.3fs\n", time:elapsed())
time:reset()
    -- store output image
    image.png.store8(file, img)
stderr("saved in %.3fs\n", time:elapsed())
end

return _M

local driver = require"driver"
local image = require"image"
local chronos = require"chronos"

local bezier = require("lua.bezier")
local unpack, pack = table.unpack, table.pack
local max, min, floor = math.max, math.min, math.floor

local _M = driver.new()

local BGColor = require("lua.color").rgb8(1,1,1,1)
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
    holder[n].virtal = virtual
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

    -- dx/dy are not valid after transformations! Is this too much of a problem?
    prepare_table.push_functions.degenerate_segment(data[offset], data[offset+1], data[offset+2], 
                                        data[offset+3], data[offset+4], data[offset+5], primitives)
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

-----------------------------------------------------------------------------------------
-------------------------------- PREPROCESSING ------------------------------------------
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
        --print(v)
        local offset = shape.offsets[i]
        prepare_table.instructions[v](shape, offset, i)
    end
end

-- prepare scene for sampling and return modified scene
local function preparescene(scene)

    for i, element in ipairs(scene.elements) do
        element.shape.xf = scene.xf * element.shape.xf
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

-----------------------------------------------------------------------------------------
--------------------------------------- SAMPLE ------------------------------------------
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
    local shape, primitives = element.shape, element.shape.primitives

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

    if paint_flag == true then return element.paint.data
    else return BGColor end
end

-- sample scene at x,y and return r,g,b,a
local function sample(scene, x, y)
    
    for i = #scene.elements, 1, -1 do
        local element = scene.elements[i]
        local temp = sample_table[element.shape.type](element, x, y)

        -- Superpose images
        if temp ~= BGColor then return unpack(temp) end
    end

    return unpack(BGColor)
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

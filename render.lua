local driver = require"driver"
local image = require"image"
local chronos = require"chronos"

local unpack, pack = table.unpack, table.pack
local max, min, floor = math.max, math.min, math.floor

local _M = driver.new()
local BGColor = require("lua.color").rgb8(1,1,1,1)

-----------------------------------------------------------------------------------------
-------------------------------- AUXILIAR FUNCTION --------------------------------------
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

-----------------------------------------------------------------------------------------
-------------------------------- PREPROCESSING ------------------------------------------
-----------------------------------------------------------------------------------------
local prepare_table = {}

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

-- prepare scene for sampling and return modified scene
local function preparescene(scene)

    for i, element in ipairs(scene.elements) do
        element.shape.xf = scene.xf * element.shape.xf
        prepare_table[element.shape.type](element)
    end

    return scene
end

-----------------------------------------------------------------------------------------
--------------------------------------- SAMPLE ------------------------------------------
-----------------------------------------------------------------------------------------
local sample_table = {}

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

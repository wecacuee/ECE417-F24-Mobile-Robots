from tvtk.api import tvtk # TODO quicker if you only import needed ones? No it's not
import numpy as np
import matplotlib.mlab as mlab
import vtk
import fcntl
import utils

''' vtk visualisation methods
Author: Julian Ryde, Vikas Dhiman
'''
IPC_VIS = False

def marchingcubes(xyzs, file_name, voxel_size=0.05):
    polydata = tvtk.PolyData()
    polydata.points = xyzs

    # set all points as vertices
    verts = np.arange(0, xyzs.shape[0], 1, 'l') # point ids
    verts.shape = (xyzs.shape[0], 1) # make it a column vector
    polydata.verts = verts

    min, max = (-5, 5)
    sample_dimensions = ((max - min)/voxel_size,) * 3
    voxelModeller = tvtk.ImplicitModeller(sample_dimensions=sample_dimensions,
                                          model_bounds=(min, max)*3,
                                          maximum_distance=2*voxel_size/(np.sqrt(3)*(max-min)),
                                          output_scalar_type='float',
                                          input=polydata)
    marchingCubes = tvtk.MarchingCubes(
        input_connection=voxelModeller.output_port,
        compute_normals=False)
    marchingCubes.set_value(0, 0.0)
    polywriter = tvtk.PolyDataWriter(file_name=file_name,
                                    input_connection=marchingCubes.output_port)
    polywriter.write()

def compute_polydata_normals(polydata):
    pdnormals = tvtk.PolyDataNormals(input=polydata,
                                     compute_cell_normals=True,
                                     splitting=False,
                                     consistency=False,
                                     auto_orient_normals=False)
    pdnormals.update()
    polydata = pdnormals.output
    normals = polydata.cell_data.normals.to_array()
    return normals


def writepolydata(polyd, vtkfile, writer=tvtk.PolyDataWriter()):
    writer.input = polyd
    writer.file_name = vtkfile
    writer.write()

def readpolydata(vtkfile):
    reader = tvtk.PolyDataReader()
    reader.file_name = vtkfile
    reader.update()
    polydata = reader.output
    return polydata

# refactor renderer into a class variable to allow multiple displays
# See Visualizer class
renderer = tvtk.Renderer()

def visvtk(vtkfile, clip_bounds=None, color_by_normals=False,
           renderer=renderer):
    """
    visualizes the vtkfile with cells colored by normals. Also, right click
    saves "screnshot.png" in the current directory.
    """
    polydata = readpolydata(vtkfile)

    if clip_bounds is not None:
        box = tvtk.Box()
        #box.set_bounds(-5, 5, -5, 2.5, -5, 5)
        box.set_bounds(*clip_bounds)
        clipper = tvtk.ClipPolyData(input=polydata)
        clipper.inside_out = True
        clipper.clip_function = box
        clipper.generate_clip_scalars = True

        polydata = clipper.output
    return visualise(polydata, color_by_normals=color_by_normals, renderer=renderer)

def show_points(xyzs, colors=None, voxelize=False, res=0.05, polydata=None,
                vtkfiletag="visvtk", renderer=renderer):

    if polydata is None:
        polydata = tvtk.PolyData()
    #zMin = xyzs[:, 2].min()
    #zMax = xyzs[:, 2].max()
    #mapper.scalar_range = (zMin, zMax)
    #mapper.scalar_visibility = 1
    polydata.points = xyzs

    # http://www.vtk.org/Wiki/VTK/Tutorials/GeometryTopology
    # VTK strongly divides GEOMETRY from TOPOLOGY. What most users would
    # think of as "points" are actually "points + vertices" in VTK. The
    # geometry is ALWAYS simply a list of coordinates - the topology
    # represents the connectedness of these coordinates. If no topology at
    # all is specified, then, as far as VTK is aware, there is NOTHING to
    # show. If you want to see points, you must tell VTK that each point
    # is independent of the rest by creating a vertex (a 0-D topology) at
    # each point.
    verts = np.arange(0, xyzs.shape[0], 1, 'l') # point ids
    verts.shape = (xyzs.shape[0], 1) # make it a column vector
    polydata.verts = verts

    if voxelize:
        cubesrc = tvtk.CubeSource()
        cubesrc.x_length = res
        cubesrc.y_length = res
        cubesrc.z_length = res
        glyph3d = tvtk.Glyph3D()
        glyph3d.source = cubesrc.output
        glyph3d.input = polydata
        glyph3d.update()
        polydata = glyph3d.output
        normals = compute_polydata_normals(polydata)
        color_polydata_by_normals(polydata, normals)
    else:
        if colors is None:
            colors = np.abs(xyzs[:, 2] / np.max(xyzs[:, 2]))

        polydata.point_data.scalars = colors
        polydata.point_data.scalars.name = 'scalars'

    return visualise(polydata, vtkfiletag=vtkfiletag, renderer=renderer)

def axes_actor(scale=0.05):
    axes_actor = tvtk.AxesActor(axis_labels=False)
    axes_actor.total_length = (scale, scale, scale)
    return axes_actor

class InteractorStyleModified(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, callback=None, key_callback=None, timer_callback=None,
                 *args, **kwargs):
        if callback is not None:
            self._callback = callback
            self.isMoved = 0
            self.AddObserver("RightButtonPressEvent", self.rightButtonPressEvent)
            self.AddObserver("MouseMoveEvent", self.mouseMoveEvent)
            self.AddObserver("RightButtonReleaseEvent", self.rightButtonReleaseEvent)
        if key_callback is not None:
            self.AddObserver("KeyPressEvent", self.on_key_down)
            self._key_callback = key_callback
        if timer_callback is not None:
            self.AddObserver("TimerEvent", self.on_timer_callback)
            self._timer_callback = timer_callback

    def rightButtonPressEvent(self, vtkIntStyle, event):
        self.isMoved = 0
        self.OnRightButtonDown()

    def mouseMoveEvent(self, vtkIntStyle, event):
        self.isMoved = 1
        self.OnMouseMove()

    def rightButtonReleaseEvent(self, vtkIntStyle, event):
        rendrr = self.GetCurrentRenderer()
        ren_win = rendrr.GetRenderWindow()
        vtkRenWinInt = ren_win.GetInteractor()
        if not self.isMoved:
            # print "Right button pressed", vtkRenWinInt.GetEventPosition()
            self._callback(vtkRenWinInt, rendrr, event)
        self.OnRightButtonUp()

    def on_key_down(self, *args):
        iren = self.GetInteractor()
        key = iren.GetKeySym()
        self._key_callback(self, key)
        self.OnKeyDown()

    def on_timer_callback(self, vtkIntStyle, event):
        vtkRenWinInt = vtkIntStyle.GetInteractor()
        self._timer_callback(vtkRenWinInt, event)

def clear_window(renderer=renderer):
    # remove all actors
    for act in renderer.actors:
        renderer.remove_actor(act)

def remove_actor(act, renderer=renderer):
    renderer.remove_actor(act)

def show_axes(renderer=renderer, **kwargs):
    renderer.add_actor(axes_actor(**kwargs))

def show_window(size=(640, 480), snapshot_file='screenshot.png',
               timer_callback=None, renderer=renderer, axes=False):
    if IPC_VIS:
        # we will show window in another process. nothing to do here
        return

    # Renderer
    #renderer = tvtk.Renderer()
    #renderer.add_actor(actor)
    if axes:
        renderer.add_actor(axes_actor(1)) # add axes

    renderer.background = (1., 1., 1.)#(0.9, 0.9, 0.8)
    #renderer.background = (0.9, 0.9, 0.8)
    renderer.reset_camera()

    # Render Window
    renderWindow = tvtk.RenderWindow()
    renderWindow.add_renderer(renderer)

    # Interactor
    renderWindowInteractor = tvtk.RenderWindowInteractor()
    renderWindowInteractor.render_window = renderWindow

    def on_key_down(iren_style, key):
        rendrr = iren_style.GetCurrentRenderer()
        rendrr = tvtk.to_tvtk(rendrr)
        if key == '+':
            for act in rendrr.actors:
                act.property.point_size *= 1.1
        elif key == '-':
            for act in rendrr.actors:
                act.property.point_size /= 1.1
        elif key == 's':
            ren_win = rendrr.render_window
            w2if = tvtk.WindowToImageFilter(input=ren_win)
            w2if.update()
            writer = tvtk.PNGWriter(file_name=snapshot_file, input=w2if.output)
            writer.write()

    iren = InteractorStyleModified(key_callback=on_key_down,
                                  timer_callback=timer_callback)
    renderWindowInteractor.interactor_style = iren

    # Begin Interaction
    renderWindow.render()
    renderWindow.size = size
    #print 'Window size:', renderWindow.size
    renderWindowInteractor.create_repeating_timer(500)
    renderWindowInteractor.start()

def devshm_publish(polydata, vtkfile='/dev/shm/visvtk.vtk',
             lockfile='/dev/shm/visvtk.lock'):
    lockfd = open(lockfile, "w")
    try:
        fcntl.lockf(lockfd, fcntl.LOCK_EX)
        writepolydata(polydata, vtkfile)
    finally:
        fcntl.lockf(lockfd, fcntl.LOCK_UN)

def visualise(polydata, color_by_normals=False, size=1, vtkfiletag="visvtk", renderer=renderer):
    if color_by_normals:
        normals = compute_polydata_normals(polydata)

        color_polydata_by_normals(polydata, normals)
        
    # visualize in another process
    if IPC_VIS:
        devshm_publish(polydata, vtkfile="/dev/shm/%s.vtk" % vtkfiletag, 
                lockfile="/dev/shm/%s.lock" % vtkfiletag)

    # Visualize
    mapper = tvtk.PolyDataMapper()
    mapper.input = polydata
     
    actor = tvtk.Actor()
    actor.mapper = mapper
    actor.property.point_size = 2
     
    renderer.add_actor(actor)
    return actor

### from https://svn.enthought.com/enthought/browser/trunk/src/lib/enthought/tvtk/examples/animated_texture.py?rev=9564 ###
# Read the texture from image and set the texture on the actor.  If
# you don't like this image, replace with your favorite -- any image
# will do (you must use a suitable reader though).
def vtkimage_from_array(ary):
    """ Create a VTK image object that references the data in ary.
        The array is either 2D or 3D with.  The last dimension
        is always the number of channels.  It is only tested
        with 3 (RGB) or 4 (RGBA) channel images.
       
        Note: This works no matter what the ary type is (accept
        probably complex...).  uint8 gives results that make since
        to me.  Int32 and Float types give colors that I am not
        so sure about.  Need to look into this...
    """
       
    sz = ary.shape
    dims = len(sz)
    ary = ary[::-1, ...]
   
    if dims == 2:
        # Image must be grayscale
        dimensions = sz[1], sz[0], 1       
        scalars = ary.ravel()
       
    elif dims == 3:
        # 3D numpy array is a 2D array of multi-component scalars.
        dimensions = sz[1], sz[0], 1
       
        # create a 2d view of the array
        scalars = ary.reshape((sz[0]*sz[1], sz[2]))
       
    else:
        raise ValueError("ary must be 3 dimensional.")
    return scalars, dimensions

def show_img(img, renderer=renderer):
    img_data = tvtk.ImageData(spacing=(1, 1, 1), origin=(0, 0, 0))
    img_data.point_data.scalars, img_data.dimensions = vtkimage_from_array(img)

    img_actor = tvtk.ImageActor(input=img_data)
    img_actor.rotate_wxyz(45, 1, 0, 0)

    renderer.add_actor(img_actor)
    return img_actor

def show_quads(positions, directions, size):
    ''' Represents points and normals as small plane patches (quads)
    is positions is plane centre and directions is plane normals '''
    D = size # size of plane patches
    #pts = np.array( [ (0, 0, 0), (X, 0, 0), (X, X, 0), (0, X, 0) ] )
    #quads = np.array( [ (0, 1, 2, 3), (0, 1, 2, 3)] )

    # TODO convert loop to array operations
    pts = []
    for pos, direction in zip(positions, directions):
        direction = direction / np.linalg.norm(direction) # normalize direction vector

        # calculate the vectors U, V which are in plane vectors perpendicular
        # to the normal and projected into the xz, yz and xy planes
        UVW = 0.5*D*np.array([(direction[2] , 0             , -direction[0]),
                              (0            , direction[2]  , -direction[1]),
                              (direction[1] , -direction[0] , 0),
                              ])

        lens = mlab.vector_lengths(UVW, axis=1)
        # one of them might be very short so use the other two
        min_ind = np.argmin(lens)
        UVW = np.delete(UVW, min_ind, axis=0)
        U = UVW[0]
        V = UVW[1]
        # normalize
        U /= np.linalg.norm(U)
        V /= np.linalg.norm(V)
        #C = np.cross(U, V) # should equal direction

        # Create four points (must be in counter clockwise order)
        pts.append(pos - U - V)
        pts.append(pos + U - V)
        pts.append(pos + U + V)
        pts.append(pos - U + V)

    pts = np.array(pts)
    pointids = np.arange(pts.shape[0], dtype=np.int)
    quads = pointids.reshape((-1, 4))

    # Create a polydata to store everything in
    polydata = tvtk.PolyData(points=pts, polys=quads)

    # TODO this section has been copy pasted merge at some point
    # Set colors from the vector direction
    color_polydata_by_normals(polydata, directions, alpha=0)
    return visualise(polydata, renderer=renderer)

def colorsys_hsv_to_rgb(h, s, v):
    ###
    # Copied from /usr/lib/python2.7/colorsys.py 
    ###
    if np.all(s == 0.0):
        return v, v, v
    i = np.int32(h*6.0) # XXX assume int() truncates!
    f = (h*6.0) - i
    p = v*(1.0 - s)
    q = v*(1.0 - s*f)
    t = v*(1.0 - s*(1.0-f))
    i = i%6
    rgb = np.empty((len(h), 3))
    #if i == 0:
    #    return v, t, p
    rgb[(i == 0), :] = np.vstack((v, t, p)).T[i == 0]
    #if i == 1:
    #    return q, v, p
    rgb[(i == 1), :] = np.vstack((q, v, p)).T[i == 1]
    #if i == 2:
    #    return p, v, t
    rgb[(i == 2), :] = np.vstack((p, v, t)).T[i == 2]
    #if i == 3:
    #    return p, q, v
    rgb[(i == 3), :] = np.vstack((p, q, v)).T[i == 3]
    #if i == 4:
    #    return t, p, v
    rgb[(i == 4), :] = np.vstack((t, p, v)).T[i == 4]
    #if i == 5:
    #    return v, p, q
    rgb[(i == 5), :] = np.vstack((v, p, q)).T[i == 5]
    return rgb
    # Cannot get here

def color_polydata_by_normals(polydata, normals, alpha=0):
    colors = utils.rgbs_by_normals(normals, alpha=0)

    # point_data.scalars override cell_data.scalars for color, hence set them
    # to none
    polydata.point_data.scalars = None

    # Remove any vertices that are their since we want only polygons
    polydata.verts = None
    polydata.lines = None

    assert np.all((colors >= 0) & (colors <= 255))
    colors = colors.astype(np.uint8)
    polydata.cell_data.scalars = colors
    polydata.cell_data.scalars.name = 'colors'


def show_lines(p1, p2, renderer=renderer):
    """ plot lines
    """
    return quiver(p1, p2 - p1, renderer=renderer)

def quiver(positions, directions, renderer=renderer):
    ''' lines is nx6 numpy array with each row containing position and
    direction  x, y, z, nx, ny, nz '''

    positions = np.array(positions, dtype=np.float32) # cast all arrays to float
    directions = np.array(directions, dtype=np.float32) # cast all arrays to float

    # Create a vtkPoints object and store the points in it
    pts = np.vstack((positions, positions + directions))

    # Create a cell array to store the lines 
    pointids = np.arange(pts.shape[0], dtype=np.int)

    # lines is nx2 array with each row containing 2 points to be connected
    lines = pointids.reshape((2, -1)).T
     
    # Create a polydata to store everything in
    linesPolyData = tvtk.PolyData()
    # Add the points to the dataset
    linesPolyData.points = pts
    # Add the lines to the dataset
    linesPolyData.lines = lines
     
    # Set line colors
    # color from the vector direction
    magnitudes = mlab.vector_lengths(directions, axis=1).reshape((-1, 1))
    magnitudes[magnitudes == 0] = 1 # make magnitudes non zero
    colors = np.abs(directions / magnitudes) * 255
    colors = colors.astype(np.uint8)
    linesPolyData.cell_data.scalars = colors
    linesPolyData.cell_data.scalars.name = 'colors'
     
    return visualise(linesPolyData, renderer=renderer)

def line(renderer=renderer):
    source = tvtk.LineSource()
    # mapper
    mapper = tvtk.PolyDataMapper()
    mapper.input = source.output
     
    # actor
    actor = tvtk.Actor()
    actor.property.line_width = 10
    actor.mapper = mapper
    renderer.add_actor(actor)
    return actor

def cylinders(positions, directions, radii, length=1, renderer=renderer):
    for position, dir, radius in zip(positions, directions, radii):
        # create source
        source = tvtk.CylinderSource()
        #source.center = position
        source.radius = float(radius)
        source.height = length
        source.resolution = 5
        source.capping = 0 # 0 no caps, 1 capped

        # mapper
        mapper = tvtk.PolyDataMapper()
        mapper.input = source.output

        # actor
        actor = tvtk.Actor()
        actor.mapper = mapper

        # cylinder is aligned with y-axis so rotation axis is cross product
        # with target direction

        initial_axis = (0, 1, 0)
        axis = np.cross(initial_axis, dir)
        # Current orientation
        #w0, x0, y0, z0 = actor.orientation_wxyz
        # The angle of rotation in degrees. 
        w = np.arccos(np.dot(dir, initial_axis))
        w = np.degrees(w)
        # The rotation is in the plane of dir and [x0, y0, z0], the axis of
        # rotation should be normal to it
        #x, y, z = np.cross(dir, [x0, y0, z0])
        actor.rotate_wxyz(w, axis[0], axis[1], axis[2])
        actor.position = position

        renderer.add_actor(actor)
        return actor

class ImageActorWrapper(object):
    """Useful for quickly generating actor to display just an image.
    Also see ImageRenderer
    """
    def __init__(self, height, width):
        self.img_data = tvtk.ImageData(spacing=(1, 1, 1))
        img = np.zeros((height, width, 3), dtype=np.uint8)
        self.update_img(img)
        self.img_actor = tvtk.ImageActor(input=self.img_data)

    def update_img(self, img):
        self.img_data.point_data.scalars, self.img_data.dimensions = \
                vtkimage_from_array(img)

    def getactor(self):
        return self.img_actor

def prepare_image_renderer(renderer, extent, origin, spacing):
    # Fix the camera with respect to the image renderer
    img_cam = renderer.active_camera
    xc = origin[0] + (extent[0] + extent[1] + 1)*spacing[0]
    #xc = origin[0] + 0.5(extent[0] + extent[1])*spacing[0]
    yc = origin[1] + 0.5*(extent[2] + extent[3] + 1)*spacing[1]
    yd = (extent[3] - extent[2] + 1)*spacing[1]
    zc = origin[2]
    #{{{ Setting up camera with parallel projection
    img_cam.parallel_projection = 1
    img_cam.position = [xc, yc, zc + img_cam.distance]
    img_cam.focal_point = [xc, yc, zc]
    img_cam.parallel_scale = 0.5 * yd
    #}}}


def vtkmatrix_from_array(ary):
    m = tvtk.Matrix4x4()
    m.from_array(ary)
    return m

class ImageRenderer(object):
    """Useful for quickly generating renderer to display just an image"""
    def __init__(self, height, width):
        self.img_actor_wrap = ImageActorWrapper(height, width)
        self.renderer = tvtk.Renderer()
        self.renderer.add_actor(self.img_actor_wrap.getactor())
        self.renderer.interactive = False
        img_data = self.img_actor_wrap.img_data
        prepare_image_renderer(self.renderer,
                               extent=img_data.extent,
                               origin=img_data.origin,
                               spacing=img_data.spacing)

    def update_img(self, img):
        self.img_actor_wrap.update_img(img)

class Visualizer(object):
    _method_list = """show_points clear_window remove_actor show_window
                      visualise show_img show_quads
                      show_lines quiver line cylinder
                      prepare_image_renderer visvtk""".split()
    def __init__(self, renderer=renderer):
        self.renderer = renderer

    def __getattr__(self, name):
        if name not in self._method_list:
            raise AttributeError("attribute '%s' not found" % name)

        def wrapper(self, *args, **kwargs):
            kwargs['renderer'] = self.renderer
            globals()[name](*args, **kwargs)
        return wrapper

# source - geometric entity
# mapper - conversion of geometric entity to renderable representation
# actor - control appearance of renderable representation

def examples():
    xyzs = np.random.random((10, 3)) * 10
    uvws = np.random.random((10, 3))
    print('Points')
    colors = np.random.random(len(xyzs))
    show_axes(scale=1)
    act = show_points(xyzs, colors)
    show_window()
    remove_actor(act)

    #xyzs = np.array([ (1, 0, 0), (0, 1, 0), (0, 0, 1) ])
    #uvws = np.array([ (1, 0, 0), (0, 1, 0), (0, 0, 1) ])

    print('Cylinders')
    cyl = cylinders(xyzs, uvws, 0.1*np.ones(len(xyzs)), length=1)


    #print 'Line'
    #line()

    print('Quads')
    quad = show_quads(xyzs, uvws, 1)

    print('Quiver' )
    quiv = quiver(xyzs, uvws)
    show_window()
    remove_actor(cyl)
    remove_actor(quad)
    remove_actor(quiv)

    #clear_window()

    print('Displaying image')
    im = np.random.randint(0, 255, (50, 100, 3))
    im = np.uint8(im)
    im_act = show_img(im)
    show_window()

    remove_actor(im_act)

    print('Colors')
    polydata = tvtk.SphereSource().output
    visualise(polydata, color_by_normals=True, renderer=renderer)

    show_window()

if __name__ == '__main__':
    ply_reader = tvtk.PLYReader()
    ply_reader.input = "00000.ply"
    ply_reader.update()
    polydata = ply_reader.output
    visualise(polydata, color_by_normals=True, renderer=renderer)

    show_window()


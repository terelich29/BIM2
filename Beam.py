import NemAll_Python_Geometry as g
import NemAll_Python_BaseElements as base
import NemAll_Python_BasisElements as basis
import NemAll_Python_Utility as util
import GeometryValidate as v
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties


print('iiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\niiiiiiiiiiiiiiiii\n')


def check_allplan_version(build_ele, version):
    del build_ele
    del version
    return True


def create_element(build_ele, doc):
    element = Beam(doc)
    return element.create(build_ele)


def move_handle(build_ele, handle_prop, input_pnt, doc):
    build_ele.change_property(handle_prop, input_pnt)
    return create_element(build_ele, doc)


class Beam:

    def __init__(self, doc):
        self.model_ele_list = []
        self.handle_list = []
        self.document = doc

    def create(self, build_ele):
        self.top(build_ele)
        self.handles(build_ele)
        return (self.model_ele_list, self.handle_list)

    def get_all_data(self, build_ele):
        Width = build_ele.Width.value
        Length = build_ele.Length.value
        Heigth = build_ele.Heigth.value
        CenterWidth = build_ele.CenterWidth.value
        CentralHeight = build_ele.CentralHeight.value
        CutTop = build_ele.CutTop.value
        Radius = build_ele.Radius.value
        WidthT = build_ele.WidthT.value
        TopTop = build_ele.TopTop.value
        PlateSpace = build_ele.PlateSpace.value
        PlateHeight = build_ele.PlateHeight.value
        Color4 = build_ele.Color4.value
        CutTopBottom = build_ele.CutTopBottom.value
        return [Width, Length, Heigth, CenterWidth, CentralHeight, CutTop, Radius, WidthT, TopTop, PlateSpace, PlateHeight, Color4, CutTopBottom]

    def bottom(self, build_ele):
        all_data = self.get_all_data(build_ele)
        cuboid = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(0, 0, 0), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), all_data[0], all_data[1], all_data[2])
        cuboid_help = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(0, 0, 0), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), all_data[0], all_data[1], all_data[2])
        cw = build_ele.CutTop.value
        cb = build_ele.CutButtom.value

        if cw > 0:
            es = util.VecSizeTList()
            es.append(1)
            es.append(3)
            e, cuboid = g.ChamferCalculus.Calculate(cuboid, es, cw, False)
            
            if not v.polyhedron(e):
                return

        if cb > 0:
            es2 = util.VecSizeTList()
            es2.append(8)
            es2.append(10)
            e, cuboid_help = g.ChamferCalculus.Calculate(cuboid_help, es2, cb, False)

            if not v.polyhedron(e):
                return

        e, final = g.MakeIntersection(cuboid, cuboid_help)
        return final

    def middle(self, build_ele):
        all_data = self.get_all_data(build_ele)
        cuboid = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(all_data[0] / 2 - all_data[3] / 2, 0, all_data[2]), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), all_data[3], all_data[1], all_data[4])
        cylinder = g.BRep3D.CreateCylinder(g.AxisPlacement3D(g.Point3D(all_data[5], all_data[1] / 8, all_data[2] + all_data[4] / 2), g.Vector3D(0, 0, 1), g.Vector3D(1, 0, 0)), all_data[6], all_data[3])
        cylinder1 = g.BRep3D.CreateCylinder(g.AxisPlacement3D(g.Point3D(all_data[5], all_data[1] - all_data[1] / 8, all_data[2] + all_data[4] / 2), g.Vector3D(0, 0, 1), g.Vector3D(1, 0, 0)), all_data[6], all_data[3])
        e, cuboid = g.MakeSubtraction(cuboid, cylinder)
        e, cuboid = g.MakeSubtraction(cuboid, cylinder1)
        e, final = g.MakeUnion(
            cuboid, self.bottom(build_ele))
        return final

    def top(self, build_ele):
        all_data = self.get_all_data(build_ele)
        cuboid = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(0 - (all_data[7] - all_data[0]) / 2, 0, all_data[2] + all_data[4]), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), all_data[7], all_data[1], all_data[8])
        cuboid_plate = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(all_data[9] - (all_data[7] - all_data[0]) / 2, 0, all_data[2] + all_data[4] + all_data[8]), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), all_data[7] - all_data[9]*2, all_data[1], all_data[10])
        com_prop = base.CommonProperties()
        com_prop.GetGlobalProperties()
        com_prop.Pen = 1
        com_prop.Color = all_data[11]
        chamfer_width_top = all_data[12]

        if chamfer_width_top > 0:
            es2 = util.VecSizeTList()
            es2.append(8)
            es2.append(10)
            e, cuboid = g.ChamferCalculus.Calculate(cuboid, es2, chamfer_width_top, False)

            if not v.polyhedron(e):
                return

        e, final = g.MakeUnion(cuboid, self.middle(build_ele))
        e, final = g.MakeUnion(final, cuboid_plate)
        self.model_ele_list.append(basis.ModelElement3D(com_prop, final))

    def handles(self, build_ele):
        all_data = self.get_all_data(build_ele)
        d = g.Point3D(all_data[0] / 2, all_data[1], all_data[4] + all_data[2])
        d2 = g.Point3D(all_data[0] / 2, 0, all_data[2] / 2)
        d3 = g.Point3D(0, all_data[1], (all_data[2] - all_data[5]) / 2)
        d4 = g.Point3D(0 - (all_data[7] - all_data[0]) / 2, all_data[1], all_data[4] + all_data[2] + all_data[12])
        d5 = g.Point3D(all_data[0] / 2, all_data[1], all_data[4] + all_data[2] - all_data[2] / 4)
        d6 = g.Point3D(all_data[0] / 2, all_data[1], all_data[4] + all_data[2] + all_data[8])
        d7 = g.Point3D(all_data[0] / 2, all_data[1], 0)
        d8 = g.Point3D(all_data[0] / 2 - all_data[3] / 2, all_data[1], all_data[4] / 2 + all_data[2])

        self.handle_list.append(HandleProperties("CentralHeight", g.Point3D(d.X, d.Y, d.Z), g.Point3D(d.X, d.Y, d.Z - all_data[4]), [("CentralHeight", HandleDirection.z_dir)], HandleDirection.z_dir, False))
        self.handle_list.append(HandleProperties("Length", g.Point3D(d2.X, d2.Y + all_data[1], d2.Z), g.Point3D(d2.X, d2.Y, d2.Z), [("Length", HandleDirection.y_dir)], HandleDirection.y_dir, False))
        self.handle_list.append(HandleProperties("Width", g.Point3D(d3.X + all_data[0], d3.Y, d3.Z), g.Point3D(d3.X, d3.Y, d3.Z), [("Width", HandleDirection.x_dir)], HandleDirection.x_dir, False))
        self.handle_list.append(HandleProperties("WidthT", g.Point3D(d4.X + all_data[7], d4.Y, d4.Z), g.Point3D(d4.X, d4.Y, d4.Z), [("WidthT", HandleDirection.x_dir)], HandleDirection.x_dir, False))
        self.handle_list.append(HandleProperties("TopTop", g.Point3D(d5.X, d5.Y, d5.Z + all_data[8]), g.Point3D(d5.X, d5.Y, d5.Z), [("TopTop", HandleDirection.z_dir)], HandleDirection.z_dir, False))
        self.handle_list.append(HandleProperties("PlateHeight", g.Point3D(d6.X, d6.Y, d6.Z + all_data[10]), g.Point3D(d6.X, d6.Y, d6.Z), [("PlateHeight", HandleDirection.z_dir)], HandleDirection.z_dir, False))
        self.handle_list.append(HandleProperties("Heigth", g.Point3D(d7.X, d7.Y, d7.Z + all_data[2]), g.Point3D(d7.X, d7.Y, d7.Z), [("Heigth", HandleDirection.z_dir)], HandleDirection.z_dir, False))
        self.handle_list.append(HandleProperties("CenterWidth", g.Point3D(d8.X + all_data[3], d8.Y, d8.Z), g.Point3D(d8.X, d8.Y, d8.Z), [("CenterWidth", HandleDirection.x_dir)], HandleDirection.x_dir, False))

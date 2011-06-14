#!/usr/bin/env python
# downloaded from https://code.ros.org/trac/ros-pkg/attachment/ticket/4440/point_cloud.py

import roslib;

import ctypes
import math
import struct

import rosbag
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    assert(cloud)
    fmt = _get_struct_fmt(cloud, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in xrange(height):
                offset = row_step * v
                for u in xrange(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in xrange(height):
                offset = row_step * v
                for u in xrange(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def create_cloud_xyz32(header, points):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    return create_cloud(header, fields, points)

def create_cloud(header, fields, points):
    cloud = PointCloud2()
    cloud.header       = header
    cloud.height       = 1
    cloud.width        = len(points)
    cloud.is_dense     = False
    cloud.is_bigendian = False
    cloud.fields       = fields
    fmt                = _get_struct_fmt(cloud)
    cloud_struct       = struct.Struct(fmt)
    cloud.point_step   = cloud_struct.size
    cloud.row_step     = cloud_struct.size * cloud.width

    buffer = ctypes.create_string_buffer(cloud_struct.size * cloud.width)

    point_step, pack_into = cloud.point_step, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buffer, offset, *p)
        offset += point_step

    cloud.data = buffer.raw

    return cloud

def _get_struct_fmt(cloud, field_names=None):
    fmt = '>' if cloud.is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(cloud.fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

if __name__ == '__main__':
    bag = rosbag.Bag(roslib.packages.find_resource('picking_util', 'laser_snapshot.bag')[0])
    _, cloud, _ = bag.read_messages().next()

    import time

    s = time.time()
    for _ in read_points(cloud):
        pass
    print '%d ms to read %d points' % ((time.time() - s) * 1000, cloud.width * cloud.height)

    pts = []
    for p in read_points(cloud):
        pts.append(p)

    s = time.time()
    cloud2 = create_cloud(cloud.header, cloud.fields, pts)
    print '%d ms to write %d points' % ((time.time() - s) * 1000, cloud2.width * cloud2.height)

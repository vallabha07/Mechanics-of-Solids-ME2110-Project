"""world.py - MuJoCo MJCF Scene & Articulated Hand-Arm Hierarchy Builder."""
import math
import os

from . import gripper_models
from . import hand as H
from .arm_ik import ARM
from .materials import OBJECTS, height, width

WORLD = 'grip_world'
TABLE_TOP_Z = 0.080

def _f(x): return '%.6g' % x
def _rgba_str(rgba): return ' '.join('%.4f' % v for v in rgba)
def _quat_zy(phi, beta):
    cz, sz, cy, sy = math.cos(phi / 2.0), math.sin(phi / 2.0), math.cos(beta / 2.0), math.sin(beta / 2.0)
    return '%s %s %s %s' % (_f(cz * cy), _f(-sz * sy), _f(cz * sy), _f(cy * sz))

def _obj_geom_xml(gname, o, crushed=False, mesh_asset_name=None):
    h, w, c_str = height(o), width(o), _rgba_str((1.0, 0.08, 0.08, 1.0) if crushed else o['rgba'])
    if o.get('shape') == 'mesh' and mesh_asset_name and o.get('mesh_ok', True): return '<geom name="%s" type="mesh" mesh="%s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, mesh_asset_name, c_str)
    if o.get('shape') == 'mesh': return '<geom name="%s" type="cylinder" size="%s %s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(w * (0.68 if crushed else 0.5)), _f(h * (0.46 if crushed else 0.5)), c_str)
    if crushed: return '<geom name="%s" type="box" size="%s %s %s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(w * 0.59), _f(w * 0.34), _f(h * 0.46), c_str)
    if o['vis'] == 'sphere': return '<geom name="%s" type="sphere" size="%s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(o['dims'][0]), c_str)
    if o['shape'] == 'box': return '<geom name="%s" type="box" size="%s %s %s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(o['dims'][0] / 2.0), _f(o['dims'][1] / 2.0), _f(o['dims'][2] / 2.0), c_str)
    return '<geom name="%s" type="cylinder" size="%s %s" rgba="%s" contype="0" conaffinity="0"/>' % (gname, _f(o['dims'][0]), _f(o['dims'][1] / 2.0), c_str)

def build_mjcf(n_fingers=None, z0=TABLE_TOP_Z):
    n = int(n_fingers or H.HAND['n_fingers'])
    H.HAND['n_fingers'] = n
    phis = H.azimuths(n)
    mesh_map = gripper_models.ensure_gripper_meshes(z0, n)
    xs, zs, L1, L2, L_p = ARM['xs'], ARM['zs'], ARM['L1'], ARM['L2'], H.HAND['L']

    xml = []
    a = xml.append
    a('<mujoco model="grip_mos_lab"><compiler angle="radian" coordinate="local"/>')
    a('<option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>')
    a('<visual><headlight ambient="0.38 0.42 0.48" diffuse="0.65 0.68 0.74" specular="0.3 0.3 0.35"/><quality shadowsize="4096" offsamples="4"/><map znear="0.01" zfar="15.0"/><global offwidth="1280" offheight="720" elevation="-18" azimuth="132"/></visual>')
    
    a('<asset>')
    a('<texture name="grid_tex" type="2d" builtin="checker" width="512" height="512" rgb1="0.11 0.13 0.17" rgb2="0.14 0.17 0.22" mark="edge" markrgb="0.20 0.24 0.30"/>')
    a('<material name="floor_mat" texture="grid_tex" texrepeat="8 8" reflectance="0.12" shininess="0.4" specular="0.3"/>')
    a('<material name="bench_mat" rgba="0.58 0.42 0.27 1" reflectance="0.10" shininess="0.5" specular="0.35"/>')
    a('<material name="table_leg_mat" rgba="0.16 0.18 0.22 1" reflectance="0.20" shininess="0.7" specular="0.6"/>')
    a('<material name="fixture_mat" rgba="0.12 0.14 0.18 1" reflectance="0.25" shininess="0.8" specular="0.7"/>')
    for cname, rgba in gripper_models.COLORS.items(): a('<material name="mat_%s" rgba="%s" shininess="0.65" specular="0.45"/>' % (cname, _rgba_str(rgba)))
    for key, entries in mesh_map.items():
        for cname, fpath in entries: a('<mesh name="mesh_%s_%s" file="%s"/>' % (key, cname, fpath))
    for obj_key, o in OBJECTS.items():
        if o.get('shape') == 'mesh' and o.get('mesh') and os.path.isfile(o['mesh']) and o.get('mesh_ok', True):
            a('<mesh name="cad_%s_intact" file="%s" scale="1 1 1"/>' % (obj_key, o['mesh']))
            a('<mesh name="cad_%s_crushed" file="%s" scale="1.18 0.68 0.92"/>' % (obj_key, o['mesh']))
    a('</asset>')

    a('<worldbody>')
    a('<light name="key_light" directional="true" castshadow="true" pos="-0.4 -0.8 2.6" dir="0.30 0.45 -0.84" diffuse="0.88 0.90 0.95" specular="0.35 0.35 0.40"/>')
    a('<light name="fill_light" directional="false" castshadow="false" pos="1.0 1.2 2.0" dir="-0.4 -0.5 -0.75" diffuse="0.36 0.42 0.52" specular="0.10 0.10 0.12"/>')
    a('<light name="gantry_spot" directional="false" castshadow="false" pos="0 0 0.78" dir="0 0 -1" diffuse="0.35 0.40 0.48" specular="0.2 0.2 0.25"/>')
    a('<geom name="lab_floor" type="plane" pos="0 0 -0.42" size="4 4 0.05" material="floor_mat" contype="0" conaffinity="0"/>')
    
    a('<body name="table_pedestal" pos="0 0 0">')
    a('<geom name="bench_top" type="box" pos="-0.18 0 0.048" size="0.59 0.41 0.028" material="bench_mat" contype="0" conaffinity="0"/>')
    # table legs (floor at z=-0.42 up to underside of the top at z=0.020), inset from the corners
    for lx in (-0.72, 0.36):
        for ly in (-0.36, 0.36):
            a('<geom name="table_leg_%s_%s" type="box" pos="%s %s -0.20" size="0.030 0.030 0.220" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if lx > 0 else 'n', 'p' if ly > 0 else 'n', _f(lx), _f(ly)))
    # apron rails just under the top, plus low stretchers for a sturdy table look
    for ly in (-0.36, 0.36):
        a('<geom name="apron_x_%s" type="box" pos="-0.18 %s -0.012" size="0.540 0.012 0.032" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if ly > 0 else 'n', _f(ly)))
        a('<geom name="stretcher_x_%s" type="box" pos="-0.18 %s -0.30" size="0.540 0.010 0.014" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if ly > 0 else 'n', _f(ly)))
    for lx in (-0.72, 0.36):
        a('<geom name="apron_y_%s" type="box" pos="%s 0 -0.012" size="0.012 0.330 0.032" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if lx > 0 else 'n', _f(lx)))
        a('<geom name="stretcher_y_%s" type="box" pos="%s 0 -0.30" size="0.010 0.330 0.014" material="table_leg_mat" contype="0" conaffinity="0"/>' % ('p' if lx > 0 else 'n', _f(lx)))
    a('<geom name="fixture_plate" type="cylinder" pos="0 0 0.078" size="0.16 0.002" material="fixture_mat" contype="0" conaffinity="0"/>')
    a('</body>')

    a('<body name="grip_arm_base" pos="%s 0 0">' % _f(xs))
    for cname, _ in mesh_map['base']: a('<geom type="mesh" mesh="mesh_base_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
    a('  <body name="grip_arm_link1" pos="0 0 %s">' % _f(zs + z0))
    a('    <joint name="shoulder" type="hinge" axis="0 1 0" limited="false"/>')
    for cname, _ in mesh_map['link1']: a('    <geom type="mesh" mesh="mesh_link1_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
    a('    <body name="grip_arm_link2" pos="%s 0 0">' % _f(L1))
    a('      <joint name="elbow" type="hinge" axis="0 1 0" limited="false"/>')
    for cname, _ in mesh_map['link2']: a('      <geom type="mesh" mesh="mesh_link2_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
    a('      <body name="grip_arm_palm" pos="%s 0 0">' % _f(L2))
    a('        <joint name="wrist" type="hinge" axis="0 1 0" limited="false"/>')
    for cname, _ in mesh_map['palm']: a('        <geom type="mesh" mesh="mesh_palm_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
    
    for i, phi in enumerate(phis):
        bx, by = H.HAND['r_base'] * math.cos(phi), H.HAND['r_base'] * math.sin(phi)
        a('        <body name="grip_f%d_p1" pos="%s %s %s" quat="%s">' % (i, _f(bx), _f(by), _f(-ARM['tcp_drop']), _quat_zy(phi, math.pi / 2.0)))
        a('          <joint name="f%d_mcp" type="hinge" axis="0 1 0" limited="false"/>' % i)
        for cname, _ in mesh_map['phalanx1']: a('          <geom type="mesh" mesh="mesh_phalanx1_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
        a('          <body name="grip_f%d_p2" pos="%s 0 0">' % (i, _f(L_p[0])))
        a('            <joint name="f%d_pip" type="hinge" axis="0 1 0" limited="false"/>' % i)
        for cname, _ in mesh_map['phalanx2']: a('            <geom type="mesh" mesh="mesh_phalanx2_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
        a('            <body name="grip_f%d_p3" pos="%s 0 0">' % (i, _f(L_p[1])))
        a('              <joint name="f%d_dip" type="hinge" axis="0 1 0" limited="false"/>' % i)
        for cname, _ in mesh_map['phalanx3']: a('              <geom type="mesh" mesh="mesh_phalanx3_%s" material="mat_%s" contype="0" conaffinity="0"/>' % (cname, cname))
        a('            </body>')
        a('          </body>')
        a('        </body>')
    a('      </body>')
    a('    </body>')
    a('  </body>')
    a('</body>')

    for obj_key, o in OBJECTS.items():
        m_i = ('cad_%s_intact' % obj_key) if o.get('shape') == 'mesh' and o.get('mesh_ok', True) else None
        m_c = ('cad_%s_crushed' % obj_key) if o.get('shape') == 'mesh' and o.get('mesh_ok', True) else None
        a('<body name="obj_%s" mocap="true" pos="0 0 -5.0">' % obj_key + _obj_geom_xml('geom_%s' % obj_key, o, False, m_i) + '</body>')
        a('<body name="crush_%s" mocap="true" pos="0 0 -5.0">' % obj_key + _obj_geom_xml('geom_crush_%s' % obj_key, o, True, m_c) + '</body>')

    a('</worldbody></mujoco>')
    return '\n'.join(xml)

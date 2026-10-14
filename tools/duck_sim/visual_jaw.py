"""Viewer-only jaw. The ONNX/MuJoCo policy remains exactly 14-dimensional.

head_roll.stl was separated into its exact 1272-face jaw component and all
remaining faces. Two concentric CAD holes (r=5/8 mm) independently give this
hinge centre, with <7e-9 m residual. See output/duck_sim/jaw-id34 proof.
"""
from copy import deepcopy

JAW_INDEX = 14
JAW_ID = 34
JAW_ANCHOR = [0., .0028, -.0179]
JAW_AXIS = [-1., 0., 0.]


def with_visual_jaw(tree, assets):
    tree, assets = deepcopy(tree), deepcopy(assets)
    def find(body):
        if (body.get('joint') or {}).get('name') == 'head_roll':
            return body
        for child in body['children']:
            found = find(child)
            if found is not None:
                return found
        return None
    head = find(tree)
    if head is None:
        raise ValueError('Head roll link missing; cannot attach jaw')
    # Keep the same coordinate frame; split only rendering geometry.
    assets['head_roll'] = dict(file='viewer/head_fixed.stl', scale=[1, 1, 1])
    assets['jaw_moving'] = dict(file='viewer/jaw_moving.stl', scale=[1, 1, 1])
    head['children'].append(dict(name='viewer_jaw', pos=[0, 0, 0], quat=[1, 0, 0, 0],
        joint=dict(name='jaw', index=JAW_INDEX, id=JAW_ID, label='嘴部开合',
                   axis=JAW_AXIS, pos=JAW_ANCHOR, range=[0, 30], visual_only=True),
        meshes=[dict(name='jaw_moving', pos=[0, 0, 0], quat=[1, 0, 0, 0])], children=[]))
    return tree, assets

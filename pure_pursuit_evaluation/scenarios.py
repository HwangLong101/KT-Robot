"""Synthetic paths and initial poses; all distances in meters."""
import numpy as np


def scenario(name):
    if name == 'straight':
        return np.array([[0., 0.], [8., 0.]]), (0., -0.6, 0.)
    if name == 's_curve':
        x = np.linspace(0., 10., 201)
        return np.column_stack((x, np.sin(2*np.pi*x/10))), (0., -0.6, 0.)
    if name == 'corners':
        return np.array([[0., 0.], [4., 0.], [4., 3.], [8., 3.], [8., 6.]]), (0., -0.6, 0.)
    raise ValueError(f'Unknown scenario: {name}')


NAMES = ('straight', 's_curve', 'corners')

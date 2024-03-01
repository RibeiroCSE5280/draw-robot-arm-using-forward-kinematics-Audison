import unittest
import importlib
import importnb
import numpy as np
from importnb import imports
#from importnb import Notebook, get_ipython, imports


from unittest.mock import patch

import cvxopt
from cvxopt import matrix, printing

from robot3D_basic import *

def forward_kinematics(Phi, L1, L2, L3, L4):
    # Initialization of the transformation matrices as identity matrices
    T_01 = np.identity(4)
    T_02 = np.identity(4)
    T_03 = np.identity(4)
    T_04 = np.identity(4)

    # Rotation and translation for each joint
    R_01 = RotationMatrix(Phi[0], 'z')
    t_01 = np.array([[L1], [0], [0], [1]])
    T_01 = getLocalFrameMatrix(R_01, t_01[:3])

    R_12 = RotationMatrix(Phi[1], 'z')
    t_12 = np.array([[L2], [0], [0], [1]])
    T_02 = T_01 @ getLocalFrameMatrix(R_12, t_12[:3])

    R_23 = RotationMatrix(Phi[2], 'z')
    t_23 = np.array([[L3], [0], [0], [1]])
    T_03 = T_02 @ getLocalFrameMatrix(R_23, t_23[:3])

    R_34 = RotationMatrix(Phi[3], 'z')
    t_34 = np.array([[L4], [0], [0], [1]])
    T_04 = T_03 @ getLocalFrameMatrix(R_34, t_34[:3])

    # Extract the end-effector position from the last column of T_04
    e = T_04[:3, 3]

    return T_01, T_02, T_03, T_04, e


class TestRobotArm(unittest.TestCase):
	
	def test_forward_kinematics1(self):
		
		# Lentghs of the parts
		L1, L2, L3, L4 = [5, 8, 3, 0]
		Phi = np.array([30, -50, -30, 0])
		T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
		
		actual = e
		expected = np.array([18.47772028, -0.71432837,  0. ])

		assert np.allclose(expected, actual)

	def test_forward_kinematics2(self):
		
		# Lentghs of the parts
		L1, L2, L3, L4 = [5, 8, 3, 0]
		Phi = np.array([0, 0, 0, 0])
		T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
		
		actual = e
		expected = np.array([21, 2,  0. ])

		assert np.allclose(expected, actual)

	def test_forward_kinematics3(self):
		
		# Lentghs of the parts
		L1, L2, L3, L4 = [5, 8, 3, 0]
		Phi = np.array([-30, 50, 30, 0])
		T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
		
		actual = e
		expected = np.array([18.47772028,  4.71432837,  0. ])

		assert np.allclose(expected, actual)



if __name__ == '__main__':
	unittest.main()




import pytest
import numpy as np

from math import isclose
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

from orbita import Actuator


def rot(axis, deg):
    return R.from_euler(axis, np.deg2rad(deg)).as_matrix()


def test_actuator_setup():
    actuator = Actuator()

    assert np.allclose(actuator.x0, np.array((1, 0, 0)))
    assert np.allclose(actuator.y0, np.array((0, 1, 0)))
    assert np.allclose(actuator.z0, np.array((0, 0, 1)))

    assert actuator.x0_quat == Quaternion(0, 1, 0, 0)
    assert actuator.y0_quat == Quaternion(0, 0, 1, 0)
    assert actuator.z0_quat == Quaternion(0, 0, 0, 1)

    R0 = np.dot(rot('z', 60), rot('y', 10))
    actuator = Actuator(R0=R0)

    assert np.allclose(actuator.x0, R0[0])
    assert np.allclose(actuator.y0, R0[1])
    assert np.allclose(actuator.z0, R0[2])

    assert actuator.x0_quat == Quaternion(0, R0[0][0], R0[0][1], R0[0][2])
    assert actuator.y0_quat == Quaternion(0, R0[1][0], R0[1][1], R0[1][2])
    assert actuator.z0_quat == Quaternion(0, R0[2][0], R0[2][1], R0[2][2])

    Pc_z = [0, 0, 89.4]
    arr = np.array(Pc_z)
    actuator = Actuator(Pc_z=Pc_z)
    Pc_z[0] = 42
    assert np.all(actuator.Pc_z == arr)

    Cp_z = [0, 0, 89.4]
    arr = np.array(Cp_z)
    actuator = Actuator(Cp_z=Cp_z)
    Cp_z[0] = 42
    assert np.all(actuator.Cp_z == arr)


def test_last_angles():
    a1 = Actuator()
    a2 = Actuator()

    assert np.allclose(a1.last_angles, [0, 2 * np.pi / 3, -2 * np.pi / 3])
    assert np.allclose(a2.last_angles, [0, 2 * np.pi / 3, -2 * np.pi / 3])

    a1.last_angles[1] = 0

    assert np.allclose(a2.last_angles, [0, 2 * np.pi / 3, -2 * np.pi / 3])


def test_new_frame_from_vector():
    a = Actuator()
    X, Y, Z = a.get_new_frame_from_vector([0, 0, 1], 90)

    assert np.allclose(X, a.y0)
    assert np.allclose(Y, -a.x0)
    assert np.allclose(Z, a.z0)

    a = Actuator(R0=rot('z', 60))
    X, Y, Z = a.get_new_frame_from_vector([0, 0, 1], 90)

    assert np.allclose(X, a.y0)
    assert np.allclose(Y, -a.x0)
    assert np.allclose(Z, a.z0)


def test_get_angles_from_vector():
    a = Actuator()

    for _ in range(5):
        x = np.random.rand() * 180 - 90
        q11, q12, q13 = a.get_angles_from_vector([0, 0, 1], x)

        assert isclose(q11, x)
        assert isclose(q12, x)
        assert isclose(q13, x)

    a = Actuator(R0=rot('z', 60))

    for _ in range(5):
        x = np.random.rand() * 180 - 90
        q11, q12, q13 = a.get_angles_from_vector([0, 0, 1], x)

        assert isclose(q11, x - 60)
        assert isclose(q12, x - 60)
        assert isclose(q13, x - 60)


def test_get_angles_from_vector_big_angles():
    a = Actuator()

    x = 180.0 + np.random.rand() * 180
    q11, q12, q13 = a.get_angles_from_vector([0, 0, 1], x)

    assert isclose(q11, x - 360)
    assert isclose(q12, x - 360)
    assert isclose(q13, x - 360)


def test_get_angles_from_vector_with_last_angles():
    a = Actuator()

    last = np.random.randint(180, 360)

    Q = []

    for x in range(0, last + 1):
        q = a.get_angles_from_vector([0, 0, 1], x)
        Q.append(q)

    q11, q12, q13 = Q[-1]

    assert np.isclose(q11, last)
    assert np.isclose(q12, last)
    assert np.isclose(q13, last)


def test_new_frame_from_quaternion():
    a = Actuator()

    q = Quaternion(axis=[0, 0, 1], degrees=90)
    X, Y, Z = a.get_new_frame_from_quaternion(q.w, q.x, q.y, q.z)

    assert np.allclose(X, a.y0)
    assert np.allclose(Y, -a.x0)
    assert np.allclose(Z, a.z0)

    q1 = Quaternion(
        axis=[np.random.rand(), np.random.rand(), np.random.rand()],
        degrees=np.random.rand()*90,
    )
    q2 = q1.inverse
    q = q1 * q2

    X, Y, Z = a.get_new_frame_from_quaternion(q.w, q.x, q.y, q.z)
    assert np.allclose(X, a.x0)
    assert np.allclose(Y, a.y0)
    assert np.allclose(Z, a.z0)


def test_multi_turn():
    a = Actuator()

    for x in range(0, 1080, 10):
        q11, q12, q13 = a.get_angles_from_vector([0, 0, 1], x)
        assert isclose(q11, x, abs_tol=1e-9)
        assert isclose(q12, x, abs_tol=1e-9)
        assert isclose(q13, x, abs_tol=1e-9)

    for x in range(1080, 0, -10):
        q11, q12, q13 = a.get_angles_from_vector([0, 0, 1], x)
        assert isclose(q11, x, abs_tol=1e-9)
        assert isclose(q12, x, abs_tol=1e-9)
        assert isclose(q13, x, abs_tol=1e-9)


def test_domain_error():
    a = Actuator()

    with pytest.raises(ValueError, match=r"math domain error"):
        X, Y, Z = a.get_angles_from_vector([1, 0, 0], np.random.rand() * 90)


def test_transform():
    a = Actuator()

    vo = np.random.rand(3)
    q = a.find_quaternion_transform(vo, vo)
    assert q == Quaternion(1, 0, 0, 0)

    vo = [1, 0, 0]
    alpha = np.random.rand() * 90
    r = R.from_euler('z', alpha, degrees=True)
    vt = np.dot(r.as_matrix(), vo)
    q = a.find_quaternion_transform(vo, vt)
    assert q == Quaternion(axis=[0, 0, 1], degrees=alpha)


def test_get_angles_from_quaternion():
    a = Actuator()

    x = np.random.rand() * 180 - 90
    q = Quaternion(axis=[0, 0, 1], degrees=x)
    q11, q12, q13 = a.get_angles_from_quaternion(q.w, q.x, q.y, q.z)

    assert isclose(q11, x)
    assert isclose(q12, x)
    assert isclose(q13, x)

    a = Actuator(R0=rot('z', 60))

    x = np.random.rand() * 180 - 90
    q = Quaternion(axis=[0, 0, 1], degrees=x)
    q11, q12, q13 = a.get_angles_from_quaternion(q.w, q.x, q.y, q.z)

    assert isclose(q11, x - 60)
    assert isclose(q12, x - 60)
    assert isclose(q13, x - 60)

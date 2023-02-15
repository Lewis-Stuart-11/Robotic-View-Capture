from math import sqrt
import geometry_msgs
import numpy as np

class QuartonianHandler():

    def Length(self, vector):
        return sqrt((vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z))

    def Normalize(self, vector):
        vectorLength = self.Length(vector)

        if vectorLength <= 0:
            return vector

        vector.x /= vectorLength
        vector.y /= vectorLength
        vector.z /= vectorLength

        return vector

    def Cross(self, vector1, vector2):
        result = geometry_msgs.msg.Point()

        result.x = (vector1.y * vector2.z) - (vector1.z * vector2.y)
        result.y = (vector1.z * vector2.x) - (vector1.x * vector2.z)
        result.z = (vector1.x * vector2.y) - (vector1.y * vector2.x)
        return result

    def SubtractVectors(self, vector1, vector2):
        result = geometry_msgs.msg.Point()

        result.x = vector1.x - vector2.x
        result.y = vector1.y - vector2.y
        result.z = vector1.z - vector2.z

        return result

    def AddVectors(self, vector1, vector2):
        result = geometry_msgs.msg.Point()

        result.x = vector1.x + vector2.x
        result.y = vector1.y + vector2.y
        result.z = vector1.z + vector2.z

        return result

    def QuaternionLookRotation(self, forward, up):
        #forward = self.Normalize(forward)
    
        vector = self.Normalize(forward)
        vector2 = self.Normalize(self.Cross(up, vector))
        vector3 = self.Cross(vector, vector2)
        m00 = vector2.x
        m01 = vector2.y
        m02 = vector2.z
        m10 = vector3.x
        m11 = vector3.y
        m12 = vector3.z
        m20 = vector.x
        m21 = vector.y
        m22 = vector.z
    
        num8 = (m00 + m11) + m22
        quaternion = geometry_msgs.msg.Quaternion()
        
        if num8 > 0.0:
            num = sqrt(num8 + 1.0)
            quaternion.w = num * 0.5
            num = 0.5 / num
            quaternion.x = (m12 - m21) * num
            quaternion.y = (m20 - m02) * num
            quaternion.z = (m01 - m10) * num
            return quaternion
        
        if ((m00 >= m11) and (m00 >= m22)):
            num7 = sqrt(((1.0 + m00) - m11) - m22)
            num4 = 0.5 / num7
            quaternion.x = 0.5 * num7
            quaternion.y = (m01 + m10) * num4
            quaternion.z = (m02 + m20) * num4
            quaternion.w = (m12 - m21) * num4
            return quaternion
        
        if m11 > m22:
            num6 = sqrt(((1.0 + m11) - m00) - m22)
            num3 = 0.5 / num6
            quaternion.x = (m10+ m01) * num3
            quaternion.y = 0.5 * num6
            quaternion.z = (m21 + m12) * num3
            quaternion.w = (m20 - m02) * num3
            return quaternion; 
        
        num5 = sqrt(((1.0 + m22) - m00) - m11);
        num2 = 0.5 / num5
        quaternion.x = (m20 + m02) * num2
        quaternion.y = (m21 + m12) * num2
        quaternion.z = 0.5 * num5
        quaternion.w = (m01 - m10) * num2
        return quaternion
    

    def convert_quart_to_rotation_matrix(self, q):

        qx = float(q[0])
        qy = float(q[1])
        qz = float(q[2])
        qw = float(q[3])

        sqw = qw*qw
        sqx = qx*qx
        sqy = qy*qy
        sqz = qz*qz

        invs = 1 / (sqx + sqy + sqz + sqw)
        m00 = ( sqx - sqy - sqz + sqw)*invs 
        m11 = (-sqx + sqy - sqz + sqw)*invs 
        m22 = (-sqx - sqy + sqz + sqw)*invs 
            
        tmp1 = qx*qy
        tmp2 = qz*qw
        m10 = 2.0 * (tmp1 + tmp2)*invs 
        m01 = 2.0 * (tmp1 - tmp2)*invs 
            
        tmp1 = qx*qz
        tmp2 = qy*qw
        m20 = 2.0 * (tmp1 - tmp2)*invs 
        m02 = 2.0 * (tmp1 + tmp2)*invs 
        tmp1 = qy*qz
        tmp2 = qx*qw
        m21 = 2.0 * (tmp1 + tmp2)*invs 
        m12 = 2.0 * (tmp1 - tmp2)*invs 
        
        r = np.array([
            [m00, m01, m02],
            [m10, m11, m12],
            [m20, m21, m22],
        ])

        return r


    def get_translation_matrix_from_transform(self, trans, rot):
        
        transformation_matrix = np.identity(4)

        rotation_matrix = self.convert_quart_to_rotation_matrix(rot)

        for i in range(3):
            for j in range(3):
                transformation_matrix[i][j] = rotation_matrix[i][j]

            transformation_matrix[i][3] = trans[i]
        
        return transformation_matrix

"""
def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    #Return quaternion from Euler angles and axis sequence.

    #ai, aj, ak : Euler's roll, pitch and yaw angles
    #axes : One of 24 axis sequences as string or encoded tuple

    #>>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
    #>>> numpy.allclose(q, [0.435953, 0.310622, -0.718287, 0.444435])
    #True

    _AXES2TUPLE = {
        'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
        'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
        'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
        'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
        'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
        'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
        'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
        'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

    _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    # axis sequences for Euler angles
    _NEXT_AXIS = [1, 2, 0, 1]

    i = firstaxis + 1
    j = _NEXT_AXIS[i+parity-1] + 1
    k = _NEXT_AXIS[i-parity] + 1

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = cos(ai)
    si = sin(ai)
    cj = cos(aj)
    sj = sin(aj)
    ck = cos(ak)
    sk = sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    if repetition:
        q[0] = cj*(cc - ss)
        q[i] = cj*(cs + sc)
        q[j] = sj*(cc + ss)
        q[k] = sj*(cs - sc)
    else:
        q[0] = cj*cc + sj*ss
        q[i] = cj*sc - sj*cs
        q[j] = cj*ss + sj*cc
        q[k] = cj*cs - sj*sc
    if parity:
        q[j] *= -1.0

    return q

def normalise_quartonian(q):
    q_dot_product = q.x**2 + q.y**2 + q.z**2 + q.w**2
    q_length = sqrt(q_dot_product)

    q.x /= q_length
    q.y /= q_length
    q.z /= q_length
    q.w /= q_length

    return q

def axis_angle_to_quartonian(axis, angle):
    q = geometry_msgs.msg.Quaternion()

    s = sin(angle/2)

    q.x = axis.x * s
    q.y = axis.y * s
    q.z = axis.z * s
    q.w = cos(angle/2)

    return normalise_quartonian(q)


def dot_product_angle_rads(vector1, vector2):
    vector_length_1 = sqrt(vector1.x**2 + vector1.y**2 + vector1.z**2)
    vector_length_2 = sqrt(vector2.x**2 + vector2.y**2 + vector2.z**2)

    dot_product = (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z)  

    return acos(dot_product/(vector_length_1*vector_length_2)) 


def cross_product_between_two_vectors(vector1, vector2):
    new_vector = geometry_msgs.msg.Point()

    new_vector.x = (vector1.y * vector2.z) - (vector1.z * vector2.y)
    new_vector.y = (vector1.z * vector2.x) - (vector1.x * vector2.z)
    new_vector.z = (vector1.x * vector2.y) - (vector1.y * vector2.x)

    return new_vector

def vects_to_rotation_matrix(p1, p2):
    v = geometry_msgs.msg.Point()

    v.x = p1.x - p2.x
    v.y = p1.y - p2.y
    v.z = p1.z - p2.z

    v_length = sqrt(v.x**2 + v.y**2 + v.z**2)

    r = geometry_msgs.msg.Point()

    v.x = v.x / v_length
    v.y = v.y / v_length
    v.z = v.z / v_length

    from math import atan2

    si = atan2(v.z, v.x)
    theta = atan2()

"""
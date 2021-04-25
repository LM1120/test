#include <cmath>
#include <vector>
#include <iostream>

#define FREQ_CAM 30
#define FREQ_IMU 100
#define T0_CAM 0.0
#define T0_IMU 0.42
#define DURATION 5
#define N 99
// Quaternion
struct Q {
    double x;
    double y;
    double z;
    double w;
    Q() : x(0), y(0), z(0), w(1) {};
    Q(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {};
};

Q NLerp(const Q& q1, const Q& q2, const double& t);
Q Normalize(const Q& q);
double GenRandom();
void GenTimestampQuaternion(std::vector<double>& cam_t, std::vector<double>& imu_t, std::vector<Q>& imu_q);
bool CalImuPose(const double& tc, const std::vector<double>& imu_t, const std::vector<Q>& imu_q, Q& qc);

double GenRandom() {
    double res = rand() % (N + 1) / double(N + 1);
    return res;
}

Q NLerp(const Q& q1, const Q& q2, const double& t) {
    Q res;
    res.x = (1 - t) * q1.x + t * q2.x;
    res.y = (1 - t) * q1.y + t * q2.y;
    res.z = (1 - t) * q1.z + t * q2.z;
    res.w = (1 - t) * q1.w + t * q2.w;
    double norm = sqrt((res.x * res.x + res.y * res.y + res.z * res.z + res.w * res.w));
    res.x /= norm;
    res.y /= norm;
    res.z /= norm;
    res.w /= norm;
    return res;
}

Q Normalize(const Q& q) {
    Q res;
    double norm = sqrt((q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w));
    res.x = q.x / norm;
    res.y = q.y / norm;
    res.z = q.z / norm;
    res.w = q.w / norm;
    return res;
}

void GenTimestampQuaternion(std::vector<double>& cam_t, std::vector<double>& imu_t, std::vector<Q>& imu_q) {
    double cam_t_i = T0_CAM;
    double imu_t_i = T0_IMU;

    cam_t.push_back(cam_t_i);
    imu_t.push_back(imu_t_i);

    Q q_t_i(0, 0, 0, 1);
    imu_q.push_back(q_t_i);

    int i = 0;
    while (cam_t_i <= 5.0) {
        i++;
        cam_t_i = T0_CAM + i * (1.0 / FREQ_CAM) + 0.5 * (1.0 / FREQ_CAM) * GenRandom();
        cam_t.push_back(cam_t_i);
    }
    i = 0;
    while (imu_t_i <= 5.0) {
        i++;
        imu_t_i = T0_IMU + i * (1.0 / FREQ_IMU) + 0.5 * (1.0 / FREQ_IMU) * GenRandom();
        imu_t.push_back(imu_t_i);

        q_t_i.x = imu_q[i - 1].x + 0.1 * GenRandom();
        q_t_i.y = imu_q[i - 1].y + 0.1 * GenRandom();
        q_t_i.z = imu_q[i - 1].z + 0.1 * GenRandom();
        q_t_i.w = imu_q[i - 1].w + 0.1 * GenRandom();
        imu_q.push_back(Normalize(q_t_i));
    }
}

bool CalImuPose(const double& tc, const std::vector<double>& imu_t, const std::vector<Q>& imu_q, Q& qc) {
    if (tc < imu_t.front() || tc > imu_t.back()) return false;
    int i = 0;
    int left, right;
    left = right = 0;
    double t_dev = imu_t.back() - imu_t.front();
    for (; i < imu_t.size(); i++) {
        if (imu_t[i] - tc > 0) {
            if (imu_t[i] - tc < t_dev) {
                t_dev = imu_t[i] - tc;
                left = i;
            }
        }
        else break;
    }
    right = left + 1;
    qc = NLerp(imu_q[left], imu_q[right], tc - imu_t[left]);
    return true;
}

int main() {
    //  do something
    std::vector<double> cam_t;
    std::vector<double> imu_t;
    std::vector<Q> imu_q;

    GenTimestampQuaternion(cam_t, imu_t, imu_q);

    //  Caculate imu's pose (qc) for each timestamp (tc) in cam_t
    Q qc;
    for (const auto& tc : cam_t) {
        if (CalImuPose(tc, imu_t, imu_q, qc)) {
            std::cout << "qc.x: " << qc.x << std::endl;
            std::cout << "qc.y: " << qc.y << std::endl;
            std::cout << "qc.z: " << qc.z << std::endl;
            std::cout << "qc.w: " << qc.w << std::endl;
        }
    }

    return 0;
}



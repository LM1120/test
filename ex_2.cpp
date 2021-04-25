#include <iostream>
#include <mutex>
#include <vector>
#include <thread>
#include <queue>
#include <cmath>
#include <condition_variable>
#include <Windows.h>

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

constexpr int MAX_SIZE = 10;
long buffer[MAX_SIZE];
std::queue<double> tc_buffer;
std::queue<Q> qc_buffer; 

std::vector<double> cam_t;
std::vector<double> imu_t;
std::vector<Q> imu_q;

int in = 0;
int out = 0;
int counter = 0;
long nextp = 0;

std::mutex print_mutex;
std::mutex condition_mutex;

std::condition_variable not_full;
std::condition_variable not_empty;

int producer_time = 1;
int consumer_time = 1;

Q NLerp(const Q& q1, const Q& q2, const double& t);
Q Normalize(const Q& q);
double GenRandom();
void GenTimestampQuaternion(std::vector<double>& cam_t, std::vector<double>& imu_t, std::vector<Q>& imu_q);
bool CalImuPose(const double& tc, const std::vector<double>& imu_t, const std::vector<Q>& imu_q, Q& qc);

void producer() {
	while (1) {
		Sleep(producer_time);
		{
			std::lock_guard<std::mutex> lock(print_mutex);
			std::cout << "生产者线程: 队列位置 " << in << " 产品编号: " << nextp + 1 << std::endl;
		}
		std::unique_lock<std::mutex> lk(condition_mutex);

		while (counter == MAX_SIZE) {
			{
				std::lock_guard<std::mutex> lock(print_mutex);
				std::cout << "等待消费者线程\n";
			}
			not_full.wait(lk);
		}

        nextp = (nextp + 1) % (cam_t.size() - 1);
        buffer[in] = nextp;
        Q qc;
        while (CalImuPose(cam_t[nextp], imu_t, imu_q, qc) == false) {
            nextp++;
            buffer[in] = nextp;
        }

        tc_buffer.push(cam_t[nextp]);
        qc_buffer.push(qc);

		in = (in + 1) % MAX_SIZE;
		counter++;
		if (counter >= 1) {
			not_empty.notify_all();
		}
		lk.unlock();
	}
}

void consumer() {
	while (1) {
		std::unique_lock<std::mutex> lk(condition_mutex);
		while (counter == 0) {
			{
				std::lock_guard<std::mutex> lock(print_mutex);
				std::cout << "等待生产者线程\n";
			}
			not_empty.wait(lk);
		}

		long nextc = buffer[out];
		int old_out = out;
		out = (out + 1) % MAX_SIZE;
        {
            std::lock_guard<std::mutex> lock(print_mutex);
            std::cout << "tc: " << tc_buffer.front() << "\n";
            std::cout << "qc.x: " << qc_buffer.front().x << "\n";
            std::cout << "qc.y: " << qc_buffer.front().y << "\n";
            std::cout << "qc.z: " << qc_buffer.front().z << "\n";
            std::cout << "qc.w: " << qc_buffer.front().w << "\n";
        }
        qc_buffer.pop();
        tc_buffer.pop();
		counter--;

		if (counter < MAX_SIZE) {
			not_full.notify_all();
		}
		lk.unlock();
		Sleep(consumer_time);
		{
			std::lock_guard<std::mutex> lock(print_mutex);
			std::cout << "消费者线程: 队列位置 " << old_out << " 产品编号: " << nextc << "\n";
		}
	}
}

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
    GenTimestampQuaternion(cam_t, imu_t, imu_q);
	std::thread t1(producer);
	std::thread t2(consumer);
	t2.join();
	t1.join();
	return 0;
}

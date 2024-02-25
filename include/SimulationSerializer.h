#pragma once

#include <string>
#include <fstream>
#include <Eigen/Dense>
#include <json/json.h>

class RigidBodySystem;

// A serialization class for exporting rigid body trajectories
//  at each frame.
class SimulationSerializer
{
public:

    static const Eigen::Vector3f sInvalidPos;

    SimulationSerializer(const std::string& _baseName);

    ~SimulationSerializer();

    void setRigidBodySystem(RigidBodySystem* _rigidBodySystem) { m_rigidBodySystem = _rigidBodySystem; }

    void setMouseSpringPoints(const Eigen::Vector3f& p0, const Eigen::Vector3f& p1) { m_p0 = p0; m_p1 = p1; }

    void setAppliedForcePoints(const Eigen::Vector3f& f0, const Eigen::Vector3f& f1) { m_f0 = f0; m_f1 = f1; }

    void start();

    void writeFrame();

    void end();

private:

    RigidBodySystem* m_rigidBodySystem;
    std::string m_baseName;
    std::ofstream m_fs;
    Json::Value m_root;
    Eigen::Vector3f m_p0, m_p1;
    Eigen::Vector3f m_f0, m_f1;
    int m_index;
};

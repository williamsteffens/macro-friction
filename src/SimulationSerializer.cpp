#include "SimulationSerializer.h"
#include "RigidBodySystem.h"
#include "RigidBody.h"
#include "MeshAssets.h"

#include <cassert>
#include <chrono>
#include <ctime>

const Eigen::Vector3f SimulationSerializer::sInvalidPos = Eigen::Vector3f(-1e5f, -1e5f, -1e5f);

namespace
{
    static inline void writeAndCloseStream(std::ofstream& fs, Json::Value& root)
    {
        Json::StreamWriterBuilder builder;
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(root, &fs);
        fs.flush();
        fs.close();
        root.clear();
    }

    static inline void writeHeader(std::ofstream& fs, RigidBodySystem* rigidBodySystem, Json::Value& root)
    {
        root.clear();
        root["num_bodies"] = rigidBodySystem->getBodies().size();

        int count = 0;
        Json::Value bodies;
        for(RigidBody* body : rigidBodySystem->getBodies())
        {
            Json::Value body_val;
            body_val["mesh"] = body->mesh->name.c_str();
            body_val["id"] = count;
            bodies[count] = body_val;
            ++count;
        }

        if( count > 0 )
        {
            root["bodies"] = bodies;
        }

    }

    static inline void writeTransforms(std::ofstream& fs, const int frameIndex, RigidBodySystem* rigidBodySystem, Json::Value& root)
    {
        Json::StreamWriterBuilder builder;

        if( rigidBodySystem->getBodies().size() < 1)
            return;

        Json::Value transforms;
        int count = 0;
        for(RigidBody* body : rigidBodySystem->getBodies())
        {
            Json::Value tm_val;
            tm_val["position"][0] = body->x.x();
            tm_val["position"][1] = body->x.y();
            tm_val["position"][2] = body->x.z();
            tm_val["orientation"][0] = body->q.w();
            tm_val["orientation"][1] = body->q.x();
            tm_val["orientation"][2] = body->q.y();
            tm_val["orientation"][3] = body->q.z();
            transforms[count] = tm_val;
            ++count;
        }

        root["transforms"][frameIndex] = transforms;
    }

    static inline void writeMouseSpring(std::ofstream& fs, const int frameIndex, const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, Json::Value& root)
    {
        Json::StreamWriterBuilder builder;

        Json::Value mouseSpring;
        mouseSpring["p0"][0] = p0.x();
        mouseSpring["p0"][1] = p0.y();
        mouseSpring["p0"][2] = p0.z();

        mouseSpring["p1"][0] = p1.x();
        mouseSpring["p1"][1] = p1.y();
        mouseSpring["p1"][2] = p1.z();

        root["mouseSpring"][frameIndex] = mouseSpring;
    }

    static inline void writeAppliedForce(std::ofstream& fs, const int frameIndex, const Eigen::Vector3f& f0, const Eigen::Vector3f& f1, Json::Value& root)
    {
        Json::StreamWriterBuilder builder;

        Json::Value applForce;
        applForce["f0"][0] = f0.x();
        applForce["f0"][1] = f0.y();
        applForce["f0"][2] = f0.z();

        applForce["f1"][0] = f1.x();
        applForce["f1"][1] = f1.y();
        applForce["f1"][2] = f1.z();

        root["applForce"][frameIndex] = applForce;
    }
}


SimulationSerializer::SimulationSerializer(const std::string& _baseName) :
    m_baseName(_baseName), m_rigidBodySystem(nullptr), m_fs(), m_index(0), m_p0(sInvalidPos), m_p1(sInvalidPos), m_f0(sInvalidPos), m_f1(sInvalidPos)
{

}

SimulationSerializer::~SimulationSerializer()
{
    writeAndCloseStream(m_fs, m_root);
}

void SimulationSerializer::start()
{
    if( m_fs.is_open() )
        writeAndCloseStream(m_fs, m_root);

    // Determine year,month,day
    auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    struct tm *localtime = std::localtime(&now_c);

    // Build filename basename and the current date and time
    std::string filename = m_baseName;
    filename.append("_");
    filename.append(std::to_string(localtime->tm_year));
    filename.append(std::to_string(localtime->tm_mon));
    filename.append(std::to_string(localtime->tm_mday));
    filename.append("_");
    filename.append(std::to_string(localtime->tm_hour));
    filename.append(std::to_string(localtime->tm_min));
    filename.append(std::to_string(localtime->tm_sec));
    filename.append(".json");

    m_index = 0;

    // Open the filestream using the new filename
    m_fs.open(filename);
    writeHeader(m_fs, m_rigidBodySystem, m_root);
}

void SimulationSerializer::end()
{
    // Close existing stream
    writeAndCloseStream(m_fs, m_root);
}

void SimulationSerializer::writeFrame()
{
    if(m_rigidBodySystem == nullptr) return;

    if( !m_fs.is_open() )
    {
        start();
    }

    writeTransforms(m_fs, m_index, m_rigidBodySystem, m_root);
    writeMouseSpring(m_fs, m_index, m_p0, m_p1, m_root);
    writeAppliedForce(m_fs, m_index, m_f0, m_f1, m_root);
    m_index++;
}

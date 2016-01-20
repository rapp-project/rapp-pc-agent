#include "ros/ros.h"
#include "sys/types.h"

#include "rapp_core_agent_msgs/Say.h"
#include "rapp_core_agent_msgs/Record.h"
#include "rapp_core_agent_msgs/PlayAudio.h"

class CoreAgent
{
  private:
    ros::ServiceServer serverSay_;
    ros::ServiceServer serverRecord_;
    ros::ServiceServer serverPlayAudio_;

    ros::ServiceClient clientAudioPlay_;
    ros::ServiceClient clientAudioRecord_;

    std::string srvSayUrl_;
    std::string srvRecordUrl_;
    std::string srvPlayAudioUrl_;

    std::string audioPlaySrvUrl_;
    std::string audioRecordSrvUrl_;

  protected:
    ros::NodeHandle nh_;
    uint16_t state_;


  public:
    CoreAgent(ros::NodeHandle nh) :
      nh_(nh)
    {
      ROS_INFO("Core Agent Initiated");
      if(stateInit())
      {
        state_ = 1;
      }
    }

    ~CoreAgent()
    {
    }

    void loadParameters(void)
    {
      int param;
      nh_.param<std::string>("service_url/say", srvSayUrl_,
        "/rapp_core_agent/say");
      nh_.param<std::string>("service_url/play_audio", srvPlayAudioUrl_,
        "/rapp_core_agent/play_audio");
      nh_.param<std::string>("service_url/record", srvRecordUrl_,
        "/rapp_core_agent/record");

      nh_.param<std::string>("audio_record_server/record_audio_srv_url",
        audioRecordSrvUrl_, "/rapp_core_agent/record_audio");
      nh_.param<std::string>("audio_play_server/play_audio_srv_url",
        audioPlaySrvUrl_, "/rapp_core_agent/play_audio");
    }

    bool stateInit(void)
    {
      loadParameters();
      ros::service::waitForService(audioPlaySrvUrl_);
      ros::service::waitForService(audioRecordSrvUrl_);

      clientAudioRecord_ = nh_.serviceClient<rapp_core_agent_msgs::Record>(audioRecordSrvUrl_);
      clientAudioPlay_ = nh_.serviceClient<rapp_core_agent_msgs::PlayAudio>(audioPlaySrvUrl_);

      serverPlayAudio_ = nh_.advertiseService(srvSayUrl_,
        &CoreAgent::srvSayCallback, this);
      serverPlayAudio_ = nh_.advertiseService(srvPlayAudioUrl_,
        &CoreAgent::srvPlayAudioCallback, this);
      serverRecord_ = nh_.advertiseService(srvRecordUrl_,
        &CoreAgent::srvRecordCallback, this);

      return true;
    }

    bool srvPlayAudioCallback(
      rapp_core_agent_msgs::PlayAudio::Request& req,
      rapp_core_agent_msgs::PlayAudio::Response& res)
    {
      rapp_core_agent_msgs::PlayAudio srv;
      srv.request.audioFile = req.audioFile;

      clientAudioPlay_.call(srv);

      res.error = srv.response.error;
      return true;
    }

    bool srvRecordCallback(
      rapp_core_agent_msgs::Record::Request& req,
      rapp_core_agent_msgs::Record::Response& res)
    {
      rapp_core_agent_msgs::Record srv;
      srv.request.recordTime = req.recordTime;
      srv.request.filename = req.filename;

      clientAudioRecord_.call(srv);

      res.error = srv.response.error;
      res.recFileDest = srv.response.recFileDest;
      return true;
    }

    bool srvSayCallback(
      rapp_core_agent_msgs::Say::Request& req,
      rapp_core_agent_msgs::Say::Response& res)
    {
      res.error = "Not yet supported";
      return true;
    }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "core_agent");
  //ROS_INFO("Core Agent node.");
  // Initiate nodeHandle under node's namespace
  ros::NodeHandle nh("~");
  // Pass nodeHandle reference to CoreAgent class.
  CoreAgent coreAgent(nh);

  ros::spin();

  return 0;
}

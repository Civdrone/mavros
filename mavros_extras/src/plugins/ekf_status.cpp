/**
 * @brief EKF Status plugin
 * @file ekf_status.cpp
 * @author Daniel Kabzon <daniel@civdrone.com>
 *
 * @addtogroup plugin
 * @{
 */


#include <mavros/mavros_plugin.h>
#include <mavros_msgs/EkfStatus.h>
#include <mavros/utils.h>


namespace mavros {
namespace extra_plugins {

/**
 * @brief System status plugin.
 *
 * Required by all plugins.
 */
class EkfStatusPlugin : public plugin::PluginBase
{
    public:
        EkfStatusPlugin() : PluginBase(),
            nh("~")
        { }

        void initialize(UAS &uas_) override
        {
            //ROS_INFO("EkfStatusPlugin - initialize");
            PluginBase::initialize(uas_);

            ekf_status_pub = nh.advertise<mavros_msgs::EkfStatus>("ekf_status", 10);
        }

        Subscriptions get_subscriptions() override {
            return {
                make_handler(&EkfStatusPlugin::handle_ekf_status),
            };
        }

    private:
        ros::NodeHandle nh;

        
        ros::Publisher ekf_status_pub;

        void handle_ekf_status(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::EKF_STATUS_REPORT &status)
        {
            using EKFF = mavlink::ardupilotmega::EKF_STATUS_FLAGS;

            auto ekf_status_msg = boost::make_shared<mavros_msgs::EkfStatus>();
            ekf_status_msg->header.stamp = ros::Time::now();

            ekf_status_msg->attitude_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::ATTITUDE));
            ekf_status_msg->velocity_horiz_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::VELOCITY_HORIZ));
            ekf_status_msg->velocity_vert_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::VELOCITY_VERT));
            ekf_status_msg->pos_horiz_rel_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::POS_HORIZ_REL));
            ekf_status_msg->pos_horiz_abs_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::POS_HORIZ_REL));
            ekf_status_msg->pos_vert_abs_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::POS_VERT_ABS));
            ekf_status_msg->pos_vert_agl_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::POS_VERT_AGL));
            ekf_status_msg->const_pos_mode_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::CONST_POS_MODE));
            ekf_status_msg->pred_pos_horiz_rel_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::PRED_POS_HORIZ_REL));
            ekf_status_msg->pred_pos_horiz_abs_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::PRED_POS_HORIZ_ABS));
            ekf_status_msg->uninitialized_status_flag = !!(status.flags & mavros::utils::enum_value(EKFF::UNINITIALIZED));
            ekf_status_msg->compass_variance = status.compass_variance;
            ekf_status_msg->pos_horiz_variance = status.pos_horiz_variance;
            ekf_status_msg->pos_vert_variance = status.pos_vert_variance;
            ekf_status_msg->terrain_alt_variance = status.terrain_alt_variance;
            ekf_status_msg->velocity_variance = status.velocity_variance;

            
            // [[[end]]] (checksum: da59238f4d4337aeb395f7205db08237)

            ekf_status_pub.publish(ekf_status_msg);
        }
    };

	
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::EkfStatusPlugin, mavros::plugin::PluginBase)

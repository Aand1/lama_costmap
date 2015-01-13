/*
 * Localizing Jockey based on local costmap (costmap position is relative to
 * the sensor but orientation is absolute).
 *
 * The role of this jockey is to get the dissimilarity of the PlaceProfile
 * descriptors of all vertices with the current PlaceProfile.
 * The action is done when the dissimilarities are computed.
 * Implemented actions:
 * - GET_VERTEX_DESCRIPTOR: return the PlaceProfile and the computed Crossing
 * - GET_SIMILARITY: return the dissimilarity based on PlaceProfile
 *
 * Interaction with the map (created by this jockey):
 * - [Getter][/][Setter], message type, interface default name
 * - Getter/Setter: PlaceProfile, jockey_name + "_place_profile"
 * - Setter: Crossing, jockey_name + "_crossing"
 *
 * Interaction with the map (created by other jockeys):
 * - [Getter][/][Setter], message type, interface default name
 *
 * Subscribers (other than map-related):
 * - message type, topic default name, description
 * - nav_msgs/OccupancyGrid, "~/local_costmap", local cost map which orientation is global
 *
 * Publishers (other than map-related):
 * - message type, topic default name, description
 *
 * Services used (other than map-related):
 * - service type, server default name, description
 * - polygon_matcher::PolygonDissimilarity, "~/dissimilarity_server", used to
 *    compare all known places (as polygons) with the current place
 *
 * Parameters:
 * - name, type, default name, description
 * - costmap_interface_name, String, jockey_name + "_place_profile", name of the map interface for place profiles.
 * - crossing_interface_name, String, jockey_name + "_crossing", name of the map interface for crossing.
 * - dissimilarity_server_name, String, "dissimilarity_server", name of the dissimilarity server.
 */

#ifndef LJ_COSTMAP_JOCKEY_H
#define LJ_COSTMAP_JOCKEY_H

#include <ros/ros.h>

#include <lama_common/place_profile_conversions.h>
#include <lama_interfaces/ActOnMap.h>
#include <lama_interfaces/AddInterface.h>
#include <lama_msgs/DescriptorLink.h>
#include <lama_jockeys/localizing_jockey.h>
#include <lama_msgs/GetPlaceProfile.h>
#include <lama_msgs/SetPlaceProfile.h>
#include <lama_msgs/SetCrossing.h>
#include <polygon_matcher/PolygonDissimilarity.h>

#include <crossing_detector/costmap_crossing_detector.h>

namespace lj_costmap {

class Jockey : public lama_jockeys::LocalizingJockey
{
  public:

    Jockey(std::string name, const double frontier_width, const double max_frontier_angle=0.785);

    virtual void onGetVertexDescriptor();
    virtual void onGetEdgesDescriptors();
    virtual void onLocalizeInVertex();
    virtual void onLocalizeEdge();
    virtual void onGetDissimilarity();
    // virtual void onInterrupt();
    // virtual void onContinue();

    void setDissimilarityServerName(std::string name) {dissimilarity_server_name_ = name;}

  private:

    void initMapPlaceProfileInterface();
    void initMapCrossingInterface();
    void getData();
    void handleMap(const nav_msgs::OccupancyGridConstPtr& msg);

    lama_msgs::DescriptorLink placeProfileDescriptorLink(const int32_t id);
    lama_msgs::DescriptorLink crossingDescriptorLink(const int32_t id);

    bool data_received_;

    // Reception and storage of OccupancyGrid and PlaceProfile.
    ros::Subscriber costmap_handler_;
    nav_msgs::OccupancyGrid map_;
    lama_msgs::PlaceProfile profile_;

    // Map interface for PlaceProfile and Crossing descriptors.
    std::string place_profile_interface_name_;
    ros::ServiceClient place_profile_getter_;
    ros::ServiceClient place_profile_setter_;
    std::string crossing_interface_name_;
    ros::ServiceClient crossing_setter_;

    // Dissimilarity server.
    std::string dissimilarity_server_name_;
    ros::ServiceClient dissimilarity_server_;

    crossing_detector::CostmapCrossingDetector crossing_detector_;
};


} // namespace lj_costmap

#endif // LJ_COSTMAP_JOCKEY_H

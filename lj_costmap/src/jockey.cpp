#include <lj_costmap/jockey.h>

namespace lj_costmap {

Jockey::Jockey(std::string name, const double frontier_width, const double max_frontier_angle) :
  LocalizingJockey(name),
  data_received_(false),
  place_profile_interface_name_(name + "_place_profile"),
  crossing_interface_name_(name + "_crossing"),
  dissimilarity_server_name_("dissimilarity_server"),
  crossing_detector_(frontier_width, max_frontier_angle)
{
  private_nh_.getParam("place_profile_interface_name", place_profile_interface_name_);
  private_nh_.getParam("crossing_interface_name", crossing_interface_name_);
  private_nh_.getParam("dissimilarity_server_name", dissimilarity_server_name_);

  initMapPlaceProfileInterface();
  initMapCrossingInterface();

  // Initialize the client for the dissimilarity server.
  dissimilarity_server_ = nh_.serviceClient<polygon_matcher::PolygonDissimilarity>(dissimilarity_server_name_);
}

/* Create the getter and setter services for LaserScan descriptors.
 */
void Jockey::initMapPlaceProfileInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  client.waitForExistence();
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = place_profile_interface_name_;
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::SERIALIZED;
  srv.request.get_service_message = "lama_msgs/GetPlaceProfile";
  srv.request.set_service_message = "lama_msgs/SetPlaceProfile";
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to create the Lama interface %s", place_profile_interface_name_.c_str());
    return;
  }
  // Initialize the clients for the getter and setter services (interface to map).
  place_profile_getter_ = nh_.serviceClient<lama_msgs::GetPlaceProfile>(srv.response.get_service_name);
  place_profile_getter_.waitForExistence();
  place_profile_setter_ = nh_.serviceClient<lama_msgs::SetPlaceProfile>(srv.response.set_service_name);
  place_profile_setter_.waitForExistence();
}

/* Create the setter services for Crossing descriptors.
 */
void Jockey::initMapCrossingInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  client.waitForExistence();
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = crossing_interface_name_;
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::CLEARTEXT;
  srv.request.get_service_message = "lama_msgs/GetCrossing";
  srv.request.set_service_message = "lama_msgs/SetCrossing";
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to create the Lama interface %s", crossing_interface_name_.c_str());
  }
  // Initialize the client for the setter service (interface to map).
  crossing_setter_ = nh_.serviceClient<lama_msgs::SetCrossing>(srv.response.set_service_name);
  crossing_setter_.waitForExistence();
}

/* Start the subscriber, wait for an OccupancyGrid and exit upon reception.
 */
void Jockey::getData()
{
  costmap_handler_ = nh_.subscribe<nav_msgs::OccupancyGrid>("local_costmap", 1, &Jockey::handleMap, this);

  ros::Rate r(100);
  while (ros::ok())
  {
    if (data_received_)
    {
      // Stop the subscribers (may be superfluous).
      costmap_handler_.shutdown();
      data_received_ = false;
      break;
    }
    ros::spinOnce();
    r.sleep();
  }
}

/* Callback for OccupancyGrid subscriber, receive a message and store it.
 */
void Jockey::handleMap(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map_ = *msg;
  profile_ = lama_common::costmapToPlaceProfile(map_);
  data_received_ = true;
}

/* Return the vertex descriptors associated with the current robot position through result_.
 *
 * The descriptors are a LaserScan and a Crossing.
 */
// TODO: Discuss with Karel the exact role of onGetVertexDescriptor
// TODO: in particular: should it save something in the database? This jockey
// is not a learning jockey.
void Jockey::onGetVertexDescriptor()
{
  if (server_.isPreemptRequested() && !ros::ok())
  {
    ROS_INFO("%s: Preempted", jockey_name_.c_str());
    // set the action state to preempted
    server_.setPreempted();
    return;
  }

  getData();

  // Add the PlaceProfile to the descriptor list.
  lama_msgs::SetPlaceProfile profile_setter_srv;
  profile_setter_srv.request.descriptor = profile_;
  if (!place_profile_setter_.call(profile_setter_srv))
  {
    ROS_ERROR("Failed to add PlaceProfile to the map");
    server_.setAborted();
    return;
  }
  ROS_INFO("Added PlaceProfile with id %d", profile_setter_srv.response.id); // DEBUG
  result_.descriptor_links.push_back(placeProfileDescriptorLink(profile_setter_srv.response.id));

  // Add the Crossing to the descriptor list.
  lama_msgs::SetCrossing crossing_setter_srv;
  lama_msgs::Crossing crossing = crossing_detector_.crossingDescriptor(map_);
  crossing_setter_srv.request.descriptor = crossing;
  if (!crossing_setter_.call(crossing_setter_srv))
  {
    ROS_ERROR("Failed to add Crossing to the map");
    server_.setAborted();
    return;
  }
  ROS_INFO("Added Crossing with id %d", crossing_setter_srv.response.id); // DEBUG
  result_.descriptor_links.push_back(crossingDescriptorLink(crossing_setter_srv.response.id));
  
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = getCompletionDuration();
  server_.setSucceeded(result_);
}

void Jockey::onGetEdgesDescriptors()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onLocalizeInVertex()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onLocalizeEdge()
{
  result_.state = lama_jockeys::LocalizeResult::NOT_SUPPORTED;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onGetDissimilarity()
{
  getData();

  // Get all scans from database.
  lama_interfaces::ActOnMap srv;
  srv.request.action = lama_interfaces::ActOnMapRequest::GET_VERTEX_LIST;
  ROS_INFO("Calling action GET_VERTEX_LIST"); // DEBUG
  if (!map_agent_.call(srv))
  {
    ROS_ERROR("Failed to call map agent");
    server_.setAborted();
    return;
  }
  ROS_INFO("Received %zu vertices", srv.response.objects.size()); // DEBUG
  
  // Iterate over vertices and get the associated Polygon (from the PlaceProfile).
  std::vector<int32_t> vertices;
  vertices.reserve(srv.response.objects.size());
  std::vector<geometry_msgs::Polygon> polygons;
  polygons.reserve(srv.response.objects.size());
  for (size_t i = 0; i < srv.response.objects.size(); ++i)
  {
    // Get all PlaceProfile descriptors associated with the current vertex.
    lama_interfaces::ActOnMap desc_srv;
    desc_srv.request.action = lama_interfaces::ActOnMapRequest::GET_DESCRIPTOR_LINKS;
    desc_srv.request.object.id = srv.response.objects[i].id;
    desc_srv.request.interface_name = place_profile_interface_name_;
    map_agent_.call(desc_srv);
    if (desc_srv.response.descriptor_links.empty())
    {
      continue;
    }
    if (desc_srv.response.descriptor_links.size() > 1)
    {
      ROS_WARN_STREAM("More than one descriptor with interface " <<
          place_profile_interface_name_ << " for vertex " <<
          desc_srv.request.object.id << ", taking the first one");
    }
    // Get the first linked PlaceProfile.
    lama_msgs::GetPlaceProfile profile_srv;
    profile_srv.request.id = desc_srv.response.descriptor_links[0].descriptor_id;
    if (!place_profile_getter_.call(profile_srv))
    {
      ROS_ERROR_STREAM(jockey_name_ << ": failed to call service \"" <<
          place_profile_interface_name_ << "\"");
      server_.setAborted();
      return;
    }
    vertices.push_back(desc_srv.request.object.id);
    polygons.push_back(profile_srv.response.descriptor.polygon);
  }
  
  // Compare them to the current polygon by calling one of the pm_* service.
  polygon_matcher::PolygonDissimilarity dissimi_srv;
  dissimi_srv.request.polygon1 = profile_.polygon;
  result_.idata.clear();
  result_.fdata.clear();
  result_.idata.reserve(vertices.size());
  result_.fdata.reserve(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    dissimi_srv.request.polygon2 = polygons[i];
    if (!dissimilarity_server_.call(dissimi_srv))
    {
      ROS_ERROR_STREAM(jockey_name_ << ": failed to call service \"" << 
          dissimilarity_server_name_ << "\"");
      server_.setAborted();
      return;
    }
    result_.idata.push_back(vertices[i]);
    result_.fdata.push_back(dissimi_srv.response.raw_dissimilarity);
  }

  ROS_INFO("%s: computed %zu dissimilarities", jockey_name_.c_str(),
      result_.idata.size());
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = getCompletionDuration();
  server_.setSucceeded(result_);
}

/* Return a DescriptorLink for the PlaceProfile interface
 */
lama_msgs::DescriptorLink Jockey::placeProfileDescriptorLink(const int32_t id)
{
  lama_msgs::DescriptorLink descriptor_link;
  descriptor_link.descriptor_id = id;
  descriptor_link.interface_name = place_profile_interface_name_;
  return descriptor_link;
}

/* Return a DescriptorLink for the Crossing interface
 */
lama_msgs::DescriptorLink Jockey::crossingDescriptorLink(const int32_t id)
{
  lama_msgs::DescriptorLink descriptor_link;
  descriptor_link.descriptor_id = id;
  descriptor_link.interface_name = crossing_interface_name_;
  return descriptor_link;
}

} // namespace lj_costmap
